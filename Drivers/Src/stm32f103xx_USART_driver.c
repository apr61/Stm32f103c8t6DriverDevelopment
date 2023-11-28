/********************************* Abbreviations *************************************/
/*
    USART : Universal Synchronous Asynchronous Receiver Transmitter
    PCLK : Peripheral Clock
    IRQ  : Interrupt Request
*/

/********************************* Includes *************************************/
#include "stm32f103xx.h"
#include "stm32f103xx_USART_driver.h"
#include "stm32f103xx_RCC_driver.h"

/* USART peripheral clock enable and disable */
/*
    Function name    :    USART_PCLK_Control
    Description      :    This function enables or disables the USART peripheral clock 
	Parameters       :    USART_RegDef_s * USARTx_p : Pointer to USART peripheral 
   						  PinStatus_e EnOrDi_e : Enable or Disable 
	Return           :    None 
   Note              :    None
*/
void USART_PCLK_Control(USART_RegDef_s *USARTx_p, PinStatus_e EnOrDi_e) {
  if (ENABLE == EnOrDi_e) {
    if (USART1 == USARTx_p) {
      USART1_PCLK_EN();
    } else if (USART2 == USARTx_p) {
      USART2_PCLK_EN();
    } else if (USART3 == USARTx_p) {
      USART3_PCLK_EN();
    }
  }
  if (DISABLE == EnOrDi_e) {
    if (USART1 == USARTx_p) {
      USART1_PCLK_DI();
    } else if (USART2 == USARTx_p) {
      USART2_PCLK_DI();
    } else if (USART3 == USARTx_p) {
      USART3_PCLK_DI();
    }
  }
} /* END of USART_PCLK_Control function*/

/*
	Function name    :    USART_PeripheralControl
	Description      :    This function initializes the USART peripheral
	Parameters       :    USART_Handle_s * USART_Handle_p : Pointer to USART user config peripheral
						  PinStatus_e EnOrDi_e : Enable Or Disable
	Return           :    None
	Note             :    None
*/
void USART_PeripheralControl(USART_RegDef_s *USARTx_p, PinStatus_e EnOrDi_e)
{
	if(EnOrDi_e == ENABLE)
	{
		USARTx_p->CR1 |= (1 << USART_CR1_UE_POS);
	}else
	{
		USARTx_p->CR1 &= ~(1 << USART_CR1_UE_POS);
	}

}/* END of USART_PeripheralControl function*/


/* USART SetBaudRate */
/*
	Function name    :    USART_SetBaudRate
	Description      :    This function sets the Baudrate in BRR register of USARTx.
	Parameters       :    USART_Handle_s * USART_Handle_p : Pointer to USART user config peripheral
						  uint32_t BaudRate_u32 : Baud rate value
	Return           :    None
	Note             :    None
*/
void USART_SetBaudRate(USART_RegDef_s *USARTx_p, uint32_t BaudRate_u32)
{
	//Variable to hold the APB clock
	uint32_t PCLKx_u32;

	uint32_t USARTDIV_u32;

	//variables to hold Mantissa and Fraction values
	uint32_t M_Part_u32,F_Part_u32;

	uint32_t TempReg_u32=0;

	//Get the value of APB bus clock in to the variable PCLKx_u32
	if(USARTx_p == USART1)
  	{
		//USART1 and USART6 are hanging on APB2 bus
		PCLKx_u32 = RCC_GetPCLK2Value();
	}
	else
	{
		PCLKx_u32 = RCC_GetPCLK1Value();
  	}

	//Check for OVER8 configuration bit
	USARTDIV_u32 = ((25u * PCLKx_u32) / (4u * BaudRate_u32));
	
  	//Calculate the Mantissa part
  	M_Part_u32 = (USARTDIV_u32 / 100u);
	
  	//Place the Mantissa part in appropriate bit position . refer USART_BRR
  	TempReg_u32 |= (M_Part_u32 << 4u);

 	//Extract the fraction part
  	F_Part_u32 = (USARTDIV_u32 - (M_Part_u32 * 100u));

  	//Calculate the final fractional
	F_Part_u32 = (((F_Part_u32 * 16u)+ 50u) / 100u) & (0xFu);
	
  	//Place the fractional part in appropriate bit position . refer USART_BRR
  	TempReg_u32 |= F_Part_u32;

  	//copy the value of TempReg_u32 in to BRR register
  	USARTx_p->BRR = TempReg_u32;
}/* END of USART_SetBaudRate */

/* USART Init, DeInit*/
/*
	Function name    :    USART_Init
	Description      :    This function initializes the USART peripheral iwth user config settings.
	Parameters       :    USART_Handle_s * USART_Handle_p : Pointer to USART user config peripheral
	Return           :    None
	Note             :    None
*/

void USART_Init(USART_Handle_s * USART_Handle_p)
{
	//Temporary variable
	uint32_t TempReg_u32=0;

	/******************************** Configuration of CR1******************************************/
	//Implement the code to enable the Clock for given USART peripheral
	USART_PCLK_Control(USART_Handle_p->USARTx_p, ENABLE);

	//Enable USART Tx and Rx engines according to the USART_Mode configuration item
	if ( USART_Handle_p->USART_Config.USART_Mode == USART_MODE_ONLY_RX)
	{
		//Implement the code to enable the Receiver bit field 
		TempReg_u32|= (1 << USART_CR1_RE_POS);
	}
	else if (USART_Handle_p->USART_Config.USART_Mode == USART_MODE_ONLY_TX)
	{
		//Implement the code to enable the Transmitter bit field 
		TempReg_u32 |= ( 1 << USART_CR1_TE_POS);
	}
	else if (USART_Handle_p->USART_Config.USART_Mode == USART_MODE_TXRX)
	{
		//Implement the code to enable the both Transmitter and Receiver bit fields 
		TempReg_u32 |= (( 1 << USART_CR1_RE_POS) | ( 1 << USART_CR1_TE_POS));
	}

	//Implement the code to configure the Word length configuration item 
	TempReg_u32 |= (USART_Handle_p->USART_Config.USART_WordLength << USART_CR1_M_POS) ;
	
	//Configuration of parity control bit fields
	if (USART_Handle_p->USART_Config.USART_ParityControl == USART_PARITY_EN_EVEN)
	{
		//Implement the code to enale the parity control 
		TempReg_u32 |= (1 << USART_CR1_PCE_POS);
		//Implement the code to enable EVEN parity 
		//Not required because by default EVEN parity will be selected once you enable the parity control 
	}
	else if (USART_Handle_p->USART_Config.USART_ParityControl == USART_PARITY_EN_ODD)
	{
		//Implement the code to enable the parity control 
		TempReg_u32 |= (1 << USART_CR1_PCE_POS);
		//Implement the code to enable ODD parity - SET
		TempReg_u32 |= ( 1 << USART_CR1_PS_POS);
	}
	
	//Program the CR1 register 
	USART_Handle_p->USARTx_p->CR1 = TempReg_u32;

	/******************************** Configuration of CR2******************************************/
	TempReg_u32=0;
	
	//Implement the code to configure the number of stop bits inserted during USART frame transmission 
	TempReg_u32 |= (USART_Handle_p->USART_Config.USART_NoOfStopBits << USART_CR2_STOP_POS);
	
	//Program the CR2 register 
	USART_Handle_p->USARTx_p->CR2 = TempReg_u32;

	/******************************** Configuration of CR3******************************************/
	TempReg_u32=0;
	
	//Configuration of USART hardware flow control 
	if (USART_Handle_p->USART_Config.USART_HwFlowControl == USART_HW_FLOW_CTRL_CTS)
	{
		//Implement the code to enable CTS flow control 
		TempReg_u32 |= (1 << USART_CR3_CTSE_POS);
	}
	else if (USART_Handle_p->USART_Config.USART_HwFlowControl == USART_HW_FLOW_CTRL_RTS)
	{
		//Implement the code to enable RTS flow control 
		TempReg_u32 |= (1 << USART_CR3_RTSE_POS);
	}
	else if (USART_Handle_p->USART_Config.USART_HwFlowControl == USART_HW_FLOW_CTRL_CTS_RTS)
	{
		//Implement the code to enable both CTS and RTS Flow control 
		TempReg_u32 |= ((1 << USART_CR3_RTSE_POS) | (1 << USART_CR3_CTSE_POS));
	}
	USART_Handle_p->USARTx_p->CR3 = TempReg_u32;

	/******************************** Configuration of BRR(BaudRate_u32 register)******************************************/
	//Implement the code to configure the baud rate
	//We will cover this in the lecture. No action required here 
	USART_SetBaudRate(USART_Handle_p->USARTx_p, USART_Handle_p->USART_Config.USART_BaudRate);
} /* END of USART_Init */

/*
	Function name    :    USART_DeInit
	Description      :    This function de-initializes the USART peripheral
	Parameters       :    USART_RegDef_s * USARTx_p : Pointer to USART peripheral
	Return           :    None
	Note             :    None
*/

void USART_DeInit(USART_RegDef_s *USARTx_p)
{
	if(USART1 == USARTx_p)
	{
		USART1_REG_RESET();
	} 
	else if (USART2 == USARTx_p) 
	{
		USART2_REG_RESET();
	} 
	else if (USART3 == USARTx_p)
	{
		USART3_REG_RESET();
	}
} /* END of USART_Init */

/*
	Function name    :    USART_Tx
	Description      :    This function used to transmit data to the connected device
	Parameters       :    
	Return           :    None
	Note             :    None
*/
void USART_Tx(USART_Handle_s *USART_Handle_p, uint8_t * TxBuffer_p, uint32_t Len_u32)
{
	uint16_t *Data_p;
	//Loop over until "Len" number of bytes are transferred
	for(uint32_t i = 0 ; i < Len_u32; i++)
	{
		//Implement the code to wait until TXE flag is set in the SR
		while(RESET == READ_BIT(USART_Handle_p->USARTx_p->SR, USART_SR_TXE_POS))
		{
			;
		}
		 //Check the USART_WordLength item for 9BIT or 8BIT in a frame
		if(USART_Handle_p->USART_Config.USART_WordLength == USART_WORDLEN_9BITS)
		{
			//if 9BIT, load the DR with 2bytes masking the bits other than first 9 bits 
			Data_p = (uint16_t*) TxBuffer_p;
			USART_Handle_p->USARTx_p->DR = (*Data_p & (uint16_t)0x01FFu);
			//check for USART_ParityControl
			if(USART_Handle_p->USART_Config.USART_ParityControl == USART_PARITY_DISABLE)
			{
				//No parity is used in this transfer. so, 9bits of user data will be sent
				//Implement the code to increment TxBuffer_p twice 
				TxBuffer_p++;
				TxBuffer_p++;
			}
			else
			{
				//Parity bit is used in this transfer . so , 8bits of user data will be sent
				//The 9th bit will be replaced by parity bit by the hardware
				TxBuffer_p++;
			}
		}
		else
		{
			//This is 8bit data transfer 
			USART_Handle_p->USARTx_p->DR = (*TxBuffer_p  & (uint8_t)0xFFu);
			//Implement the code to increment the buffer address
			TxBuffer_p++;
		}
	}
	//Implement the code to wait till TC flag is set in the SR
	while(RESET == READ_BIT(USART_Handle_p->USARTx_p->SR, USART_SR_TC_POS))
	{
		;
	}
} /* END of USART_Tx */

/*
	Function name    :    USART_Rx
	Description      :    This function used to received data to the connected device
	Parameters       :    
	Return           :    None
	Note             :    None
*/

void USART_Rx(USART_Handle_s *USART_Handle_p, uint8_t * RxBuffer_p, uint32_t Len_u32)
{
	//Loop over until "Len" number of bytes are transferred
	for(uint32_t i = 0 ; i < Len_u32; i++)
	{
		//Implement the code to wait until RXNE flag is set in the SR
		while(RESET == READ_BIT(USART_Handle_p -> USARTx_p->SR, USART_SR_RXNE_POS))
		{
			;
		}

		//Check the USART_WordLength to decide whether we are going to receive 9bit of data in a frame or 8 bit
		if(USART_Handle_p->USART_Config.USART_WordLength == USART_WORDLEN_9BITS)
		{
			//We are going to receive 9bit data in a frame

			//check are we using USART_ParityControl control or not
			if(USART_Handle_p->USART_Config.USART_ParityControl == USART_PARITY_DISABLE)
			{
				//No parity is used. so, all 9bits will be of user data

				//read only first 9 bits. so, mask the DR with 0x01FF
				*((uint16_t*) RxBuffer_p) = (USART_Handle_p->USARTx_p->DR  & (uint16_t)0x01FF);

				//Now increment the RxBuffer_p two times
				RxBuffer_p++;
				RxBuffer_p++;
			}
			else
			{
				//Parity is used, so, 8bits will be of user data and 1 bit is parity
				 *RxBuffer_p = (USART_Handle_p->USARTx_p->DR  & (uint8_t)0xFF);
				 //Increment the RxBuffer_p
				RxBuffer_p++;
			}
		}
		else
		{
			//We are going to receive 8bit data in a frame
			//check are we using USART_ParityControl control or not
			if(USART_Handle_p->USART_Config.USART_ParityControl == USART_PARITY_DISABLE)
			{
				//No parity is used , so all 8bits will be of user data
				//read 8 bits from DR
				*RxBuffer_p = (uint8_t) (USART_Handle_p->USARTx_p->DR  & (uint8_t)0xFF);
			}
			else
			{
				//Parity is used, so , 7 bits will be of user data and 1 bit is parity
				//read only 7 bits , hence mask the DR with 0X7F
				*RxBuffer_p = (uint8_t) (USART_Handle_p->USARTx_p->DR  & (uint8_t)0x7F);
			}
			//increment the RxBuffer_p
			RxBuffer_p++;
		}
	}
} /* END of USART_Rx */

/*
	Function name    :    USART_TxIT
	Description      :    This function used to transmit data to the connected device
	Parameters       :    
	Return           :    None
	Note             :    Interrupt based
*/
uint8_t USART_TxIT(USART_Handle_s *USART_Handle_p, uint8_t * TxBuffer_p, uint32_t Len_u32)
{
	uint8_t TxState_u8 = USART_Handle_p->TxState_u8;
	if(TxState_u8 != USART_BUSY_IN_TX)
	{
		USART_Handle_p->TxLen_u32 = Len_u32;
		USART_Handle_p->TxBuffer_p = TxBuffer_p;
		USART_Handle_p->TxState_u8 = USART_BUSY_IN_TX;

		//Implement the code to enable interrupt for TXE
		USART_Handle_p->USARTx_p->CR1 |= (1 << USART_CR1_TXEIE_POS);
		//Implement the code to enable interrupt for TC 
		USART_Handle_p->USARTx_p->CR1 |= (1 << USART_CR1_TCIE_POS);
	}
	return TxState_u8;
} /* END of USART_TxIT */

/*
	Function name    :    USART_RxIT
	Description      :    This function used to received data to the connected device
	Parameters       :    
	Return           :    None
	Note             :    Interrupt Based
*/

uint8_t USART_RxIT(USART_Handle_s *USART_Handle_p, uint8_t * RxBuffer_p, uint32_t Len_u32)
{
	uint8_t RxState_u8 = USART_Handle_p->RxState_u8;
	if(RxState_u8 != USART_BUSY_IN_RX)
	{
		USART_Handle_p->RxLen_u32 = Len_u32;
		USART_Handle_p->RxBuffer_p = RxBuffer_p;
		USART_Handle_p->RxState_u8 = USART_BUSY_IN_RX;
		//Implement the code to enable interrupt for RXNE
		USART_Handle_p->USARTx_p->CR1 |= (1 << USART_CR1_RXNEIE_POS);
	}
	return RxState_u8;
} /* END of USART_RxIT */

/* USART interrupt Configuration and handling */
/*
	Function name    :    USART_IRQ_Config
	Description      :    This function configures the USART interrupt
	Parameters       :    uint8_t IRQ_Number_u8 : IRQ number of the USART
						  PinStatus_e EnOrDi_e) : ENABLE or DISABLE IRQ
	Return           :    None
	Note             :    None
*/
void USART_IRQ_Config(uint8_t IRQ_Number_u8, PinStatus_e EnOrDi_e)
{
	if(EnOrDi_e == ENABLE)
	{
		if(IRQ_Number_u8 <= 31u)
		{
			*NVIC_ISER0 |= (uint32_t)(1 << IRQ_Number_u8);
		}
		else /* IRQ Number 32 to 64 */
		{
			*NVIC_ISER1 |= (uint32_t)(1 << (IRQ_Number_u8 % 32u));
		}
	}
	else
	{
		if(IRQ_Number_u8 <= 31u)
		{
			*NVIC_ICER0 |= (uint32_t)(1 << IRQ_Number_u8);
		}
		else /* IRQ Number 32 to 64 */
		{
			*NVIC_ICER1 |= (uint32_t)(1 << (IRQ_Number_u8 % 32u));
		}
	}
}/* END of USART_IRQ_Config */

/*
	Function name    :    USART_IRQ_Priority
	Description      :    This function configures the USART interrupt
	Parameters       :    uint8_t IRQ_Number_u8 : IRQ number of the USART
						  uint8_t IRQ_Priority_u8 : Priority of the IRQ
	Return           :    None
	Note             :    None
*/
void USART_IRQ_Priority(uint8_t IRQ_Number_u8, uint8_t IRQ_Priority_u8)
{
	uint8_t IPRx_u8 = IRQ_Number_u8 / 4u; // Get the required IPR register number
	uint8_t IPRx_Section_u8 = IRQ_Number_u8 % 4u; // Get the required IPR register number
	uint8_t ShiftAmount_u8 = (IPRx_Section_u8 * 8u) + (8u - NUM_PR_BITS_IMPLEMENTED); // Last 4 bits are not implemented

	*(NVIC_PR_BASE_ADDR + IPRx_u8) |= (uint32_t)(IRQ_Priority_u8 << ShiftAmount_u8);
}/* END of USART_IRQ_Priority */

/*
	Function name    :    USART_IRQ_Handling
	Description      :    This function handles the USART interrupt
	Parameters       :    USART_Handle_s * USART_Handle_p : USART handle
	Return           :    None
	Note             :    None
*/
void USART_IRQ_Handling(USART_Handle_s * USART_Handle_p)
{
	
}/* END of USART_IRQ_Handling */

/********************************* END of file ************************************/
