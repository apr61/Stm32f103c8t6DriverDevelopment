

/********************************* Abbreviations ************************************/
/*
	I2C : Inter Integrated Circuit
	PCLK : Peripheral Clock
	IRQ  : Interrupt Request 
*/

/********************************* Includes ************************************/
#include "stm32f103xx.h"
#include "stm32f103xx_I2C_driver.h"
#include "stm32f103xx_RCC_driver.h"

/* Private function declarations */
static void I2C_GenerateStartCondition(I2C_RegDef_s * I2Cx_p);
static void I2C_CompleteAddressPhase(I2C_RegDef_s * I2Cx_p, uint8_t SlaveAddress, I2C_RW_Mode_e I2C_RW_Mode);
static void I2C_ClearAddressFlag(I2C_RegDef_s * I2Cx_p);
static void I2C_GenerateStopCondition(I2C_RegDef_s * I2Cx_p);


void I2C_PCLK_Control(I2C_RegDef_s * I2Cx_p, Status_e EnOrDi_e)
{
	if(EnOrDi_e == ENABLE)
	{
		if(I2Cx_p == I2C1)
		{
			I2C1_PCLK_EN();
		}
		if(I2Cx_p == I2C2)
		{
			I2C2_PCLK_EN();
		}
	} 
	else
	{
		if(I2Cx_p == I2C1)
		{
			I2C1_PCLK_DI();
		}
		if(I2Cx_p == I2C2)
		{
			I2C2_PCLK_DI();
		}
	}
} /* END of I2C_PCLK_Control */

void I2C_PERIPH_Enable(I2C_RegDef_s * I2Cx_p, Status_e EnOrDi_e)
{
	if(EnOrDi_e == ENABLE)
	{
		I2Cx_p->CR1 |= (1 << I2C_CR1_PE_POS);
	} 
	else
	{
		I2Cx_p->CR1 &= ~(1 << I2C_CR1_PE_POS);
	}
}/* END of I2C_PERIPH_Enable */


void I2C_Init(I2C_Handle_s * I2C_Handle_p)
{
	uint32_t TempReg_u32 = 0;
	uint16_t CCR_Value_u16 = 0;
	/* 1. Configure the mode (Standard or Fast) */
	if(I2C_Handle_p->I2C_Config.I2C_SCL_Speed <= I2C_SCL_SPEED_SM_100K)
	{
		/* Standard mode */
		/* By default mode is in Standard 0 */
		/* Calculate CCR value */
		CCR_Value_u16 = (RCC_GetPCLK1Value() / (2 * I2C_Handle_p->I2C_Config.I2C_SCL_Speed));
		TempReg_u32 |= (CCR_Value_u16 & 0xFFF);
	} 
	else
	{
		/* Set Fast Mode */
		TempReg_u32 |= (1 << I2C_CCR_F_S_POS);
		/* Set Duty Cycle for Fast mode */
		TempReg_u32 |= (1 << I2C_CCR_DUTY_POS);
		/* Calculate CCR value */
		if(I2C_Handle_p->I2C_Config.I2C_FmDutyCycle == I2C_FM_DUTY_CYCLE_2)
		{
			/* Fm mode duty cycle = 0 */
			CCR_Value_u16 = (RCC_GetPCLK1Value() / (3 * I2C_Handle_p->I2C_Config.I2C_SCL_Speed));
		}
		else
		{
			/* Fm mode duty cycle = 1 */
			CCR_Value_u16 = (RCC_GetPCLK1Value() / (25 * I2C_Handle_p->I2C_Config.I2C_SCL_Speed));
		}
		TempReg_u32 |= (CCR_Value_u16 & 0xFFF);
	}
	I2C_Handle_p->I2Cx_p->CCR = TempReg_u32;
	
	/* 2. Configure the Speed of SCLK (SCL) */
	TempReg_u32 = 0;
	TempReg_u32 = (RCC_GetPCLK1Value() / 1000000u);
	I2C_Handle_p->I2Cx_p->CR2 = (TempReg_u32 & 0x3F);
	
	/* 3. Configure the device address (When device is slave) */
	TempReg_u32 = 0;
	TempReg_u32 |= (I2C_Handle_p->I2C_Config.I2C_DeviceAddress << 1u);
	TempReg_u32 |= (1u << 14u); /* Mentioned in Reference Manual */
	I2C_Handle_p->I2Cx_p->OAR1 |= TempReg_u32;
	
	/* 4. Enable ACK */
	TempReg_u32 = 0;
	TempReg_u32 |= (I2C_Handle_p->I2C_Config.I2C_ACK_Control << I2C_CR1_ACK_POS);
	I2C_Handle_p->I2Cx_p->CR1 |= TempReg_u32;
	
	/* 5. Configure the rise time for I2C Pins (Standard or Fast) */
	TempReg_u32 = 0;
	if(I2C_Handle_p->I2C_Config.I2C_SCL_Speed <= I2C_SCL_SPEED_SM_100K)
	{
		/* Standard mode */
		TempReg_u32 = ((RCC_GetPCLK1Value() * 1000u) / 1000000000u);
	}
	else
	{
		/* Fast mode */
		TempReg_u32 = ((RCC_GetPCLK1Value() * 300u) / 1000000000u);
	}
	I2C_Handle_p->I2Cx_p->TRISE = (TempReg_u32 & 0x3Fu);
	
}/* END of I2C_Init */


void I2C_DeInit(I2C_RegDef_s * I2Cx_p)
{
	if(I2Cx_p == I2C1)
	{
		I2C1_REG_RESET();
	}
	if(I2Cx_p == I2C2)
	{
		I2C2_REG_RESET();
	}
} /* END of I2C_DeInit */


void I2C_MasterSendData(I2C_Handle_s * I2C_Handle_p, uint8_t * TxBuffer_p,uint32_t Len_u32, uint8_t SlaveAddress, I2C_RepeatedStart_e RepeatedStart_e)
{
	// 1. Generate start condition
	I2C_GenerateStartCondition(I2C_Handle_p->I2Cx_p);

	// 2. Confirm whether Start condtion is completed by checking the SB flag of SR1
	// NOTE : Until SB is cleared, SCL will be streched(Pulled LOW)
	while(READ_BIT(I2C_Handle_p->I2Cx_p->SR1, I2C_SR1_SB_POS) == SET);

	// 3. Send the slave address with Read(1) or Write(0) bit (8 bits)
	I2C_CompleteAddressPhase(I2C_Handle_p->I2Cx_p, SlaveAddress, I2C_Write_Mode);

	// 4. Check wheather the address phase is completed by checking the ADDR flag of SR1
	while(READ_BIT(I2C_Handle_p->I2Cx_p->SR1, I2C_SR1_ADDR_POS) == SET);

	// 5. Clear the ADDR flag of SR1 according to software sequence
	// NOTE : Untill ADDR is cleared, SCL will be streched(Pulled LOW)
	I2C_ClearAddressFlag(I2C_Handle_p->I2Cx_p);

	// 6. Send data until Len_u32 == 0
	while(Len_u32 > 0)
	{	
		while(READ_BIT(I2C_Handle_p->I2Cx_p->SR1, I2C_SR1_TxE_POS) == SET);
		I2C_Handle_p->I2Cx_p->DR = *TxBuffer_p;
		TxBuffer_p++;
		Len_u32--;
	}

	// 7. When Len_u32 == 0, wait until TXE == 1 and BTF == 1 before generating stop condition
	// NOTE : Until TXE = 1, BTF = 1, means DR are empty and Next Byte Transfer should begin
	// When BTF = 1, SCL will be streched(Pulled LOW)
	
	while(READ_BIT(I2C_Handle_p->I2Cx_p->SR1, I2C_SR1_TxE_POS) == SET);
	while(READ_BIT(I2C_Handle_p->I2Cx_p->SR1, I2C_SR1_BTF_POS) == SET);

	// 8. Generate stop condition
	// NOTE : Generating Stops condition will clear the BTF flag of SR1
	if(RepeatedStart_e == I2C_RepeatedStart_Disable)
	{	
		I2C_GenerateStopCondition(I2C_Handle_p->I2Cx_p);
	}
	
}/* END of I2C_MasterSendData */



void I2C_MasterReceiveData(I2C_Handle_s * I2C_Handle_p, uint8_t * RxBuffer_p,uint32_t Len_u32, uint8_t SlaveAddress, I2C_RepeatedStart_e RepeatedStart_e)
{
	// 1. Generate start condition
	I2C_GenerateStartCondition(I2C_Handle_p->I2Cx_p);
	
	// 2. Confirm whether Start condtion is completed by checking the SB flag of SR1
	// NOTE : Until SB is cleared, SCL will be streched(Pulled LOW)
	while(READ_BIT(I2C_Handle_p->I2Cx_p->SR1, I2C_SR1_SB_POS) == SET);

	// 3. Send the slave address with Read(1) or Write(0) bit (8 bits)
	I2C_CompleteAddressPhase(I2C_Handle_p->I2Cx_p, SlaveAddress, I2C_Read_Mode);

	// 4. Check wheather the address phase is completed by checking the ADDR flag of SR1
	while(READ_BIT(I2C_Handle_p->I2Cx_p->SR1, I2C_SR1_ADDR_POS) == SET);

	// Procedure for 1 byte of length
	if(Len_u32 == 1)
	{
		// Disable Acknowldegement
		I2C_ManageAcking(I2C_Handle_p->I2Cx_p, I2C_Ack_Disable);

		// Clear address flag
		I2C_ClearAddressFlag(I2C_Handle_p->I2Cx_p);

		// Wait until RxNE = 1
		while(READ_BIT(I2C_Handle_p->I2Cx_p->SR1, I2C_SR1_RxNE_POS) == SET);

		// Generate Stop condition
		if(RepeatedStart_e == I2C_RepeatedStart_Disable)
		{	
			I2C_GenerateStopCondition(I2C_Handle_p->I2Cx_p);
		}
		
		// Read from DR
		*RxBuffer_p = I2C_Handle_p->I2Cx_p->DR;
	}
	else
	{
		// Clear ADDrR flag
		I2C_ClearAddressFlag(I2C_Handle_p->I2Cx_p);
		
		while(Len_u32 > 0)
		{
			// Wait until RxNE = 1
			while(READ_BIT(I2C_Handle_p->I2Cx_p->SR1, I2C_SR1_RxNE_POS) == SET);

			if(Len_u32 == 2)
			{
				// Clear the ACK bit
				I2C_ManageAcking(I2C_Handle_p->I2Cx_p, I2C_Ack_Disable);
				
				// Generate Stop condition
				if(RepeatedStart_e == I2C_RepeatedStart_Disable)
				{	
					I2C_GenerateStopCondition(I2C_Handle_p->I2Cx_p);
				}
			}
			// Read from DR
			*RxBuffer_p = I2C_Handle_p->I2Cx_p->DR;
			
			// Increment pointer
			RxBuffer_p++;
			
			// Decrement length
			Len_u32--;
		}
	}
	// Enable Acknowledgement
	if(I2C_Handle_p->I2C_Config.I2C_ACK_Control == I2C_Ack_Enable)
	{	
		I2C_ManageAcking(I2C_Handle_p->I2Cx_p, I2C_Ack_Enable);
	}
}/* END of I2C_MasterReceiveData */


void I2C_ManageAcking(I2C_RegDef_s * I2Cx_p, I2C_ACK_Control_e Ack_Status)
{
	if(Ack_Status == I2C_Ack_Enable)
	{
		I2Cx_p -> CR1 |= (1 << I2C_CR1_ACK_POS);
	}
	else
	{
		I2Cx_p -> CR1 &= ~(1 << I2C_CR1_ACK_POS);
	}
} /* END of I2C_ManageAcking */


/* Private function Definitions */

void I2C_GenerateStartCondition(I2C_RegDef_s * I2Cx_p)
{
	I2Cx_p -> CR1 |= (1 << I2C_CR1_START_POS);
}/* END of I2C_GenerateStartCondition */

void I2C_CompleteAddressPhase(I2C_RegDef_s * I2Cx_p, uint8_t SlaveAddress, I2C_RW_Mode_e I2C_RW_Mode)
{
	SlaveAddress = (SlaveAddress << 1u); /* Make space for Read/Write bit */
	if(I2C_RW_Mode == I2C_Read_Mode)
	{
		SlaveAddress |= (1u << 0); /* Set bit for Read mode */
	}
	else
	{
		SlaveAddress &= ~(1u << 0); /* Clearing for Write mode */
	}	
	I2Cx_p->DR = SlaveAddress;
}/* END of I2C_CompleteAddressPhase */

void I2C_ClearAddressFlag(I2C_RegDef_s * I2Cx_p)
{
	uint32_t TempReg_u32 = 0;
	TempReg_u32 = I2Cx_p->SR1;
	TempReg_u32 = I2Cx_p->SR2;
	(void) TempReg_u32;
}/* END of I2C_ClearAddressFlag */

void I2C_GenerateStopCondition(I2C_RegDef_s * I2Cx_p)
{
	I2Cx_p -> CR1 |= (1 << I2C_CR1_STOP_POS);
}/* END of I2C_GenerateStopCondition */
