
#ifndef STM32F103XX_USART_DRIVER_H_
#define STM32F103XX_USART_DRIVER_H_

/********************************* Abbreviations *************************************/
/*
    USART : Universal Synchronous Asynchronous Receiver Transmitter
    PCLK : Peripheral Clock
    IRQ  : Interrupt Request
*/

/********************************* Includes *************************************/
#include "stm32f103xx.h"

/********************************* USART Config *************************************/
typedef struct {
  uint32_t USART_Mode; /* Ref @USART_Mode */
  uint32_t USART_BaudRate; /* Ref @USART_BaudRate */
  uint32_t USART_NoOfStopBits; /* Ref @USART_NoOfStopBits */
  uint32_t USART_WordLength; /* Ref @USART_WordLength */
  uint32_t USART_ParityControl; /* Ref @USART_ParityControl */
  uint32_t USART_HwFlowControl; /* Ref @USART_HWFlowControl */
} USART_Config_s;

typedef struct {
  USART_RegDef_s *USARTx_p;
  USART_Config_s USART_Config;
  uint8_t *TxBuffer_p;
  uint8_t *RxBuffer_p;
  uint8_t TxState_u8;
  uint8_t RxState_u8;
  uint32_t RxLen_u32;
  uint32_t TxLen_u32;
} USART_Handle_s;

/*
 *@USART_Mode
 *Possible options for USART_Mode
 */
#define USART_MODE_ONLY_TX                                   0
#define USART_MODE_ONLY_RX                                   1
#define USART_MODE_TXRX                                      2

/*
 *@USART_BaudRate
 *Possible options for USART_Baud
 */
#define USART_STD_BAUD_1200					                         1200
#define USART_STD_BAUD_2400					                         2400
#define USART_STD_BAUD_9600					                         9600
#define USART_STD_BAUD_19200 				                         19200
#define USART_STD_BAUD_38400 				                         38400
#define USART_STD_BAUD_57600 				                         57600
#define USART_STD_BAUD_115200 				                       115200
#define USART_STD_BAUD_230400 				                       230400
#define USART_STD_BAUD_460800 				                       460800
#define USART_STD_BAUD_921600 				                       921600
#define USART_STD_BAUD_2M 					                         2000000
#define SUART_STD_BAUD_3M 					                         3000000


/*
 *@USART_ParityControl
 *Possible options for USART_ParityControl
 */
#define USART_PARITY_EN_ODD                                  2
#define USART_PARITY_EN_EVEN                                 1
#define USART_PARITY_DISABLE                                 0

/*
 *@USART_WordLength
 *Possible options for USART_WordLength
 */
#define USART_WORDLEN_8BITS                                  0
#define USART_WORDLEN_9BITS                                  1

/*
 *@USART_NoOfStopBits
 *Possible options for USART_NoOfStopBits
 */
#define USART_STOPBITS_1                                     0
#define USART_STOPBITS_0_5                                   1
#define USART_STOPBITS_2                                     2
#define USART_STOPBITS_1_5                                   3

/*
 *@USART_HWFlowControl
 *Possible options for USART_HWFlowControl
 */
#define USART_HW_FLOW_CTRL_NONE    	                         0
#define USART_HW_FLOW_CTRL_CTS    	                         1
#define USART_HW_FLOW_CTRL_RTS    	                         2
#define USART_HW_FLOW_CTRL_CTS_RTS	                         3

/*
 * Application states
 */
#define USART_BUSY_IN_RX 1
#define USART_BUSY_IN_TX 2
#define USART_READY 0

/********************************* API's ************************************/

/* USART Peripheral Clock Control */
void USART_PCLK_Control(USART_RegDef_s *USARTx_p, PinStatus_e EnOrDi_e);
void USART_PeripheralControl(USART_RegDef_s *USARTx_p, PinStatus_e EnOrDi_e);

/* USART Init and Deinit */
void USART_Init(USART_Handle_s * USART_Handle_p);
void USART_DeInit(USART_RegDef_s *USARTx_p);

/* USART Transmitter and Receiver */
void USART_Tx(USART_Handle_s *USART_Handle_p, uint8_t * TxBuffer_p, uint32_t Len_u32);
void USART_Rx(USART_Handle_s *USART_Handle_p, uint8_t * RxBuffer_p, uint32_t Len_u32);
/* USART Transmitter and Receiver Interrupt based*/
uint8_t USART_TxIT(USART_Handle_s *USART_Handle_p, uint8_t * TxBuffer_p, uint32_t Len_u32);
uint8_t USART_RxIT(USART_Handle_s *USART_Handle_p, uint8_t * RxBuffer_p, uint32_t Len_u32);

/* USART interrupt Configuration and handling */
void USART_IRQ_Config(uint8_t IRQ_Number_u8, uint8_t EnOrDi_u8);
void USART_IRQ_Priority(uint8_t IRQ_Number_u8, uint8_t IRQ_Priority_u8);
void USART_IRQ_Handling(USART_Handle_s * USART_Handle_p);

/* Other USART API's */

#endif /* STM32F103XX_USART_DRIVER_H_ */
