
#ifndef STM32F103XX_USART_DRIVER_H_
#define STM32F103XX_USART_DRIVER_H_

/********************************* Abbreviations
 * ************************************/
/*
    USART : Universal Synchronous Asynchronous Receiver Transmitter
    PCLK : Peripheral Clock
    IRQ  : Interrupt Request
*/

/********************************* Includes
 * ************************************/
#include "stm32f103xx.h"

/********************************* USART Config
 * ************************************/
typedef struct {
  uint32_t USART_Mode;
  uint32_t USART_BaudRate;
  uint32_t USART_NoOfStopBits;
  uint32_t USART_WordLength;
  uint32_t USART_ParityControl;
  uint32_t USART_HwFlowControl;
} USART_Config_s;

typedef struct {
  USART_Config_s *USART_Config_p;
  USART_RegDef_s *USARTx_p;
} USART_Handle_s;

/********************************* API's ************************************/

/* USART Peripheral Clock Control */
void USART_PCLK_Control(USART_RegDef_s *USARTx_p);

/* USART Init and Deinit */
void USART_Init(void);
void USART_Deinit(void);

/* USART Transmitter and Receiver */
void USART_Tx(void);
void USART_Rx(void);

/* USART interrupt Configuration and handling */
void USART_IRQ_Config(uint8_t IRQ_Number_u8, uint8_t EnOrDi_u8);
void USART_IRQ_Priority(uint8_t IRQ_Number_u8, uint8_t IRQ_Priority_u8);
void USART_IRQ_Handling(uint8_t PinNumber_u8);

#endif /* STM32F103XX_USART_DRIVER_H_ */