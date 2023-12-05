#ifndef STM32F103XX_RCC_DRIVER_H_
#define STM32F103XX_RCC_DRIVER_H_

/********************************* Abbreviations *************************************/
/*
	RCC : Reset and Clock Control
	PCLK : Peripheral Clock
	IRQ  : Interrupt Request
*/

/********************************* Includes *************************************/
#include "stm32f103xx.h"

uint32_t RCC_GetPCLK1Value(void);
uint32_t RCC_GetPCLK2Value(void);
uint32_t RCC_GetPLLOutputClock(void);

#endif /* STM32F103XX_RCC_DRIVER_H_ */
