/*
 * stm32f103xx_GPIO_driver.h
 *
 *  Created on: Nov 14, 2023
 *      Author: Pradeep
 */

#ifndef STM32F103XX_GPIO_DRIVER_H_
#define STM32F103XX_GPIO_DRIVER_H_

#include "stm32f103xx.h"

// GPIO configuration
typedef struct {
  uint8_t GPIOPinNumber;
  uint8_t GPIOPinMode;
  uint8_t GPIOPinSpeed;
  uint8_t GPIOPinPuPdControl;
  uint8_t GPIOPinOPType;
  uint8_t GPIOPinAltFunction;
} GPIO_PinConfig_t;

// GPIO Handle
typedef struct {
  GPIO_RegDef_s *GPIOx_p;
  GPIO_PinConfig_t GPIO_PinConfig;
} GPIO_Handle_s;

/* GPIO API's */

/* GPIO Init, DeInit*/
void GPIO_Init(void);
void GPIO_DeInit(void);

/* GPIO read/write */
void GPIO_ReadInputPin(void);
void GPIO_ReadInputPort(void);
void GPIO_WriteInputPin(void);
void GPIO_WriteInputPort(void);
void GPIO_TogglePin(void);

/* GPIO peripheral clock enable and disable */
void GPIO_PCLK_Control(void);

/* GPIO interrupt Configuration and handling */
void GPIO_IQR_Config(void);
void GPIO_IQR_Handling(void);

#endif /* STM32F103XX_GPIO_DRIVER_H_ */
