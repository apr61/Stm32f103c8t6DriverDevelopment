/*
 * stm32f103xx_GPIO_driver.h
 *
 *  Created on: Nov 14, 2023
 *      Author: Pradeep
 */

#ifndef STM32F103XX_GPIO_DRIVER_H_
#define STM32F103XX_GPIO_DRIVER_H_

/********************************* Abbreviations
 * ************************************/
/*
    GPIO : General Purpose Input and Output
    PCLK : Peripheral Clock
    IRQ  : Interrupt Request
*/

/********************************* Includes
 * ************************************/
#include "stm32f103xx.h"

// GPIO configuration
typedef struct {
  uint32_t GPIOPinNumber; /* Refer to @GPIO_PIN_NUMBERS */
  uint32_t GPIOPinMode;   /* Refer to @GPIO_PIN_MODES */
  uint32_t GPIOPinSpeed;  /* Refer to @GPIO_SPEED */
  uint32_t GPIOPinPull;   /* Refer to @GPIO_Pull_up_down */
} GPIO_PinConfig_s;

// GPIO Handle
typedef struct {
  GPIO_RegDef_s *GPIOx_p;
  GPIO_PinConfig_s GPIO_PinConfig;
} GPIO_Handle_s;

/* @GPIO_PIN_NUMBERS */
#define GPIO_PIN_0 0u
#define GPIO_PIN_1 1u
#define GPIO_PIN_2 2u
#define GPIO_PIN_3 3u
#define GPIO_PIN_4 4u
#define GPIO_PIN_5 5u
#define GPIO_PIN_6 6u
#define GPIO_PIN_7 7u
#define GPIO_PIN_8 8u
#define GPIO_PIN_9 9u
#define GPIO_PIN_10 10u
#define GPIO_PIN_11 11u
#define GPIO_PIN_12 12u
#define GPIO_PIN_13 13u
#define GPIO_PIN_14 14u
#define GPIO_PIN_15 15u

/*
    @GPIO_PIN_MODES
*/

#define EXTI_MODE 0x10000000u
#define GPIO_MODE_IT 0x00010000u
#define GPIO_MODE_EVT 0x00020000u
#define RISING_EDGE 0x00100000u
#define FALLING_EDGE 0x00200000u

/* General Purpose input mode */
#define GPIO_MODE_INPUT 1u
#define GPIO_MODE_ANALOG 2u

/* General Purpose output mode */
#define GPIO_MODE_OUT_PUSH_PULL 3u
#define GPIO_MODE_OUT_OD 4u

/* Alternate functionality mode for Output */
#define GPIO_MODE_ALT_PUSH_PULL 5u
#define GPIO_MODE_ALT_OD 6u /* Open Drain */

/* GPIO External interrupt - Event */
#define GPIO_MODE_INT_FALLING_TRI                                              \
  0x10110000u /* External Interrupt Falling Edge */
#define GPIO_MODE_INT_RAISING_TRI                                              \
  0x10210000u /* External Interrupt Raising Edge */
#define GPIO_MODE_INT_RAISING_FALLING                                          \
  0x10310000u /* External Interrupt Raising - Falling */

#define GPIO_MODE_EVT_FALLING_TRI 0x10120000u /* External Event Falling Edge   \
                                               */
#define GPIO_MODE_EVT_RAISING_TRI 0x10220000u /* External Event Raising Edge   \
                                               */
#define GPIO_MODE_EVT_RAISING_FALLING                                          \
  0x10320000u /* External Event Raising - Falling */

/*
 * 	@GPIO_Pull_up_down defines
 *
 */
#define GPIO_NO_PULL 0x00000000u   /*!< No Pull-up or Pull-down activation  */
#define GPIO_PULL_UP 0x00000001u   /*!< Pull-up activation                  */
#define GPIO_PULL_DOWN 0x00000002u /*!< Pull-down activation                */

// Speed of output mode, @GPIO_SPEED
#define GPIO_SPEED_MEDIUM 0x1u /* GPIO MODE OUT, MAX speed 10MHz */
#define GPIO_SPEED_LOW 0x2u    /* GPIO MODE OUT, MAX speed 2MHz */
#define GPIO_SPEED_HIGH 0x3u   /* GPIO MODE OUT, MAX speed 50MHz */

/*
 *	Bit manipulations for GPIO CRL and CRH registers
 * */

#define GPIO_CR_MODE_IN 0u           /* 00: GPIO MODE INPUT */
#define GPIO_CR_CNF_IN_ANALOG 0u     /* 00: Analog mode */
#define GPIO_CR_CNF_IN_FLOATING 1u   /* 01: Floating input (reset state) */
#define GPIO_CR_CNF_IN_PU_UP_DOWN 2u /* 10: Input with pull-up / pull-down */
#define GPIO_CR_CNF_OUT_GP_PS_PL 0u  /* 00: General purpose output push-pull */
#define GPIO_CR_CNF_OUT_GP_OPEN_DR                                             \
  1u /* 01: General purpose output Open-drain */
#define GPIO_CR_CNF_OUT_ALT_FUN_PS_PL                                          \
  2u /* 10: Alternate function output Push-pull */
#define GPIO_CR_CNF_OUT_ALT_FUN_OPEN_DR                                        \
  3u /* 11: Alternate function output Open-drain */

/* GPIO API's */

/* GPIO peripheral clock enable and disable */
void GPIO_PCLK_Control(GPIO_RegDef_s *GPIOx_p, uint8_t EnOrDi_u8);

/* GPIO Init, DeInit*/
void GPIO_Init(GPIO_Handle_s *GPIO_Handle_p);
void GPIO_DeInit(GPIO_RegDef_s *GPIOx_p);

/* GPIO read/write */
uint8_t GPIO_ReadInputPin(GPIO_RegDef_s *GPIOx_p, uint8_t PinNumber_u8);
uint16_t GPIO_ReadInputPort(GPIO_RegDef_s *GPIOx_p);
void GPIO_WriteOutputPin(GPIO_RegDef_s *GPIOx_p, uint8_t PinNumber_u8,
                         uint8_t Value_u8);
void GPIO_WriteOutputPort(GPIO_RegDef_s *GPIOx_p, uint16_t Value_u16);
void GPIO_TogglePin(GPIO_RegDef_s *GPIOx_p, uint8_t PinNumber_u8);

/* GPIO interrupt Configuration and handling */
void GPIO_IRQ_Config(uint8_t IRQ_Number_u8, uint8_t EnOrDi_u8);
void GPIO_IRQ_Priority(uint8_t IRQ_Number_u8, uint8_t IRQ_Priority_u8);
void GPIO_IRQ_Handling(uint8_t PinNumber_u8);

#endif /* STM32F103XX_GPIO_DRIVER_H_ */

/********************************* END of file
 * ************************************/
