/*
 * stm32f103xx_GPIO_driver.h
 *
 *  Created on: Nov 14, 2023
 *      Author: Pradeep
 */

#ifndef STM32F103XX_GPIO_DRIVER_H_
#define STM32F103XX_GPIO_DRIVER_H_

/********************************* Abbreviations ************************************/
/*
    GPIO : General Purpose Input and Output
    PCLK : Peripheral Clock
    IRQ  : Interrupt Request 
*/

/********************************* Includes ************************************/
#include "stm32f103xx.h"

// GPIO configuration
typedef struct {
  uint8_t GPIOPinNumber; /* Refer to @GPIO_PIN_NUMBERS */
  uint8_t GPIOPinMode;  /* Refer to @GPIO_PIN_MODES, @GPIO_OUT_SPEED */
  uint8_t GPIOPinSpeed; /* Refer to @GPIO_OUT_SPEED */
  uint8_t GPIOPinCNF; /* Refer to @GPIO_PIN_IN_CNF, @GPIO_PIN_OUT_CNF */
} GPIO_PinConfig_s;

// GPIO Handle
typedef struct {
  GPIO_RegDef_s *GPIOx_p;
  GPIO_PinConfig_s GPIO_PinConfig;
} GPIO_Handle_s;

/* @GPIO_PIN_NUMBERS */
#define GPIO_PIN_0                           0
#define GPIO_PIN_1                           1
#define GPIO_PIN_2                           2
#define GPIO_PIN_3                           3
#define GPIO_PIN_4                           4
#define GPIO_PIN_5                           5
#define GPIO_PIN_6                           6
#define GPIO_PIN_7                           7
#define GPIO_PIN_8                           8
#define GPIO_PIN_9                           9
#define GPIO_PIN_10                          10
#define GPIO_PIN_11                          11
#define GPIO_PIN_12                          12
#define GPIO_PIN_13                          13
#define GPIO_PIN_14                          14
#define GPIO_PIN_15                          15

/*
    @GPIO_PIN_MODES
*/

/* General Purpose input mode */
#define GPIO_MODE_IN                       0x00000000u; /* 0b00000000000000000000000000000000 */
#define GPIO_MODE_IN_ANALOG                0x00000001u; /* 0b00000000000000000000000000000001 */
#define GPIO_MODE_IN_PU_PD                 0x00000002u; /* 0b00000000000000000000000000000010 - PULL UP/DOWN */
#define GPIO_MODE_IN_FLOATING              0x00000003u; /* 0b00000000000000000000000000000011 */

/* General Purpose output mode */
#define GPIO_MODE_OUT                      0x00000004u; /* 0b00000000000000000000000000000100 */
#define GPIO_MODE_OUT_PUSH_PULL            0x00000005u; /* 0b00000000000000000000000000000101 */
#define GPIO_MODE_OUT_OD                   0x00000006u; /* 0b00000000000000000000000000000110 - Open Drain */

/* Alternate functionality mode for Output */
#define GPIO_MODE_OUT_ALT_PUSH_PULL        0x00000007u; /* 0b00000000000000000000000000000111 */
#define GPIO_MODE_OUT_ALT_OD               0x00000008u; /* 0b00000000000000000000000000001000 - Open Drain */

#define GPIO_MODE_IN                          0 /* GPIO MODE INT */

// Input Types @GPIO_PIN_IN_CNF
#define GPIO_CNF_IN_ANALOG                    0 /* GPIO INPUT ANALOG */
#define GPIO_CNF_IN_FLOATING                  1 /* GPIO INPUT Pull Up Pull Down */
#define GPIO_CNF_IN_PU_UP_DOWN                2 /* GPIO INPUT Pull Up Pull Down */

// Output Types @GPIO_PIN_OUT_CNF
#define GPIO_CNF_OUT_PS_PL                    0 /* GPIO PUSH PULL */
#define GPIO_CNF_OUT_OPEN_DR                  1 /* GPIO OPEN DRAIN */
#define GPIO_CNF_OUT_ALT_FUN_PS_PL            2 /* Alt fun PUSH PULL */   
#define GPIO_CNF_OUT_ALT_FUN_OPEN_DR          3 /* Alt fun OPEN DRAIN */   

// Speed of output mode, @GPIO_OUT_SPEED
#define GPIO_SPEED_MEDIUM            1 /* GPIO MODE OUT, MAX speed 10MHz */
#define GPIO_SPEED_LOW               2 /* GPIO MODE OUT, MAX speed 2MHz */
#define GPIO_SPEED_HIGH              3 /* GPIO MODE OUT, MAX speed 50MHz */

/* GPIO API's */

/* GPIO peripheral clock enable and disable */
void GPIO_PCLK_Control(GPIO_RegDef_s * GPIOx_p, uint8_t EnOrDi_u8);

/* GPIO Init, DeInit*/
void GPIO_Init(GPIO_Handle_s * GPIO_Handle_p);
void GPIO_DeInit(GPIO_RegDef_s * GPIOx_p);

/* GPIO read/write */
uint8_t GPIO_ReadInputPin(GPIO_RegDef_s * GPIOx_p, uint8_t PinNumber_u8);
uint16_t GPIO_ReadInputPort(GPIO_RegDef_s * GPIOx_p);
void GPIO_WriteInputPin(GPIO_RegDef_s * GPIOx_p, uint8_t PinNumber_u8, uint8_t Value_u8);
void GPIO_WriteInputPort(GPIO_RegDef_s * GPIOx_p, uint16_t Value_u16);
void GPIO_TogglePin(GPIO_RegDef_s * GPIOx_p, uint8_t PinNumber_u8);

/* GPIO interrupt Configuration and handling */
void GPIO_IRQ_Config(uint8_t IRQ_Number_u8, uint8_t IRQ_Priority_u8, uint8_t EnOrDi_u8);
void GPIO_IRQ_Handling(uint8_t PinNumber_u8);

#endif /* STM32F103XX_GPIO_DRIVER_H_ */

/********************************* END of file ************************************/

