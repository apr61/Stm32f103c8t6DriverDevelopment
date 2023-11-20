/**
 ******************************************************************************
 * @file           : main.c
 * @author         : Auto-generated by STM32CubeIDE
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2023 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */

#include "stm32f103xx.h"
#include "stm32f103xx_GPIO_driver.h"

void delay(void)
{
    for(uint32_t i=0;i<500000/4;i++);
}

int main(void)
{
    GPIO_Handle_s GPIO_Led, GPIO_Btn;
    GPIO_Led.GPIOx_p = GPIOA;
    GPIO_Led.GPIO_PinConfig.GPIOPinNumber = GPIO_PIN_5;
    GPIO_Led.GPIO_PinConfig.GPIOPinMode = GPIO_MODE_OUT_PUSH_PULL;
    GPIO_Led.GPIO_PinConfig.GPIOPinSpeed = GPIO_SPEED_HIGH;
    GPIO_Led.GPIO_PinConfig.GPIOPinPull = GPIO_NO_PULL;

    GPIO_Btn.GPIOx_p = GPIOB;
    GPIO_Btn.GPIO_PinConfig.GPIOPinNumber = GPIO_PIN_0;
    GPIO_Btn.GPIO_PinConfig.GPIOPinMode = GPIO_MODE_INPUT;
    GPIO_Btn.GPIO_PinConfig.GPIOPinSpeed = GPIO_SPEED_HIGH;
    GPIO_Btn.GPIO_PinConfig.GPIOPinPull = GPIO_PULL_UP;

    GPIO_PCLK_Control(GPIOB, ENABLE);
    GPIO_PCLK_Control(GPIOA, ENABLE);
    GPIO_Init(&GPIO_Led);
    GPIO_Init(&GPIO_Btn);

    while(1)
    {
        if(GPIO_ReadInputPin(GPIOB, GPIO_PIN_0) == GPIO_RESET_PIN)
        {
            delay();    
            GPIO_WriteOutputPin(GPIOA, GPIO_PIN_5, GPIO_SET_PIN);
        }
        else
        {
        	delay();
        	GPIO_WriteOutputPin(GPIOA, GPIO_PIN_5, GPIO_RESET_PIN);
        }
    }
}
