/*
 * stm32f103xx_GPIO_driver.c
 *
 *  Created on: Nov 14, 2023
 *      Author: Pradeep
 */

/********************************* Abbreviations ************************************/
/*
    GPIO : General Purpose Input and Output
    PCLK : Peripheral Clock
    IRQ  : Interrupt Request 
*/

/********************************* Includes ************************************/
#include "stm32f103xx_GPIO_driver.h"

/* GPIO peripheral clock enable and disable */
/*
    Function name    :    GPIO_PCLK_Control
    Description      :    This function enables or disables the GPIO peripheral clock
    Parameters       :    GPIO_RegDef_s * GPIOx_p : Pointer to GPIO peripheral
                          uint8_t EnOrDi_u8 : Enable or Disable
    Return           :    None
    Note             :    None
*/
void GPIO_PCLK_Control(GPIO_RegDef_s * GPIOx_p, uint8_t EnOrDi_u8)
{
    if (ENABLE == EnOrDi_u8)
    {
        if(GPIOA == GPIOx_p)
        {
            GPIOA_PCLK_EN();
        }
        else if (GPIOB == GPIOx_p)
        {
            GPIOB_PCLK_EN();
        }
        else if (GPIOC == GPIOx_p)
        {
            GPIOC_PCLK_EN();
        }
        else if (GPIOD == GPIOx_p)
        {
            GPIOD_PCLK_EN();
        }
        else if (GPIOE == GPIOx_p)
        {
            GPIOE_PCLK_EN();
        }
    }
    if (DISABLE == EnOrDi_u8)
    {
        if(GPIOA == GPIOx_p)
        {
            GPIOA_PCLK_DI();
        }
        else if (GPIOB == GPIOx_p)
        {
            GPIOB_PCLK_DI();
        }
        else if (GPIOC == GPIOx_p)
        {
            GPIOC_PCLK_DI();
        }
        else if (GPIOD == GPIOx_p)
        {
            GPIOD_PCLK_DI();
        }
        else if (GPIOE == GPIOx_p)
        {
            GPIOE_PCLK_DI();
        }
    }
} /* END of GPIO_PCLK_Control function*/

/* GPIO Init, DeInit*/
/*
    Function name    :    GPIO_Init
    Description      :    This function initializes the GPIO peripheral
    Parameters       :    GPIO_Handle_s * GPIO_Handle_p : Pointer to GPIO peripheral
    Return           :    None
    Note             :    None
*/
void GPIO_Init(GPIO_Handle_s * GPIO_Handle_p)
{
    uint32_t Temp_u32 = 0; // Temp Register
    
    /*
        1. Configure mode of GPIO
            - Input Mode
    */ 
    if(GPIO_Handle_p->GPIO_PinConfig.GPIOPinNumber <= 7)
    {
        Temp_u32 = (GPIO_Handle_p->GPIO_PinConfig.GPIOPinMode << (4 * GPIO_Handle_p->GPIO_PinConfig.GPIOPinNumber));
        GPIO_Handle_p->GPIOx_p->CRL |= Temp_u32;
    }
    else
    {
        Temp_u32 = (GPIO_Handle_p->GPIO_PinConfig.GPIOPinMode << (4 * (GPIO_Handle_p->GPIO_PinConfig.GPIOPinNumber - 8)));
        GPIO_Handle_p->GPIOx_p->CRH |= Temp_u32;
    }

    Temp_u32 = 0;
    
    // 2. Configure the speed
    if(GPIO_Handle_p->GPIO_PinConfig.GPIOPinNumber <= 7)
    {
        Temp_u32 = (GPIO_Handle_p->GPIO_PinConfig.GPIOPinSpeed << (4 * GPIO_Handle_p->GPIO_PinConfig.GPIOPinNumber));
        GPIO_Handle_p->GPIOx_p->CRL |= Temp_u32;
    }
    else
    {
        Temp_u32 = (GPIO_Handle_p->GPIO_PinConfig.GPIOPinSpeed << (4 * (GPIO_Handle_p->GPIO_PinConfig.GPIOPinNumber - 8)));
        GPIO_Handle_p->GPIOx_p->CRH |= Temp_u32;
    }

    Temp_u32 = 0;
    
    /*
    *    3. Configure the GPIO Pin, 
    *    Input - (Pull Up - Pull Down or Floating or Analog), 
    *    Output - General Purpose / Alternate Function (Open-Drain or Push-Pull) 
    */
    
    if(GPIO_Handle_p->GPIO_PinConfig.GPIOPinNumber <= 7)
    {
        Temp_u32 = (GPIO_Handle_p->GPIO_PinConfig.GPIOPinCNF << ((4 * GPIO_Handle_p->GPIO_PinConfig.GPIOPinNumber) + 2));
        GPIO_Handle_p->GPIOx_p->CRL |= Temp_u32;
    }
    else
    {
        Temp_u32 = (GPIO_Handle_p->GPIO_PinConfig.GPIOPinSpeed << ((4 * (GPIO_Handle_p->GPIO_PinConfig.GPIOPinNumber - 8)) + 2));
        GPIO_Handle_p->GPIOx_p->CRH |= Temp_u32;
    }

    // 4. Alternate Functioning
    // TODO :: 
    
    // 5. Interrupt based
    // TODO :: 
} /* END of GPIO_Init */

/*
    Function name    :    GPIO_DeInit
    Description      :    This function de-initializes the GPIO peripheral
    Parameters       :    GPIO_RegDef_s * GPIOx_p : Pointer to GPIO peripheral
    Return           :    None
    Note             :    None
*/
void GPIO_DeInit(GPIO_RegDef_s * GPIOx_p)
{
    if(GPIOA == GPIOx_p)
    {
        GPIOA_REG_RESET();
    }
    else if (GPIOB == GPIOx_p)
    {
        GPIOB_REG_RESET();
    }
    else if (GPIOC == GPIOx_p)
    {
        GPIOC_REG_RESET();
    }
    else if (GPIOD == GPIOx_p)
    {
        GPIOD_REG_RESET();
    }
    else if (GPIOE == GPIOx_p)
    {
        GPIOE_REG_RESET();
    }
}/* END of GPIO_DeInit */

/* GPIO read/write */
/*
    Function name    :    GPIO_ReadInputPin
    Description      :    This function reads the input pin value
    Parameters       :    GPIO_RegDef_s * GPIOx_p : Pointer to GPIO peripheral
                          uint8_t GPIO_Pin_u8 : GPIO pin Number
    Return           :    Values stored in the particular GPIO_Pin which is (0 or 1)
    Note             :    None
*/
uint8_t GPIO_ReadInputPin(GPIO_RegDef_s * GPIOx_p, uint8_t PinNumber_u8)
{
    uint8_t TempValue_u8 = 0;
    TempValue_u8 = (uint8_t)((GPIOx_p->IDR >> PinNumber_u8) & 0x00000001); 
    return TempValue_u8;
}/* END of GPIO_ReadInputPin */

/*
    Function name    :    GPIO_ReadInputPort
    Description      :    This function reads the input port value
    Parameters       :    GPIO_RegDef_s * GPIOx_p : Pointer to GPIO peripheral
    Return           :    Values stored in the particular GPIO_Port of type (uint16_t)
    Note             :    None
*/
uint16_t GPIO_ReadInputPort(GPIO_RegDef_s * GPIOx_p)
{
    uint16_t TempValue_u16 = 0;
    TempValue_u16 = (uint16_t)(GPIOx_p->IDR); 
    return TempValue_u16;
}/* END of GPIO_ReadInputPort */

/*
    Function name    :    GPIO_WriteInputPin
    Description      :    This function writes the input pin value
    Parameters       :    GPIO_RegDef_s * GPIOx_p : Pointer to GPIO peripheral
                          uint8_t GPIO_Pin_u8 : GPIO pin Number
                          uint8_t GPIO_PinValue_u8 : GPIO pin value (0 or 1)
    Return           :    None
    Note             :    None
*/
void GPIO_WriteInputPin(GPIO_RegDef_s * GPIOx_p, uint8_t PinNumber_u8, uint8_t Value_u8)
{
    if((uint8_t)GPIO_SET_PIN == Value_u8)
    {
        GPIOx_p->ODR |= (uint32_t)(1 << PinNumber_u8);
    }
    else
    {
        GPIOx_p->ODR &= ~(uint32_t)(1 << PinNumber_u8);
    }
}/* END of GPIO_WriteInputPin */

/*
    Function name    :    GPIO_WriteInputPort
    Description      :    This function writes the input port value
    Parameters       :    GPIO_RegDef_s * GPIOx_p : Pointer to GPIO peripheral
                          uint16_t Value_u16 : GPIO 16-bit value
    Return           :    None
    Note             :    None
*/
void GPIO_WriteInputPort(GPIO_RegDef_s * GPIOx_p, uint16_t Value_u16)
{
    GPIOx_p->ODR = Value_u16;
}/* END of GPIO_WriteInputPort */

/*
    Function name    :    GPIO_TogglePin
    Description      :    This function toggles the pin value of a GPIO port
    Parameters       :    GPIO_RegDef_s * GPIOx_p : Pointer to GPIO peripheral
                          uint8_t PinNumber_u8 : GPIO pin to toggle
    Return           :    None
    Note             :    None
*/
void GPIO_TogglePin(GPIO_RegDef_s * GPIOx_p, uint8_t PinNumber_u8)
{
    GPIOx_p -> ODR ^= (uint32_t)(1 << PinNumber_u8);
}/* END of GPIO_TogglePin */

/* GPIO interrupt Configuration and handling */
/*
    Function name    :    GPIO_IRQ_Config
    Description      :    This function configures the GPIO interrupt
    Parameters       :    uint8_t IRQ_Number_u8 : IRQ number of the GPIO
                          uint8_t IRQ_Priority_u8 : Priority of the IRQ
                          uint8_t IRQ_Mode_u8 : ENABLE or DISABLE IRQ
    Return           :    None
    Note             :    None
*/
void GPIO_IRQ_Config(uint8_t IRQ_Number_u8, uint8_t IRQ_Priority_u8, uint8_t EnOrDi_u8)
{
    
}/* END of GPIO_IRQ_Config */

/*
    Function name    :    GPIO_IRQ_Handling
    Description      :    This function handles the GPIO interrupt
    Parameters       :    uint8_t PinNumber_u8 : GPIO pin of the PORT
    Return           :    None
    Note             :    None
*/
void GPIO_IRQ_Handling(uint8_t PinNumber_u8)
{
    
}/* END of GPIO_IRQ_Handling */


/********************************* END of file ************************************/
