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
    uint8_t Mode_CNF = 0x0U; // Store MODEx and CNFx value for each pin (x = 0,1,2,.....15)
    uint8_t PinNumber = GPIO_Handle_p->GPIO_PinConfig.GPIOPinNumber;
    uint8_t bitFieldOffSet;

    // MODEx [1:0] && CNFx [1:0] Configurations
    switch(GPIO_Handle_p->GPIO_PinConfig.GPIOPinMode)
    {
        // GPIO output mode PUSH PULL
        case GPIO_MODE_OUT_PUSH_PULL:
            Mode_CNF = ((GPIO_CR_CNF_OUT_GP_PS_PL << 2) | GPIO_Handle_p->GPIO_PinConfig.GPIOPinSpeed);
            break;
        // GPIO output mode Open Drain
        case GPIO_MODE_OUT_OD:
            Mode_CNF = ((GPIO_CR_CNF_OUT_GP_OPEN_DR << 2) | GPIO_Handle_p->GPIO_PinConfig.GPIOPinSpeed);
            break;
        // Alternate functionality PUSH PULL
        case GPIO_MODE_ALT_PUSH_PULL:
            Mode_CNF = ((GPIO_CR_CNF_OUT_ALT_FUN_PS_PL << 2) | GPIO_Handle_p->GPIO_PinConfig.GPIOPinSpeed);
            break;
        // Alternate functionality open drain
        case GPIO_MODE_ALT_OD:
            Mode_CNF = ((GPIO_CR_CNF_OUT_ALT_FUN_OPEN_DR << 2) | GPIO_Handle_p->GPIO_PinConfig.GPIOPinSpeed);
            break;
        // GPIO Input mode
        case GPIO_MODE_INPUT:
        case GPIO_MODE_INT_FALLING_TRI:
        case GPIO_MODE_INT_RAISING_TRI:
        case GPIO_MODE_INT_RAISING_FALLING:
            // GPIO pull Floating
            if(GPIO_Handle_p->GPIO_PinConfig.GPIOPinPull == GPIO_NO_PULL)
            {
                Mode_CNF = ((GPIO_CR_CNF_IN_FLOATING << 2) | GPIO_CR_MODE_IN);
            } 
            else if (GPIO_Handle_p->GPIO_PinConfig.GPIOPinPull == GPIO_PULL_UP) // GPIO input PULL UP
            {
                Mode_CNF = ((GPIO_CR_CNF_IN_PU_UP_DOWN << 2) | GPIO_CR_MODE_IN);
                GPIO_Handle_p->GPIOx_p->BSRR |= (1<<PinNumber);
            }
            else // GPIO input PULL DOWN
            {
                Mode_CNF = ((GPIO_CR_CNF_IN_PU_UP_DOWN << 2) | GPIO_CR_MODE_IN);
                GPIO_Handle_p->GPIOx_p->BRR |= (1<<PinNumber);
            }
            break;
        // GPIO INPUT analog
        case GPIO_MODE_ANALOG:
            Mode_CNF = ((GPIO_CR_CNF_IN_ANALOG << 2) | GPIO_CR_MODE_IN);
            break;
        default:
            break;
    }
    /* Check for First half bits (0 - 7) */
    if(PinNumber <= 7)
    {
    	GPIO_Handle_p->GPIOx_p->CRL &= ~(0xF << (PinNumber * 4));
    	GPIO_Handle_p->GPIOx_p->CRL |= (Mode_CNF << (PinNumber * 4));
    }
    else     /* Check for Second half bits (8 - 15) */
    {
    	GPIO_Handle_p->GPIOx_p->CRH &= ~(0xF << (PinNumber * 4));
    	GPIO_Handle_p->GPIOx_p->CRH |= (Mode_CNF << (PinNumber * 4));
    }


    /*--------------------- EXTI Mode Configuration ------------------------*/
    if((GPIO_Handle_p->GPIO_PinConfig.GPIOPinMode & EXTI_MODE) == EXTI_MODE)
    {
        /* Enable AFIO Clock */
        AFIO_PCLK_EN();

        /* Enable EXTICR register for given pin */
        // Here divison is used to get the EXTICR register out of 4 register
        // Modulo is used to get the bit field offSet
        // GPIO_GET_INDEX macro gives the index of GPIO port
        /*
            Example: PinNumber = 12
            PinNumber / 4 = 3 (3rd EXTICR register)
            PinNumber % 4 = 0 (0th bit field)
        */
        bitFieldOffSet = (PinNumber % 4) * 4; // Range in register that is 0,4,8,12
        AFIO->EXTICR[PinNumber / 4] &= ~(0xF << bitFieldOffSet);  // Clearing the current register Bitfield offset
        AFIO->EXTICR[PinNumber / 4] |= (GPIO_GET_INDEX(GPIO_Handle_p->GPIOx_p) << bitFieldOffSet);  // Setting the current register Bitfield offset with required port

        /* Edge detection for Interrupt */
        /* Falling edge trigger */
        if(GPIO_Handle_p->GPIO_PinConfig.GPIOPinMode == GPIO_MODE_INT_FALLING_TRI)
        {
            EXTI->FTSR |= (1 << PinNumber);
        }
        else
        {
            EXTI->FTSR &= ~(1 << PinNumber);
        }

        /* Raisng edge trigger */
        if(GPIO_Handle_p->GPIO_PinConfig.GPIOPinMode == GPIO_MODE_INT_RAISING_TRI)
        {
            EXTI->RTSR |= (1 << PinNumber);
        }
        else
        {
            EXTI->RTSR &= ~(1 << PinNumber);
        }


        /* Raising - Falling edge trigger */
        if(GPIO_Handle_p->GPIO_PinConfig.GPIOPinMode == GPIO_MODE_INT_RAISING_FALLING)
        {
            EXTI->RTSR |= (1 << PinNumber);
            EXTI->FTSR |= (1 << PinNumber);
        }
        else
        {
            EXTI->RTSR &= ~(1 << PinNumber);
            EXTI->FTSR &= ~(1 << PinNumber);
        }

        /* Configure IMR for EXTI interrupt delivery */
        EXTI->IMR |= (1 << PinNumber);
    }

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
    TempValue_u8 = (uint8_t)((GPIOx_p->IDR >> PinNumber_u8) & 0x1);
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
    Function name    :    GPIO_WriteOutputPin
    Description      :    This function writes the input pin value
    Parameters       :    GPIO_RegDef_s * GPIOx_p : Pointer to GPIO peripheral
                          uint8_t GPIO_Pin_u8 : GPIO pin Number
                          uint8_t GPIO_PinValue_u8 : GPIO pin value (0 or 1)
    Return           :    None
    Note             :    None
*/
void GPIO_WriteOutputPin(GPIO_RegDef_s * GPIOx_p, uint8_t PinNumber_u8, uint8_t Value_u8)
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
    Function name    :    GPIO_WriteOutputPort
    Description      :    This function writes the input port value
    Parameters       :    GPIO_RegDef_s * GPIOx_p : Pointer to GPIO peripheral
                          uint16_t Value_u16 : GPIO 16-bit value
    Return           :    None
    Note             :    None
*/
void GPIO_WriteOutputPort(GPIO_RegDef_s * GPIOx_p, uint16_t Value_u16)
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
                          uint8_t IRQ_Mode_u8 : ENABLE or DISABLE IRQ
    Return           :    None
    Note             :    None
*/
void GPIO_IRQ_Config(uint8_t IRQ_Number_u8, uint8_t EnOrDi_u8)
{
    if(EnOrDi_u8 == ENABLE)
    {
        if(IRQ_Number_u8 <= 31 && IRQ_Number_u8 >= 0)
        {
            *NVIC_ISER0 |= (uint32_t)(1 << IRQ_Number_u8);
        }
        else /* IRQ Number 32 to 64 */
        {
            *NVIC_ISER1 |= (uint32_t)(1 << (IRQ_Number_u8 % 32));
        }
    }
    else
    {
        if(IRQ_Number_u8 <= 31 && IRQ_Number_u8 >= 0)
        {
            *NVIC_ICER0 |= (uint32_t)(1 << IRQ_Number_u8);
        }
        else /* IRQ Number 32 to 64 */
        {
            *NVIC_ICER1 |= (uint32_t)(1 << (IRQ_Number_u8 % 32));
        }
    }
}/* END of GPIO_IRQ_Config */

/*
    Function name    :    GPIO_IRQ_Priority
    Description      :    This function configures the GPIO interrupt
    Parameters       :    uint8_t IRQ_Number_u8 : IRQ number of the GPIO
                          uint8_t IRQ_Priority_u8 : Priority of the IRQ
    Return           :    None
    Note             :    None
*/
void GPIO_IRQ_Priority(uint8_t IRQ_Number_u8, uint8_t IRQ_Priority_u8)
{
    uint8_t IPRx_u8 = IRQ_Number_u8 / 4; // Get the required IPR register number
    uint8_t IPRx_Section_u8 = IRQ_Number_u8 % 4; // Get the required IPR register number
    uint8_t ShiftAmount_u8 = (IPRx_Section_u8 * 8) + (8 - NUM_PR_BITS_IMPLEMENTED); // Last 4 bits are not implemented

    *(NVIC_PR_BASE_ADDR + IPRx_u8) |= (uint32_t)(IRQ_Priority_u8 << ShiftAmount_u8);
}/* END of GPIO_IRQ_Priority */

/*
    Function name    :    GPIO_IRQ_Handling
    Description      :    This function handles the GPIO interrupt
    Parameters       :    uint8_t PinNumber_u8 : GPIO pin of the PORT
    Return           :    None
    Note             :    None
*/
void GPIO_IRQ_Handling(uint8_t PinNumber_u8)
{
    // Clear the EXTI PR register corresponding to the pin number
    if((EXTI->PR & (1 << PinNumber_u8)) == SET)
    {
        // Clear the bit by setting it to 1
        EXTI->PR |= (uint32_t)(1 << PinNumber_u8);
    }
}/* END of GPIO_IRQ_Handling */


/********************************* END of file ************************************/
