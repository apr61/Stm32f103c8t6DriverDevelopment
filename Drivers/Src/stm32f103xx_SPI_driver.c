/********************************* Abbreviations ************************************/
/*
    SPI : General Purpose Input and Output
    PCLK : Peripheral Clock
    IRQ  : Interrupt Request 
*/

/********************************* Includes ************************************/
#include "stm32f103xx_SPI_driver.h"

/* Private Function Declarations */
static void SPI_TXE_Interrupt_Handle(SPI_Handle_s * SPI_Handle_p);
static void SPI_RXNE_Interrupt_Handle(SPI_Handle_s * SPI_Handle_p);
static void SPI_OVR_Interrupt_Handle(SPI_Handle_s * SPI_Handle_p);

/* SPI peripheral clock enable and disable */
/*
    Function name    :    SPI_PCLK_Control
    Description      :    This function enables or disables the SPI peripheral clock
    Parameters       :    SPI_RegDef_s * SPIx_p : Pointer to SPIx peripheral
                          uint8_t EnOrDi_u8 : Enable or Disable
    Return           :    None
    Note             :    None
*/
void SPI_PCLK_Control(SPI_RegDef_s * SPIx_p, uint8_t EnOrDi_u8)
{
    if(ENABLE == EnOrDi_u8)
    {
        if(SPI1 == SPIx_p)
        {
            SPI1_PCLK_EN();
        }
        if(SPI2 == SPIx_p)
        {
            SPI2_PCLK_EN();
        }
    } else {
        if(SPI1 == SPIx_p)
        {
            SPI1_PCLK_DI();
        }
        if(SPI2 == SPIx_p)
        {
            SPI2_PCLK_DI();
        }
    }
} /* END of SPI_PCLK_Control*/

/* SPI Init, DeInit*/
/*
    Function name    :    SPI_Init
    Description      :    This function initializes the SPIx peripheral
    Parameters       :    SPI_Handle_s * SPI_Handle_p : Pointer to SPI peripheral
    Return           :    None
    Note             :    None
*/
void SPI_Init(SPI_Handle_s * SPI_Handle_p)
{
    uint32_t TempReg_u32 = 0x00000000;
    /* 1. Configure Device Mode - Master (1) & Slave (0 - Default) */
    TempReg_u32 |= (SPI_Handle_p->SPI_Config.SPI_DeviceMode << SPI_CR1_MSTR_POS);

    /* 2. Configure Bus Config */
    if(SPI_Handle_p->SPI_Config.SPI_BusConfig == SPI_BUS_CONFIG_FULL)
    {
        /* BIDI_MODE should be cleared - Full Duplex */
        TempReg_u32 &= ~(1 << SPI_CR1_BIDI_MODE_POS);
    } 
    else if (SPI_Handle_p->SPI_Config.SPI_BusConfig == SPI_BUS_CONFIG_HALF)
    {
        /* BIDI_MODE should be set - Half duplex */
        TempReg_u32 |= (1 << SPI_CR1_BIDI_MODE_POS);
    }
    else if (SPI_Handle_p->SPI_Config.SPI_BusConfig == SPI_BUS_CONFIG_SIMPLEX_Rx)
    {
        /* BIDI_MODE should be cleared - Simplex Rx Only*/
         TempReg_u32 &= ~(1 << SPI_CR1_BIDI_MODE_POS);
        /* Rx Only needs tobe set */
        TempReg_u32 |= (1 << SPI_CR1_RXONLY_POS);
    }
    /* 3. Configure Clock Speed */
    TempReg_u32 |= (SPI_Handle_p->SPI_Config.SPI_SCLK_Speed << SPI_CR1_BR_POS);
    /* 4. Configure DFF */
    TempReg_u32 |= (SPI_Handle_p->SPI_Config.SPI_DFF << SPI_CR1_DFF_POS);
    /* 5. Configure CPOL */
    TempReg_u32 |= (SPI_Handle_p->SPI_Config.SPI_CPOL << SPI_CR1_CPOL_POS);
    /* 6. Configure CPHA */
    TempReg_u32 |= (SPI_Handle_p->SPI_Config.SPI_CPHA << SPI_CR1_CPHA_POS);
    /* 7. Configure SSM */
    TempReg_u32 |= (SPI_Handle_p->SPI_Config.SPI_SSM << SPI_CR1_SSM_POS);

    /* Update the SPI CR1 Register */
    SPI_Handle_p->SPIx_p->CR1 = TempReg_u32;
    
} /* END of SPI_Init */

/*
    Function name    :    SPI_DeInit
    Description      :    This function de-initializes the SPI peripheral
    Parameters       :    SPI_RegDef_s * SPIx_p : Pointer to SPI peripheral
    Return           :    None
    Note             :    None
*/
void SPI_DeInit(SPI_RegDef_s * SPIx_p)
{
    if(SPI1 == SPIx_p)
    {
        SPI1_PERI_RESET();
    }
    else if (SPI2 == SPIx_p)
    {
        SPI2_PERI_RESET();
    }
}/* END of SPI_DeInit */

/*
    Function name    :    SPI_PERIPH_Control
    Description      :    This function takes the SPIx base address, enables or disables a SPIx Peripheral.
    Parameters       :    SPI_RegDef_s * SPIx_p : Pointer to SPIx peripheral
                          PinStatus_e PinStatus : Enable or Disable
    Return           :    None
    Note             :    None
*/
void SPI_PERIPH_Control(SPI_RegDef_s * SPIx_p, PinStatus_e PinStatus)
{
    if(PinStatus == ENABLE)
    {
        /* Enable the SPIx Peripheral */
        SPIx_p->CR1 |= (1 << SPI_CR1_SPE_POS);
    }
    else
    {
        /* Disable the SPIx Peripheral */
        /* Wait until the BUS is free (BSY is 0 )*/
        while((SPIx_p->SR & SPI_SR_BSY_FLAGMASK) == SET)
        {
             /* Wait until the BUS is free */
        }
        SPIx_p->CR1 &= ~(1 << SPI_CR1_SPE_POS);
    }
}/* END of SPI_PERIPH_Control */

/*
    Function name    :    SPI_SendData
    Description      :    This function takes the TxBuffer as argument and sends the data 
                          using SPI communication
    Parameters       :    SPI_RegDef_s * SPIx_p : Pointer to SPIx peripheral
                          uint8_t *TxBuffer_p : Pointer to SPI Tx Buffer
                          uint32_t DataLen_u32 : Length of data that need to be transmitted
    Return           :    None
    Note             :    Polling method
*/
void SPI_SendData(SPI_RegDef_s * SPIx_p, uint8_t *TxBuffer_p, uint32_t DataLen_u32)
{
    while(DataLen_u32 > 0)
    {
        /* Wait till Tx Buffer is Empty */
        while(SPI_SR_GetFlagStatus(SPIx_p, SPI_SR_TXE_FLAGMASK) == FLAG_RESET);

        /* Check for DDF */
        if((SPIx_p->CR1 & (1u << SPI_CR1_DFF_POS)) == SET)
        {
            /* 16 bit DFF */
            SPIx_p->DR = *(uint16_t *)TxBuffer_p;
            /* Decrement by 2 for 16 bit */
            DataLen_u32--;
            DataLen_u32--;
            /* Increment pointer index to next Data */
            (uint16_t *)TxBuffer_p++;
        }
        else
        {
            /* 8 bit DFF */
            SPIx_p->DR = *TxBuffer_p;
            /* Decrement by 1 for 8 bit */
            DataLen_u32--;
            /* Increment pointer index to next Data */
            TxBuffer_p++;
        }
    }
}/* END of SPI_SendData */

/*
    Function name    :    SPI_ReceiveData
    Description      :    This function takes the RxBuffer as argument and updated 
                          RxBuffer with the received data using SPI communication
    Parameters       :    SPI_RegDef_s * SPIx_p : Pointer to SPIx peripheral
                          uint8_t *TxBuffer_p : Pointer to SPI Tx Buffer
                          uint32_t DataLen_u32 : Length of data that need to be transmitted
    Return           :    None
    Note             :    Polling Method
*/
void SPI_ReceiveData(SPI_RegDef_s * SPIx_p, uint8_t *RxBuffer_p, uint32_t DataLen_u32)
{
    while(DataLen_u32 > 0)
    {
        /* Wait till Tx Buffer is Empty */
        while(SPI_SR_GetFlagStatus(SPIx_p, SPI_SR_RXNE_FLAGMASK) == FLAG_RESET);

        /* Check for DDF */
        if((SPIx_p->CR1 & (1u << SPI_CR1_DFF_POS)) == SET)
        {
            /* 16 bit DFF */
            *((uint16_t *)RxBuffer_p) = SPIx_p->DR;
            /* Decrement by 2 for 16 bit */
            DataLen_u32--;
            DataLen_u32--;
            /* Increment pointer index to next Data */
            (uint16_t *)RxBuffer_p++;
        }
        else
        {
            /* 8 bit DFF */
            *(RxBuffer_p) = SPIx_p->DR;
            /* Decrement by 1 for 8 bit */
            DataLen_u32--;
            /* Increment pointer index to next Data */
            RxBuffer_p++;
        }
    }
}/* END of SPI_ReceiveData */

/*
    Function name    :    SPI_SR_GetFlagStatus
    Description      :    This function takes the SPIx base address, checks whether 
                          a particular bit is set or not and returns the SET or RESET falg.
    Parameters       :    SPI_RegDef_s * SPIx_p : Pointer to SPIx peripheral
                          uint16_t FlagMask : Flag mask for SR register
    Return           :    FlagValue_e {FLAG_SET, FLAG_RESET}
    Note             :    None
*/
FlagValue_e SPI_SR_GetFlagStatus(SPI_RegDef_s * SPIx_p, uint16_t FlagMask)
{
    FlagValue_e FlagStatus;
    if((SPIx_p->SR & FlagMask) == FLAG_SET)
    {
        FlagStatus = FLAG_SET;
    }
    else
    {
        FlagStatus = FLAG_RESET;
    }
    return FlagStatus;
}/* END of SPI_ReceiveData */

/* SPI interrupt Configuration and handling */
/*
    Function name    :    SPI_IRQ_Config
    Description      :    This function configures the SPI interrupt
    Parameters       :    uint8_t IRQ_Number_u8 : IRQ number of the SPI
                          uint8_t IRQ_Mode_u8 : ENABLE or DISABLE IRQ
    Return           :    None
    Note             :    None
*/
void SPI_IRQ_Config(uint8_t IRQ_Number_u8, uint8_t EnOrDi_u8)
{
    if(EnOrDi_u8 == ENABLE)
    {
        if(IRQ_Number_u8 <= 31u)
        {
            *NVIC_ISER0 |= (1 << IRQ_Number_u8);
        }
        else /* IRQ Number 32 to 64 */
        {
            *NVIC_ISER1 |= (1 << (IRQ_Number_u8 % 32u));
        }
    }
    else
    {
        if(IRQ_Number_u8 <= 31u)
        {
            *NVIC_ICER0 |= (uint32_t)(1 << IRQ_Number_u8);
        }
        else /* IRQ Number 32 to 64 */
        {
            *NVIC_ICER1 |= (uint32_t)(1 << (IRQ_Number_u8 % 32u));
        }
    }
}/* END of SPI_IRQ_Config */

/*
    Function name    :    SPI_IRQ_Priority
    Description      :    This function configures the SPI interrupt
    Parameters       :    uint8_t IRQ_Number_u8 : IRQ number of the SPI
                          uint8_t IRQ_Priority_u8 : Priority of the IRQ
    Return           :    None
    Note             :    None
*/
void SPI_IRQ_Priority(uint8_t IRQ_Number_u8, uint8_t IRQ_Priority_u8)
{
    uint8_t IPRx_u8 = IRQ_Number_u8 / 4u; // Get the required IPR register number
    uint8_t IPRx_Section_u8 = IRQ_Number_u8 % 4u; // Get the required IPR register number
    uint8_t ShiftAmount_u8 = (IPRx_Section_u8 * 8u) + (8u - NUM_PR_BITS_IMPLEMENTED); // Last 4 bits are not implemented

    *(NVIC_PR_BASE_ADDR + IPRx_u8) |= (uint32_t)(IRQ_Priority_u8 << ShiftAmount_u8);
}/* END of SPI_IRQ_Priority */

/*
    Function name    :    SPI_IRQ_Handling
    Description      :    This function handles the SPI interrupt
    Parameters       :    uint8_t PinNumber_u8 : SPI pin of the PORT
    Return           :    None
    Note             :    None
*/
void SPI_IRQ_Handling(SPI_Handle_s * SPI_Handle_p)
{
    uint8_t Temp1_u8, Temp2_u8;
    /* Handling for Tx Interrupt */
    Temp1_u8 = (SPI_Handle_p -> SPIx -> SR & (1u << SPI_SR_TXE_POS));
    Temp2_u8 = (SPI_Handle_p -> SPIx -> CR2 & (1u << SPI_CR2_TXEIE_Pos));
    if(Temp1_u8 == SET && Temp2_u8 == SET)
    {
        SPI_TXE_Interrupt_Handle(SPI_Handle_p);
    }
    /* Handling for Rx Interrupt */
    Temp1_u8 = (SPI_Handle_p -> SPIx -> SR & (1u << SPI_SR_RXNE_POS));
    Temp2_u8 = (SPI_Handle_p -> SPIx -> CR2 & (1u << SPI_CR2_RXNEIE_Pos));
    if(Temp1_u8 == SET && Temp2_u8 == SET)
    {
        SPI_RXNE_Interrupt_Handle(SPI_Handle_p);
    }
    /* Handling for OVERFLOW bit */
    Temp1_u8 = (SPI_Handle_p -> SPIx -> SR & (1u << SPI_SR_OVR_POS));
    Temp2_u8 = (SPI_Handle_p -> SPIx -> CR2 & (1u << SPI_CR2_ERRIE_POS));
    if(Temp1_u8 == SET && Temp2_u8 == SET)
    {
        SPI_OVR_Error_Interrupt_Handle(SPI_Handle_p);
    }
}/* END of SPI_IRQ_Handling */

/* SPI Send and Receive data Interrupt methods */
/*
    Function name    :    SPI_ReceiveDataIT
    Description      :    This function takes the RxBuffer as argument and updated 
                          RxBuffer with the received data using SPI communication
    Parameters       :    SPI_Handle_s * SPI_Handle_p : Pointer to SPI Handle
                          uint8_t *TxBuffer_p : Pointer to SPI Tx Buffer
                          uint32_t DataLen_u32 : Length of data that need to be transmitted
    Return           :    uint8_t - Status of the SPI Ref @SPI_States
    Note             :    Interrupt Method
*/
uint8_t SPI_SendDataIT(SPI_Handle_s * SPI_Handle_p, uint8_t *TxBuffer_p, uint32_t DataLen_u32)
{
    uint8_t State_u8 = SPI_Handle_p->TxState_u8;
    if(State_u8 != SPI_BUSY_IN_TX)
    {
        /* 1. Save the TxBuffer Address and TxLen in Some Global Varaible (SPI_Handle_p) */
        SPI_Handle_p->TxBuffer_p = TxBuffer_p;
        SPI_Handle_p->TxLen_u32 = DataLen_u32;
        
        /* 2. Mark the SPI state as busy in transmission so that no other code 
            can take over the same SPI peripeheral until Tx is Complete 
        */
        SPI_Handle_p->TxState_u8 = SPI_BUSY_IN_TX;
        
        /* 3. Enable TXEIE control bit in SPI CR2, whenever TXE flag bit of SPI_SR is SET */
        SPI_Handle_p->SPIx_p->CR2 |= (1u << SPI_CR2_TXEIE_POS);
        
        /* Data Transmission will be handled by the ISR */
    }
    return State_u8;
}/* END of SPI_SendDataIT */

/*
    Function name    :    SPI_ReceiveDataIT
    Description      :    This function takes the RxBuffer as argument and updated 
                          RxBuffer with the received data using SPI communication
    Parameters       :    SPI_Handle_s * SPI_Handle_p : Pointer to SPI Handle
                          uint8_t *RxBuffer_p : Pointer to SPI Tx Buffer
                          uint32_t DataLen_u32 : Length of data that need to be transmitted
    Return           :    uint8_t - Status of the SPI Ref @SPI_States
    Note             :    Interrupt Method
*/
void SPI_ReceiveDataIT(SPI_Handle_s * SPI_Handle_p, uint8_t *RxBuffer_p, uint32_t DataLen_u32)
{
    uint8_t State_u8 = SPI_Handle_p->RxState_u8;
    if(State_u8 != SPI_BUSY_IN_RX)
    {
        /* 1. Save the TxBuffer Address and TxLen in Some Global Varaible (SPI_Handle_p) */
        SPI_Handle_p->RxBuffer_p = RxBuffer_p;
        SPI_Handle_p->RxLen_u32 = DataLen_u32;

        /* 2. Mark the SPI state as busy in transmission so that no other code 
            can take over the same SPI peripeheral until Tx is Complete 
        */
        SPI_Handle_p->RxState_u8 = SPI_BUSY_IN_RX;

        /* 3. Enable RXNEIE control bit in SPI CR2, whenever RXNE flag bit of SPI_SR is SET */
        SPI_Handle_p->SPIx_p->CR2 |= (1u << SPI_CR2_RXNEIE_POS);

        /* Data Transmission will be handled by the ISR */
    }
    return State_u8;
}/* END of SPI_ReceiveDataIT */


/*
    Function name    :    SPI_TXE_Interrupt_Handle
    Description      :    This function takes the SPI_Handle_p as argument and updates the 
                          Required SPI config for closing TxHandling
    Parameters       :    SPI_Handle_s * SPI_Handle_p : Pointer to SPI Handle
    Return           :    none
    Note             :    Private Interrupt handling function for Tx of SPI
*/
static void SPI_TXE_Interrupt_Handle(SPI_Handle_s * SPI_Handle_p)
{
    /* Check for DDF */
    if((SPI_Handle_p->SPIx_p->CR1 & (1u << SPI_CR1_DFF_POS)) == SET)
    {
        /* 16 bit DFF */
        SPI_Handle_p->SPIx_p->DR = *(uint16_t *)TxBuffer_p;
        /* Decrement by 2 for 16 bit */
        SPI_Handle_p->TxLen_u32--;
        SPI_Handle_p->TxLen_u32--;
        /* Increment pointer index to next Data */
        (uint16_t *)SPI_Handle_p->TxBuffer_p++;
    }
    else
    {
        /* 8 bit DFF */
        SPI_Handle_p->SPIx_p->DR = *TxBuffer_p;
        /* Decrement by 1 for 8 bit */
        SPI_Handle_p->TxLen_u32--;
        /* Increment pointer index to next Data */
        SPI_Handle_p->TxBuffer_p++;
    }

    /* Check if length of data transmitted is equal to ZERO */
    if(SPI_Handle_p->TxLen_u32 == 0)
    {
        SPI_CloseTransmission(SPI_Handle_p);
        /* Calling Application Callback for User Indication */
        SPI_AppLicationEventCallback(SPI_Handle_p, SPI_EventTxComplete);
    }
    
}/* END of SPI_TXE_Interrupt_Handle */

/*
    Function name    :    SPI_RXNE_Interrupt_Handle
    Description      :    This function takes the SPI_Handle_p as argument and updates the 
                          Required SPI config for closing RxHandling
    Parameters       :    SPI_Handle_s * SPI_Handle_p : Pointer to SPI Handle
    Return           :    none
    Note             :    Private Interrupt handling function for Rx of SPI
*/
static void SPI_RXNE_Interrupt_Handle(SPI_Handle_s * SPI_Handle_p)
{
    /* Check for DDF */
    if((SPI_Handle_s->SPIx_p->CR1 & (1u << SPI_CR1_DFF_POS)) == SET)
    {
        /* 16 bit DFF */
        *((uint16_t *)SPI_Handle_s->RxBuffer_p) = SPI_Handle_s->SPIx_p->DR;
        /* Decrement by 2 for 16 bit */
        SPI_Handle_s->RxLen_u32--;
        SPI_Handle_s->RxLen_u32--;
        /* Increment pointer index to next Data */
        (uint16_t *)SPI_Handle_s->RxBuffer_p++;
    }
    else
    {
        /* 8 bit DFF */
        *(SPI_Handle_s->RxBuffer_p) = SPI_Handle_s->SPIx_p->DR;
        /* Decrement by 1 for 8 bit */
        SPI_Handle_s->RxLen_u32--;
        /* Increment pointer index to next Data */
        SPI_Handle_s->RxBuffer_p++;
    }

    /* Check if length of data transmitted is equal to ZERO */
    if(SPI_Handle_p->RxLen_u32 == 0)
    {
        SPI_CloseReceiving(SPI_Handle_p);
        /* Calling Application Callback for User Indication */
        SPI_AppLicationEventCallback(SPI_Handle_p, SPI_EventRxComplete);
    }
}/* END of SPI_RXNE_Interrupt_Handle */

/*
    Function name    :    SPI_OVR_Error_Interrupt_Handle
    Description      :    This function takes the SPI_Handle_p as argument and updates the 
                          Required SPI config for OVR flow ERROR
    Parameters       :    SPI_Handle_s * SPI_Handle_p : Pointer to SPI Handle
    Return           :    none
    Note             :    Private Interrupt handling function for OVR error of SPI
*/
static void SPI_OVR_Error_Interrupt_Handle(SPI_Handle_s * SPI_Handle_p)
{
    uint8_t Temp_u8;
    /* Clear the OVR error flag */
    if(SPI_Handle_p->TxState != SPI_BUSY_IN_TX)
    {
        Temp_u8 = SPI_Handle_p->SPIx_p->DR;
        Temp_u8 = SPI_Handle_p->SPIx_p->SR;
    }
    /* Callback for Application */
    SPI_AppLicationEventCallback(SPI_Handle_p, SPI_OVR_Flow_Error);
}/* END of SPI_OVR_Error_Interrupt_Handle */

/*
    Function name    :    SPI_CloseTransmission
    Description      :    This function takes the SPI_Handle_p as argument and closes the 
                          transmission for passed SPI peripheral.
    Parameters       :    SPI_Handle_s * SPI_Handle_p : Pointer to SPI Handle
    Return           :    none
    Note             :    none
*/
void SPI_CloseTransmission(SPI_Handle_s * SPI_Handle_p)
{
    /* Clearing the TXEIE flag bit to Stop getting further interrupts */
    SPI_Handle_p->SPIx_p->CR2 &= ~(1u << SPI_CR2_TXEIE_POS);

    /* Setting the TxBuffer Address to NULL */
    SPI_Handle_p->TxBuffer_p = NULL;

    /* Setting the TxLen to Zero */
    SPI_Handle_p->TxLen_u32 = 0;

    /* Setting TxStatus to Ready */
    SPI_Handle_p->TxState_u8 = SPI_READY;
}/* END of SPI_CloseTransmission */

/*
    Function name    :    SPI_CloseReceiving
    Description      :    This function takes the SPI_Handle_p as argument and closes the 
                          receiveing for passed SPI peripheral.
    Parameters       :    SPI_Handle_s * SPI_Handle_p : Pointer to SPI Handle
    Return           :    none
    Note             :    none
*/
void SPI_CloseReceiving(SPI_Handle_s * SPI_Handle_p)
{
    /* Clearing the TXEIE flag bit to Stop getting further interrupts */
    SPI_Handle_p->SPIx_p->CR2 &= ~(1u << SPI_CR2_RXNEIE_POS);

    /* Setting the TxBuffer Address to NULL */
    SPI_Handle_p->RxBuffer_p = NULL;

    /* Setting the TxLen to Zero */
    SPI_Handle_p->RxLen_u32 = 0;

    /* Setting TxStatus to Ready */
    SPI_Handle_p->RxState_u8 = SPI_READY;
}/* END of SPI_CloseReceiving */

/*
    Function name    :    SPI_Clear_OVR_Flag
    Description      :    This function is used to clear the OVR flag when an OVR error occurs.
    Parameters       :    SPI_Handle_s * SPI_Handle_p : Pointer to SPI Handle
    Return           :    none
    Note             :    none
*/
void SPI_Clear_OVR_Flag(SPI_Handle_s * SPI_Handle_p)
{
    uint8_t Temp_u8;
    Temp_u8 = SPI_Handle_p->SPIx_p->DR;
    Temp_u8 = SPI_Handle_p->SPIx_p->SR;
}/* END of SPI_Clear_OVR_Flag */

/*
    Function name    :    SPI_AppLicationEventCallback
    Description      :    This function is used as a collback for SPI application layer.
    Parameters       :    SPI_Handle_s * SPI_Handle_p : Pointer to SPI Handle
    Return           :    none
    Note             :    none
*/
__weak void SPI_AppLicationEventCallback(SPI_Handle_s * SPI_Handle_p, SPI_Events_e SPI_Event)
{
    /*
        It is a weak implementation. 
        The application may override the current declaration.
    */
}/* END of SPI_AppLicationEventCallback */

/********************************* END of file ************************************/
