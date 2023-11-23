/********************************* Abbreviations ************************************/
/*
    SPI : General Purpose Input and Output
    PCLK : Peripheral Clock
    IRQ  : Interrupt Request 
*/

/********************************* Includes ************************************/
#include "stm32f103xx_SPI_driver.h"

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
    Note             :    None
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

/********************************* END of file ************************************/
