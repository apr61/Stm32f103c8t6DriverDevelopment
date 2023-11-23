
#ifndef STM32F103XX_SPI_DRIVER_H_
#define STM32F103XX_SPI_DRIVER_H_

/********************************* Abbreviations ************************************/
/*
    SPI : Serial Peripheral Interface
    PCLK : Peripheral Clock
    IRQ  : Interrupt Request 
*/

/********************************* Includes ************************************/
#include "stm32f103xx.h"

/* SPI configuration */
typedef struct
{
    uint32_t SPI_DeviceMode; /* Ref @SPI_DEVICE_MODES */
    uint32_t SPI_BusConfig; /* Ref @SPI_BUS_CONFIG */
    uint32_t SPI_SCLK_Speed; /* Ref @SPI_SCLK_SPEED */
    uint32_t SPI_DFF; /* Ref @SPI_DFF */
    uint32_t SPI_CPOL; /* Ref @SPI_CPOL */
    uint32_t SPI_CPHA; /* Ref @SPI_CPHA */
    uint32_t SPI_SSM; /* Ref @SPI_SSM */
} SPI_Config_s;

/* SPI Handle */
typedef struct{
    SPI_RegDef_s *SPIx_p;
    SPI_Config_s SPI_Config;
} SPI_Handle_s;

/************************************ SPI Config Macros *****************************/
/* @SPI_DEVICE_MODES */
#define SPI_DEVICE_MODE_SLAVE             0x0u
#define SPI_DEVICE_MODE_MASTER            0x1u

/* @SPI_BUS_CONFIG */
#define SPI_BUS_CONFIG_FULL               0x0u
#define SPI_BUS_CONFIG_HALF               0x1u
#define SPI_BUS_CONFIG_SIMPLEX_Rx         0x2u

/* @SPI_SCLK_SPEED */
#define SPI_SCLK_SPEED_DIV2               0x0u
#define SPI_SCLK_SPEED_DIV4               0x1u
#define SPI_SCLK_SPEED_DIV8               0x2u
#define SPI_SCLK_SPEED_DIV16              0x3u
#define SPI_SCLK_SPEED_DIV32              0x4u
#define SPI_SCLK_SPEED_DIV64              0x5u
#define SPI_SCLK_SPEED_DIV128             0x6u
#define SPI_SCLK_SPEED_DIV256             0x7u

/* @SPI_DFF */
#define SPI_DFF_8BITS                     0x0u
#define SPI_DFF_16BITS                    0x1u

/* @SPI_CPOL */
#define SPI_CPOL_LOW                      0x0u
#define SPI_CPOL_HIGH                     0x1u

/* @SPI_CPHA */
#define SPI_CPHA_LOW                      0x0u
#define SPI_CPHA_HIGH                     0x1u

/* @SPI_SSM */
#define SPI_SSM_DI                        0x0u
#define SPI_SSM_EN                        0x1u


/********************************* SPI SR register Flag Masks **********************/

#define SPI_SR_RXNE_FLAGMASK                (1u << SPI_SR_RXNE_POS)
#define SPI_SR_TXE_FLAGMASK                 (1u << SPI_SR_TXE_POS)
#define SPI_SR_BSY_FLAGMASK                 (1u << SPI_SR_BSY_POS)

/**************************************** SPI API'S *******************************/

/* SPI peripheral clock enable and disable */
void SPI_PCLK_Control(SPI_RegDef_s * SPIx_p, uint8_t EnOrDi_u8);

/* SPI Init, DeInit*/
void SPI_Init(SPI_Handle_s * SPI_Handle_p);
void SPI_DeInit(SPI_RegDef_s * SPIx_p);

/* Data send and receive registers */
/* LENGTH of data always need tobe of type unsigned int */
void SPI_SendData(SPI_RegDef_s * SPIx_p, uint8_t *TxBuffer_p, uint32_t DataLen_u32);
void SPI_ReceiveData(SPI_RegDef_s * SPIx_p, uint8_t *RxBuffer_p, uint32_t DataLen_u32);


/* SPI interrupt Configuration and handling */
void SPI_IRQ_Config(uint8_t IRQ_Number_u8, uint8_t EnOrDi_u8);
void SPI_IRQ_Priority(uint8_t IRQ_Number_u8, uint8_t IRQ_Priority_u8);
void SPI_IRQ_Handling(SPI_RegDef_s * SPIx_p);

/* Other SPI API's */
FlagValue_e SPI_SR_GetFlagStatus(SPI_RegDef_s * SPIx_p, uint16_t FlagMask);

#endif /* STM32F103XX_SPI_DRIVER_H_ */

/********************************* END of file ************************************/
