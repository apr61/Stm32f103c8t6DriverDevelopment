/*
 * stm32f103xx.h
 *
 *  Created on: Nov 14, 2023
 *      Author: Pradeep
 */

#ifndef STM32F103XX_H_
#define STM32F103XX_H_

/********************************* Abbreviations ************************************/
/*
    GPIO : General Purpose Input and Output
    PCLK : Peripheral Clock
    PERIPH : Peripheral
    APB : Advanced Peripheral Bus
    AHB : Advanced High Performance Bus
    EXTI : External Interrupt / Event controller
    TIM : Timer
    RCC : Reset and Clock Control
    USART : Universal Synchronous Asynchronous Receiver Transmitter
    SPI : Serial Peripheral Interface
    I2C : Inter - Integrated Circuit
    PERI : Peripheral
*/

/********************************* Includes ************************************/

#include <stdint.h>

typedef enum {
    FLAG_RESET = 0,
    FLAG_SET
} FlagValue_e;

typedef enum {
    DISABLE = 0,
    ENABLE
} PinStatus_e;

// Some general macros
#define SET                                  ENABLE
#define RESET                                DISABLE
#define GPIO_SET_PIN                         ENABLE
#define GPIO_RESET_PIN                       DISABLE


#define GPIO_GET_INDEX(__gpio_address__)   (((__gpio_address__) == (GPIOA))? 0u :\
                                            ((__gpio_address__) == (GPIOB))? 1u :\
                                            ((__gpio_address__) == (GPIOC))? 2u :\
                                            ((__gpio_address__) == (GPIOD))? 3u :4u)


/* Arm Cortex Mx NVIC ISERx register - Set */
#define NVIC_ISER0                         ((volatile uint32_t *) 0xE000E100u)  /* 0 - 31 IRQs */
#define NVIC_ISER1                         ((volatile uint32_t *) 0xE000E104u)  /* 32 - 63 IRQs */

/* Arm Cortex Mx NVIC ICERx register - Clear*/
#define NVIC_ICER0                         ((volatile uint32_t *) 0xE000E180u)  /* 0 - 31 IRQs */
#define NVIC_ICER1                         ((volatile uint32_t *) 0xE000E184u)  /* 32 - 63 IRQs */

/* Arm Cortex Mx NVIC Priporty register */
#define NVIC_PR_BASE_ADDR                  ((volatile uint32_t *) 0xE000E400u)
#define NUM_PR_BITS_IMPLEMENTED            4u

/*
    Flash and SRAM base addresses
*/

#define FLASH_BASE_ADDR                     0x08000000U /* Flash memory base address */
#define SRAM_BASE_ADDR                      0x20000000U  /* SRAM base address */
#define ROM_BASE_ADDR                       0x1FFFF000U /* ROM base address where ROM bootloader is stored by ST */

/*
    AHB and APBx bus peripheral base addresses
*/

#define PERIPH_BASE_ADDR                     0x40000000U
#define APB1_PERIPH_BASE_ADDR                PERIPH_BASE_ADDR
#define APB2_PERIPH_BASE_ADDR                0x40010000U
#define AHB_PERIPH_BASE_ADDR                 0x40020000U

/*
    BASE address of peripherals which are hanging to APB2
*/

#define GPIOA_BASE_ADDR                      (APB2_PERIPH_BASE_ADDR + 0x0800U)
#define GPIOB_BASE_ADDR                      (APB2_PERIPH_BASE_ADDR + 0x0C00u)
#define GPIOC_BASE_ADDR                      (APB2_PERIPH_BASE_ADDR + 0x1000u)
#define GPIOD_BASE_ADDR                      (APB2_PERIPH_BASE_ADDR + 0x1400u)
#define GPIOE_BASE_ADDR                      (APB2_PERIPH_BASE_ADDR + 0x1800u)
#define EXTI_BASE_ADDR                       (APB2_PERIPH_BASE_ADDR + 0x0400u)
#define AFIO_BASE_ADDR						 (APB2_PERIPH_BASE_ADDR + 0x0000u)
#define SPI1_BASE_ADDR                       (APB2_PERIPH_BASE_ADDR + 0x3000u)
#define USART1_BASE_ADDR                     (APB2_PERIPH_BASE_ADDR + 0x3800u)
#define TIM1_BASE_ADDR                       (APB2_PERIPH_BASE_ADDR + 0x2C00u)

/*
    BASE address of peripherals which are hanging to APB1
*/

#define TIM2_BASE_ADDR                       (APB1_PERIPH_BASE_ADDR + 0x0000u)
#define TIM3_BASE_ADDR                       (APB1_PERIPH_BASE_ADDR + 0x0400u)
#define TIM4_BASE_ADDR                       (APB1_PERIPH_BASE_ADDR + 0x0800u)
#define USART2_BASE_ADDR                     (APB1_PERIPH_BASE_ADDR + 0x4400u)
#define USART3_BASE_ADDR                     (APB1_PERIPH_BASE_ADDR + 0x4800u)
#define USART4_BASE_ADDR                     (APB1_PERIPH_BASE_ADDR + 0x4C00u)
#define SPI2_BASE_ADDR                       (APB1_PERIPH_BASE_ADDR + 0x3800u)
#define I2C1_BASE_ADDR                       (APB1_PERIPH_BASE_ADDR + 0x5400u)
#define I2C2_BASE_ADDR                       (APB1_PERIPH_BASE_ADDR + 0x5800u)

/*
    BASE address of peripherals which are hanging to AHB
*/

#define RCC_BASE_ADDR                        (AHB_PERIPH_BASE_ADDR + 0x1000u)

/* Peripheral register types */

/* General purpose Input output structure */
typedef struct {
  volatile uint32_t CRL; /*Port configuration register low ,Address offset : 0x00*/
  volatile uint32_t CRH; /*Port configuration register high ,Address offset : 0x04*/
  volatile uint32_t IDR;  /*Port input data register ,Address offset : 0x08*/
  volatile uint32_t ODR;  /*Port output data register ,Address offset : 0x0C*/
  volatile uint32_t BSRR; /*Port bit set/reset register ,Address offset : 0x10*/
  volatile uint32_t BRR;  /*Port bit reset register ,Address offset : 0x14*/
  volatile uint32_t LCKR; /*Port configuration lock register ,Address offset : 0x18*/
} GPIO_RegDef_s;

/* Alternate function Input output structure */
typedef struct {
  volatile uint32_t EVCR;    /*Event control register ,Address offset : 0x00*/
  volatile uint32_t MAPR;    /*AF remap and debug I/O configuration register,Address offset : 0x04*/
  volatile uint32_t EXTICR[4]; /*External interrupt configuration register,Address offset : 0x08*/
  uint32_t RESERVED;         /*Reserved ,Address offset : 0x18*/
  volatile uint32_t MAPR2;   /*AF remap and debug I/O configuration register2,Address offset : 0x1C*/
} AFIO_RegDef_s;

/* RCC Register def structure */
typedef struct {
  volatile uint32_t CR; /* Clock control register, Address offset : 0x00 */
  volatile uint32_t CFGR; /* Clock configuration register, Address offset : 0x04 */
  volatile uint32_t CIR; /* Clock interrupt register, Address offset : 0x08 */
  volatile uint32_t APB2RSTR; /* APB2 peripheral reset register, Address offset : 0x0C */
  volatile uint32_t APB1RSTR; /* APB1 peripheral reset register, Address offset : 0x10 */
  volatile uint32_t AHBENR; /* AHB peripheral clock enable register, Address offset : 0x14 */
  volatile uint32_t APB2ENR; /* APB2 peripheral clock enable register, Address offset : 0x18 */
  volatile uint32_t APB1ENR; /* APB1 peripheral clock enable register, Address offset : 0x1C */
  volatile uint32_t BDCR; /* Backup domain control register, Address offset : 0x20 */
  volatile uint32_t CSR; /* Control/status register, Address offset : 0x24 */
} RCC_RegDef_t;

/* EXTI Register def structure */
typedef struct {
	volatile uint32_t IMR; /* Interrupt mask register , Address offset : 0x00 */
	volatile uint32_t EMR; /* Event mask register , Address offset : 0x04 */
	volatile uint32_t RTSR; /* Rising trigger selection register , Address offset : 0x08 */
	volatile uint32_t FTSR; /* Falling trigger selection register , Address offset : 0x0C */
	volatile uint32_t SWIER; /* Software interrupt event register , Address offset : 0x10 */
	volatile uint32_t PR; /* Pending register , Address offset : 0x14 */
} EXTI_RegDef_t;

/* SPI register */

typedef struct {
    volatile uint32_t CR1; /* SPI control register 1 , Address offset : 0x00 */
    volatile uint32_t CR2; /* SPI control register 2 , Address offset : 0x04 */
    volatile uint32_t SR; /* SPI status register , Address offset : 0x08 */
    volatile uint32_t DR; /* SPI data register , Address offset : 0x0C */
    volatile uint32_t CRCPR; /* SPI CRC polynomial register , Address offset : 0x10 */
    volatile uint32_t RXCRCR; /* SPI RX CRC register , Address offset : 0x14 */
    volatile uint32_t TXCRCR; /* SPI TX CRC register , Address offset : 0x18 */
    volatile uint32_t I2SCFGR; /* SPI_I2S configuration register , Address offset : 0x1C */
    volatile uint32_t I2SPR; /* SPI_I2S prescaler register , Address offset : 0x20 */
} SPI_RegDef_s;

/* GPIO Peripheral defines */

#define GPIOA                                ((GPIO_RegDef_s *)GPIOA_BASE_ADDR)
#define GPIOB                                ((GPIO_RegDef_s *)GPIOB_BASE_ADDR)
#define GPIOC                                ((GPIO_RegDef_s *)GPIOC_BASE_ADDR)
#define GPIOD                                ((GPIO_RegDef_s *)GPIOD_BASE_ADDR)
#define GPIOE                                ((GPIO_RegDef_s *)GPIOE_BASE_ADDR)

#define RCC                                  ((RCC_RegDef_t *)RCC_BASE_ADDR)
#define EXTI                                 ((EXTI_RegDef_t *)EXTI_BASE_ADDR)
#define AFIO                                 ((AFIO_RegDef_s *)AFIO_BASE_ADDR)

#define SPI1                                 ((SPI_RegDef_s *)SPI1_BASE_ADDR)
#define SPI2                                 ((SPI_RegDef_s *)SPI2_BASE_ADDR)


/*
    Clock enable macros for GPIO, AFIO
*/

#define GPIOA_PCLK_EN()                      (RCC->APB2ENR |= (1u << 2u))
#define GPIOB_PCLK_EN()                      (RCC->APB2ENR |= (1u << 3u))
#define GPIOC_PCLK_EN()                      (RCC->APB2ENR |= (1u << 4u))
#define GPIOD_PCLK_EN()                      (RCC->APB2ENR |= (1u << 5u))
#define GPIOE_PCLK_EN()                      (RCC->APB2ENR |= (1u << 6u))
#define AFIO_PCLK_EN()                       (RCC->APB2ENR |= (1u << 0u))

/*
    Clock disable macros for GPIO
*/

#define GPIOA_PCLK_DI()                      (RCC->APB2ENR &= ~(1u << 2u))
#define GPIOB_PCLK_DI()                      (RCC->APB2ENR &= ~(1u << 3u))
#define GPIOC_PCLK_DI()                      (RCC->APB2ENR &= ~(1u << 4u))
#define GPIOD_PCLK_DI()                      (RCC->APB2ENR &= ~(1u << 5u))
#define GPIOE_PCLK_DI()                      (RCC->APB2ENR &= ~(1u << 6u))
#define AFIO_PCLK_DI()                       (RCC->APB2ENR &= ~(1u << 0u))

/*
    GPIO Register Reset macros
*/

#define GPIOA_REG_RESET()                    do{(RCC->APB2RSTR |= (1u << 2u)); (RCC->APB2RSTR &= ~(1u << 2u));} while(0)
#define GPIOB_REG_RESET()                    do{(RCC->APB2RSTR |= (1u << 3u)); (RCC->APB2RSTR &= ~(1u << 3u));} while(0)
#define GPIOC_REG_RESET()                    do{(RCC->APB2RSTR |= (1u << 4u)); (RCC->APB2RSTR &= ~(1u << 4u));} while(0)
#define GPIOD_REG_RESET()                    do{(RCC->APB2RSTR |= (1u << 5u)); (RCC->APB2RSTR &= ~(1u << 5u));} while(0)
#define GPIOE_REG_RESET()                    do{(RCC->APB2RSTR |= (1u << 6u)); (RCC->APB2RSTR &= ~(1u << 6u));} while(0)

/*
    SPI Peripheral Reset macros
*/

#define SPI1_PERI_RESET()                    do{(RCC->APB2RSTR |= (1u << 12u)); (RCC->APB2RSTR &= ~(1u << 12u));}while(0)

#define SPI2_PERI_RESET()                    do{(RCC->APB1RSTR |= (1u << 14u)); (RCC->APB1RSTR &= ~(1u << 14u));}while(0)

/*
    Clock enable macros for I2C
*/

#define I2C1_PCLK_EN()                       (RCC->APB1ENR |= (1 << 21u))
#define I2C2_PCLK_EN()                       (RCC->APB1ENR |= (1 << 22u))

/*
    Clock disable macros for I2C
*/

#define I2C1_PCLK_DI()                       (RCC->APB1ENR &= ~(1 << 21u))
#define I2C2_PCLK_DI()                       (RCC->APB1ENR &= ~(1 << 22u))

/*
    Clock enable macros for SPI
*/

#define SPI1_PCLK_EN()                       (RCC->APB2ENR |= (1 << 12u))
#define SPI2_PCLK_EN()                       (RCC->APB1ENR |= (1 << 14u))

/*
    Clock disable macros for SPI
*/

#define SPI1_PCLK_DI()                       (RCC->APB2ENR &= ~(1 << 12u))
#define SPI2_PCLK_DI()                       (RCC->APB1ENR &= ~(1 << 14u))

/*
    Clock enable macros for USART
*/

#define USART1_PCLK_EN()                     (RCC->APB2ENR |= (1 << 14u))
#define USART2_PCLK_EN()                     (RCC->APB1ENR |= (1 << 17u))
#define USART3_PCLK_EN()                     (RCC->APB1ENR |= (1 << 18u))

/*
    Clock disable macros for USART
*/
#define USART1_PCLK_DI()                     (RCC->APB2ENR &= ~(1 << 14u))
#define USART2_PCLK_DI()                     (RCC->APB1ENR &= ~(1 << 17u))
#define USART3_PCLK_DI()                     (RCC->APB1ENR &= ~(1 << 18u))


/*
    IRQ (Interrupt request) Numbers
*/
#define IRQ_NO_EXTI0                        6u
#define IRQ_NO_EXTI1                        7u
#define IRQ_NO_EXTI2                        8u
#define IRQ_NO_EXTI3                        9u
#define IRQ_NO_EXTI4                        10u
#define IRQ_NO_EXTI9_5                      23u
#define IRQ_NO_EXTI15_10                    40u


/* 
*    SPI CR1 Register bits Position
*/

#define SPI_CR1_CPHA_POS                    0u
#define SPI_CR1_CPOL_POS                    1u
#define SPI_CR1_MSTR_POS                    2u
#define SPI_CR1_BR_POS                      3u
#define SPI_CR1_SPE_POS                     6u
#define SPI_CR1_LSB_FIRST_POS               7u
#define SPI_CR1_SSI_POS                     8u
#define SPI_CR1_SSM_POS                     9u
#define SPI_CR1_RXONLY_POS                  10u
#define SPI_CR1_DFF_POS                     11u
#define SPI_CR1_CRC_NEXT_POS                12u
#define SPI_CR1_CRC_EN_POS                  13u
#define SPI_CR1_BIDI_OE_POS                 14u
#define SPI_CR1_BIDI_MODE_POS               15u

/* 
*    SPI CR2 Register bits Position
*/

#define SPI_CR2_RXDMAEN_POS                 0u
#define SPI_CR2_TXDMAEN_POS                 1u
#define SPI_CR2_SSOE_POS                    2u
#define SPI_CR2_ERRIE_POS                   5u
#define SPI_CR2_RXNEIE_POS                  6u
#define SPI_CR2_TXEIE_POS                   7u


/* 
*    SPI SR Register bits Position
*/

#define SPI_SR_RXNE_POS                    0u
#define SPI_SR_TXE_POS                     1u
#define SPI_SR_CHSIDE_POS                  2u
#define SPI_SR_UDR_POS                     3u
#define SPI_SR_CRCERR_POS                  4u
#define SPI_SR_MODF_POS                    5u
#define SPI_SR_OVR_POS                     6u
#define SPI_SR_BSY_POS                     7u


#endif /* STM32F103XX_H_ */


/********************************* END of file ************************************/
