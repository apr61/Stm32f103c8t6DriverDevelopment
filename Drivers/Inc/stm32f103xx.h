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
*/

/********************************* Includes ************************************/

#include <stdint.h>

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

#define GPIOA_BASE_ADDR                      (APB2_PERIPH_BASE_ADDR + 0x00000800UL) //0x00000800UL
#define GPIOB_BASE_ADDR                      (APB2_PERIPH_BASE_ADDR + 0x0C00)
#define GPIOC_BASE_ADDR                      (APB2_PERIPH_BASE_ADDR + 0x1000)
#define GPIOD_BASE_ADDR                      (APB2_PERIPH_BASE_ADDR + 0x1400)
#define GPIOE_BASE_ADDR                      (APB2_PERIPH_BASE_ADDR + 0x1800)
#define EXTI_BASE_ADDR                       (APB2_PERIPH_BASE_ADDR + 0x0400)
#define SPI1_BASE_ADDR                       (APB2_PERIPH_BASE_ADDR + 0x3000)
#define USART1_BASE_ADDR                     (APB2_PERIPH_BASE_ADDR + 0x3800)
#define TIM1_BASE_ADDR                       (APB2_PERIPH_BASE_ADDR + 0x2C00)

/*
    BASE address of peripherals which are hanging to APB1
*/

#define TIM2_BASE_ADDR                       (APB1_PERIPH_BASE_ADDR + 0x0000)
#define TIM3_BASE_ADDR                       (APB1_PERIPH_BASE_ADDR + 0x0400)
#define TIM4_BASE_ADDR                       (APB1_PERIPH_BASE_ADDR + 0x0800)
#define USART2_BASE_ADDR                     (APB1_PERIPH_BASE_ADDR + 0x4400)
#define USART3_BASE_ADDR                     (APB1_PERIPH_BASE_ADDR + 0x4800)
#define USART4_BASE_ADDR                     (APB1_PERIPH_BASE_ADDR + 0x4C00)
#define SPI2_BASE_ADDR                       (APB1_PERIPH_BASE_ADDR + 0x3800)
#define I2C1_BASE_ADDR                       (APB1_PERIPH_BASE_ADDR + 0x5400)
#define I2C2_BASE_ADDR                       (APB1_PERIPH_BASE_ADDR + 0x5800)

/*
    BASE address of peripherals which are hanging to AHB
*/

#define RCC_BASE_ADDR                        (AHB_PERIPH_BASE_ADDR + 0x1000)

/* Peripheral register types */

// General purpose Input output structure
typedef struct {
  volatile uint32_t CRL; /*Port configuration register low ,Address offset : 0x00*/
  volatile uint32_t CRH; /*Port configuration register high ,Address offset : 0x04*/
  volatile uint32_t IDR;  /*Port input data register ,Address offset : 0x08*/
  volatile uint32_t ODR;  /*Port output data register ,Address offset : 0x0C*/
  volatile uint32_t BSRR; /*Port bit set/reset register ,Address offset : 0x10*/
  volatile uint32_t BRR;  /*Port bit reset register ,Address offset : 0x14*/
  volatile uint32_t LCKR; /*Port configuration lock register ,Address offset : 0x18*/
} GPIO_RegDef_s;

// Alternate function Input output structure
typedef struct {
  volatile uint32_t EVCR;    /*Event control register ,Address offset : 0x00*/
  volatile uint32_t MAPR;    /*AF remap and debug I/O configuration register,Address offset : 0x04*/
  volatile uint32_t EXTICR1; /*External interrupt configuration register 1,Address offset : 0x08*/
  volatile uint32_t EXTICR2; /*External interrupt configuration register 2,Address offset : 0x0C*/
  volatile uint32_t EXTICR3; /*External interrupt configuration register 3,Address offset : 0x10*/
  volatile uint32_t EXTICR4; /*External interrupt configuration register 4,Address offset : 0x14*/
  uint32_t RESERVED;         /*Reserved ,Address offset : 0x18*/
  volatile uint32_t MAPR2;   /*AF remap and debug I/O configuration register2,Address offset : 0x1C*/
} AFIO_RegDef_s;

// RCC Register def structure
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

// EXTI Register def structure



/* GPIO Peripheral defines */

#define GPIOA                                ((GPIO_RegDef_s *)GPIOA_BASE_ADDR)
#define GPIOB                                ((GPIO_RegDef_s *)GPIOB_BASE_ADDR)
#define GPIOC                                ((GPIO_RegDef_s *)GPIOC_BASE_ADDR)
#define GPIOD                                ((GPIO_RegDef_s *)GPIOD_BASE_ADDR)
#define GPIOE                                ((GPIO_RegDef_s *)GPIOE_BASE_ADDR)

#define RCC                                  ((RCC_RegDef_t *)RCC_BASE_ADDR)
#define EXTI                                 ((RCC_RegDef_t *)EXTI_BASE_ADDR)

/*
    Clock enable macros for GPIO, AFIO
*/

#define GPIOA_PCLK_EN()                      (RCC->APB2ENR |= (0x1UL << (2U)))
#define GPIOB_PCLK_EN()                      (RCC->APB2ENR |= (1 << 3))
#define GPIOC_PCLK_EN()                      (RCC->APB2ENR |= (1 << 4))
#define GPIOD_PCLK_EN()                      (RCC->APB2ENR |= (1 << 5))
#define GPIOE_PCLK_EN()                      (RCC->APB2ENR |= (1 << 6))
#define AFIO_PCLK_EN()                       (RCC->APB2ENR |= (1 << 0))

/*
    Clock disable macros for GPIO
*/

#define GPIOA_PCLK_DI()                      (RCC->APB2ENR &= ~(1 << 2))
#define GPIOB_PCLK_DI()                      (RCC->APB2ENR &= ~(1 << 3))
#define GPIOC_PCLK_DI()                      (RCC->APB2ENR &= ~(1 << 4))
#define GPIOD_PCLK_DI()                      (RCC->APB2ENR &= ~(1 << 5))
#define GPIOE_PCLK_DI()                      (RCC->APB2ENR &= ~(1 << 6))
#define AFIO_PCLK_DI()                       (RCC->APB2ENR &= ~(1 << 0))

/*
    GPIO Register Reset macros
*/

#define GPIOA_REG_RESET()                    do{(RCC->APB2RSTR |= (1 << 2)); (RCC->APB2RSTR &= ~(1 << 2));} while(0)
#define GPIOB_REG_RESET()                    do{(RCC->APB2RSTR |= (1 << 3)); (RCC->APB2RSTR &= ~(1 << 3));} while(0)
#define GPIOC_REG_RESET()                    do{(RCC->APB2RSTR |= (1 << 4)); (RCC->APB2RSTR &= ~(1 << 4));} while(0)
#define GPIOD_REG_RESET()                    do{(RCC->APB2RSTR |= (1 << 5)); (RCC->APB2RSTR &= ~(1 << 5));} while(0)
#define GPIOE_REG_RESET()                    do{(RCC->APB2RSTR |= (1 << 6)); (RCC->APB2RSTR &= ~(1 << 6));} while(0)

/*
    Clock enable macros for I2C
*/

#define I2C1_PCLK_EN()                       (RCC->APB1ENR |= (1 << 21))
#define I2C2_PCLK_EN()                       (RCC->APB1ENR |= (1 << 22))

/*
    Clock disable macros for I2C
*/

#define I2C1_PCLK_DI()                       (RCC->APB1ENR &= ~(1 << 21))
#define I2C2_PCLK_DI()                       (RCC->APB1ENR &= ~(1 << 22))

/*
    Clock enable macros for SPI
*/

#define SPI1_PCLK_EN()                       (RCC->APB2ENR |= (1 << 12))
#define SPI2_PCLK_EN()                       (RCC->APB1ENR |= (1 << 14))

/*
    Clock disable macros for SPI
*/

#define SPI1_PCLK_DI()                       (RCC->APB2ENR &= ~(1 << 12))
#define SPI2_PCLK_DI()                       (RCC->APB1ENR &= ~(1 << 14))

/*
    Clock enable macros for USART
*/

#define USART1_PCLK_EN()                     (RCC->APB2ENR |= (1 << 14))
#define USART2_PCLK_EN()                     (RCC->APB1ENR |= (1 << 17))
#define USART3_PCLK_EN()                     (RCC->APB1ENR |= (1 << 18))

/*
    Clock disable macros for USART
*/
#define USART1_PCLK_DI()                     (RCC->APB2ENR &= ~(1 << 14))
#define USART2_PCLK_DI()                     (RCC->APB1ENR &= ~(1 << 17))
#define USART3_PCLK_DI()                     (RCC->APB1ENR &= ~(1 << 18))


// Some general macros
#define ENABLE                               1
#define DISABLE                              0
#define SET                                  ENABLE
#define RESET                                DISABLE
#define GPIO_SET_PIN                         ENABLE
#define GPIO_RESET_PIN                       DISABLE

#endif /* STM32F103XX_H_ */


/********************************* END of file ************************************/
