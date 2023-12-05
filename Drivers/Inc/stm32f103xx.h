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

typedef enum {FLAG_RESET, FLAG_SET} FlagValue_e;
typedef enum {DISABLE, ENABLE} Status_e;

// Some general macros
#define SET                                  1u
#define RESET                                0
#define GPIO_SET_PIN                         1u
#define GPIO_RESET_PIN                       0

#define READ_BIT(REG, INDEX)                 (REG & (1u << INDEX))

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

#define GPIOA_BASE_ADDR                      (APB2_PERIPH_BASE_ADDR + 0x0800U) //0x00000800UL
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
  volatile uint32_t EXTICR[4]; /*External interrupt configuration register,Address offset : 0x08*/
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
typedef struct {
	volatile uint32_t IMR; /* Interrupt mask register , Address offset : 0x00 */
	volatile uint32_t EMR; /* Event mask register , Address offset : 0x04 */
	volatile uint32_t RTSR; /* Rising trigger selection register , Address offset : 0x08 */
	volatile uint32_t FTSR; /* Falling trigger selection register , Address offset : 0x0C */
	volatile uint32_t SWIER; /* Software interrupt event register , Address offset : 0x10 */
	volatile uint32_t PR; /* Pending register , Address offset : 0x14 */
} EXTI_RegDef_t;

/* I2C Register Def */
typedef struct {
  volatile uint32_t CR1; /* Control register 1, Address offset : 0x00 */
  volatile uint32_t CR2; /* Control register 2 , Address offset : 0x04 */
  volatile uint32_t OAR1; /* Own address register 1 , Address offset : 0x08 */
  volatile uint32_t OAR2; /* Own address register 2 , Address offset : 0x0C */
  volatile uint32_t DR; /* Data register , Address offset : 0x10 */
  volatile uint32_t SR1; /* Status register 1 , Address offset : 0x14 */
  volatile uint32_t SR2; /* Status register 2 , Address offset : 0x18 */
  volatile uint32_t CCR; /* Clock control register , Address offset : 0x1C */
  volatile uint32_t TRISE; /* TRISE register , Address offset : 0x20 */
} I2C_RegDef_s;

/* GPIO Peripheral defines */

#define GPIOA                                ((GPIO_RegDef_s *)GPIOA_BASE_ADDR)
#define GPIOB                                ((GPIO_RegDef_s *)GPIOB_BASE_ADDR)
#define GPIOC                                ((GPIO_RegDef_s *)GPIOC_BASE_ADDR)
#define GPIOD                                ((GPIO_RegDef_s *)GPIOD_BASE_ADDR)
#define GPIOE                                ((GPIO_RegDef_s *)GPIOE_BASE_ADDR)

#define RCC                                  ((RCC_RegDef_t *)RCC_BASE_ADDR)
#define EXTI                                 ((EXTI_RegDef_t *)EXTI_BASE_ADDR)
#define AFIO                                 ((AFIO_RegDef_s *)AFIO_BASE_ADDR)

#define I2C1                                 ((I2C_RegDef_s *)I2C1_BASE_ADDR);
#define I2C2                                 ((I2C_RegDef_s *)I2C2_BASE_ADDR);

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
    I2C Reset
*/

#define I2C1_REG_RESET()                     do{(RCC->APB1RSTR |= (1u << 21u)); (RCC->APB1RSTR &= ~(1u << 21u));} while(0)
#define I2C2_REG_RESET()                     do{(RCC->APB1RSTR |= (1u << 22u)); (RCC->APB1RSTR &= ~(1u << 22u));} while(0)

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


#define I2C_CR1_PE_POS                       0u
#define I2C_CR1_SMBUS_POS                    1u
#define I2C_CR1_SMBTYPE_POS                  3u
#define I2C_CR1_ENARP_POS                    4u
#define I2C_CR1_ENPEC_POS                    5u
#define I2C_CR1_ENGC_POS                     6u
#define I2C_CR1_NOSTRETCH_POS                7u
#define I2C_CR1_START_POS                    8u
#define I2C_CR1_STOP_POS                     9u
#define I2C_CR1_ACK_POS                      10u
#define I2C_CR1_POS_POS                      11u
#define I2C_CR1_PEC_POS                      12u
#define I2C_CR1_ALERT_POS                    13u
#define I2C_CR1_SWRST_POS                    15u

#define I2C_CR2_FREQ_POS                     0u
#define I2C_CR2_TERREN_POS                   8u
#define I2C_CR2_TEVTEN_POS                   9u
#define I2C_CR2_ITBUFEN_POS                  10u
#define I2C_CR2_DMAEN_POS                    11u
#define I2C_CR2_LAST_POS                     12u

#define I2C_OAR1_ADD0_POS                    0u
#define I2C_OAR1_ADD1_POS                    1u
#define I2C_OAR1_ADD8_POS                    8u
#define I2C_OAR1_ADDMODE_POS                 15u

#define I2C_OAR2_ENDUAL_POS                  0u
#define I2C_OAR2_ADD2_POS                    1u

#define I2C_SR1_SB_POS                       0u
#define I2C_SR1_ADDR_POS                     1u
#define I2C_SR1_BTF_POS                      2u
#define I2C_SR1_ADD10_POS                    3u
#define I2C_SR1_STOPF_POS                    4u
#define I2C_SR1_RxNE_POS                     6u
#define I2C_SR1_TxE_POS                      7u
#define I2C_SR1_BERR_POS                     8u
#define I2C_SR1_RLO_POS                      9u
#define I2C_SR1_AF_POS                       10u
#define I2C_SR1_OVR_POS                      11u
#define I2C_SR1_PECERR_POS                   12u
#define I2C_SR1_TIMEOUT_POS                  14u
#define I2C_SR1_SMBALERT_POS                 15u

#define I2C_SR2_MSL_POS                      0u
#define I2C_SR2_BUSY_POS                     1u
#define I2C_SR2_TRA_POS                      2u
#define I2C_SR2_GENCALL_POS                  4u
#define I2C_SR2_SMBDEFAUL_POS                5u
#define I2C_SR2_SMBHOST_POS                  6u
#define I2C_SR2_DUALF_POS                    7u
#define I2C_SR2_PEC_POS                      8u

#define I2C_CCR_DUTY_POS                     14u
#define I2C_CCR_F_S_POS                      15u

#endif /* STM32F103XX_H_ */


/********************************* END of file ************************************/
