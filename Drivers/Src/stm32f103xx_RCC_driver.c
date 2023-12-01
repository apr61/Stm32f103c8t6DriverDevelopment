/********************************* Abbreviations *************************************/
/*
 RCC : Reset and Clock Control
 PCLK : Peripheral Clock
 IRQ  : Interrupt Request
 */

/********************************* Includes *************************************/
#include "stm32f103xx.h"
#include "stm32f103xx_RCC_driver.h"

uint32_t SystemCoreClock = 8000000;
const uint8_t AHBPrescTable[16U] = {0, 0, 0, 0, 0, 0, 0, 0, 1, 2, 3, 4, 6, 7, 8, 9};
const uint8_t APBPrescTable[8U] =  {0, 0, 0, 0, 1, 2, 3, 4};
#define RCC_CFGR_PPRE1 (0x7UL << (8U))
#define RCC_CFGR_PPRE2 (0x7UL << (11U))
#define RCC_CFGR_PPRE1_Pos 8u
#define RCC_CFGR_PPRE2_Pos 11u

/*
 Function name    :    RCC_GetPCLK1Value
 Description      :    Returns the PCLK of the APB2 peripheral
 Parameters       :    None
 Return           :    None
 Note             :    None
 */
uint32_t RCC_GetPCLK1Value(void) {
//	uint32_t pclk1, SystemClk;
//
//	uint8_t clksrc, temp, ahbp, apb1p;
//
//	clksrc = ((RCC->CFGR >> 2) & 0x3);
//
//	if (clksrc == 0) {
//		SystemClk = 16000000;
//	} else if (clksrc == 1) {
//		SystemClk = 8000000;
//	} else if (clksrc == 2) {
//		SystemClk = RCC_GetPLLOutputClock();
//	}
//
//	//for ahb
//	temp = ((RCC->CFGR >> 4) & 0xF);
//
//	if (temp < 8) {
//		ahbp = 1;
//	} else {
//		ahbp = AHB_PreScaler[temp - 8];
//	}
//
//	//apb1
//	temp = ((RCC->CFGR >> 10) & 0x7);
//
//	if (temp < 4) {
//		apb1p = 1;
//	} else {
//		apb1p = APB1_PreScaler[temp - 4];
//	}
//
//	pclk1 = (SystemClk / ahbp) / apb1p;
//
//	return pclk1;
	/* Get HCLK source and Compute PCLK1 frequency ---------------------------*/
	  return (SystemCoreClock >> APBPrescTable[(RCC->CFGR & RCC_CFGR_PPRE1) >> RCC_CFGR_PPRE1_Pos]);
}/* END of RCC_GetPCLK1Value */

/*
 Function name    :    RCC_GetPCLK2Value
 Description      :    Returns the PCLK of the APB1 peripheral
 Parameters       :    None
 Return           :    None
 Note             :    None
 */
uint32_t RCC_GetPCLK2Value(void) {
//	uint32_t SystemClock = 0, tmp, pclk2;
//	uint8_t clk_src = ( RCC->CFGR >> 2) & 0X3;
//
//	uint8_t ahbp, apb2p;
//
//	if (clk_src == 0) {
//		SystemClock = 16000000;
//	} else {
//		SystemClock = 8000000;
//	}
//	tmp = (RCC->CFGR >> 4) & 0xF;
//
//	if (tmp < 0x08) {
//		ahbp = 1;
//	} else {
//		ahbp = AHB_PreScaler[tmp - 8];
//	}
//
//	tmp = (RCC->CFGR >> 13) & 0x7;
//	if (tmp < 0x04) {
//		apb2p = 1;
//	} else {
//		apb2p = APB1_PreScaler[tmp - 4];
//	}
//
//	pclk2 = (SystemClock / ahbp) / apb2p;
//
//	return pclk2;
	return (SystemCoreClock >> APBPrescTable[(RCC->CFGR & RCC_CFGR_PPRE2) >> RCC_CFGR_PPRE2_Pos]);
}/* END of RCC_GetPCLK2Value */


/*
 Function name    :    RCC_GetPLLOutputClock
 Description      :    Returns the PCLK of the APB1 peripheral
 Parameters       :    None
 Return           :    None
 Note             :    None
 */

uint32_t RCC_GetPLLOutputClock(void)
{
	return 0;
}/* END of RCC_GetPLLOutputClock */

/********************************* END of File *************************************/
