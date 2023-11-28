
#include "stm32f103xx.h"
#include "stm32f103xx_USART_driver.h"
#include "stm32f103xx_GPIO_driver.h"

void USART2_Init()
{
	
}

void USART2_GPIO_Init()
{
	GPIO_Handle_s  USART_GPIO;
	USART_GPIO.GPIO_PinConfig.GPIOPinMode = GPIO_MODE_ALT_PUSH_PULL;
}

int main(void)
{
	
}