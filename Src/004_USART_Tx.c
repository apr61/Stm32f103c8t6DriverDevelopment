#include "stm32f103xx.h"
#include "stm32f103xx_USART_driver.h"
#include "stm32f103xx_GPIO_driver.h"
#include <string.h>


char msg[1024] = "Hello";

void delay(void) {
	for (uint32_t i = 0; i < 500000 / 4; i++) {
		;
	}
}

void USART2_Init(USART_Handle_s * USART2_handle) {
	USART2_handle->USARTx_p = USART1;
	USART2_handle->USART_Config.USART_WordLength = USART_WORDLEN_8BITS;
	USART2_handle->USART_Config.USART_BaudRate = USART_STD_BAUD_9600;
	USART2_handle->USART_Config.USART_Mode = USART_MODE_ONLY_TX;
	USART2_handle->USART_Config.USART_NoOfStopBits = USART_STOPBITS_1;
	USART2_handle->USART_Config.USART_ParityControl = USART_PARITY_DISABLE;
	USART2_handle->USART_Config.USART_HwFlowControl = USART_HW_FLOW_CTRL_NONE;
	USART_Init(USART2_handle);
}

void USART2_GPIO_Init() {
	GPIO_Handle_s USART_GPIO;
	USART_GPIO.GPIOx_p = GPIOA;
	USART_GPIO.GPIO_PinConfig.GPIOPinNumber = GPIO_PIN_9;
	USART_GPIO.GPIO_PinConfig.GPIOPinMode = GPIO_MODE_ALT_PUSH_PULL;
	USART_GPIO.GPIO_PinConfig.GPIOPinSpeed = GPIO_SPEED_HIGH;
	USART_GPIO.GPIO_PinConfig.GPIOPinPull = GPIO_NO_PULL;

	GPIO_PCLK_Control(GPIOA, ENABLE);

	/* USART Tx Init */
	GPIO_Init(&USART_GPIO);

	USART_GPIO.GPIO_PinConfig.GPIOPinNumber = GPIO_PIN_10;
	USART_GPIO.GPIO_PinConfig.GPIOPinMode = GPIO_MODE_INPUT;
	USART_GPIO.GPIO_PinConfig.GPIOPinPull = GPIO_NO_PULL;

	/* USART Rx Init */
	GPIO_Init(&USART_GPIO);
}

void GPIO_BtnInit(void) {
	GPIO_Handle_s GPIO_Btn;

	GPIO_Btn.GPIOx_p = GPIOB;
	GPIO_Btn.GPIO_PinConfig.GPIOPinNumber = GPIO_PIN_0;
	GPIO_Btn.GPIO_PinConfig.GPIOPinMode = GPIO_MODE_INPUT;
	GPIO_Btn.GPIO_PinConfig.GPIOPinSpeed = GPIO_SPEED_HIGH;
	GPIO_Btn.GPIO_PinConfig.GPIOPinPull = GPIO_PULL_UP;

	GPIO_PCLK_Control(GPIOB, ENABLE);

	GPIO_Init(&GPIO_Btn);
}

int main(void) {
	USART_Handle_s USART2_handle;

	GPIO_BtnInit();
	USART2_GPIO_Init();
	USART2_Init(&USART2_handle);
	USART_PeripheralControl(USART1, ENABLE);

	while (1) {
		if (GPIO_ReadInputPin(GPIOB, GPIO_PIN_0) == GPIO_RESET_PIN) {
			delay();
			USART_Tx(&USART2_handle, (uint8_t*) "Hello", 6);
		}
	}
}
