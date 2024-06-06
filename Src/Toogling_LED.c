/*
 * 001LEDToogle.c
 *
 *  Created on: Feb 11, 2024
 *      Author: pldha
 */

#include "stm32f401xe.h"
#include "stm32f401xe_gpio_driver.h"
//#include"stm32f401xe_gpio_driver.c"

void delay(void)
{
	for(uint32_t i=0 ; i< (500000/2) ; i++);
}

int main(void)
{
	GPIO_Handle_t Gpioled;
	Gpioled.pGPIOx =GPIOA;
	Gpioled.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_5;
	Gpioled.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
	Gpioled.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	Gpioled.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	Gpioled.GPIO_PinConfig.GPIO_PinPuPdControl= GPIO_NO_PUPD;

	GPIO_PeriClockControl(GPIOA,ENABLE);

	GPIO_Init(&Gpioled);

	while(1)
	{
		//GPIO_WriteToOutputPin(GPIOA,GPIO_PIN_NO_5, 0);
		GPIO_ToggleOutputPin(GPIOA , GPIO_PIN_NO_5);
		delay();
	}
	return 0;
}

