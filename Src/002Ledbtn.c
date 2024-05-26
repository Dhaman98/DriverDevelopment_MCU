/*
 * 002Ledbtn.c
 *
 *  Created on: Feb 12, 2024
 *      Author: pldha
 */


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
	GPIO_Handle_t Gpioled , GPIOBtn;

	Gpioled.pGPIOx =GPIOA;
	Gpioled.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_5;
	Gpioled.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
	Gpioled.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	Gpioled.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	Gpioled.GPIO_PinConfig.GPIO_PinPuPdControl= GPIO_NO_PUPD;

	GPIO_PeriClockControl(GPIOA,ENABLE);
	GPIO_Init(&Gpioled);

	GPIOBtn.pGPIOx =GPIOC;
	GPIOBtn.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_13;
	GPIOBtn.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IN;
	GPIOBtn.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	GPIOBtn.GPIO_PinConfig.GPIO_PinPuPdControl= GPIO_NO_PUPD;

	GPIO_PeriClockControl(GPIOC,ENABLE);

	GPIO_Init(&GPIOBtn);

	while(1)
	{
		//GPIO_WriteToOutputPin(GPIOA,GPIO_PIN_NO_5, 0);
		if(GPIO_ReadFromInputPin(GPIOC, GPIO_PIN_NO_13))
		{
			delay();
			GPIO_ToggleOutputPin(GPIOA , GPIO_PIN_NO_5);
		}

	}
	return 0;
}

