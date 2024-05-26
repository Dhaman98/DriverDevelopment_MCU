/*
 * stm32f401xe_rcc_driver.h
 *
 *  Created on: Mar 4, 2024
 *      Author: pldha
 */

#ifndef INC_STM32F401XE_RCC_DRIVER_H_
#define INC_STM32F401XE_RCC_DRIVER_H_

#include "stm32f401xe.h"

//This returns the APB1 clock value
uint32_t RCC_GetPCLK1Value(void);

//This returns the APB2 clock value
uint32_t RCC_GetPCLK2Value(void);


uint32_t  RCC_GetPLLOutputClock(void);


#endif /* INC_STM32F401XE_RCC_DRIVER_H_ */
