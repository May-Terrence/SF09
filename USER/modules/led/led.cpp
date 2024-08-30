/*
 * led.cpp
 *
 *  Created on: 2020年9月2日
 *      Author: 17900
 */
#include "led.hpp"


//uint32_t PinMask
/*
 LL_GPIO_PIN_2
 LL_GPIO_PIN_3
 LL_GPIO_PIN_4
 LL_GPIO_PIN_5
*/
//GPIOE 2 3 4 5

void LED::LED_Tog()
{
	LL_GPIO_TogglePin(GPIOx,PinMask);
}

void LED::LED_Set()
{
	LL_GPIO_ResetOutputPin(GPIOx,PinMask);
}

void LED::LED_Clr()
{
	LL_GPIO_SetOutputPin(GPIOx,PinMask);
}

