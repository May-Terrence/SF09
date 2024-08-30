/*
 * led.cpp
 *
 *  Created on: 2020年9月2日
 *      Author: 17900
 */
#include "led.hpp"

LED::LED()
{
	LL_GPIO_SetOutputPin(LED1_GPIO_Port,LED1_Pin);
	LL_GPIO_SetOutputPin(LED2_GPIO_Port,LED2_Pin);
}

LED::~LED()
{

}



void LED::LED_log()
{
	LL_GPIO_TogglePin(LED1_GPIO_Port,LED1_Pin);
	LL_GPIO_TogglePin(LED2_GPIO_Port,LED2_Pin);
}


LED led;

extern "C" void led_main(void *argument)
{
	osDelay(2);
	for(;;)
	{
		osSemaphoreAcquire(semLed,0xffffffff);
		led.LED_log();
	}
}
