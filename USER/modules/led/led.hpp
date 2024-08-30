/*
 * led.hpp
 *
 *  Created on: 2020年9月2日
 *      Author: 17900
 */

#ifndef __LED_HPP
#define __LED_HPP

//#include "main.h"
#include "system/system.hpp"
#include "userlib/userlib.hpp"

#ifdef __cplusplus


//#include <iostream>
using namespace std;

class LED
{
public:
	LED(){}
	LED(GPIO_TypeDef *gpiox,uint32_t Pin): GPIOx(gpiox),PinMask(Pin){}
	~LED(){}
	void LED_Tog();
	void LED_Set();
	void LED_Clr();

	GPIO_TypeDef *GPIOx;
	uint32_t PinMask;
private:
};


#endif


#ifdef __cplusplus
extern "C"
{
#endif


#ifdef __cplusplus
}
#endif


#endif /* LED_HPP_ */
