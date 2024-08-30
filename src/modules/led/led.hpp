/*
 * led.hpp
 *
 *  Created on: 2020年9月2日
 *      Author: 17900
 */

#ifndef LED_HPP_
#define LED_HPP_

#include "main.h"
#include "FreeRTOS.h"
#include "cmsis_os.h"
#include "semphr.h"                     // ARM.FreeRTOS::RTOS:Core
#include "system/system.hpp"


#ifdef __cplusplus


#include <iostream>
#include <Dense>
using namespace std;
using namespace Eigen;

class LED
{
public:
	LED();
	~LED();
	void LED_log();
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
