/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : adc.hpp
  * Description        :
  ******************************************************************************
  * Function           :
  *
  ******************************************************************************
  * Author             : Zhiping Fu
  * Creation Date      : Oct 22, 2020
  ******************************************************************************
  */
/* USER CODE END Header */

#ifndef __POWER_HPP
#define __POWER_HPP

#include "system/system.hpp"

#ifdef __cplusplus
extern "C" {
#endif

#define VOL_RES1 15.0f  //电压分阻1
#define VOL_RES2 3.0f   //电压分阻2
#define VOL_NORM 14.8f
#define VOL_CRITICAL 14.0f

void DMA2_Stream0_IRQHandler(void);


#ifdef __cplusplus
}
#endif


#ifdef __cplusplus

using namespace std;

class POWER
{
public:
	POWER(){}
	POWER(ADC_TypeDef * h,char * n,uint32_t c) : hadc(h),name(n),chn(c){};
	~POWER(){}

	void power_Init(void);

	float get_Voltage(void);

	void battery_Update(void);

private:
	ADC_TypeDef * hadc;
	char *name;
	uint32_t chn;
	bool Update;
	uint16_t AdcRaw;
	float AdcRel;

	battery_msg battery;

};

#endif

#endif


/************************ (C) COPYRIGHT Longmen Drone Team *****END OF FILE****/
