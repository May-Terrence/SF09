
/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : bell.hpp
  * Description        :
  ******************************************************************************
  * Function           :
  *
  ******************************************************************************
  * Author             : Zhiping Fu
  * Creation Date      : 2020年11月8日
  ******************************************************************************
  */
/* USER CODE END Header */
#ifndef __BELL_HPP
#define __BELL_HPP

#include "system/system.hpp"
#include "userlib/userlib.hpp"
#include "led/led.hpp"
#ifdef __cplusplus
extern "C" {
#endif
#define TONE_NUM 20
#define VOL_RES1 15.0f  //电压分阻1
#define VOL_RES2 3.0f   //电压分阻2
#define VOL_NORM 14.8f
#define VOL_CRITICAL 14.0f

#ifdef __cplusplus
}
#endif

#ifdef __cplusplus

using namespace std;

class BELL:public LED
{
public:
	BELL(){}
	BELL(TIM_TypeDef *ht,char *n,u32 ch,GPIO_TypeDef *gpiox,uint32_t Pin):htim(ht),name(n),chn(ch),led(gpiox,Pin){}
	BELL(TIM_TypeDef *ht,char *n,u32 ch):htim(ht),name(n),chn(ch){}
	~BELL(){}
	void Bell_Init(void);
	void Bell_Loop_1ms(void);
	void Bell_Calc(void);
	void Bell_Sound(u16 *tone,u16 *time,u16 *idle,u8 num);
	void Bell_Start(void);
	void Bell_Error(void);
	void Bell_Cali(void);

	const u16 TONE_TAB[21]={3816,3400,3030,2864,2550,2272,2024,
		1912,1704,1517,1432,1276,1136,1012,
		956, 850, 758, 716, 638, 568, 508};

private:
	TIM_TypeDef *htim;
	char *name;
	u32 chn;
	LED led;
	bool Update;
	bool HoldOn;               //刚报完警1s内不再报警
	u16 TONE_ARRAY[TONE_NUM];  //音调  周期us（1/频率）
	u16 TIME_ARRAY[TONE_NUM];  //持续时间  ms
	u16 IDLE_ARRAY[TONE_NUM];  //空闲时间  ms
	u16 TimeCnt;
	u8  ToneIndex;
	u8  ToneNum;

	sCNT  Filt;       //均值滤波器  --
	eERR  Err;        //错误信息  --
	eBATTARY_STA Battary_sta;	//电池状态

	battery_msg battery;
};


#endif


#endif


/************************ (C) COPYRIGHT Longmen Drone Team *****END OF FILE****/
