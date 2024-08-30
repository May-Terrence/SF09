/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : bell.cpp
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

#include "bell.hpp"


BELL bell(TIM1,(char *)"bell",TIM_CHANNEL_4,GPIOE,LL_GPIO_PIN_5);


void BELL::Bell_Init()
{

	Err = ERR_NONE;

	Update=false;
	HoldOn=false;
	TimeCnt=0;
	ToneIndex=0;
	ToneNum=0;

	LL_TIM_EnableCounter(htim);
//	TIM1->ARR =2000;
	LL_TIM_OC_SetCompareCH4(htim,0);
	LL_TIM_CC_EnableChannel(htim, LL_TIM_CHANNEL_CH4);
	LL_TIM_EnableAllOutputs(htim);

	osDelay(10);
}



void BELL::Bell_Loop_1ms()
{
	if(Update==false)return;  //无任务
	if(HoldOn==true)
	{
		if(--TimeCnt==0)
		{
			Update=false;
			HoldOn=false;
		}
		return;
	}
	if(TimeCnt==IDLE_ARRAY[ToneIndex])
		LL_TIM_OC_SetCompareCH4(htim, 0);
	if(--TimeCnt==0)
	{
		if(ToneIndex<ToneNum)
		{
			TimeCnt=TIME_ARRAY[ToneIndex];
			LL_TIM_OC_SetCompareCH4(htim, TONE_ARRAY[ToneIndex]>>1);
			LL_TIM_SetAutoReload(htim,TONE_ARRAY[ToneIndex]);
			TimeCnt=TIME_ARRAY[ToneIndex];
			htim->CCR4=(TONE_ARRAY[ToneIndex]>>1);
			htim->ARR =TONE_ARRAY[ToneIndex];
			htim->CNT =0;   //恢复计数器不然CNT>ARR时将发生错误
			ToneIndex++;

		}
		else
		{
			HoldOn=true;
			TimeCnt=1000;
			LL_TIM_OC_SetCompareCH4(htim, 0);
			return;
		}
	}
}


void BELL::Bell_Calc()
{
	xQueuePeek(queueBattery, &battery, 0);

	if(battery.battery_Voltage <= VOL_CRITICAL)
	{
		Battary_sta = BATTARY_CRITICAL;
	    led.LED_Tog();
	}
	else if(battery.battery_Voltage < VOL_NORM)
	{
		Err = ERR_SOFT;
		Battary_sta = BATTARY_LOW;
		led.LED_Set();
	}
	else
	{
		Err = ERR_NONE;
		Battary_sta = BATTARY_NORM;
		led.LED_Clr();
	}
}



void BELL::Bell_Sound(u16 *tone,u16 *time,u16 *idle,u8 num)
{
	if(num>TONE_NUM)num=TONE_NUM;
	ToneIndex=0;
	ToneNum=num;
	TimeCnt=1;
	for(int i=0;i<num;i++)
	{
		TONE_ARRAY[i]=tone[i];
		TIME_ARRAY[i]=time[i];
		IDLE_ARRAY[i]=idle[i];
	}
	Update=true;
}



void BELL::Bell_Start()
{
	if(Update==true)return;
	u16 time[6]={200,200,200,200,200,200};
	u16 idle[6]={100,100,100,100,100,100};
	u16 tone[6]={3800,3000,2200,2200,3000,3800};
	Bell_Sound(tone,time,idle,6);
}

void BELL::Bell_Error()
{
	if(Update==true)return;
	u16 time[2]={500,500};
	u16 idle[2]={200,200};
	u16 tone[2]={3500,2000};
	Bell_Sound(tone,time,idle,2);
}

void BELL::Bell_Cali()
{
	if(Update==true)return;
	u16 time[3]={300,300,300};
	u16 idle[3]={100,100,100};
	u16 tone[3]={3000,3000,3000};
	Bell_Sound(tone,time,idle,3);
}

extern "C" void bell_main(void *argument)
{
	bell.Bell_Init();
	osDelay(10);
	bell.Bell_Start();
	osDelay(10);
	static u8 cnt = 0;
	for(;;)
	{
		osSemaphoreAcquire(semBell,0xffffffff);
		bell.Bell_Loop_1ms();
		if(cnt%100==0)
		{
			bell.Bell_Calc();
			//bell.Bell_Error();
		}
		cnt++;
	}

}

/************************ (C) COPYRIGHT Longmen Drone Team *****END OF FILE****/
