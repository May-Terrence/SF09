/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : motor.cpp
  * Description        : 
  ******************************************************************************
  * Function           :
  *
  ******************************************************************************
  * Author             : Zhiping Fu
  * Creation Date      : Oct 23, 2020
  ******************************************************************************
  */
/* USER CODE END Header */

#include "motor.hpp"


void MOTOR::motor_Init(void)
{
	Sta = STA_INI;
	Err = ERR_NONE;
	UnLock = false;
	executionTime_us = 0;
	cycleTime_us = 0;
	for(uint8_t i=0;i<PWM_OUT_NUM;i++)
	{
		PWM[i] = INI_PWM;
	}

	osDelay(200);
	//启动定时器
	LL_TIM_EnableCounter(htimA);
	LL_TIM_OC_SetCompareCH1(htimA, INI_PWM);
	LL_TIM_OC_SetCompareCH2(htimA, INI_PWM);
	LL_TIM_OC_SetCompareCH3(htimA, INI_PWM);
	LL_TIM_OC_SetCompareCH4(htimA, INI_PWM);
	LL_TIM_CC_EnableChannel(htimA, LL_TIM_CHANNEL_CH1);
	LL_TIM_CC_EnableChannel(htimA, LL_TIM_CHANNEL_CH2);
	LL_TIM_CC_EnableChannel(htimA, LL_TIM_CHANNEL_CH3);
	LL_TIM_CC_EnableChannel(htimA, LL_TIM_CHANNEL_CH4);
	LL_TIM_EnableAllOutputs(htimA);


	LL_TIM_EnableCounter(htimB);
	LL_TIM_OC_SetCompareCH1(htimB, INI_PWM);
	LL_TIM_OC_SetCompareCH2(htimB, INI_PWM);
	LL_TIM_OC_SetCompareCH3(htimB, INI_PWM);
	LL_TIM_OC_SetCompareCH4(htimB, INI_PWM);
	LL_TIM_CC_EnableChannel(htimB, LL_TIM_CHANNEL_CH1);
	LL_TIM_CC_EnableChannel(htimB, LL_TIM_CHANNEL_CH2);
	LL_TIM_CC_EnableChannel(htimB, LL_TIM_CHANNEL_CH3);
	LL_TIM_CC_EnableChannel(htimB, LL_TIM_CHANNEL_CH4);
	LL_TIM_EnableAllOutputs(htimB);

	Sta = STA_RUN;

}


void MOTOR::motor_run(void)
{
	startTimerLast = startTimer;
	getTimer_us(&startTimer);
	cycleTime_us = startTimer - startTimerLast;

	xQueuePeek(queueRCCommand, &rcCommand, 0);
	xQueuePeek(queueMotorPWM,&motorPWM, 0);
	if(rcCommand.Key[3]<1) UnLock = false;
	else if(rcCommand.Key[3]>1) UnLock = true;

	if(UnLock == false)
	{
		for(uint8_t i=0;i<PWM_OUT_NUM;i++)
		{
			PWM[i] = INI_PWM;
		}
	}
	else
	{
		for(uint8_t i=0;i<PWM_OUT_NUM;i++)
		{
			PWM[i] = motorPWM.PWM[i]<Min_PWM_Out?Min_PWM_Out:(motorPWM.PWM[i]>Max_PWM_Out?Max_PWM_Out:motorPWM.PWM[i]);
		}
	}
	LL_TIM_OC_SetCompareCH1(htimA, PWM[0]);
	LL_TIM_OC_SetCompareCH2(htimA, PWM[1]);
	LL_TIM_OC_SetCompareCH3(htimA, PWM[2]);
	LL_TIM_OC_SetCompareCH4(htimA, PWM[3]);

	getTimer_us(&stopTimer);

	executionTime_us = stopTimer - startTimer;
}

MOTOR motor(TIM3,TIM4,(char *)"MOTOR");
extern "C" void motor_main(void *argument)
{
	motor.motor_Init();
	osDelay(1);
	for(;;)
	{
		osSemaphoreAcquire(semMotor,0xffffffff);
		motor.motor_run();
	}
}


/************************ (C) COPYRIGHT Longmen Drone Team *****END OF FILE****/
