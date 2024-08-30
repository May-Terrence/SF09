/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : motor.hpp
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

#ifndef __MOTOR_HPP
#define __MOTOR_HPP

#include "userlib/userlib.hpp"
#include "system/system.hpp"

#ifdef __cplusplus
extern "C" {
#endif

#define quadrocopter
//#define helicopter
//#define duct

#define PWM_OUT_NUM 4
#define INI_PWM 1002
#define Min_PWM_Out  1002 //0
#define Max_PWM_Out  1996//2047

#ifdef __cplusplus
}
#endif


#ifdef __cplusplus

using namespace std;

class MOTOR
{
public:
	MOTOR(){}
	MOTOR(TIM_TypeDef *ha,char *n) : htimA(ha),name(n){}
	MOTOR(TIM_TypeDef *ha,TIM_TypeDef *hb,char *n) : htimA(ha),htimB(hb),name(n){}
	~MOTOR(){}

	void motor_Init(void);


	void motor_run(void);

private:
	TIM_TypeDef *htimA;
	TIM_TypeDef *htimB;
	char *name;

	eSTA  Sta;        //状态  --
	eERR  Err;        //错误信息  --
	bool  UnLock;     //解锁

	uint32_t  startTimer;			//计时器
	uint32_t  stopTimer;			//计时器
	uint32_t  executionTime_us;			//计时器

	uint32_t  startTimerLast;			//计时器
	uint32_t  cycleTime_us;


	uint16_t PWM[PWM_OUT_NUM];


	RC_command_msg rcCommand;
	Motor_PWM_msg motorPWM;

};

#endif


#endif


/************************ (C) COPYRIGHT Longmen Drone Team *****END OF FILE****/
