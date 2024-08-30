/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : pid.hpp
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

#ifndef __PID_HPP
#define __PID_HPP


#include "userlib/userlib.hpp"
#include "system/system.hpp"

#ifdef __cplusplus
extern "C" {
#endif

#define ERRDEF_SLOPE_NUM 20

#ifdef __cplusplus
}
#endif


#ifdef __cplusplus

using namespace std;

class PID
{
public:
	PID();
	~PID();

	float Pid_Controller(float SP, float FB, float Dt, u8 Mode);
	float Pid_Controller(float SP, float FB, float Dt, u8 Mode, float minValue, float maxValue);
	void Pid_Reset(void);

	float get_PIDOutput(void);

	void Pid_Set_Para(float Kp,float Ki,float Kd,float Kb,float iLimit,float eLimit,float dLimit);
	void Pid_Set_Para(float Kp,float Ki,float Kd);

	float Pid_Get_Kp();
	float Pid_Get_Ki();
	float Pid_Get_Kd();

private:

	float setpoint;		// 设定值
	float feedback;		// 反馈值
	float lastfeedback; // 上一个反馈值
	float error;		// 误差
	float lastError;    // 上一个误差
	float integral;		// 当前积分值
	float Kp;			// 比例系数 proportional gain
	float Ki;			// 积分系数 integral gain
	float Kd;			// 微分系数 differential gain
	float Kb;           // 微分先行系数
	float pout;			// (debugging)
	float iout;			// (debugging)
	float dout;			// (debugging)
	float output;		// 当前PID的输出
	float dt;

	float ErrDefTmp[ERRDEF_SLOPE_NUM];
	float dout2;
	float dout3;

	float iLimit;
	float eLimit;
	float dLimit;



};


#endif


#endif


/************************ (C) COPYRIGHT Longmen Drone Team *****END OF FILE****/
