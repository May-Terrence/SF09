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

#include "system/system.hpp"
#include "userlib/userlib.hpp"


#ifdef __cplusplus
extern "C" {
#endif

#define ERRDEF_SLOPE_NUM 20

#define PID_NUM 18

#ifdef __cplusplus
}
#endif


#ifdef __cplusplus

using namespace std;

class PID
{
public:
	PID(){}
	~PID(){}

	float PID_Controller(double err,float Dt);
	float PID_Controller(double err,double rate_err,float Dt);
	float PID_tmp(void);
	float PID_anti_windup(float err, float Dt);
	void Pid_Reset(void);
	float get_PIDOutput(void);

	void Pid_Set_Para(float Kp,float Ki,float Kd,float Kb,float iLimit,float eLimit,float dLimit);
	void Pid_Set_Para(float Kp,float Ki,float Kd);

	float Pid_Get_Kp();
	float Pid_Get_Ki();
	float Pid_Get_Kd();


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
	float signal;		// 阶跃信号
	float filter_para; //低通滤波参数
	float ErrDefTmp[ERRDEF_SLOPE_NUM];
	float dout2;
	float dout3;

	float iLimit;
	float eLimit;
	float dLimit;
	float Beta;
	float lowerBound;
	float upperBound;
	float u_unsat;
private:


};


#endif


#endif


/************************ (C) COPYRIGHT Longmen Drone Team *****END OF FILE****/
