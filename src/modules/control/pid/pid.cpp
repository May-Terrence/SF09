/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : pid.cpp
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

#include "pid.hpp"

PID::PID()
{

}

PID::~PID()
{

}

float PID::Pid_Controller(float SP, float FB, float Dt, u8 Mode)
{
	lastError=error;
	lastfeedback = feedback;
	setpoint = SP;
	feedback = FB;
	dt = Dt;
	if(Mode == 4)
		error = LoopConstrain(setpoint - feedback, -180.0f, 180.0f);
	else
		error=setpoint - feedback;
	pout = error * Kp;
	integral += error*dt;
	iout = fConstrain(integral * Ki,-iLimit,iLimit);	//积分限幅

	if(Mode == 0 || Mode == 4)
		dout = (error - lastError) * Kd / dt;
	else if(Mode == 1)		//不完全微分
		dout = (1.0f - Kb)*(error - lastError) * Kd / dt + Kb*dout;
	else if (Mode == 2)		//微分先行
		dout = (feedback - lastfeedback) * Kd /dt;
	else if(Mode == 3)		//不完全微分+微分先行
		dout = (1.0f - Kb) * (lastfeedback - feedback) * Kd / dt + Kb*dout;

	output = pout + iout + dout;
	return output;
}

float PID::Pid_Controller(float SP, float FB, float Dt, u8 Mode, float minValue, float maxValue)
{
	lastError=error;
	lastfeedback = feedback;
	setpoint = SP;
	feedback = FB;
	dt = Dt;
	if(Mode == 4)
		error = LoopConstrain(setpoint - feedback, -180.0f, 180.0f);
	else
		error=setpoint - feedback;
	pout = error * Kp;
	integral += error*dt;
	iout = fConstrain(integral * Ki,-iLimit,iLimit);	//积分限幅

	if(Mode == 0 || Mode == 4)
		dout = (error - lastError) * Kd / dt;
	else if(Mode == 1)		//不完全微分
		dout = (1.0f - Kb)*(error - lastError) * Kd / dt + Kb*dout;
	else if (Mode == 2)		//微分先行
		dout = (feedback - lastfeedback) * Kd /dt;
	else if(Mode == 3)		//不完全微分+微分先行
		dout = (1.0f - Kb) * (lastfeedback - feedback) * Kd / dt + Kb*dout;

	output = pout + iout + dout;
	if (output < minValue)
		output = minValue;
	else if (output > maxValue)
		output = maxValue;
	return output;
}

void PID::Pid_Reset(void)
{
	integral = 0;
}

void PID::Pid_Set_Para(float Kp,float Ki,float Kd,float Kb,float iLimit,float eLimit,float dLimit)
{
	this->Kp = Kp;
	this->Ki = Ki;
	this->Kd = Kd;
	this->Kb = Kb;
	this->iLimit = iLimit;
	this->eLimit = eLimit;
	this->dLimit = dLimit;
}


void PID::Pid_Set_Para(float Kp,float Ki,float Kd)
{
	this->Kp = Kp;
	this->Ki = Ki;
	this->Kd = Kd;
}

float PID::get_PIDOutput(void)
{
	return output;
}

float PID::Pid_Get_Kp()
{
	return Kp;
}

float PID::Pid_Get_Ki()
{
	return Ki;
}

float PID::Pid_Get_Kd()
{
	return Kd;
}

/************************ (C) COPYRIGHT Longmen Drone Team *****END OF FILE****/
