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

float PID::PID_Controller(double err,float Dt)
{
	lastError=error;
//	ele->error=fConstrain(err,-ele->eLimit,ele->eLimit);
	error=err;
	pout = error * Kp;
	integral += error*Dt;
	iout = fConstrain(integral * Ki,-iLimit,iLimit);
	dout = (error-lastError)/Dt;
	//--------------误差导数最小二乘滤波--------------
	if((error>0 && lastError<0) || (error<0 && lastError>0)) memset(ErrDefTmp, 0, sizeof(ErrDefTmp));
	for(u8 i=1;i<ERRDEF_SLOPE_NUM;i++)
		ErrDefTmp[i-1]=ErrDefTmp[i];
	ErrDefTmp[ERRDEF_SLOPE_NUM-1]=error;
	float ab[2];
	LineFit(ErrDefTmp,ERRDEF_SLOPE_NUM,ab);
	dout2 = ab[0]/Dt;
	//------------------------------------------------
//	ele->dout = fConstrain(ele->dout,-ele->dLimit,ele->dLimit);
	dout3 = dout2*Kd;
    if ((pout > 0 && dout3 < 0) || (pout < 0 && dout3 > 0))dout3 = 0;
	output = pout + iout + dout3;
	return output;
}

float PID::PID_Controller(double err,double rate_err,float Dt)
{
	lastError=error;
	error=fConstrain(err,-eLimit,eLimit);
	pout = error * Kp;
	integral += error*Dt;
	integral = fConstrain(integral,-iLimit,iLimit);
	iout = integral * Ki;
	dout = rate_err*Kd;
	output = pout + iout + dout;
	return output;
}

float PID::PID_tmp()
{
	error=setpoint - feedback;
	if(iout <800.0f&&iout>-800.0f)
	{
		if(error> 5.0f||error<- 5.0f) iout=0;    //积分分离
		else
		{
			integral += error;
			iout=Ki*integral;
		}
	}
	else if  (((iout>800||iout==800)&&error<0)||((iout<-800||iout==-800)&&error>0))
    {
		integral += error;
		iout=Ki*integral;
    }

	pout=Kp*error;
	//dout=(Kd*(error-lastError)+filter_para*lastdout)/(0.001+filter_para);  //低通滤波微分公式
	dout=Kd*(error-lastError);
	output=pout+iout;//+ele->dout;
	//output=pout+iout;
	if(output < 0) output=0;
	lastError=error;
	if(output >999)output=999;
	//lastdout=dout;  //此处为上一微分输出值
	return output;
}
float PID::PID_anti_windup(float err, float Dt)//反饱和处理
{
	lastError = error;
	error = err;
	// propotional control
	pout =  error * Kp;
	// integral control
	integral += Dt/2*(lastError+err);	//trapazoidal rule
	iout = Ki*integral;
	// derivative control
	Beta = 47/50;		// gain on dirty derivative:Beta = (2*tau−Ts)/(2*tau+Ts), 1/tau is the bandwidth of the differentiator
	dout = Beta*dout + (1-Beta)*(error - lastError)/Dt;
	dout2 = fConstrain(dout*Kd, lowerBound, upperBound);
	u_unsat = fConstrain(pout + dout2, lowerBound, upperBound) + iout;
	output = fConstrain(u_unsat, lowerBound, upperBound);
	if(Ki != 0.0f)
	{
		integral = integral + 1/Ki*(output - u_unsat);
	}
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
	this->integral = 0.0f;
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
