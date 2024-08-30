/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : control.cpp
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

#include "control.hpp"

PID pid[PID_NUM];
PID *pidRolRate=&pid[0],*pidPitRate=&pid[1],*pidYawRate=&pid[2];
PID *pidRol=&pid[3],*pidPit=&pid[4],*pidYaw=&pid[5];
PID *pidU=&pid[6],*pidV=&pid[7],*pidW=&pid[8];
PID *pidX=&pid[9],*pidY=&pid[10],*pidZ=&pid[11];

ATTITUDE_CTRL::ATTITUDE_CTRL()
{
	pidRolRate->Pid_Set_Para(1.3f, 0.14f, 0.08f, 0.5f, 10000*M_PI, 4*M_PI, M_PI);
	pidPitRate->Pid_Set_Para(1.3f, 0.14f, 0.08f, 0.5f, 10000*M_PI, 4*M_PI, M_PI);
	pidYawRate->Pid_Set_Para(1.3f, 0.14f, 0.08f, 0.0f, 10000*M_PI, 4*M_PI, M_PI);
	pidRol->Pid_Set_Para(4.8f, 0.0f, 2.0f, 0.0f, 100*M_PI, 4*M_PI, M_PI);
	pidPit->Pid_Set_Para(4.8f, 0.0f, 2.0f, 0.0f, 100*M_PI, 4*M_PI, M_PI);
	pidYaw->Pid_Set_Para(4.5f, 0.0f, 2.0f, 0.0f, 100*M_PI, 4*M_PI, M_PI);
	pidU->Pid_Set_Para(20.0f, 0.0f, 0.0f, 0.0f, 100*M_PI, 4*M_PI, M_PI);
	pidV->Pid_Set_Para(20.0f, 0.0f, 0.0f, 0.0f, 100*M_PI, 4*M_PI, M_PI);
	pidW->Pid_Set_Para(40.0f, 8.0f, 0.2f, 0.5f, 100*M_PI, 4*M_PI, M_PI);
	pidX->Pid_Set_Para(1.0f, 0.0f, 0.0f, 0.0f, 100*M_PI, 4*M_PI, M_PI);
	pidY->Pid_Set_Para(1.0f, 0.0f, 0.0f, 0.0f, 100*M_PI, 4*M_PI, M_PI);
	pidZ->Pid_Set_Para(1.6f, 0.1f, 0.0f, 0.0f, 100*M_PI, 4*M_PI, M_PI);

}

ATTITUDE_CTRL::~ATTITUDE_CTRL()
{

}

void ATTITUDE_CTRL::flight_Mode_Update()
{
	/* 判别飞行模式 ******************************************/
	switch(rcCommand.Key[0])
	{
		case 0: //姿态
			Mode = QAUD_MANUAL;
			break;
		case 1: //姿态+定高
			Mode = QAUD_ATTITUDE;
			break;
		case 2: //全自控	//定点+定高
			Mode = QAUD_ALT_HOLD;
			break;
	}
	if(rcCommand.Key[2] == 0)	//锁尾
	{
		LockHead = true;
		if(LockHeadFirst)
		{
			HoldYaw = ahrsEuler.Ang[2]*R2D;
			LockHeadFirst = false;
		}
	}
	else
	{
		LockHead = false;
		LockHeadFirst = true;
	}
}

void ATTITUDE_CTRL::motor_Output()
{
	if(THROTTLE <1040)
	{
		#ifdef QUADX	//X字型的四轴飞行器
			motorPWM.PWM[0] = PWM_IDLE; //FRONT_L 前左电机
			motorPWM.PWM[1] = PWM_IDLE; //REAR_L  后左电机
			motorPWM.PWM[2] = PWM_IDLE; //FRONT_R 前右电机
			motorPWM.PWM[3] = PWM_IDLE; //REAR_R  后右电机
		#endif
	}
	else
	{
		#define MIX(X,Y,Z) THROTTLE + FOREWARD + Z_OUT + ROL_OUT*(X) + PIT_OUT*(Y) + YAW_OUT*(Z)

		#ifdef QUADP	//十字型的四轴飞行器
			motorPWM.PWM[4] = MIX( 0,+1,+1); //FRONT 前面电机
			motorPWM.PWM[5] = MIX(+1, 0,-1); //LEFT	 左边电机
			motorPWM.PWM[6] = MIX( 0,-1,+1); //REAR	 后尾电机
			motorPWM.PWM[7] = MIX(-1, 0,-1); //RIGHT 右边电机
		#endif
		#ifdef QUADX	//X字型的四轴飞行器
			motorPWM.PWM[0] = MIX(+1,+1,+1); //FRONT_L 前左电机
			motorPWM.PWM[1] = MIX(+1,-1,-1); //REAR_L  后左电机
			motorPWM.PWM[2] = MIX(-1,+1,-1); //FRONT_R 前右电机
			motorPWM.PWM[3] = MIX(-1,-1,+1); //REAR_R  后右电机
		#endif
		#ifdef OCTAP	//十字型的八轴飞行器
			const float rt2 = 1.414213562373f;
			motorPWM.PWM[4] = MIX(   0,  +2,-1); //1号电机
			motorPWM.PWM[5] = MIX(-rt2, rt2,+1); //2号电机
			motorPWM.PWM[6] = MIX(  -2,   0,-1); //3号电机
			motorPWM.PWM[7] = MIX(-rt2,-rt2,+1); //4号电机
			motorPWM.PWM[0] = MIX(   0,  -2,-1); //5号电机
			motorPWM.PWM[1] = MIX( rt2,-rt2,+1); //6号电机
			motorPWM.PWM[2] = MIX(  +2,   0,-1); //7号电机
			motorPWM.PWM[3] = MIX( rt2, rt2,+1); //8号电机
		#endif
	}
	xQueueOverwrite(queueMotorPWM,&motorPWM);
}

void ATTITUDE_CTRL::Attitude_Ctrl_Init()
{
	LockHeadFirst = true;
	isFirstAltHold = true;
	Mode = QAUD_MANUAL;

	osDelay(100);
	for(uint8_t i=0;i<12;i++)
	{
		pid_para.kp[i] = pid[i].Pid_Get_Kp();
		pid_para.ki[i] = pid[i].Pid_Get_Ki();
		pid_para.kd[i] = pid[i].Pid_Get_Kd();
	}
	xQueueOverwrite(queuePID,&pid_para);

}

void ATTITUDE_CTRL::Attitude_ctrl_run()
{

	getTimer_us(&startTimer);
	xQueuePeek(queueGyrDat, &gyro, 0);
	xQueuePeek(queueRCCommand, &rcCommand, 0);
	xQueuePeek(queueAhrsEuler, &ahrsEuler, 0);

	pid_Para_Update();

	flight_Mode_Update();
	if(rcCommand.Thr < 400.0f)
	{
		for(u8 i=0; i<PID_NUM; i++)
			pid[i].Pid_Reset();
	}
	switch(Mode)
	{
		case QAUD_MANUAL:
			SetDAng[0] = rcCommand.dAng[0];
			SetDAng[1] = -rcCommand.dAng[1];
			if(LockHead == true)
			{
				SetAng[2] = -rcCommand.Ang[2] + HoldYaw;//锁尾，航向控角度
				FeedBackAng[2] = ahrsEuler.Ang[2]*R2D;
				SetDAng[2] = pidYaw->Pid_Controller(SetAng[2], FeedBackAng[2],0.005f,0);
			}
			else
			{
				SetDAng[2] = -rcCommand.dAng[2];//不锁尾，航向控角速度
			}
			THROTTLE = rcCommand.Thr;
			Z_OUT = 0;
			break;
		case QAUD_ATTITUDE:
		case QAUD_ALT_HOLD:
		case QAUD_POS_HOLD:
		case QAUD_TRAJECTORY:

			SetAng[0] = rcCommand.Ang[0];
			SetAng[1] = -rcCommand.Ang[1];

			//-------------------高度控制----------------------//

			if(Mode == QAUD_ALT_HOLD||Mode == QAUD_POS_HOLD)	//高度和位置控制模式都需要定高
			{
				if(isFirstAltHold)	//第一次进入定高模式，保存此时的油门值，然后将遥控器油门通道作为高度的设定值
				{
					THROTTLE = rcCommand.Thr;
					isFirstAltHold = false;
				}

				SetXyz[2] = rcCommand.Thr/800.0f;
				SetXyz[2] = fConstrain(SetXyz[2], 0.0f, 2.5f);
				SetUvw[2] = pidZ->Pid_Controller(SetXyz[2],FeedBackXyz[2],0.005f,0, -0.4f, 0.4f);
				Z_OUT = pidW->Pid_Controller(SetUvw[2],FeedBackUvw[2],0.005f,1);
				Z_OUT = Z_OUT*10.0f;
			}
			else	//否则为姿态控制模式
			{
				isFirstAltHold = true;	//将是否第一次进入定高模式标志位置1，方便下一次进入定高模式时可以保存油门值
				THROTTLE = rcCommand.Thr;
				Z_OUT = 0;
			}
			//-------------------高度控制----------------------//

			//------------姿态角控制-------------------//
			//姿态角反馈值
			FeedBackAng[0] = ahrsEuler.Ang[0]*R2D;
			FeedBackAng[1] = ahrsEuler.Ang[1]*R2D;

			pidRol->Pid_Controller(SetAng[0], FeedBackAng[0],0.005f,0);
			pidPit->Pid_Controller(SetAng[1], FeedBackAng[1],0.005f,0);
			float sinR, cosR, sinP, cosP;
			if(LockHead == true)		//锁尾，航向控角度
			{
				SetAng[2] = -rcCommand.Ang[2] + HoldYaw;
				if(SetAng[2]>180.0f)
					SetAng[2] -= 360.0f;
				else if(SetAng[2]<-180.0f)
					SetAng[2] += 360.0f;
				FeedBackAng[2] = ahrsEuler.Ang[2]*R2D;		//航向角反馈值
				pidYaw->Pid_Controller(SetAng[2], FeedBackAng[2],0.005f,4);
				//欧拉角速度转pqr
				sinR = sinf(ahrsEuler.Ang[0]);
				cosR = cosf(ahrsEuler.Ang[0]);
				sinP = sinf(ahrsEuler.Ang[1]);
				cosP = cosf(ahrsEuler.Ang[1]);
				SetDAng[0] =  pidRol->get_PIDOutput()      - pidYaw->get_PIDOutput()*sinP;
				SetDAng[1] =  pidPit->get_PIDOutput()*cosR + pidYaw->get_PIDOutput()*cosP*sinR;
				SetDAng[2] = -pidPit->get_PIDOutput()*sinR + pidYaw->get_PIDOutput()*cosP*cosR;

			}
			else		//不锁尾，航向控角速度
			{
				SetDAng[0] =  pidRol->get_PIDOutput();
				SetDAng[1] =  pidPit->get_PIDOutput();
				SetDAng[2] = -rcCommand.dAng[2];
			}
			//------------姿态角控制-------------------//

			break;
	}

	FeedBackDAng[0] = gyro.gyro[0]*R2D;
	FeedBackDAng[1] = gyro.gyro[1]*R2D;
	FeedBackDAng[2] = gyro.gyro[2]*R2D;
	pidRolRate->Pid_Controller(SetDAng[0],FeedBackDAng[0],0.005f,1);
	pidPitRate->Pid_Controller(SetDAng[1],FeedBackDAng[1],0.005f,1);
	pidYawRate->Pid_Controller(SetDAng[2],FeedBackDAng[2],0.005f,1);

	ROL_OUT = pidRolRate->get_PIDOutput();
	PIT_OUT = pidPitRate->get_PIDOutput();
	YAW_OUT = pidYawRate->get_PIDOutput();

	motor_Output();

	getTimer_us(&stopTimer);

	executionTime_us = stopTimer - startTimer;
}

void ATTITUDE_CTRL::pid_Para_Update()
{
	xQueuePeek(queuePID, &pid_para, 0);
	for(uint8_t i=0;i<12;i++)
	{
		pid[i].Pid_Set_Para(pid_para.kp[i], pid_para.ki[i], pid_para.kd[i]);
	}
}

ATTITUDE_CTRL attitude_ctrl;
extern "C" void attitudeCtrl_main(void *argument)
{
	attitude_ctrl.Attitude_Ctrl_Init();
	osDelay(200);
	for(;;)
	{
		osSemaphoreAcquire(semAttitudeCtrl,0xffffffff);
		attitude_ctrl.Attitude_ctrl_run();
	}
}




/************************ (C) COPYRIGHT Longmen Drone Team *****END OF FILE****/
