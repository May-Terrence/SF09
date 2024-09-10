/*
 * control.cpp
 *
 *  Created on: 2020年9月2日
 *      Author: 17900
 */
#include "control.hpp"
#include "alloc/dir_alloc.hpp"
#include "claw_tran/claw_tran.hpp"
#include "openlog/USART_SD_Log.hpp"

CONTROL control;
DIR_ALLOC dir_alloc;

void CONTROL::Control_Init()
{
	CtrlIO.control_mode   = 0;
	CtrlIO.FlightStatus   = GROUND;
	CtrlIO.fly_mode       = 0;
	CtrlIO.yaw_mode       = 0;
	CtrlLpIO.gyro_gain    = 0.03f;
	CtrlIO.mid_trim[0]    = 0.0f;
	CtrlIO.mid_trim[1]    = 0.0f;
	CtrlIO.mid_trim[2]    = 0.0f;
	CtrlKiTemp.int_reset  = true;
	CtrlFbck.engine_speed = 0.0f;
	CtrlLpIO.AngleLimitR  = 20*D2R;
	CtrlLpIO.AngleLimitP  = 20*D2R;

	CtrlLpIO.XY_phase          = 0;
	CtrlLpIO.XY_phase1          = 0;
	CtrlLpIO.count=0;
	CtrlLpIO.count1=0;
	CtrlLpIO.X_pos0=0.0;
	CtrlLpIO.Y_pos0=0.0;
	CtrlLpIO.Z_pos0=0.0;

	Ctrltrack.PTP_Status = PTP_hover;
	Ctrltrack.Circle_Status = Circle_hover;

	CtrlLpIO.XY_phase2          = 0;
	Ctrltrack.POS_err          = 0.0;
	CtrlLpIO.POS_thr          = 0.3;

	CtrlLpIO.Z_phase           = 0;
	CtrlLpIO.Z_phase1           = 5;

	CtrlLpIO.brake_mode        = false;
	CtrlLpIO.alt_brake_mode    = false;
	CtrlLpIO.landing_acc_mode  = false;
	CtrlLpIO.trajectory_start  = true;



	CtrlLpIO.circle_start      = false;
	CtrlLpIO.brake_cnt.CNT     = 0;
	CtrlLpIO.alt_brake_cnt.CNT = 0;
	CtrlLpIO.alt_landing_cnt.CNT = 0;

	CtrlLpIO.brake_cnt.CCR     = 50;
	CtrlLpIO.alt_brake_cnt.CCR = 50;
	CtrlLpIO.alt_landing_cnt.CCR = 50;
	CtrlIO.JUMP = 0;
	CtrlIO.rc_cnt    = 0;
	CtrlIO.rc_check  = 0;
	CtrlIO.ever_rc   = 0;
	CtrlIO.rc_status = NORMAL;

	CtrlLpIO.VelSet=0.0;
	CtrlLpIO.relative_height = 0.0f;
	CtrlLpIO.indi_fltr_band = 16.7532f; // 单位Hz

	for(int i=0; i<RC_CHECK_NUM; i++)
	{
		CtrlIO.rc_buffer[i] = 0;
	}

	//------------外环控制-----------------------------------
	CtrlLpIO.Phase   = 0;
	CtrlIO.tran_mode = 3;
	CtrlIO.level_mode = 1;
	CtrlLpIO.ko1     = 2.0f;


	//---------控制分配--------------------------------------

	dir_alloc.dir_alloc_mch_initialize();
//	wls_alloc_mch_initialize();

	//--------INDI-----------------------------------------

	CtrlINDI.pqr_d0[0]          = 0.0f;
	CtrlINDI.pqr_d0[1]          = 0.0f;
	CtrlINDI.pqr_d0[2]          = 0.0f;
	CtrlINDI.output_0[0]        = 0.0f;
	CtrlINDI.output_0[1]        = 0.0f;
	CtrlINDI.output_0[2]        = 0.0f;
	CtrlINDI.Filt_AngularAcc[0] = 40;
	CtrlINDI.Filt_AngularAcc[1] = 40;
	CtrlINDI.Filt_AngularAcc[2] = 30;
	CtrlINDI.Filt_Output[0]     = 40;
	CtrlINDI.Filt_Output[1]     = 40;
	CtrlINDI.Filt_Output[2]     = 30;
	CtrlINDI.output_acc0[0]     = 0;
	CtrlINDI.output_acc0[1]     = 0;
	CtrlINDI.output_acc0[2]     = -OneG;

	//------------------------------------------------------

	for(u8 i=0;i<PID_NUM;i++)
	{
		pid[i].Kp = 0;
		pid[i].Ki = 0;
		pid[i].Kd = 0;
	}

	//pidRolRate
		pidRolRate->Kp = 0.3;//0.25f;
		pidRolRate->Ki = 0.0;//0.3f;
		pidRolRate->Kd = 0.0f;
		pidRolRate->Kb = 0.0f;
		pidRolRate->eLimit = 4*PI;
		pidRolRate->iLimit = 30*D2R;
		pidRolRate->dLimit = PI;

		//pidPitRate
		pidPitRate->Kp = 0.3;//0.25f;
		pidPitRate->Ki = 0.0;//0.3f;
		pidPitRate->Kd = 0.0f;
		pidPitRate->Kb = 0.0f;
		pidPitRate->eLimit = 4*PI;
		pidPitRate->iLimit = 30*D2R;
		pidPitRate->dLimit = PI;

		//pidYawRate
		pidYawRate->Kp = 0.18;//0.13f;
		pidYawRate->Ki = 0.0;//0.25f;
		pidYawRate->Kd = 0.00f;
		pidYawRate->Kb = 0.0f;
		pidYawRate->eLimit = 4*PI;
		pidYawRate->iLimit = 30*D2R;
		pidYawRate->dLimit = PI;

		//pidRol
		pidRol->Kp = 5.5;//4.0f;
		pidRol->Ki = 0.0f;
		pidRol->Kd = 0.0f;
		pidRol->Kb = 0.0f;
		pidRol->eLimit = PI;
		pidRol->iLimit = PI;

		//pidPit
		pidPit->Kp = 5.5;//4.0f;
		pidPit->Ki = 0.0f;
		pidPit->Kd = 0.0f;
		pidPit->Kb = 0.0f;
		pidPit->eLimit = PI;
		pidPit->iLimit = PI;

		//pidYaw
		pidYaw->Kp = 5.0f;
		pidYaw->Ki = 0.1f;
		pidYaw->Kd = 0.0f;
		pidYaw->Kb = 0.0f;
		pidYaw->eLimit = PI;
		pidYaw->iLimit = PI;

		//pidXRate
		pidXRate->Kp = 1.5f;
		pidXRate->Ki = 0.0f;
		pidXRate->Kd = 0.0f;
		pidXRate->Kb = 0.0f;
		pidXRate->eLimit = 20;
		pidXRate->iLimit = 20;
		pidXRate->dLimit = 20;

		//pidYRate
		pidYRate->Kp = 1.5f;
		pidYRate->Ki = 0.0f;
		pidYRate->Kd = 0.0f;
		pidYRate->Kb = 0.0f;
		pidYRate->eLimit = 20;
		pidYRate->iLimit = 20;
		pidYRate->dLimit = 20;

		//pidZRate
		pidZRate->Kp = 2.0f;
		pidZRate->Ki = 0.05f;
		pidZRate->Kd = 0.0f;
		pidZRate->Kb = 0.0f;
		pidZRate->eLimit = 20;
		pidZRate->iLimit = 200;
		pidZRate->dLimit = 20;

		//pidX
		pidX->Kp = 0.5f;
		pidX->Ki = 0.0f;
		pidX->Kd = 0.0f;
		pidX->Kb = 0.0f;
		pidX->eLimit = 200;
		pidX->iLimit = 200;
		pidX->dLimit = 200;

		//pidY
		pidY->Kp = 0.5f;
		pidY->Ki = 0.0f;
		pidY->Kd = 0.0f;
		pidY->Kb = 0.0f;
		pidY->eLimit = 200;
		pidY->iLimit = 200;
		pidY->dLimit = 200;

		//pidZ
		pidZ->Kp = 2.0f;
		pidZ->Ki = 0.0f;
		pidZ->Kd = 0.0f;
		pidZ->Kb = 0.0f;
		pidZ->eLimit = 20;
		pidZ->iLimit = 200;
		pidZ->dLimit = 20;

		//pidTmp
//		pidTmp->Kp = 30.0f;
//		pidTmp->Ki = 0.07f;
//		pidTmp->Kd = 20.02f;
//		pidTmp->filter_para = 0.557f;
//		pidTmp->setpoint=40.0f;

		//pidThrust
		pidThrust->Kp = 0.1f; //PID13
		pidThrust->Ki = 0.05f;
		pidThrust->Kd = 0.0f;
		pidThrust->Kb = 0.0f;
		pidThrust->eLimit = 200;
		pidThrust->iLimit = 200;
		pidThrust->dLimit = 200;

	//调试信号
//	pid[13].Kp = CtrlLpIO.gyro_gain;
//
//	pid[14].Kp = CtrlIO.mid_trim[0];
//	pid[14].Ki = CtrlIO.mid_trim[1];
//	pid[14].Kd = CtrlIO.mid_trim[2];
//
//	pid[15].Kp = CtrlLpIO.ko1;
//	pid[15].Ki = 25;
//	pid[15].Kd = 1;
//
//
//	pid[12].Kp = 0.5f;//调试互补滤波用，初始值
//	pid[12].Ki = 0.5f;

	pid[17].Kp = CtrlLpIO.ko1;
	pid[17].Ki = CtrlLpIO.VelSet;
	pid[17].Kd = CtrlLpIO.relative_height;

	for(u8 i=0;i<PID_NUM;i++)
	{
		pid_msg.kp[i] = pid[i].Kp;
		pid_msg.ki[i] = pid[i].Ki;
		pid_msg.kd[i] = pid[i].Kd;
	}
	xQueueOverwrite(queuePID,&pid_msg);
}

void CONTROL::Control_Feedback()
{
	xQueuePeek(queueGyrDat, &gyro, 0);					//从队列中获取陀螺仪数据
	xQueuePeek(queueGyrDatFil, &gyro_filt,0);
	xQueuePeek(queueESKF,&eskf,0);
	xQueuePeek(queueHeight,&height,0);
	xQueuePeek(queueAhrsEuler, &ahrsEuler, 0);

	CtrlFbck.AirSpeed_Last   = CtrlFbck.AirSpeed;
	static bool iniPos=false;
	if(eskf.update && !iniPos){
		CtrlLpIO.X_pos0 = CtrlFbck.X[0];//记录初始位置
		CtrlLpIO.Y_pos0 = CtrlFbck.Y[0];
		CtrlLpIO.Z_pos0 = CtrlFbck.Z[0];
		iniPos=true;
	}
//位置
	CtrlFbck.X[0] = eskf.Pos[0];
	CtrlFbck.Y[0] = eskf.Pos[1];
	CtrlFbck.Z[0] = eskf.Pos[2];
//	CtrlFbck.Z[0] = -height.height*0.01;
//速度
	CtrlFbck.X[1] = eskf.Ned_spd[0];
	CtrlFbck.Y[1] = eskf.Ned_spd[1];
	CtrlFbck.Z[1] = eskf.Ned_spd[2];
//机头
	float sinY, cosY;
	sinY = sinf(eskf.Attitude[2]);
	cosY = cosf(eskf.Attitude[2]);

//	CtrlFbck.XH[0] =  CtrlFbck.X[0]*cosY + CtrlFbck.Y[0]*sinY;
//	CtrlFbck.YH[0] = -CtrlFbck.X[0]*sinY + CtrlFbck.Y[0]*cosY;

	CtrlFbck.XH[1] =  CtrlFbck.X[1]*cosY + CtrlFbck.Y[1]*sinY;
	CtrlFbck.YH[1] = -CtrlFbck.X[1]*sinY + CtrlFbck.Y[1]*cosY;
//机体速度
	CtrlFbck.U[0] = CtrlFbck.X[1]*eskf.rotation[0][0] + CtrlFbck.Y[1]*eskf.rotation[1][0] + CtrlFbck.Z[1]*eskf.rotation[2][0];
	CtrlFbck.V[0] = CtrlFbck.X[1]*eskf.rotation[0][1] + CtrlFbck.Y[1]*eskf.rotation[1][1] + CtrlFbck.Z[1]*eskf.rotation[2][1];
	CtrlFbck.W[0] = CtrlFbck.X[1]*eskf.rotation[0][2] + CtrlFbck.Y[1]*eskf.rotation[1][2] + CtrlFbck.Z[1]*eskf.rotation[2][2];

	Rb2n << eskf.rotation[0][0], eskf.rotation[0][1], eskf.rotation[0][2],
			eskf.rotation[1][0], eskf.rotation[1][1], eskf.rotation[1][2],
			eskf.rotation[2][0], eskf.rotation[2][1], eskf.rotation[2][2];

	for (u8 i=0;i<3;i++)
	{
//		CtrlFbck.pqr[i]=gyro.gyro[i];
		CtrlFbck.pqr[i]= gyro_filt.gyro_filter[i];//调试用临时注释
//		CtrlFbck.Ang[i]= eskf.Attitude[i];
		CtrlFbck.Ang[i] = ahrsEuler.Ang[i];
		Ctrltrack.angeskf[i] = eskf.Attitude[i];
	}
	xQueueOverwrite(queuetrajectoryData,&Ctrltrack);

	//----------北东地位置----------------------

	CtrlFbck.AirSpeed = sqrt(CtrlFbck.X[1]*CtrlFbck.X[1]+CtrlFbck.Y[1]*CtrlFbck.Y[1]+CtrlFbck.Z[1]*CtrlFbck.Z[1]);

	if (CtrlIO.control_mode==2 || CtrlIO.control_mode==3)
	{
		CtrlFbck.engine_speed=fConstrain(CtrlFbck.engine_speed,800,2000);
	}
}

//void CONTROL::Control_Feedback_ahrs()
//{
//	xQueuePeek(queueGyrDat, &gyro, 0);					//从队列中获取陀螺仪数据
//	xQueuePeek(queueGyrDatFil, &gyro_filt,0);
//	xQueuePeek(queueAhrsEuler, &ahrsEuler, 0);					//从队列中获取角度、旋转矩阵数据
//	xQueuePeek(queueEKF,&cplfil,0);
//
//	CtrlFbck.AirSpeed_Last   = CtrlFbck.AirSpeed;
//
//
////位置
//	CtrlFbck.X[0] = cplfil.Ned[0];
//	CtrlFbck.Y[0] = cplfil.Ned[1];
//	CtrlFbck.Z[0] = cplfil.Ned[2];
////速度
//	CtrlFbck.X[1] = cplfil.Ned_spd[0];
//	CtrlFbck.Y[1] = cplfil.Ned_spd[1];
//	CtrlFbck.Z[1] = cplfil.Ned_spd[2];
////机头
//	float sinY, cosY;
//	sinY = sinf(ahrsEuler.Ang[2]);
//	cosY = cosf(ahrsEuler.Ang[2]);
//	CtrlFbck.XH[1] =  CtrlFbck.X[1]*cosY + CtrlFbck.Y[1]*sinY;
//	CtrlFbck.YH[1] = -CtrlFbck.X[1]*sinY + CtrlFbck.Y[1]*cosY;
////机体速度
//	CtrlFbck.U[0] = CtrlFbck.X[1]*ahrsEuler.rotation[0][0] + CtrlFbck.Y[1]*ahrsEuler.rotation[1][0] + CtrlFbck.Z[1]*ahrsEuler.rotation[2][0];
//	CtrlFbck.V[0] = CtrlFbck.X[1]*ahrsEuler.rotation[0][1] + CtrlFbck.Y[1]*ahrsEuler.rotation[1][1] + CtrlFbck.Z[1]*ahrsEuler.rotation[2][1];
//	CtrlFbck.W[0] = CtrlFbck.X[1]*ahrsEuler.rotation[0][2] + CtrlFbck.Y[1]*ahrsEuler.rotation[1][2] + CtrlFbck.Z[1]*ahrsEuler.rotation[2][2];
//
//
//	for (u8 i=0;i<3;i++)
//	{
////		CtrlFbck.pqr[i]=gyro.gyro[i];
//		CtrlFbck.pqr[i]=gyro_filt.gyro_filter[i];
//		CtrlFbck.Ang[i]=ahrsEuler.Ang[i];
//	}
//	//----------北东地位置----------------------
//
//	CtrlFbck.AirSpeed = sqrt(CtrlFbck.X[1]*CtrlFbck.X[1]+CtrlFbck.Y[1]*CtrlFbck.Y[1]+CtrlFbck.Z[1]*CtrlFbck.Z[1]);
//
//	if (CtrlIO.control_mode==2 || CtrlIO.control_mode==3)
//	{
//		CtrlFbck.engine_speed=fConstrain(CtrlFbck.engine_speed,800,2000);
//	}
//}


void CONTROL::ControlRC_Check()
{

	xQueuePeek(queueRC_Status,&rc_status_msg,0);					//遥控器状态

	CtrlIO.rc_status = rc_status_msg.rc_status;
	CtrlIO.ever_rc = rc_status_msg.ever_rc;

}

void CONTROL::Integral_Reset()
{

	for (int i=0;i<CTRLPID_NUM;i++)
	{
		CtrlKiTemp.pid_Ki_temp[i]=pid[i].Ki;//暂存积分系数
	}
	/********************起降阶段内环暂停积分***************/
	if (CtrlKiTemp.int_reset==true)
	{
		for (int i=0;i<CTRLPID_NUM;i++)
		{
			pid[i].Ki=0.0f;
			pid[i].integral=0.0f;
		}
		//----------------------------------------//
		CtrlINDI.output_0[0]        = 0.0f;
		CtrlINDI.output_0[1]        = 0.0f;
		CtrlINDI.output_0[2]        = 0.0f;
		//---------------------------------------//
		if (CtrlIO.output[3]>(0.8f*OneG))//转速高于悬停转速-100，重置并开始积分
		{
			CtrlKiTemp.int_reset=false;
		}
	}
	else if (CtrlKiTemp.int_reset==false)
	{

		for (int i=0;i<CTRLPID_NUM;i++)
		{
			pid[i].Ki = CtrlKiTemp.pid_Ki_temp[i];
		}
		if (CtrlIO.output[3]<(0.1f*OneG))//转速低于100，停止积分
		{
			CtrlKiTemp.int_reset=true;
		}
		switch(CtrlIO.control_mode)
		{
			case 0://手控，姿态环积分系数置零
			{
				for (u8 i=3;i<12;i++)
				{
					pid[i].Ki=0.0f;
					pid[i].integral=0.0f;
				}
				CtrlLpIO.u1_Tilt[0]=0.0f;
			}
			break;

			case 1://半自控
			{
				for (u8 i=6;i<12;i++)
				{
					pid[i].Ki=0.0f;
					pid[i].integral=0.0f;
				}
				CtrlLpIO.u1_Tilt[0]=0.0f;
			}
			break;
			case 2://自控
			break;
			case 3://自控
			break;
			default:break;
		}
	}
}

void CONTROL::Integral_Restore()
{
	for (int i=0;i<CTRLPID_NUM;i++)
	{
	  pid[i].Ki=CtrlKiTemp.pid_Ki_temp[i];
	}
}
void CONTROL::StatusClear()
{
	CtrlLpIO.iniLanding =false;
	CtrlLpIO.circle_start=false;
	CtrlLpIO.trajectory_start  = false;
	Ctrltrack.flypoint_start=false;
	Ctrltrack.flycircle_start=false;
	Ctrltrack.dubins_start=false;
	rcCommand.OneKeyTakeoff = false;
	rcCommand.OneKeyLanding = false;
	CtrlLpIO.Ang_command[2] = CtrlFbck.Ang[2];//锁航向
	CtrlLpIO.Acc_command[0] = 0.0f;
	CtrlLpIO.Acc_command[1] = 0.0f;
	CtrlLpIO.Acc_command[2] = 0.0f;
	CtrlLpIO.AngVel_command[0] =0.0f;
	CtrlLpIO.AngVel_command[1] =0.0f;
	CtrlLpIO.AngVel_command[2] =0.0f;
}
void CONTROL::virtualFence(){
	float zLimit=10;//高度限制
	if(CtrlFbck.Z[0] < CtrlLpIO.Z_pos0-zLimit){
		  CtrlLpIO.Ang_command[0] = 0.0f;      //滚转角度输入0
		  CtrlLpIO.Ang_command[1] = 0.0f;      //俯仰角度输入0
		  CtrlLpIO.AngVel_command[2] =0.0f;
		  isexceedLimit=true;
		  In_Loop_Step2();
	}
	else isexceedLimit=false;
}

void CONTROL::add_differentiation(){
	pidXRate->Kd = 0.2f;
	pidYRate->Kd = 0.2f;
	pidX->Kd = 0.05f;
	pidY->Kd = 0.05f;
}

void CONTROL::remove_differentiation(){
	pidXRate->Kd = 0.0f;
	pidYRate->Kd = 0.0f;
	pidX->Kd = 0.0f;
	pidY->Kd = 0.0f;
}

void CONTROL::Out_Loop_XY_Pre2()
{
	double X_err,Y_err,XY_err;
	bool   X_command_flag,Y_command_flag;
	static bool X_Vel_flag=true,Y_Vel_flag=true;
	float  Vel_tol=0.4;
	float sinY,cosY;
	sinY = sinf(CtrlFbck.Ang[2]);
	cosY = cosf(CtrlFbck.Ang[2]);
	XY_err = sqrt(SQR(CtrlFbck.XH[1])+SQR(CtrlFbck.YH[1]));

	//----------------------检测打杆与否--------------------------------------
	if (CtrlIO.input[1]<50 && CtrlIO.input[1]>-50) X_command_flag = true;
	else  	                                       X_command_flag = false;
	if (CtrlIO.input[0]<50 && CtrlIO.input[0]>-50) Y_command_flag = true;
	else                                           Y_command_flag = false;

	//-----------------------模式切换-----------------------------------------
	if ((CtrlIO.last_control_mode!=2) || (CtrlLpIO.XY_phase==2 && CtrlLpIO.brake_mode==false))
	{
		CtrlLpIO.XY_phase = 0;
		StatusClear();
		CtrlLpIO.Pos_command[0] = CtrlFbck.X[0];//当前位置作为位置给定
		CtrlLpIO.Pos_command[1] = CtrlFbck.Y[0];
		pid[9].integral         = 0.0;  //X位置积分
		pid[10].integral        = 0.0; //Y位置积分
		X_Vel_flag = true;
		Y_Vel_flag = true;
	}
	if (X_command_flag==false || Y_command_flag==false)
	{
		CtrlLpIO.XY_phase = 1;
	}
	if (CtrlLpIO.XY_phase==1 && X_command_flag==true && Y_command_flag==true)
	{
		CtrlLpIO.XY_phase = 2;
		CtrlLpIO.brake_cnt.CNT = 0;
		CtrlLpIO.brake_mode    = true;
		CtrlLpIO.Vel_command[0]=CtrlFbck.XH[1];
		CtrlLpIO.Vel_command[1]=CtrlFbck.YH[1];
		pidXRate->integral       = 0.0;
		pidYRate->integral       = 0.0;
	}

	//-----------------------模式功能-----------------------------------------
	switch(CtrlLpIO.XY_phase)
	{
	case 0://定点模式
		X_err = CtrlLpIO.Pos_command[0] - CtrlFbck.X[0];
		Y_err = CtrlLpIO.Pos_command[1] - CtrlFbck.Y[0];
		CtrlLpIO.Pos_err[0] = (X_err* cosY + Y_err*sinY);
		CtrlLpIO.Pos_err[1] = (X_err*-sinY + Y_err*cosY);
		CtrlLpIO.Vel_command[0] = pidX->PID_Controller(CtrlLpIO.Pos_err[0],CtrlDt);
		CtrlLpIO.Vel_command[1] = pidY->PID_Controller(CtrlLpIO.Pos_err[1],CtrlDt);
		CtrlLpIO.Vel_command[0] = fConstrain(CtrlLpIO.Vel_command[0],-1,1);
		CtrlLpIO.Vel_command[1] = fConstrain(CtrlLpIO.Vel_command[1],-1,1);

        break;
	case 1:
		CtrlLpIO.Vel_command[0] = -CtrlIO.input[1]*0.0125f;
		CtrlLpIO.Vel_command[1] =  CtrlIO.input[0]*0.0125f;
		break;
	case 2://刹车模式
		CtrlLpIO.brake_mode     = TarHit(&CtrlLpIO.brake_cnt,Vel_tol,XY_err);
		if(X_Vel_flag==true){
			CtrlLpIO.Acc_command[0] = -Sign(CtrlFbck.XH[1])*2.0f;
			CtrlLpIO.Vel_command[0] = CtrlLpIO.Vel_command[0] + CtrlLpIO.Acc_command[0]*CtrlDt;
		}
		if(Y_Vel_flag==true){
			CtrlLpIO.Acc_command[1] = -Sign(CtrlFbck.YH[1])*2.0f;
			CtrlLpIO.Vel_command[1] = CtrlLpIO.Vel_command[1] + CtrlLpIO.Acc_command[1]*CtrlDt;
		}
		if(absf(CtrlFbck.XH[1])<Vel_tol && X_Vel_flag==true)
		{
			CtrlLpIO.Acc_command[0] = 0.0f;
			CtrlLpIO.Vel_command[0] = 0.0f;
			X_Vel_flag=false;
		}
		if(absf(CtrlFbck.YH[1])<Vel_tol && Y_Vel_flag==true)
		{
			CtrlLpIO.Acc_command[1] = 0.0f;
			CtrlLpIO.Vel_command[1] = 0.0f;
			Y_Vel_flag=false;
		}
		break;
	default:break;
	}
//	CtrlLpIO.Acc_command[0] = 0.0f;
//	CtrlLpIO.Acc_command[1] = 0.0f;
}


bool CONTROL::frog_jump(void)
{
	xQueuePeek(queueRCCommand, &rcCommand, 0);					//从队列中获取遥控器数据
	double X_err,Y_err,Z_err;
	static float ini_yaw_angle = 0.0;
	float  radius = 5.0;
	float sinY, cosY;
	static Vector2f end(0,0),start(0,0);
	Vector2f cur(CtrlFbck.X[0],CtrlFbck.Y[0]);

	static u16 count = 0, num = 0;
	static bool state_transition = false;
	sinY = sinf(CtrlFbck.Ang[2]);
	cosY = cosf(CtrlFbck.Ang[2]);
	float sin_azimuth = sinf(ini_yaw_angle);
	float cos_azimuth = cosf(ini_yaw_angle);
	CtrlFbck.XH[0] =  CtrlFbck.X[0]*cos_azimuth + CtrlFbck.Y[0]*sin_azimuth;
	CtrlFbck.YH[0] = -CtrlFbck.X[0]*sin_azimuth + CtrlFbck.Y[0]*cos_azimuth;
		//-----------------------模式切换-----------------------------------------
	if(rcCommand.OneKeyTakeoff == true ||(num==0 && CtrlIO.FlightStatus != AIR))  OneKeyTakeOff();
	if(rcCommand.OneKeyLanding == true)  OneKeyLanding(end(0),end(1),true,true);
	if(CtrlIO.FlightStatus == GE && num!=2 && rcCommand.OneKeyTakeoff != true){
		if(++count>200){
			CtrlIO.FlightStatus = IDLING;
			count=0;
		}
	}
	if(CtrlIO.FlightStatus == IDLING && num!=0){
		if(++count>300){
			count=0;
			rcCommand.OneKeyTakeoff = true;
			state_transition=false;
		}
	}
	if(CtrlIO.FlightStatus == AIR) {
		if(num == 0)ini_yaw_angle = CtrlLpIO.Ang_command[2];
		if(state_transition == false)
		{
			StatusClear();
			num++;
			CtrlLpIO.XYZ_phase = 1;//转头
			state_transition = true;
		}
			X_err = CtrlLpIO.Pos_command[0] - CtrlFbck.X[0];
			Y_err = CtrlLpIO.Pos_command[1] - CtrlFbck.Y[0];
			Z_err = CtrlLpIO.Pos_command[2] - CtrlFbck.Z[0];
			CtrlLpIO.Pos_err[0] = (X_err* cosY + Y_err*sinY);
			CtrlLpIO.Pos_err[1] = (X_err*-sinY + Y_err*cosY);
			CtrlLpIO.Pos_err[2] = Z_err;
			CtrlLpIO.Vel_command[0] = pidX->PID_Controller(CtrlLpIO.Pos_err[0],CtrlDt);
			CtrlLpIO.Vel_command[1] = pidY->PID_Controller(CtrlLpIO.Pos_err[1],CtrlDt);
			CtrlLpIO.Vel_command[2] = pidZ->PID_Controller(CtrlLpIO.Pos_err[2],CtrlDt);
			CtrlLpIO.Vel_command[2] = fConstrain(CtrlLpIO.Vel_command[2],-1,0.5);

		//-----------------------模式功能-----------------------------------------
		switch(CtrlLpIO.XYZ_phase)
		{
		case 0://hover 准备降落
			CtrlLpIO.Vel_command[0] = fConstrain(CtrlLpIO.Vel_command[0],-1,1);
			CtrlLpIO.Vel_command[1] = fConstrain(CtrlLpIO.Vel_command[1],-1,1);
			if(count < (2 / CtrlDt)) {
                count++;
            }
			else {
				state_transition = true;
				rcCommand.OneKeyLanding = true;
				count=0;
			}
			break;
		case 1://转头
			CtrlLpIO.Vel_command[0] = fConstrain(CtrlLpIO.Vel_command[0],-1,1);
			CtrlLpIO.Vel_command[1] = fConstrain(CtrlLpIO.Vel_command[1],-1,1);
			Ctrltrack.azimuth = atan2f((CtrlLpIO.Pos_command[1]+radius*sinf(ini_yaw_angle)-CtrlFbck.Y[0]),(CtrlLpIO.Pos_command[0]+radius*cosf(ini_yaw_angle)-CtrlFbck.X[0]));
			if(num==2)Ctrltrack.azimuth = atan2f((CtrlLpIO.Y_pos0-CtrlFbck.Y[0]),(CtrlLpIO.X_pos0-CtrlFbck.X[0]));
			if (Turn_Heading(ini_yaw_angle)){
				CtrlLpIO.XYZ_phase = 2;//直线
				start << CtrlFbck.X[0],CtrlFbck.Y[0];
				TempBodyFrame << CtrlFbck.XH[0],CtrlFbck.YH[0];
				CtrlLpIO.Pos_command[0] = CtrlLpIO.Pos_command[0]+radius*cosY; //设定目标航点
				CtrlLpIO.Pos_command[1] = CtrlLpIO.Pos_command[1]+radius*sinY;
				if(num==2){
					CtrlLpIO.Pos_command[0] = CtrlLpIO.X_pos0; //设定目标航点
					CtrlLpIO.Pos_command[1] = CtrlLpIO.Y_pos0;
				}
				end << CtrlLpIO.Pos_command[0],CtrlLpIO.Pos_command[1];
			}
			break;
		case 2://直线
			Ctrltrack.distance_XY = (end-cur).norm();
			if(Ctrltrack.distance_XY>=0.3f && Ctrltrack.brake_finish==false){
				Ctrltrack.brake_finish = followStraightPath(&start,&end,1.0,false);
			}
			if(Ctrltrack.distance_XY<0.3f || Ctrltrack.brake_finish==true){
				Ctrltrack.brake_finish = true;
				CtrlLpIO.Acc_command[0] = 0;
				count++;
				if(count>200){
					count = 0;
					isComplete = true;
					Ctrltrack.brake_finish=false;
					CtrlLpIO.XYZ_phase = 0;
				}
			}
			CtrlLpIO.Vel_command[0] = fConstrain(CtrlLpIO.Vel_command[0],-5,5);//
			CtrlLpIO.Vel_command[1] = fConstrain(CtrlLpIO.Vel_command[1],-1,1);
			break;
		default:break;
		}
	}
	xQueueOverwrite(queueRCCommand,&rcCommand);
	xQueueOverwrite(queuetrajectoryData,&Ctrltrack);
	return true;
}

void CONTROL::OneKeyTakeOff(void)
{
	xQueuePeek(queueRCCommand, &rcCommand, 0);					//从队列中获取遥控器数据
	double X_err,Y_err,Z_err;
	float sinY,cosY;
	sinY = sinf(CtrlFbck.Ang[2]);
	cosY = cosf(CtrlFbck.Ang[2]);

//	if(!slam.Ready_Take_off || !claw.isOpen) return;

	if(CtrlIO.FlightStatus == IDLING)
	{
		static bool ini=false;
		if( !ini ) {
			CtrlLpIO.X_pos0 = CtrlFbck.X[0];//记录xy当前位置
			CtrlLpIO.Y_pos0 = CtrlFbck.Y[0];
			CtrlLpIO.Z_pos0 = CtrlFbck.Z[0];

			CtrlLpIO.Pos_command[0] = CtrlLpIO.X_pos0;
			CtrlLpIO.Pos_command[1] = CtrlLpIO.Y_pos0;
			CtrlLpIO.Pos_command[2] = CtrlLpIO.Z_pos0 - 0.8f;//当前高度+3m作为高度给定
			CtrlLpIO.Ang_command[2] = CtrlFbck.Ang[2];//锁航向

			ini = true;
		}
		CtrlIO.FlightStatus = LAUNCH;
	}
	if(CtrlIO.FlightStatus == LAUNCH )
	{
		static int num = 0;
		num++;
		if( num > 50 ){
			num = 0;
			CtrlIO.FlightStatus = TAKEOFF;
		}
	}
	X_err = CtrlLpIO.Pos_command[0] - CtrlFbck.X[0];
	Y_err = CtrlLpIO.Pos_command[1] - CtrlFbck.Y[0];
	Z_err = CtrlLpIO.Pos_command[2] - CtrlFbck.Z[0];
	CtrlLpIO.Pos_err[0] = (X_err* cosY + Y_err*sinY);
	CtrlLpIO.Pos_err[1] = (X_err*-sinY + Y_err*cosY);
	CtrlLpIO.Pos_err[2] = Z_err;

	CtrlLpIO.Vel_command[0] = pidX->PID_Controller(CtrlLpIO.Pos_err[0],CtrlDt);
	CtrlLpIO.Vel_command[1] = pidY->PID_Controller(CtrlLpIO.Pos_err[1],CtrlDt);
	CtrlLpIO.Vel_command[2] = pidZ->PID_Controller(CtrlLpIO.Pos_err[2],CtrlDt);

	CtrlLpIO.Vel_command[0] = fConstrain(CtrlLpIO.Vel_command[0],-1,1);
	CtrlLpIO.Vel_command[1] = fConstrain(CtrlLpIO.Vel_command[1],-1,1);
	CtrlLpIO.Vel_command[2] = fConstrain(CtrlLpIO.Vel_command[2],-1,0.5);

	Ctrltrack.POS_err = absf(CtrlLpIO.Pos_err[2]);
	if ( Ctrltrack.POS_err <= CtrlLpIO.POS_thr && CtrlIO.FlightStatus == TAKEOFF )
	{
		CtrlIO.FlightStatus = AIR;
		rcCommand.IsAir = 1;
		rcCommand.OneKeyTakeoff = false;
		rcCommand.TakeOffFinish = true;
	}
	CtrlLpIO.Acc_command[0] = 0.0f;
	CtrlLpIO.Acc_command[1] = 0.0f;
	CtrlLpIO.Acc_command[2] = 0.0f;
	xQueueOverwrite(queueRCCommand,&rcCommand);
	xQueueOverwrite(queuetrajectoryData,&Ctrltrack);

}

void CONTROL::Auto_flypoint()
{
	xQueuePeek(queueRCCommand, &rcCommand, 0);					//从队列中获取遥控器数据
	double X_err,Y_err,Z_err,XY_err,distance_Z;
	static float distance=0.0,temp_height=0.0;
	static Vector2f end,start; //终点 起点
	Vector2f cur(CtrlFbck.X[0],CtrlFbck.Y[0]); //当前位置
	static u16 count=0,num=0;
	float sinY = sinf(CtrlFbck.Ang[2]);
	float cosY = cosf(CtrlFbck.Ang[2]);
	XY_err = sqrt(SQR(CtrlFbck.XH[1])+SQR(CtrlFbck.YH[1]));
	float sin_azimuth = sinf(Ctrltrack.azimuth);
	float cos_azimuth = cosf(Ctrltrack.azimuth);
	CtrlFbck.XH[0] =  CtrlFbck.X[0]*cos_azimuth + CtrlFbck.Y[0]*sin_azimuth; //临时坐标系下的X位置
	CtrlFbck.YH[0] = -CtrlFbck.X[0]*sin_azimuth + CtrlFbck.Y[0]*cos_azimuth; //临时坐标系下的Y位置
		//-----------------------模式切换-----------------------------------------
	if(Ctrltrack.flypoint_start==false)
	{
		StatusClear(); //初始各状态清零
		num = 0; //航点序号
		Ctrltrack.PTP_Status = PTP_hover; //悬停
		CtrlLpIO.Pos_command[0] = CtrlLpIO.X_pos0;//CtrlFbck.X[0];
		CtrlLpIO.Pos_command[1] = CtrlLpIO.Y_pos0;//CtrlFbck.Y[0];
		CtrlLpIO.Pos_command[2] = CtrlLpIO.Z_pos0-1.5f;//CtrlFbck.Z[0];
		CtrlLpIO.Ang_command[2] = CtrlFbck.Ang[2];
		Ctrltrack.flypoint_start=true;
	}
		X_err = CtrlLpIO.Pos_command[0] - CtrlFbck.X[0];
		Y_err = CtrlLpIO.Pos_command[1] - CtrlFbck.Y[0];
		Z_err = CtrlLpIO.Pos_command[2] - CtrlFbck.Z[0];
		CtrlLpIO.Pos_err[0] = (X_err* cosY + Y_err*sinY);
		CtrlLpIO.Pos_err[1] = (X_err*-sinY + Y_err*cosY);
		CtrlLpIO.Pos_err[2] = Z_err;
		CtrlLpIO.Vel_command[0] = pidX->PID_Controller(CtrlLpIO.Pos_err[0],CtrlDt);
		CtrlLpIO.Vel_command[1] = pidY->PID_Controller(CtrlLpIO.Pos_err[1],CtrlDt);
		CtrlLpIO.Vel_command[2] = pidZ->PID_Controller(CtrlLpIO.Pos_err[2],CtrlDt);
		CtrlLpIO.Vel_command[2] = fConstrain(CtrlLpIO.Vel_command[2],-1,0.5);

	//-----------------------模式功能-----------------------------------------
	switch(Ctrltrack.PTP_Status)
	{
	case PTP_hover://定点
		add_differentiation();
		CtrlLpIO.Vel_command[0] = fConstrain(CtrlLpIO.Vel_command[0],-1,1);
		CtrlLpIO.Vel_command[1] = fConstrain(CtrlLpIO.Vel_command[1],-1,1);
		if(point_list[num].enable ==1 && Ctrltrack.flypoint_start == true){
			Ctrltrack.PTP_Status = PTP_TurnHead; //转头，指向目标点
			point_list[num].GetYaw(&fly_yaw);//获得航点偏航信息
			point_list[num].GetPosition(&position);//获得航点位置信息，注意Z给的是相对高度！
			point_list[num].GetSpeed(&vel);//获得航点速度信息
			point_list[num].GetStayTime(&staytime);//获得停留时间信息
		}
        break;
	case PTP_TurnHead://转头
		CtrlLpIO.Vel_command[0] = fConstrain(CtrlLpIO.Vel_command[0],-1,1);
		CtrlLpIO.Vel_command[1] = fConstrain(CtrlLpIO.Vel_command[1],-1,1);
		Ctrltrack.azimuth = atan2f((position.data.y-CtrlFbck.Y[0]),(position.data.x-CtrlFbck.X[0])); //方位角
//		if(int(fly_yaw) == 0)Turn_Heading(azimuth);
//		else Turn_Heading(fly_yaw*D2R);
		if (Turn_Heading(Ctrltrack.azimuth)){
				Ctrltrack.PTP_Status = PTP_XY_Fly;
				remove_differentiation();
				start << CtrlFbck.X[0],CtrlFbck.Y[0];//以当前点为起点
				TempBodyFrame << CtrlFbck.XH[0],CtrlFbck.YH[0]; //临时坐标系原点
				temp_height = CtrlFbck.Z[0]; //当前高度为临时高度
				CtrlLpIO.Pos_command[0]=position.data.x; //目标X位置
				CtrlLpIO.Pos_command[1]=position.data.y; //目标Y位置
				end << position.data.x,position.data.y; //目标点
				isComplete = true; //初始时路径完成
		}
		break;
	case PTP_XY_Fly://
		Ctrltrack.distance_XY = (end-cur).norm();
		if(Ctrltrack.distance_XY>=0.4f && Ctrltrack.brake_finish==false){ //当前位置与终点位置小于0.4或者刹车完成就进入下一个状态
			Ctrltrack.brake_finish = followStraightPath(&start,&end,vel,false);

		}
		if(Ctrltrack.distance_XY<0.4f || Ctrltrack.brake_finish==true){
			Ctrltrack.brake_finish = true;
			CtrlLpIO.Acc_command[0] = 0;
			CtrlLpIO.Jerk_command[0]= 0;
			if(++count>200){
				count = 0;
				isComplete = true;
				Ctrltrack.brake_finish=false;
				Ctrltrack.PTP_Status = PTP_Z_Fly;
			}
		}
		CtrlLpIO.Vel_command[0] = fConstrain(CtrlLpIO.Vel_command[0],-5,5);
		CtrlLpIO.Vel_command[1] = fConstrain(CtrlLpIO.Vel_command[1],-1,1);
		break;
	case PTP_Z_Fly://
		CtrlLpIO.Pos_command[2] = temp_height-position.data.z;
//		CtrlLpIO.Pos_command[2] = fConstrain(temp_height-position.data.z,-10,-1.5);
		CtrlLpIO.Vel_command[0] = fConstrain(CtrlLpIO.Vel_command[0],-1,1);
		CtrlLpIO.Vel_command[1] = fConstrain(CtrlLpIO.Vel_command[1],-1,1);
//		distance=sqrt(SQR(position.data.x-CtrlFbck.X[0])+SQR(position.data.y-CtrlFbck.Y[0])+SQR(position.data.z-CtrlFbck.Z[0]));
		distance_Z=absf(CtrlLpIO.Pos_command[2]-CtrlFbck.Z[0]);
		if(distance_Z<0.2f){
			if(++count>200)
			{
				count = 0;
				Ctrltrack.PTP_Status = PTP_Arrive;
			}
		}
		break;
	case PTP_XYZ_Fly:// TODO 规划上升时间与xy移动时间一致
		CtrlLpIO.Pos_command[0] = position.data.x;
		CtrlLpIO.Pos_command[1] = position.data.y;
		CtrlLpIO.Pos_command[2] = temp_height-position.data.z;
//		CtrlLpIO.Pos_command[2] = fConstrain(temp_height-position.data.z,-10,-1.5);
		CtrlLpIO.Vel_command[0] = fConstrain(CtrlLpIO.Vel_command[0],-vel,vel);////////
		CtrlLpIO.Vel_command[1] = fConstrain(CtrlLpIO.Vel_command[1],-1,1);
		distance=sqrt(SQR(position.data.x-CtrlFbck.X[0])+SQR(position.data.y-CtrlFbck.Y[0])+SQR(CtrlLpIO.Pos_command[2]-CtrlFbck.Z[0]));
//		distance_Z=absf(position.data.z-CtrlFbck.Z[0]);
		if(distance<0.3f)
		{
			if(++count>200)
			{
				count = 0;
				Ctrltrack.PTP_Status = PTP_Arrive;
			}
		}
		break;
	case PTP_Arrive://到达
		CtrlLpIO.Vel_command[0] = fConstrain(CtrlLpIO.Vel_command[0],-1,1);
		CtrlLpIO.Vel_command[1] = fConstrain(CtrlLpIO.Vel_command[1],-1,1);
		if(count<(staytime/CtrlDt)) count++;
		else
		{
			Ctrltrack.PTP_Status = PTP_hover;
			count=0;
			num++;
		}
        break;
	case NonPTP:
		break;
	default:break;
	}
	xQueueOverwrite(queueRCCommand,&rcCommand);
	xQueueOverwrite(queuetrajectoryData,&Ctrltrack);
}
void CONTROL::Auto_flycircle()
{
	xQueuePeek(queueRCCommand, &rcCommand, 0);					//从队列中获取遥控器数据
	double X_err,Y_err,Z_err,XY_err,distance_Z;
	float  Vel_tol=0.4;
	static float YawAngle_last=0.0,temp_height=0.0,line_vel=0.0;
	float sinY,cosY,tangential_ang;
	static u16 count=0,num=0;
	static Vector2f center(0,0),end(0,0),start(0,0);
	Vector2f cur(CtrlFbck.X[0],CtrlFbck.Y[0]);
//	static int circleRemaining = 0,circleTraveled = 0;
	sinY = sinf(CtrlFbck.Ang[2]);
	cosY = cosf(CtrlFbck.Ang[2]);
	XY_err = sqrt(SQR(CtrlFbck.XH[1])+SQR(CtrlFbck.YH[1]));
	float sin_azimuth = sinf(Ctrltrack.azimuth);
	float cos_azimuth = cosf(Ctrltrack.azimuth);
	CtrlFbck.XH[0] =  CtrlFbck.X[0]*cos_azimuth + CtrlFbck.Y[0]*sin_azimuth;
	CtrlFbck.YH[0] = -CtrlFbck.X[0]*sin_azimuth + CtrlFbck.Y[0]*cos_azimuth;
		//-----------------------模式切换-----------------------------------------
	if (Ctrltrack.flycircle_start==false)
	{
		StatusClear();
		Ctrltrack.Circle_Status = Circle_hover;
		CtrlLpIO.Pos_command[0] = CtrlFbck.X[0];
		CtrlLpIO.Pos_command[1] = CtrlFbck.Y[0];
		CtrlLpIO.Pos_command[2] = CtrlFbck.Z[0];//
		CtrlLpIO.Ang_command[2] = CtrlFbck.Ang[2];
		Ctrltrack.flycircle_start=true;
		num = 0;
	}
		X_err = CtrlLpIO.Pos_command[0] - CtrlFbck.X[0];
		Y_err = CtrlLpIO.Pos_command[1] - CtrlFbck.Y[0];
		Z_err = CtrlLpIO.Pos_command[2] - CtrlFbck.Z[0];
		CtrlLpIO.Pos_err[0] = (X_err* cosY + Y_err*sinY);
		CtrlLpIO.Pos_err[1] = (X_err*-sinY + Y_err*cosY);
		CtrlLpIO.Pos_err[2] = Z_err;
		CtrlLpIO.Vel_command[0] = pidX->PID_Controller(CtrlLpIO.Pos_err[0],CtrlDt);
		CtrlLpIO.Vel_command[1] = pidY->PID_Controller(CtrlLpIO.Pos_err[1],CtrlDt);
		CtrlLpIO.Vel_command[2] = pidZ->PID_Controller(CtrlLpIO.Pos_err[2],CtrlDt);
		CtrlLpIO.Vel_command[2] = fConstrain(CtrlLpIO.Vel_command[2],-1,0.5);

	//-----------------------模式功能-----------------------------------------
	switch(Ctrltrack.Circle_Status)
	{
	case Circle_hover://定点
		CtrlLpIO.Vel_command[0] = fConstrain(CtrlLpIO.Vel_command[0],-1,1);
		CtrlLpIO.Vel_command[1] = fConstrain(CtrlLpIO.Vel_command[1],-1,1);
		if(point_list[num].enable ==1 && Ctrltrack.flycircle_start == true)
		{
			Ctrltrack.Circle_Status = Circle_TurnHead;
			point_list[num].GetYaw(&Ctrltrack.goalAng); //临时使用  此处作为飞过的角度（弧度制）
			circleRemaining = static_cast<int>(floor(Ctrltrack.goalAng)/(2*PI)); //将画多少个圆 floor函数获得不超过结果的最大整数
			point_list[num].GetPosition(&position); //获得圆心位置信息
			center << position.data.x,position.data.y;
			point_list[num].GetSpeed(&vel); //获得画圆的速度信息
			point_list[num].GetStayTime(&staytime);//
			temp_height = CtrlFbck.Z[0];
			fly_radius = SQR(vel)/OneG/tan(1.5*D2R); //tan6-1.028  tan5-0.856  tan4-0.684  tan3-0.512  tan2-0.341 tan1.5-0.256
//			fly_radius = sqrt(SQR(position.data.x-CtrlFbck.X[0])+SQR(position.data.y-CtrlFbck.Y[0]));
		}
        break;
	case Circle_TurnHead://转头
		CtrlLpIO.Vel_command[0] = fConstrain(CtrlLpIO.Vel_command[0],-1,1);
		CtrlLpIO.Vel_command[1] = fConstrain(CtrlLpIO.Vel_command[1],-1,1);
		Ctrltrack.azimuth = atan2f((position.data.y-CtrlFbck.Y[0]),(position.data.x-CtrlFbck.X[0]));
//		Ctrltrack.distance_XY = (center-cur).norm();
//		if(Ctrltrack.distance_XY < fly_radius) Ctrltrack.azimuth = Ctrltrack.azimuth>=0? Ctrltrack.azimuth-PI :Ctrltrack.azimuth+PI; //如果当前位置在圆内
		if(Turn_Heading(Ctrltrack.azimuth)) Ctrltrack.Circle_Status = Circle_Z_Fly;
		break;
	case Circle_Z_Fly://飞到画圆平面
		CtrlLpIO.Pos_command[2] = temp_height-position.data.z;
		CtrlLpIO.Vel_command[0] = fConstrain(CtrlLpIO.Vel_command[0],-1,1);
		CtrlLpIO.Vel_command[1] = fConstrain(CtrlLpIO.Vel_command[1],-1,1);
		distance_Z=absf(CtrlLpIO.Pos_command[2]-CtrlFbck.Z[0]);
		if(distance_Z<0.2f){
			count++;
			if(count>200){
				count = 0;
				Ctrltrack.Circle_Status = Circle_Line;
				Ctrltrack.distance_XY = (center-cur).norm();
				CtrlLpIO.Pos_command[0] = CtrlFbck.X[0]+absf(Ctrltrack.distance_XY-fly_radius)*cosY;//设定目标航点
				CtrlLpIO.Pos_command[1] = CtrlFbck.Y[0]+absf(Ctrltrack.distance_XY-fly_radius)*sinY;
				end << CtrlLpIO.Pos_command[0],CtrlLpIO.Pos_command[1];
				start = cur;
				line_vel = fConstrain(std::ceil((end-cur).norm()/10),1,5);
				isComplete = true;
				TempBodyFrame << CtrlFbck.XH[0],CtrlFbck.YH[0];
			}
		}
		break;
	case Circle_Line://直线
//		Ctrltrack.distance_XY = (center-cur).norm();
//		CtrlLpIO.Pos_command[0] = CtrlFbck.X[0]+absf(Ctrltrack.distance_XY-fly_radius)*cosY;//设定目标航点
//		CtrlLpIO.Pos_command[1] = CtrlFbck.Y[0]+absf(Ctrltrack.distance_XY-fly_radius)*sinY;
		Ctrltrack.distance_XY = (end-cur).norm();
		if(Ctrltrack.distance_XY>=0.4f && Ctrltrack.brake_finish==false){
			Ctrltrack.brake_finish = followStraightPath(&start,&end,line_vel,false);
		}
//		if(absf(Ctrltrack.distance_XY-fly_radius) < 0.2){
		if(Ctrltrack.distance_XY<0.4f || Ctrltrack.brake_finish==true){
			Ctrltrack.brake_finish = true;
			CtrlLpIO.Acc_command[0] = 0;
			CtrlLpIO.Jerk_command[0]= 0;
			if(count++>200){
				count = 0;
				isComplete = true;
				Ctrltrack.brake_finish=false;
				Ctrltrack.Circle_Status = Circle_TurnHead1;
				Ctrltrack.azimuth = atan2f((position.data.y-CtrlFbck.Y[0]),(position.data.x-CtrlFbck.X[0]));
				YawAngle_last = Ctrltrack.azimuth;
			}
		}
		CtrlLpIO.Vel_command[0] = fConstrain(CtrlLpIO.Vel_command[0],-5,5);
		CtrlLpIO.Vel_command[1] = fConstrain(CtrlLpIO.Vel_command[1],-1,1);
		break;
	case Circle_TurnHead1://转头 此时机头指向圆心,CW-应先逆时针旋转90°,CCW-应先顺时针旋转90°
//		if(YawAngle_last<-PI/2) tangential_ang = YawAngle_last+3*PI/2; //逆时针旋转90
//		else tangential_ang = YawAngle_last-PI/2;
		if(YawAngle_last>PI/2) tangential_ang = YawAngle_last-3*PI/2; //顺时针旋转90
		else tangential_ang = YawAngle_last+PI/2;
		if(Turn_Heading(tangential_ang)){
			Ctrltrack.Circle_Status = Circle_ing;
			Ctrltrack.iniAng = CtrlFbck.Ang[2];
			isComplete = true;
			Ctrltrack.YawAngle_traveled = 0.0;
			YawAngle_last = 0.0;
			circleTraveled = 0;
		}
		CtrlLpIO.Vel_command[0] = fConstrain(CtrlLpIO.Vel_command[0],-1,1);
		CtrlLpIO.Vel_command[1] = fConstrain(CtrlLpIO.Vel_command[1],-1,1);
		break;
	case Circle_ing://
		calcCircularAngle(Ctrltrack.iniAng,Ctrltrack.goalAng,-1);
		if( (Ctrltrack.YawAngle_remaining>=0.3/fly_radius) && Ctrltrack.brake_finish==false){
			Ctrltrack.brake_finish = followCirclePath(fly_radius,vel,center,Ctrltrack.goalAng,false,-1);
//			CtrlLpIO.Ang_command[2] = CtrlLpIO.Ang_command[2]+ Ctrltrack.Vel_command_ref[0]*CtrlDt/fly_radius;
//			CtrlLpIO.Acc_command[1] = SQR(CtrlFbck.XH[1])/fly_radius; //向心加速度 Y方向
		}
		if( (Ctrltrack.YawAngle_remaining<0.3/fly_radius) || Ctrltrack.brake_finish==true)
		{
			isComplete = true;
			Ctrltrack.brake_finish = false;
			Ctrltrack.Circle_Status = Circle_brake;
			CtrlLpIO.brake_cnt.CNT = 0;
			CtrlLpIO.brake_mode    = true;
			CtrlLpIO.Ang_command[2] = CtrlFbck.Ang[2];//**
			Ctrltrack.YawAngle_traveled = 0.0;
			Ctrltrack.YawAngle_remaining =0.0;
			CtrlLpIO.Acc_command[1] = 0.0;
			CtrlLpIO.Acc_command[0]=0;
			CtrlLpIO.Jerk_command[0]= 0;
			CtrlLpIO.AngVel_command[2] =0.0f;
		}
		CtrlLpIO.Vel_command[0] = fConstrain(CtrlLpIO.Vel_command[0],-5,5);
		CtrlLpIO.Vel_command[1] = fConstrain(CtrlLpIO.Vel_command[1],-1,1);
		break;
	case Circle_brake://刹车
		CtrlLpIO.brake_mode     = TarHit(&CtrlLpIO.brake_cnt,Vel_tol,XY_err);
		CtrlLpIO.Vel_command[0] = 0.0f;
		CtrlLpIO.Vel_command[1] = 0.0f;
		CtrlLpIO.Vel_command[2] = 0.0f;
		CtrlLpIO.Acc_command[0] = 0.0f;

		if(CtrlLpIO.brake_mode == false){
			Ctrltrack.Circle_Status  = Circle_Arrive;
			CtrlLpIO.Pos_command[0] = CtrlFbck.X[0];//当前位置作为位置给定
			CtrlLpIO.Pos_command[1] = CtrlFbck.Y[0];
			CtrlLpIO.Pos_command[2] = CtrlFbck.Z[0];
		}
		break;
	case Circle_Arrive://到达
		CtrlLpIO.Vel_command[0] = fConstrain(CtrlLpIO.Vel_command[0],-1,1);
		CtrlLpIO.Vel_command[1] = fConstrain(CtrlLpIO.Vel_command[1],-1,1);
		if(count<(staytime/CtrlDt)) count++;
//		if(count<100) count++;
		else
		{
			Ctrltrack.Circle_Status = Circle_hover;
			count=0;
			num++;
		}
        break;
	case NonCircle:
		break;
	default:break;
	}
	xQueueOverwrite(queueRCCommand,&rcCommand);
	xQueueOverwrite(queuetrajectoryData,&Ctrltrack);

}
void CONTROL::OneKeyLanding(float X,float Y,bool isAppoint,bool isUnderControl)
{
	xQueuePeek(queueRCCommand, &rcCommand, 0);
	double X_err,Y_err,Z_err;
	float  Vel_tol=0.4;
	static u16 count=0;
	float  GE1=GE_SPEED;
	float  GE2=GE_ACC;
	static Vector2f end,start;
	Vector2f cur(CtrlFbck.X[0],CtrlFbck.Y[0]);
	static bool readyReturn;
	readyReturn = isUnderControl;
	static float return_vel=1.0,curAlti;
	float 	XYZ_err = sqrt(SQR(CtrlFbck.XH[1])+SQR(CtrlFbck.YH[1])+SQR(CtrlFbck.Z[1]));

	float sinY = sinf(CtrlFbck.Ang[2]);
	float cosY = cosf(CtrlFbck.Ang[2]);
	float sin_azimuth = sinf(Ctrltrack.azimuth);
	float cos_azimuth = cosf(Ctrltrack.azimuth);
	CtrlFbck.XH[0] =  CtrlFbck.X[0]*cos_azimuth + CtrlFbck.Y[0]*sin_azimuth;
	CtrlFbck.YH[0] = -CtrlFbck.X[0]*sin_azimuth + CtrlFbck.Y[0]*cos_azimuth;
	if ( !CtrlLpIO.iniLanding ) {
//		CtrlLpIO.Pos_command[0] = CtrlFbck.X[0];
//		CtrlLpIO.Pos_command[1] = CtrlFbck.Y[0];
//		CtrlLpIO.Pos_command[2] = CtrlFbck.Z[0];//为了蛙跳暂时注释
//		curAlti = CtrlFbck.Z[0];
		CtrlLpIO.brake_cnt.CNT = 0;
		CtrlLpIO.brake_mode    = true;
		CtrlLpIO.iniLanding =true;
	}
	if( CtrlLpIO.brake_mode ) CtrlLpIO.brake_mode = TarHit(&CtrlLpIO.brake_cnt,Vel_tol,XYZ_err);  //准备返航前确保定点
	if( !readyReturn && !CtrlLpIO.brake_mode ) { //若失去遥控器的控制 保险起见返航前先升高3m
		CtrlLpIO.Pos_command[2]= curAlti-3.0;
		if(absf(CtrlLpIO.Pos_command[2]-CtrlFbck.Z[0])<0.2f){
			if(++count>200){
				count = 0;
				readyReturn = true;
			 }
		}
	}
	if(CtrlIO.FlightStatus == AIR && !CtrlLpIO.brake_mode && readyReturn)
	{
		Ctrltrack.PTP_Status = NonPTP;
		Ctrltrack.Circle_Status = NonCircle;
		if( !isAppoint ) // 若未指定降落地点，则在起飞位置降落
		{
			end << CtrlLpIO.X_pos0,CtrlLpIO.Y_pos0;
			Ctrltrack.azimuth = (end -cur).norm() > 0.5 ? atan2f((CtrlLpIO.Y_pos0-CtrlFbck.Y[0]),(CtrlLpIO.X_pos0-CtrlFbck.X[0])) : CtrlFbck.Ang[2];
			if( Turn_Heading(Ctrltrack.azimuth) )
			{
				CtrlIO.FlightStatus = RETURNING;
				remove_differentiation();
				CtrlLpIO.Pos_command[0] = CtrlLpIO.X_pos0;
				CtrlLpIO.Pos_command[1] = CtrlLpIO.Y_pos0;
			}
		}
		else { //指定降落地点
			end << X,Y;
			Ctrltrack.azimuth = (end -cur).norm() > 0.5 ? atan2f((Y-CtrlFbck.Y[0]),(X-CtrlFbck.X[0])) : CtrlFbck.Ang[2];
			if( Turn_Heading( Ctrltrack.azimuth ) )
			{
				CtrlIO.FlightStatus = RETURNING;
				remove_differentiation();
				CtrlLpIO.Pos_command[0] = X;
				CtrlLpIO.Pos_command[1] = Y;
			}
		}
		if(CtrlIO.FlightStatus != AIR){
			CtrlLpIO.alt_landing_cnt.CNT  = 0;
			CtrlLpIO.landing_acc_mode = true;
			CtrlLpIO.Ang_command[2] = CtrlFbck.Ang[2];//锁航向
			CtrlLpIO.Pos_command[2] = CtrlFbck.Z[0];
			start << CtrlFbck.X[0],CtrlFbck.Y[0];
			end << CtrlLpIO.Pos_command[0],CtrlLpIO.Pos_command[1];
			TempBodyFrame << CtrlFbck.XH[0],CtrlFbck.YH[0];
			Ctrltrack.brake_finish = false;
			CtrlLpIO.Acc_command[0] = 0.0f;
			CtrlLpIO.Acc_command[1] = 0.0f;
			CtrlLpIO.Acc_command[2] = 0.0f;
			return_vel = fConstrain(std::ceil((end-cur).norm()/15),1,5);
		}
	}
	X_err = CtrlLpIO.Pos_command[0] - CtrlFbck.X[0];
	Y_err = CtrlLpIO.Pos_command[1] - CtrlFbck.Y[0];
	Z_err = CtrlLpIO.Pos_command[2] - CtrlFbck.Z[0];
	CtrlLpIO.Pos_err[0] = (X_err* cosY + Y_err*sinY);
	CtrlLpIO.Pos_err[1] = (X_err*-sinY + Y_err*cosY);
	CtrlLpIO.Pos_err[2] = Z_err;
	CtrlLpIO.Vel_command[0] = pidX->PID_Controller(CtrlLpIO.Pos_err[0],CtrlDt);
	CtrlLpIO.Vel_command[1] = pidY->PID_Controller(CtrlLpIO.Pos_err[1],CtrlDt);
	CtrlLpIO.Vel_command[2] = pidZ->PID_Controller(CtrlLpIO.Pos_err[2],CtrlDt);
	if( CtrlIO.FlightStatus != AIR ) {
		if(CtrlIO.FlightStatus == RETURNING){
			Ctrltrack.distance_XY = (end-cur).norm();
			if(Ctrltrack.distance_XY>=0.3f && Ctrltrack.brake_finish==false){
				Ctrltrack.brake_finish = followStraightPath(&start,&end,return_vel,false);
			}
			if (Ctrltrack.distance_XY<0.3f || Ctrltrack.brake_finish==true){
				count++;
				Ctrltrack.brake_finish = true;
				if(count>100){
					isComplete = true;
					Ctrltrack.brake_finish=false;
					CtrlIO.FlightStatus = LANDING;
					count=0;
				}
			}
		}
		if(CtrlIO.FlightStatus == LANDING){
//			CtrlLpIO.Vel_command[2] = 0.5;
//			CtrlLpIO.Vel_command[2] = fConstrain(CtrlLpIO.Vel_command[2],-1,0.5);
//			CtrlLpIO.landing_acc_mode = TarHit(&CtrlLpIO.alt_landing_cnt,Vel_tol,CtrlLpIO.Vel_err[2]);
//			if(CtrlLpIO.landing_acc_mode == false){
//				if(CtrlFbck.Z[1]<=GE1  && (acc_fil.acc_filter[2]+OneG)<=GE2){
//					CtrlIO.FlightStatus = GE;
//					rcCommand.OneKeyLanding = false;
//				}
//			}

//			if(Turn_Heading(CtrlLpIO.end_yaw)){
				CtrlLpIO.Pos_command[2] = CtrlLpIO.Z_pos0-0.15f;
				Z_err = CtrlLpIO.Pos_command[2] - CtrlFbck.Z[0];
				CtrlLpIO.Pos_err[2] = Z_err;
				CtrlLpIO.Vel_command[2] = pidZ->PID_Controller(CtrlLpIO.Pos_err[2],CtrlDt);
//			}

//			if(rcCommand.ReLanding){
//				CtrlLpIO.Vel_command[2] = 0.3;
//				CtrlLpIO.Vel_command[2] = fConstrain(CtrlLpIO.Vel_command[2],-1,0.5);
//				CtrlLpIO.landing_acc_mode = TarHit(&CtrlLpIO.alt_landing_cnt,Vel_tol,CtrlLpIO.Vel_err[2]);
//				if(CtrlLpIO.landing_acc_mode == false){
//					if(CtrlFbck.Z[1]<=GE1  && (acc_fil.acc_filter[2]+OneG)<=GE2){
//						CtrlIO.FlightStatus = GE;
//						rcCommand.OneKeyLanding = false;
//						rcCommand.IsAir = 0;
//						rcCommand.ReLanding = false;
//					}
//				}
//			}
			//NED系下
			static int cnt = 0;
			float ax = acc_fil.acc_filter[0];
			float ay = acc_fil.acc_filter[1];
			float AX = cos(CtrlFbck.Ang[1])*ax + sin(CtrlFbck.Ang[0])*sin(CtrlFbck.Ang[1])*ay;//机体系到地球系的旋转矩阵只取前两列
			float AY = cos(CtrlFbck.Ang[0])*ay;
			CtrlLpIO.Pos_estimate[0] = CtrlFbck.X[0] + CtrlFbck.X[1] + 0.5*AX;//一秒后位置预测
			CtrlLpIO.Pos_estimate[1] = CtrlFbck.Y[0] + CtrlFbck.Y[1] + 0.5*AY;
			CtrlLpIO.X_err_estimate = abs(CtrlLpIO.Pos_command[0] - CtrlLpIO.Pos_estimate[0]);
			CtrlLpIO.Y_err_estimate = abs(CtrlLpIO.Pos_command[1] - CtrlLpIO.Pos_estimate[1]);
			CtrlLpIO.XY_err_estimate = sqrt(SQR(CtrlLpIO.X_err_estimate)+SQR(CtrlLpIO.Y_err_estimate));
			if(abs(CtrlLpIO.Pos_command[2] - CtrlFbck.Z[0])<0.02){
				if(CtrlLpIO.X_err_estimate<0.02 && CtrlLpIO.Y_err_estimate<0.02 && CtrlLpIO.XY_err_estimate<0.05){
					CtrlLpIO.enable_Grab_flag = true;
					if(++cnt>10){
						claw.Close_Request_Tran();
					}
					else{
						cnt = 0;
						CtrlLpIO.enable_Grab_flag = false;
					}
				}
			}
		}
	}
	CtrlLpIO.Vel_command[0] = fConstrain(CtrlLpIO.Vel_command[0],-5,5);
	CtrlLpIO.Vel_command[1] = fConstrain(CtrlLpIO.Vel_command[1],-1,1);
	CtrlLpIO.Vel_command[2] = fConstrain(CtrlLpIO.Vel_command[2],-1,0.5);

	xQueueOverwrite(queueRCCommand,&rcCommand);
	xQueueOverwrite(queuetrajectoryData,&Ctrltrack);
}

bool CONTROL::followStraightPath(Vector2f *line_start,Vector2f *line_end,float vel,bool isContinuation)
{
	float rho_orbit = SQR(Dubins_Vel)/OneG/tan(DubinsTiltAng); // Dubins转弯半径
//	float t0 = isContinuation?vel:2*vel/3.0;
	float t0 = vel;

	float t1 = t0/4.0,t2 = 3.0*t0/4.0;
	float a0 = 4.0*vel/(3.0*t0);
	float v1 = a0*t1/2.0,v2 = a0*(2*t2-t1)/2.0;
	Vector2f end=*line_end,start=*line_start;
	Vector2f cur(CtrlFbck.X[0],CtrlFbck.Y[0]);
	float distance=(end - start).norm(); //起点到终点的距离

	float traveled_X = CtrlFbck.XH[0]-TempBodyFrame(0);
	float remaining_X = distance - traveled_X;
	if(isComplete){
		isComplete      = false;
		CtrlLpIO.Acc_command[0]    = 0;
		CtrlLpIO.Jerk_command[0] = 0;
		Ctrltrack.Vel_command_ref[0]= 0;
		Ctrltrack.Pos_command_ref[0]= 0;
		Ctrltrack.isUniform = false;
		Ctrltrack.isBrake = false;
		Ctrltrack.accelerationAngle =0;
		Ctrltrack.accelerationDistance =0;
	}
	if(Ctrltrack.Vel_command_ref[0]<vel && !Ctrltrack.isBrake && !Ctrltrack.isUniform) //从起点加速过程
	{
		if(Ctrltrack.Vel_command_ref[0]<v1){
		CtrlLpIO.Jerk_command[0]=a0/t1;
		CtrlLpIO.Acc_command[0]+=a0/t1*CtrlDt;
		}
		else if(Ctrltrack.Vel_command_ref[0]>=v1 && Ctrltrack.Vel_command_ref[0]<v2)
		{
			CtrlLpIO.Acc_command[0] = a0;
			CtrlLpIO.Jerk_command[0]= 0;
		}
		else {
			CtrlLpIO.Acc_command[0] -= a0/t1*CtrlDt;
			CtrlLpIO.Jerk_command[0]=-a0/t1;
		}
		CtrlLpIO.Acc_command[0] = fConstrain(CtrlLpIO.Acc_command[0],0.01,a0);
		Ctrltrack.Vel_command_ref[0]+=CtrlLpIO.Acc_command[0]*CtrlDt;
		Ctrltrack.Pos_command_ref[0]+=Ctrltrack.Vel_command_ref[0]*CtrlDt;
//		Acc_command_max = CtrlLpIO.Acc_command[0]; ////实际最大加速度
		Ctrltrack.accelerationDistance = traveled_X + Ctrltrack.accelerationAngle*rho_orbit; //实际加速距离
	}
	if(Ctrltrack.Vel_command_ref[0]>=vel && (remaining_X>Ctrltrack.accelerationDistance||isContinuation)) //匀速过程  vel
	{
		Ctrltrack.isUniform = true; //匀速标志位
	}
	if(remaining_X<=Ctrltrack.accelerationDistance && !isContinuation) //快到目标点时的减速过程
	{
		Ctrltrack.isBrake = true; //刹车标志位
		Ctrltrack.isUniform = false;
	}
	if(Ctrltrack.isUniform){
		CtrlLpIO.Acc_command[0]=0;
		Ctrltrack.Vel_command_ref[0]=vel;
		Ctrltrack.Pos_command_ref[0]+=Ctrltrack.Vel_command_ref[0]*CtrlDt;
	}
	if(Ctrltrack.isBrake){
		if(Ctrltrack.Vel_command_ref[0]>v2){
		CtrlLpIO.Acc_command[0]-=a0/t1*CtrlDt;
		CtrlLpIO.Jerk_command[0]=-a0/t1;
		}
		else if(Ctrltrack.Vel_command_ref[0]>v1 && Ctrltrack.Vel_command_ref[0]<=v2)
		{
			CtrlLpIO.Acc_command[0] = -a0;
			CtrlLpIO.Jerk_command[0]= 0;
		}
		else {
			CtrlLpIO.Acc_command[0] += a0/t1*CtrlDt;
			CtrlLpIO.Jerk_command[0]=a0/t1;
		}
		CtrlLpIO.Acc_command[0] = fConstrain(CtrlLpIO.Acc_command[0],-a0,-0.01);
		Ctrltrack.Vel_command_ref[0]+=CtrlLpIO.Acc_command[0]*CtrlDt;
		Ctrltrack.Pos_command_ref[0]+=Ctrltrack.Vel_command_ref[0]*CtrlDt;
		if(CtrlFbck.XH[1]<0.1*vel) //机头速度小于0.1倍的设定速度表明刹车完成，to be modified
		{
			CtrlLpIO.Acc_command[0] = 0;
			CtrlLpIO.Jerk_command[0]= 0;
			isComplete = true;
			Ctrltrack.isBrake = false;
			Ctrltrack.accelerationDistance = 0.0;
			return true;
		}
	}
	Ctrltrack.Pos_track[0] = (CtrlFbck.XH[0]-TempBodyFrame(0));
	Ctrltrack.Pos_track[1] = (CtrlFbck.YH[0]-TempBodyFrame(1));
	float X_err_ref = Ctrltrack.Pos_command_ref[0] - Ctrltrack.Pos_track[0];
	float Y_err_ref = 0 - Ctrltrack.Pos_track[1];
	CtrlLpIO.Vel_command[0] = Ctrltrack.Vel_command_ref[0] + fConstrain(pidX->PID_Controller(X_err_ref,CtrlDt),-0.5,0.5);
	CtrlLpIO.Vel_command[1] = pidY->PID_Controller(Y_err_ref,CtrlDt);
	if((end-cur).norm()>2.0)CtrlLpIO.Ang_command[2] = atan2f(end(1)-cur(1),end(0)-cur(0));
	return false;
}

bool CONTROL::followCirclePath(float radius,float vel,Vector2f center,float goal_angle,bool isContinuation,int direction)
{
	/*
	 * radius: 飞行半径
	 * vel: 速度
	 * center： 圆心
	 * goal_angle：飞过多少角度，2*PI表示飞过一圈
	 * isContinuation：表明是否继续飞行
	 * direction：方向，1表示顺时针，-1表示逆时针
	 */
	float t0 = vel;
	float t1 = t0/4.0,t2 = 3.0*t0/4.0;
	float a0 = 4.0*vel/(3.0*t0);
	float v1 = a0*t1/2.0,v2 = a0*(2*t2-t1)/2.0;
	Vector2f cur(CtrlFbck.X[0],CtrlFbck.Y[0]);
	float traveled_X = Ctrltrack.YawAngle_traveled;
	float remaining_X = goal_angle - traveled_X;
	if(isComplete){
		isComplete       =false;
		CtrlLpIO.Acc_command[0]    = 0;
		CtrlLpIO.Jerk_command[0]= 0;
		Ctrltrack.Vel_command_ref[0]= 0;
		Ctrltrack.Pos_command_ref[0]= 0;
		Ctrltrack.isUniform = false;
		Ctrltrack.isBrake = false;
		Ctrltrack.accelerationAngle =0;
		Ctrltrack.accelerationDistance =0;
	}
	if(Ctrltrack.Vel_command_ref[0]<vel && !Ctrltrack.isBrake && !Ctrltrack.isUniform) //从起点加速过程
	{
		if(Ctrltrack.Vel_command_ref[0]<v1){
		CtrlLpIO.Acc_command[0]+=a0/t1*CtrlDt;
		CtrlLpIO.Jerk_command[0]= a0/t1;
		}
		else if(Ctrltrack.Vel_command_ref[0]>=v1 && Ctrltrack.Vel_command_ref[0]<v2){
			CtrlLpIO.Acc_command[0] = a0;
			CtrlLpIO.Jerk_command[0]= 0;
		}
		else {
			CtrlLpIO.Acc_command[0] -= a0/t1*CtrlDt;
			CtrlLpIO.Jerk_command[0]= -a0/t1;
		}
		CtrlLpIO.Acc_command[0] = fConstrain(CtrlLpIO.Acc_command[0],0.01,a0);
		Ctrltrack.Vel_command_ref[0]+=CtrlLpIO.Acc_command[0]*CtrlDt;
		Ctrltrack.Pos_command_ref[0]+=Ctrltrack.Vel_command_ref[0]*CtrlDt;
		Ctrltrack.accelerationAngle = traveled_X + Ctrltrack.accelerationDistance/radius; //实际加速到vel走过的角度
	}
	if(Ctrltrack.Vel_command_ref[0]>=vel && (remaining_X>Ctrltrack.accelerationAngle||isContinuation)) //匀速过程  vel
	{
		Ctrltrack.isUniform = true; //匀速运动标志位
	}
	if(remaining_X<=Ctrltrack.accelerationAngle && !isContinuation) //快到目标点时的减速过程
	{
		Ctrltrack.isBrake = true; //刹车标志位
		Ctrltrack.isUniform = false;
	}
	if(Ctrltrack.isUniform){
		CtrlLpIO.Acc_command[0]=0;
		Ctrltrack.Vel_command_ref[0]=vel;
		Ctrltrack.Pos_command_ref[0]+=Ctrltrack.Vel_command_ref[0]*CtrlDt;
	}
	if(Ctrltrack.isBrake){
		if(Ctrltrack.Vel_command_ref[0]>v2){
		CtrlLpIO.Acc_command[0]-=a0/t1*CtrlDt;
		CtrlLpIO.Jerk_command[0]= -a0/t1;
		}
		else if(Ctrltrack.Vel_command_ref[0]>v1 && Ctrltrack.Vel_command_ref[0]<=v2){
			CtrlLpIO.Acc_command[0] = -a0;
			CtrlLpIO.Jerk_command[0]= 0;
		}
		else {
			CtrlLpIO.Acc_command[0] += a0/t1*CtrlDt;
			CtrlLpIO.Jerk_command[0]= a0/t1;
		}
		CtrlLpIO.Acc_command[0] = fConstrain(CtrlLpIO.Acc_command[0],-a0,-0.01);
		Ctrltrack.Vel_command_ref[0]+=CtrlLpIO.Acc_command[0]*CtrlDt;
		Ctrltrack.Pos_command_ref[0]+=Ctrltrack.Vel_command_ref[0]*CtrlDt;
		if(CtrlFbck.XH[1]<0.1*vel){ //机头速度小于0.1倍的设定速度表明刹车完成，to be modified
			CtrlLpIO.Acc_command[0]=0;
			CtrlLpIO.Jerk_command[0]= 0;
			Ctrltrack.Vel_command_ref[0]= 0;
			Ctrltrack.Pos_command_ref[0]= 0;
			isComplete=true;
			Ctrltrack.isBrake = false;
			Ctrltrack.accelerationAngle = 0.0;
			return true;
		}
	}
	float X_err_ref = Ctrltrack.Pos_command_ref[0] - traveled_X*radius;
	CtrlLpIO.Vel_command[0] = Ctrltrack.Vel_command_ref[0] + fConstrain(pidX->PID_Controller(X_err_ref,CtrlDt),-0.5,0.5);

	float Y_err_ref = (cur - center).norm()-radius;
	CtrlLpIO.Vel_command[1] = direction*pidY->PID_Controller(Y_err_ref,CtrlDt);
	CtrlLpIO.Acc_command[1] = direction*SQR(CtrlLpIO.Vel_command[0])/radius; //向心加速度 Y方向

	CtrlLpIO.AngVel_command[2] = direction*CtrlLpIO.Vel_command[0]/radius;

	Ctrltrack.azimuth = atan2f((center(1)-CtrlFbck.Y[0]),(center(0)-CtrlFbck.X[0])); //方位角 面向圆心
	if(direction == 1){ //CW 顺时针
		if(Ctrltrack.azimuth<-PI/2) Ctrltrack.desiredHeading = Ctrltrack.azimuth+3*PI/2;
		else Ctrltrack.desiredHeading = Ctrltrack.azimuth-PI/2;
	}
	else{ //CCW 逆时针
		if(Ctrltrack.azimuth>PI/2) Ctrltrack.desiredHeading = Ctrltrack.azimuth-3*PI/2;
		else Ctrltrack.desiredHeading = Ctrltrack.azimuth+PI/2;
	}
	Ctrltrack.desiredHeading +=CtrlLpIO.AngVel_command[2]*CtrlDt;
	CtrlLpIO.Ang_command[2]=Ctrltrack.desiredHeading;
//	Ctrltrack.headingAng = CtrlFbck.Ang[2];
	return false;
}

void CONTROL::trackPath(void)
{
//	PATHFOLLOW::Stateflag state_transition;
//	PATHFOLLOW::get_Stateflag(state_transition);
	static std::list<PATHFOLLOW::Path> paths;
	static std::list<PATHFOLLOW::Path>::iterator it = paths.begin();
	std::list<PATHFOLLOW::Path>::iterator last = paths.end();

	double X_err,Y_err,Z_err,XY_err;
	static float iniAng=0.0;
	float sinY,cosY;
	static Vector2f end(0,0),start(0,0),center(0,0);
	Vector2f cur(CtrlFbck.X[0],CtrlFbck.Y[0]);
	static u16 count=0,num=0;
	static bool TurnHead=false;
	sinY = sinf(CtrlFbck.Ang[2]);
	cosY = cosf(CtrlFbck.Ang[2]);
	float sin_azimuth = sinf(Ctrltrack.azimuth);
	float cos_azimuth = cosf(Ctrltrack.azimuth);
	CtrlFbck.XH[0] =  CtrlFbck.X[0]*cos_azimuth + CtrlFbck.Y[0]*sin_azimuth;
	CtrlFbck.YH[0] = -CtrlFbck.X[0]*sin_azimuth + CtrlFbck.Y[0]*cos_azimuth;
	float rho_orbit = SQR(Dubins_Vel)/OneG/tan(DubinsTiltAng); // 转弯半径
	fly_radius=rho_orbit;
	XY_err = sqrt(SQR(CtrlFbck.XH[1])+SQR(CtrlFbck.YH[1]));
		//-----------------------模式切换-----------------------------------------
	if(Ctrltrack.dubins_start == false)
	{
		StatusClear();
		PATHFOLLOW::get_paths(paths);
		it = paths.begin();
		Ctrltrack.Dubins_Status = Dubins_TurnHead;
		CtrlLpIO.Pos_command[0] = CtrlFbck.X[0];
		CtrlLpIO.Pos_command[1] = CtrlFbck.Y[0];
		CtrlLpIO.Pos_command[2] = CtrlFbck.Z[0];//
//		CtrlLpIO.Ang_command[2] = CtrlFbck.Ang[2];
		isSegmentComplete = true;
		TurnHead = false;
		Ctrltrack.dubins_start=true;
	}
	if((it == paths.end() || paths.empty()) && Ctrltrack.Dubins_Status != Dubins_hover){ //定点
		CtrlLpIO.Pos_command[0] = CtrlFbck.X[0];
		CtrlLpIO.Pos_command[1] = CtrlFbck.Y[0];
		CtrlLpIO.Pos_command[2] = CtrlFbck.Z[0];
		Ctrltrack.Dubins_Status = Dubins_hover;
	}
	if(TurnHead == true && isSegmentComplete){
		if(it->lambda == 0) {//直线
			start = it->r; //起点
			end = it->w; //终点
			Ctrltrack.Dubins_Status = Dubins_Straight;
			point_list[num++].GetPosition(&position);//获得航点位置信息
			Ctrltrack.azimuth = atan2f((position.data.y-CtrlFbck.Y[0]),(position.data.x-CtrlFbck.X[0]));
			CtrlFbck.XH[0] =  CtrlFbck.X[0]*cosf(Ctrltrack.azimuth) + CtrlFbck.Y[0]*sinf(Ctrltrack.azimuth);
			CtrlFbck.YH[0] = -CtrlFbck.X[0]*sinf(Ctrltrack.azimuth) + CtrlFbck.Y[0]*cosf(Ctrltrack.azimuth);
			TempBodyFrame << CtrlFbck.XH[0],CtrlFbck.YH[0];
			Ctrltrack.goalDistance=it->len;
			Ctrltrack.goalAng = 0; //弧度
		}
		else{
			center = it->c; //圆心
			Ctrltrack.goalAng = it->len/rho_orbit; //弧度
			circleRemaining = static_cast<int>(floor(Ctrltrack.goalAng)/(2*PI)); //将画多少个圆
			Ctrltrack.Dubins_Status = Dubins_Orbit;
			Ctrltrack.YawAngle_traveled = 0.0;
			iniAng = CtrlFbck.Ang[2];
			Ctrltrack.goalDistance=0;
		}
		Ctrltrack.Pos_command_ref[0]=0.0;
		isSegmentComplete=false;
	}
		X_err = CtrlLpIO.Pos_command[0] - CtrlFbck.X[0];
		Y_err = CtrlLpIO.Pos_command[1] - CtrlFbck.Y[0];
		Z_err = CtrlLpIO.Pos_command[2] - CtrlFbck.Z[0];
		CtrlLpIO.Pos_err[0] = (X_err* cosY + Y_err*sinY);
		CtrlLpIO.Pos_err[1] = (X_err*-sinY + Y_err*cosY);
		CtrlLpIO.Pos_err[2] = Z_err;
		CtrlLpIO.Vel_command[0] = pidX->PID_Controller(CtrlLpIO.Pos_err[0],CtrlDt);
		CtrlLpIO.Vel_command[1] = pidY->PID_Controller(CtrlLpIO.Pos_err[1],CtrlDt);
		CtrlLpIO.Vel_command[2] = pidZ->PID_Controller(CtrlLpIO.Pos_err[2],CtrlDt);
		CtrlLpIO.Vel_command[2] = fConstrain(CtrlLpIO.Vel_command[2],-1,0.5);
	switch(Ctrltrack.Dubins_Status)
	{
	case Dubins_hover://定点
		CtrlLpIO.Vel_command[0] = fConstrain(CtrlLpIO.Vel_command[0],-1,1);
		CtrlLpIO.Vel_command[1] = fConstrain(CtrlLpIO.Vel_command[1],-1,1);
        break;
	case Dubins_TurnHead://转头
		point_list[0].GetPosition(&position);//获得航点位置信息
		CtrlLpIO.Vel_command[0] = fConstrain(CtrlLpIO.Vel_command[0],-1,1);
		CtrlLpIO.Vel_command[1] = fConstrain(CtrlLpIO.Vel_command[1],-1,1);
		Ctrltrack.azimuth = atan2f((position.data.y-CtrlFbck.Y[0]),(position.data.x-CtrlFbck.X[0]));
		if (Turn_Heading(Ctrltrack.azimuth))TurnHead = true;
		break;
	case Dubins_Straight://
		Ctrltrack.distance_XY = (cur - start).norm();
		if(it != (--last))followStraightPath(&start,&end,Dubins_Vel,true);
		else Ctrltrack.brake_finish=followStraightPath(&start,&end,Dubins_Vel,false);
//		Ctrltrack.distance_XY = it->len+0.5;
		if(it != (--last)){	//若不是最后一段轨迹
			if(Ctrltrack.distance_XY>=it->len){
				isSegmentComplete=true;
				++it;
			}
		}
		else if(absf(Ctrltrack.distance_XY-it->len)<1.0 || Ctrltrack.brake_finish==true){ //是最后一段轨迹
			isSegmentComplete=true;
			isComplete=true;
			Ctrltrack.brake_finish=false;
			++it;
		}
		CtrlLpIO.Vel_command[0] = fConstrain(CtrlLpIO.Vel_command[0],-Dubins_Vel,Dubins_Vel);////////
		CtrlLpIO.Vel_command[1] = fConstrain(CtrlLpIO.Vel_command[1],-1,1);
		break;
	case Dubins_Orbit://
		calcCircularAngle(iniAng,Ctrltrack.goalAng,it->lambda);
		if(it != (--last))followCirclePath(rho_orbit,Dubins_Vel,center,Ctrltrack.goalAng,true,it->lambda);
		else Ctrltrack.brake_finish=followCirclePath(rho_orbit,Dubins_Vel,center,Ctrltrack.goalAng,false,it->lambda);

		if(it != (--last)){
			if (Ctrltrack.YawAngle_traveled>= Ctrltrack.goalAng){
				CtrlLpIO.Ang_command[2] = CtrlFbck.Ang[2];//**
				Ctrltrack.YawAngle_traveled = 0.0;
				CtrlLpIO.Acc_command[1] = 0;
				isSegmentComplete=true;
				++it;
			}
		}
		else if(absf(Ctrltrack.YawAngle_traveled-Ctrltrack.goalAng)<1.0/rho_orbit || Ctrltrack.brake_finish==true){
			CtrlLpIO.Ang_command[2] = CtrlFbck.Ang[2];//**
			Ctrltrack.YawAngle_traveled = 0.0;
			CtrlLpIO.Acc_command[1] = 0;
			isSegmentComplete=true;
			isComplete=true;
			Ctrltrack.brake_finish=false;
			++it;
		}
		CtrlLpIO.Vel_command[0] = fConstrain(CtrlLpIO.Vel_command[0],-Dubins_Vel,Dubins_Vel);
		CtrlLpIO.Vel_command[1] = fConstrain(CtrlLpIO.Vel_command[1],-1,1);
		break;
	case Dubins_Arrive://到达
		CtrlLpIO.Vel_command[0] = fConstrain(CtrlLpIO.Vel_command[0],-1,1);
		CtrlLpIO.Vel_command[1] = fConstrain(CtrlLpIO.Vel_command[1],-1,1);
		if(count<(2/CtrlDt)) count++;
		else{
			Ctrltrack.Dubins_Status = Dubins_hover;
			count=0;
		}
        break;
	case NonDubins:
		break;
	default:break;
	}
	xQueueOverwrite(queuetrajectoryData,&Ctrltrack);

}

bool CONTROL::Turn_Heading(float SetAng)//返回True表明转到目标角度
{
	float Vel_Ang=0.4;//偏航角速度
	static u16 count=0;
	float AngleDiff = absf(SetAng-CtrlFbck.Ang[2]);
	if(AngleDiff>PI)AngleDiff=2*PI-AngleDiff;
	if(AngleDiff < (4*D2R_D) )  //判断角度差小于4°的时候，认为已经转到了目标角度（阈值可以根据情况更改）
	{
		CtrlLpIO.Ang_command[2] = SetAng;
		count++;	//延时1s等待稳定
		if(count>100)
		{
			CtrlLpIO.Ang_command[2] = SetAng;
			count = 0;
			return true;
		}
	}
	if(SetAng>=0)
	{
		if(CtrlFbck.Ang[2]>=SetAng || CtrlFbck.Ang[2]<=(SetAng-PI))
		{
			CtrlLpIO.Ang_command[2] = CtrlLpIO.Ang_command[2]- Vel_Ang*CtrlDt; //CCW 逆时针转
			if(CtrlLpIO.Ang_command[2]<-PI)CtrlLpIO.Ang_command[2]+=2*PI;//如果小于 -PI,应加上2*PI
		}
		else //CW 顺时针转
		{
			CtrlLpIO.Ang_command[2] = CtrlLpIO.Ang_command[2]+ Vel_Ang*CtrlDt; //这种情况下不会大于2*PI
		}
	}
	else
	{
		if(CtrlFbck.Ang[2]<=SetAng || CtrlFbck.Ang[2]>=(SetAng+PI))
		{
			CtrlLpIO.Ang_command[2] = CtrlLpIO.Ang_command[2]+ Vel_Ang*CtrlDt; //CW 顺时针转
			if(CtrlLpIO.Ang_command[2]>PI)CtrlLpIO.Ang_command[2]-=2*PI;//如果大于 PI,应减去2*PI
		}
		else
		{
			CtrlLpIO.Ang_command[2] = CtrlLpIO.Ang_command[2]- Vel_Ang*CtrlDt; //这种情况下不会小于-2*PI
		}
	}
	return false;
}
void CONTROL::calcCircularAngle(float iniAng,float goalAng,int direction)
{
	float thresholdAng=0.0;
	float YawAngle_last=0;
	static bool isBegin=true;
	YawAngle_last = Ctrltrack.YawAngle_traveled - circleTraveled*2*PI;
	if(direction == 1){ //CW
		thresholdAng = CtrlFbck.Ang[2]>iniAng?CtrlFbck.Ang[2]-iniAng:0.0;
		if(thresholdAng<0.2 && Ctrltrack.Pos_command_ref[0]/fly_radius<0.2){
			isBegin = true; //认为还在起点附近,若此时利用航向角判断是否到达终点大概率有误差
			if(CtrlFbck.Ang[2]>=iniAng) {
				Ctrltrack.YawAngle_traveled = CtrlFbck.Ang[2] - iniAng;
			}
			else Ctrltrack.YawAngle_traveled = Ctrltrack.Pos_command_ref[0]/fly_radius; //假定初始阶段没有跟踪误差(后续再优化)
		}
		else{
			isBegin = false;
			if(CtrlFbck.Ang[2]>=iniAng) Ctrltrack.YawAngle_traveled = CtrlFbck.Ang[2] - iniAng;
			else Ctrltrack.YawAngle_traveled = 2*PI-(iniAng-CtrlFbck.Ang[2]);
		}
	}
	else{ //CCW
		thresholdAng = CtrlFbck.Ang[2]<iniAng?iniAng-CtrlFbck.Ang[2]:0.0;
		if(thresholdAng<0.2 && Ctrltrack.Pos_command_ref[0]/fly_radius<0.2){
			isBegin = true; //认为还在起点附近,若此时利用航向角判断是否到达终点大概率有误差
			if(CtrlFbck.Ang[2]<=iniAng){
				Ctrltrack.YawAngle_traveled = -(CtrlFbck.Ang[2] - iniAng);
			}
			else Ctrltrack.YawAngle_traveled = Ctrltrack.Pos_command_ref[0]/fly_radius; //假定初始阶段没有跟踪误差(后续再优化)
		}
		else{
			isBegin = false;
			if(CtrlFbck.Ang[2]<=iniAng) Ctrltrack.YawAngle_traveled = -(CtrlFbck.Ang[2] - iniAng);
			else Ctrltrack.YawAngle_traveled = 2*PI+(iniAng-CtrlFbck.Ang[2]);
		}
	}
	if(circleRemaining != 0 && YawAngle_last>1.99*PI && Ctrltrack.YawAngle_traveled<0.01*PI) //走过整个圆
	{
		circleRemaining--;
		circleTraveled++;
	}
	Ctrltrack.YawAngle_traveled += circleTraveled*2*PI;
	Ctrltrack.YawAngle_remaining = goalAng-Ctrltrack.YawAngle_traveled;
}
void CONTROL::Out_Loop_Z_Pre2()
{
	int Z_command_flag;
	float  Vel_tol=0.2f;
//	float  Velz_end=0.6f;
//	float  Acc_alt_manu=0.3f;
	float  GE1=GE_SPEED;
	float  GE2=GE_ACC;
	//----------------------检测打杆与否--------------------------------------
	if      (CtrlIO.input[3]>1630 ) Z_command_flag = 1;//遥控器在上部，上升
	else if (CtrlIO.input[3]<1370 ) Z_command_flag = 2;//遥控器在下部，下降
	else                            Z_command_flag = 3;//遥控器在中间，悬停

	//-----------------------模式切换-----------------------------------------
	if ((CtrlIO.last_control_mode!=2) &&( CtrlIO.FlightStatus == TAKEOFF ||CtrlIO.FlightStatus == RETURNING||CtrlIO.FlightStatus == LANDING))
	{
		CtrlIO.FlightStatus = AIR;
	}
	if ((CtrlIO.last_control_mode!=2) || (CtrlLpIO.Z_phase==2 && CtrlLpIO.alt_brake_mode==false))
	{
		CtrlLpIO.Z_phase = 0; //定高
		CtrlLpIO.Pos_command[2] = CtrlFbck.Z[0];//当前位置作为位置给定
		pid[11].integral        = 0.0;
	}
	if (CtrlLpIO.Z_phase!=1 && Z_command_flag!=3 && CtrlIO.FlightStatus == AIR)
	{
		CtrlLpIO.Z_phase = 1;
		if (Z_command_flag==2)
		{
			CtrlLpIO.alt_landing_cnt.CNT  = 0;
		    CtrlLpIO.landing_acc_mode = true;
		}
	}
	if (CtrlLpIO.Z_phase==1 && Z_command_flag==3)
	{
		CtrlLpIO.Z_phase = 2;
		CtrlLpIO.alt_brake_cnt.CNT = 0;
		CtrlLpIO.alt_brake_mode    = true;
		CtrlIO.FlightStatus = AIR;
	}
	//-------------------------地面状态起飞前怠速----------------------------
	if (Z_command_flag!=1 && CtrlIO.FlightStatus == GROUND)
	{
		CtrlIO.FlightStatus = IDLING;
	}
	//-------------------------怠速状态下切换起飞----------------------------
	if (Z_command_flag==1 && CtrlIO.FlightStatus == IDLING)
	{
		CtrlLpIO.Z_phase = 1;
		CtrlIO.FlightStatus = LAUNCH;
	}
	if(CtrlIO.FlightStatus == LAUNCH) //发射
	{
		static int count = 0;
		count++;
		if(count>50){
			count = 0;
			CtrlIO.FlightStatus = TAKEOFF;
		}
	}
	//-------------------------准备降落判断是否进入地效区----------------------------
	if (CtrlLpIO.Z_phase==1 && Z_command_flag==2 && CtrlLpIO.landing_acc_mode==false && CtrlIO.FlightStatus != GE)
	{
		CtrlIO.FlightStatus = LANDING;
	}
	if(CtrlIO.FlightStatus == LANDING && Z_command_flag==2)
	{
		if(CtrlFbck.Z[1]<=GE1  && (acc_fil.acc_filter[2]+OneG)<=GE2) CtrlIO.FlightStatus = GE;
	}

	//-----------------------模式功能-----------------------------------------
	switch(CtrlLpIO.Z_phase)
	{
	case 0://定高
		CtrlLpIO.Pos_err[2]     = CtrlLpIO.Pos_command[2]-CtrlFbck.Z[0];
		CtrlLpIO.Vel_command[2] = pidZ->PID_Controller(CtrlLpIO.Pos_err[2],CtrlDt);
		CtrlLpIO.Vel_command[2] = fConstrain(CtrlLpIO.Vel_command[2],-1,0.5);
        break;
	case 1:
		if (Z_command_flag==2)
			{
			   CtrlLpIO.Vel_command[2] = 0.5f;
			   CtrlLpIO.landing_acc_mode = TarHit(&CtrlLpIO.alt_landing_cnt,Vel_tol,CtrlLpIO.Vel_err[2]);
			}
		else if (Z_command_flag==1)
		{
			CtrlLpIO.Vel_command[2] = -1.0f;
		}
		if(CtrlIO.FlightStatus == GE)
		{
			   CtrlLpIO.Vel_command[2] = 0.0f;
		}
	break;
	case 2:
		CtrlLpIO.alt_brake_mode = TarHit(&CtrlLpIO.alt_brake_cnt,Vel_tol,CtrlLpIO.Vel_err[2]);
		CtrlLpIO.Vel_command[2] = 0.0f;
		break;
	default:break;
	}
	CtrlLpIO.Acc_command[2] = 0.0f;
}

void CONTROL::Out_Loop_Step2()
{

	xQueuePeek(queueAccDatFil, &acc_fil, 0);					//从队列中获取高度数据
	float u1_Tilt_D;
	float Rd,Pd,Yd;
	float A_f;


	Vector3f A;
	Vector3f x_c;
	Vector3f a_c1,a_c2,a_c3;
	Vector3f a_c2r;
	Vector3f e3;
	Matrix<float,3,3> Rb2n_f;
	//-----------------------NEDH速度误差-----------------------------//

		CtrlLpIO.Vel_err[0] = CtrlLpIO.Vel_command[0]-CtrlFbck.XH[1];
		CtrlLpIO.Vel_err[1] = CtrlLpIO.Vel_command[1]-CtrlFbck.YH[1];
		CtrlLpIO.Vel_err[2] = CtrlLpIO.Vel_command[2]-CtrlFbck.Z[1];

	//----------------------------------------------------------------

	float ad =   fConstrain(pidXRate->PID_Controller(CtrlLpIO.Vel_err[0],CtrlDt),-3.5,3.5)
			   + CtrlLpIO.Acc_command[0];

	float bd =   fConstrain(pidYRate->PID_Controller(CtrlLpIO.Vel_err[1],CtrlDt),-3.5,3.5)
			   + CtrlLpIO.Acc_command[1];

	float cd =   fConstrain(pidZRate->PID_Controller(CtrlLpIO.Vel_err[2],CtrlDt),-OneG,OneG)
			   + CtrlLpIO.Acc_command[2];
	float ax = acc_fil.acc_filter[0] ;//- gps->acc_bias_EKF[0];
	float ay = acc_fil.acc_filter[1] ;//- gps->acc_bias_EKF[1];
	float az = acc_fil.acc_filter[2] ;//- gps->acc_bias_EKF[2];
	float A1 = ad - cos(CtrlFbck.Ang[1])*ax - sin(CtrlFbck.Ang[0])*sin(CtrlFbck.Ang[1])*ay;//机体系到地球系的旋转矩阵只取前两列
	float A2 = bd - cos(CtrlFbck.Ang[0])*ay;
	float A3 = cd + sin(CtrlFbck.Ang[1])*ax - sin(CtrlFbck.Ang[0])*cos(CtrlFbck.Ang[1])*ay - OneG;
//	//---------------------速度环INDI-------------------------//改为和角速度内环INDI一样的参数,    改加速度计滤波值，  取消拉力补偿
//	float A1 = ad - cos(CtrlFbck.Ang[1])*ax - sin(CtrlFbck.Ang[0])*sin(CtrlFbck.Ang[1])*ay - cos(CtrlFbck.Ang[0])*sin(CtrlFbck.Ang[1])*az + CtrlINDI.output_acc0[0];
//	float A2 = bd - cos(CtrlFbck.Ang[0])*ay + sin(CtrlFbck.Ang[0])*az + CtrlINDI.output_acc0[1];
//	float A3 = cd + sin(CtrlFbck.Ang[1])*ax - sin(CtrlFbck.Ang[0])*cos(CtrlFbck.Ang[1])*ay - cos(CtrlFbck.Ang[0])*cos(CtrlFbck.Ang[1])*az - OneG + CtrlINDI.output_acc0[2];
//	CtrlINDI.output_acc[0]=A1;
//	CtrlINDI.output_acc[1]=A2;
//	CtrlINDI.output_acc[2]=A3;
//	//---------------------输出滤波-------------------------//
//
//	for (u8 i=0;i<3;i++)
//	{
//		CtrlINDI.output_acc0[i] = (CtrlINDI.output_acc0[i]*(CtrlINDI.Filt_Output[i]-1) + CtrlINDI.output_acc[i])/CtrlINDI.Filt_Output[i];
//	}
	//-----------------------几何控制-----------------------

	A  << A1,A2,A3;
	A_f=A.norm();
	x_c << 1.0f,0.0f,0.0f;
	a_c3  = -A/A_f;
	a_c2r = a_c3.cross(x_c);
	a_c2  = a_c2r/a_c2r.norm();
	a_c1  = a_c2.cross(a_c3);
	Rb2n_f << a_c1[0],a_c2[0],a_c3[0],
				  a_c1[1],a_c2[1],a_c3[1],
				  a_c1[2],a_c2[2],a_c3[2];
	Pd = asin(fConstrain(-Rb2n_f(2,0),-1,1));
	Rd = atan2f(Rb2n_f(2,1),Rb2n_f(2,2));
	Yd = atan2f(Rb2n_f(1,0),Rb2n_f(0,0));
	CtrlLpIO.u1_Bar[0] = A_f;
//	CtrlLpIO.u1_Bar[0] = -(cd - OneG);

	//-------------------拉力补偿-------------------------

	if (CtrlLpIO.thrust_command[0]>(0.2f*OneG))
	{
		u1_Tilt_D = CtrlLpIO.ko1*(az + CtrlLpIO.u1_Bar[0]);
		CtrlLpIO.u1_Tilt[0] = CtrlLpIO.u1_Tilt[0] + u1_Tilt_D*CtrlDt;
	}

	//-------------------------------------------------------

	CtrlLpIO.thrust_command[0] = CtrlLpIO.u1_Bar[0] + CtrlLpIO.u1_Tilt[0];
//	CtrlLpIO.thrust_command[0] = CtrlLpIO.u1_Bar[0];
	CtrlLpIO.Ang_command[0]    = Rd;
	CtrlLpIO.Ang_command[1]    = Pd;
//	CtrlLpIO.Ang_command[0] =  CtrlIO.input[0]*0.05f*D2R;//滚转角度输入0.05对应20°
//	CtrlLpIO.Ang_command[1] =  CtrlIO.input[1]*0.05f*D2R;//俯仰角度输入0.05对应20°
	CtrlLpIO.thrust_command[0] = fConstrain(CtrlLpIO.thrust_command[0],0.2f*OneG,2.0f*OneG);

	//-------------------计算角速度前馈-------------------------
	float u1=1.0,phi=CtrlFbck.Ang[0],theta=CtrlFbck.Ang[1],psi=CtrlFbck.Ang[2];
	Vector3f acc_d(ad,bd,cd); //期望的加速度矢量
	Vector3f acc_measurement(ax,ay,az),hw,Wb2w;
	Vector3f Jerk(CtrlLpIO.Jerk_command[0],CtrlLpIO.Jerk_command[1],CtrlLpIO.Jerk_command[2]);
	Vector3f Zw(0.0,0.0,1.0); //惯性系Z轴
	Rbe = (Matrix3f() << cosf(theta)*cosf(psi), -cosf(phi)*sinf(psi)-sinf(phi)*sinf(theta)*cosf(psi),sinf(phi)*sinf(psi)-cosf(phi)*sinf(theta)*cosf(psi),
	                  cosf(theta)*sinf(psi), cosf(phi)*cosf(psi)-sinf(phi)*sinf(theta)*sinf(psi),  -sinf(phi)*cosf(psi)-cosf(phi)*sinf(theta)*sinf(psi),
	                  -sinf(theta),sinf(phi)*cosf(theta), cosf(phi)*cosf(theta)
	                  ).finished();
	float p,q,r;
	Matrix3f Omega = (Matrix3f() << 0, -CtrlFbck.pqr[2],CtrlFbck.pqr[1],
									CtrlFbck.pqr[2],0,-CtrlFbck.pqr[0],
	                             -CtrlFbck.pqr[1],CtrlFbck.pqr[0],0).finished();
	Vector3f Amdot(acc_fil.jerk_filter[0],acc_fil.jerk_filter[1],acc_fil.jerk_filter[2]); //是否是这样的滤波
	for(int j=0;j<3;++j){
		if(a_c3(j)!=0){
			u1 = (GROSSMASS * (acc_d - OneG * Zw-Rbe*acc_measurement))(j)/a_c3(j);
			break;
		}
	}
    hw = GROSSMASS * (Jerk-(a_c3.dot(Jerk))*a_c3-Rbe*Omega*acc_measurement-Rbe*Amdot)/u1;
    p = (-hw).dot(a_c2);
    q = (hw).dot(a_c1);
    r = 0;
    Wb2w=p*a_c1+q*a_c2+r*a_c3;  //期望输出与角速度的关系
    CtrlLpIO.AngVel_command[0]=Wb2w(0);
    CtrlLpIO.AngVel_command[1]=Wb2w(1);
}

//void CONTROL::In_Loop_Step()
///*
// *功能：角度控制
// *
// * 输入：CtrlLpIO.Ang_command[]
// *
// * 输出：	CtrlLpIO.Ang_err[0]
//		CtrlLpIO.Ang_err[1]
//		CtrlLpIO.Ang_err[2]
//
//		CtrlLpIO.pqr_command[0]
//		CtrlLpIO.pqr_command[1]
//		CtrlLpIO.pqr_command[2]
// */
//
//{
//	double R_err,P_err,Y_err;
//	float yaw_rate;
//	//----------------滚转俯仰角输入限幅----------------//
//	CtrlLpIO.Ang_command[0]   = fConstrain(CtrlLpIO.Ang_command[0],-CtrlLpIO.AngleLimitR,CtrlLpIO.AngleLimitR);
//	CtrlLpIO.Ang_command[1]   = fConstrain(CtrlLpIO.Ang_command[1],-CtrlLpIO.AngleLimitP,CtrlLpIO.AngleLimitP);
//	//----------------滚转俯仰角误差----------------//
//	R_err = CtrlLpIO.Ang_command[0]-CtrlFbck.Ang[0];
//	P_err = CtrlLpIO.Ang_command[1]-CtrlFbck.Ang[1];
//	//----------------滚转俯仰角变化率----------------//
//	float ab1[2],ab2[2],ab3[2];
//
//
//	if(CtrlIO.last_control_mode != 1)
//	{
//		for (u8 i=0;i<3;i++)
//		{
//			for(u8 j=0;j<RATE_SLOPE_NUM;j++)
//			{
//				CtrlLpIO.last_Ang_command[i][j] = CtrlLpIO.Ang_command[i];
//				CtrlLpIO.last_pqr_command[i][j] = CtrlLpIO.pqr_command[i];
//			}
//		}
//	}
//
//	LineFit(CtrlLpIO.last_Ang_command[0],RATE_SLOPE_NUM,ab1);
//	LineFit(CtrlLpIO.last_Ang_command[1],RATE_SLOPE_NUM,ab2);
//	LineFit(CtrlLpIO.last_Ang_command[2],RATE_SLOPE_NUM,ab3);
//
//	CtrlLpIO.Ang_command_d[0] = ab1[0]/CtrlDt;
//	CtrlLpIO.Ang_command_d[1] = ab2[0]/CtrlDt;
//
//	//----------------------记录杆量回到阈值内时的偏航角--------------------------------------
//
//	if ((CtrlIO.last_yaw_mode==0 && CtrlIO.yaw_mode==1 ) || (CtrlIO.last_fly_mode != CtrlIO.fly_mode ))
//	//锁尾不锁尾切换，轨迹模式切换
//	{
//		CtrlLpIO.yaw_mid = CtrlFbck.Ang[2];//记录偏航角
//		for(u8 j=0;j<RATE_SLOPE_NUM;j++)
//		{
//			CtrlLpIO.last_Ang_command[2][j] = CtrlLpIO.yaw_mid;//线性拟合数组记录为偏航角
//		}
//	}
//
//
//	//--------------------------偏航控制----------------------------------
//	if(CtrlIO.fly_mode == 0 && CtrlIO.yaw_mode == 0)//操控模式不锁尾
//	{
//		yaw_rate = CtrlIO.input[2]*0.225f*D2R;
//		CtrlLpIO.yaw_mid += yaw_rate*CtrlDt;
//	}
//
//	if (CtrlIO.fly_mode ==1 && rcCommand.TakeOffFinish != 0)//轨迹模式
//	{
//		CtrlLpIO.yaw_mid = trajectory.yaw_command;
//	}
//
//	if(rcCommand.Key[2] == 1)//轨迹模式完成后切回控角度模式，此时fly_mode ==1 ，但需要直接控制偏航或者起飞失败需要手动救回时，来不及切换fly_mode，需要能直接控制偏航角
//	{
//		yaw_rate = CtrlIO.input[2]*0.225f*D2R;
//		CtrlLpIO.yaw_mid += yaw_rate*CtrlDt;
//	}
//
//    //----------------------------------------------------
//	CtrlLpIO.Ang_command[2] = CtrlLpIO.yaw_mid;
//	Y_err                   = CtrlLpIO.Ang_command[2]-CtrlFbck.Ang[2];
//	//----------------消除-180°到+180°的跳变----------------
//	if      (Y_err<-PI)   Y_err+=2*PI;
//	else if (Y_err> PI)   Y_err-=2*PI;
//	//------------------------------------------
//	CtrlLpIO.Ang_command_d[2] = ab3[0]/CtrlDt;
//
//	//----------------欧拉角变化率到pqr----------------
//	CtrlLpIO.pqr_command[0] = CtrlLpIO.Ang_command_d[0] - CtrlLpIO.Ang_command_d[2]*sin(CtrlFbck.Ang[1]);
//	CtrlLpIO.pqr_command[1] = CtrlLpIO.Ang_command_d[1]*cos(CtrlFbck.Ang[0]) + CtrlLpIO.Ang_command_d[2]*cos(CtrlFbck.Ang[1])*sin(CtrlFbck.Ang[0]);
//	CtrlLpIO.pqr_command[2] =-CtrlLpIO.Ang_command_d[1]*sin(CtrlFbck.Ang[0]) + CtrlLpIO.Ang_command_d[2]*cos(CtrlFbck.Ang[1])*cos(CtrlFbck.Ang[0]);
//
//	CtrlLpIO.Ang_err[0] = R_err - Y_err*sin(CtrlFbck.Ang[1]);
//	CtrlLpIO.Ang_err[1] = P_err*cos(CtrlFbck.Ang[0]) + Y_err*cos(CtrlFbck.Ang[1])*sin(CtrlFbck.Ang[0]);
//	CtrlLpIO.Ang_err[2] =-P_err*sin(CtrlFbck.Ang[0]) + Y_err*cos(CtrlFbck.Ang[1])*cos(CtrlFbck.Ang[0]);
//
//	CtrlLpIO.pqr_command[0] = fConstrain(CtrlLpIO.pqr_command[0],-45*D2R,45*D2R);
//	CtrlLpIO.pqr_command[1] = fConstrain(CtrlLpIO.pqr_command[1],-45*D2R,45*D2R);
//	CtrlLpIO.pqr_command[2] = fConstrain(CtrlLpIO.pqr_command[2],-90*D2R,90*D2R);
//}

void CONTROL::In_Loop_Step2()
{
	float R_err,P_err,Y_err;
	float yaw_rate;
	//----------------滚转俯仰角输入限幅----------------//

	CtrlLpIO.Ang_command[0] = fConstrain(CtrlLpIO.Ang_command[0],-CtrlLpIO.AngleLimitR,CtrlLpIO.AngleLimitR);
	CtrlLpIO.Ang_command[1] = fConstrain(CtrlLpIO.Ang_command[1],-CtrlLpIO.AngleLimitP,CtrlLpIO.AngleLimitP);

	//----------------滚转俯仰角误差----------------//

	R_err = CtrlLpIO.Ang_command[0] - CtrlFbck.Ang[0];
	P_err = CtrlLpIO.Ang_command[1] - CtrlFbck.Ang[1];

//	//----------------滚转俯仰角变化率----------------//
//
//	CtrlLpIO.Ang_command_d[0] = pidRol->PID_Controller(R_err,CtrlDt);
//	CtrlLpIO.Ang_command_d[1] = pidPit->PID_Controller(P_err,CtrlDt);

	//--------------------------偏航控制----------------------------------
	if (CtrlIO.control_mode!=3 && CtrlIO.control_mode!=5)
	{

		if (CtrlIO.last_control_mode==0)
		{
			CtrlLpIO.Ang_command[2] = CtrlFbck.Ang[2];
		}
		else
		{
			if (CtrlIO.yaw_mode==0)  yaw_rate = CtrlIO.input[2]*0.225*D2R;
			else                     yaw_rate = 0.0f;
			CtrlLpIO.Ang_command[2] = CtrlLpIO.Ang_command[2] + yaw_rate*CtrlDt;
		}
	}

	Y_err = CtrlLpIO.Ang_command[2] - CtrlFbck.Ang[2];
	modPI(&Y_err);
	    //----------------消除-180°到+180°的跳变----------------
//	    if      (Y_err<-PI)   Y_err+=2*PI;
//	    else if (Y_err> PI)   Y_err-=2*PI;
	    //------------------------------------------

	//----------------欧拉角角给定变化率----------------//
	float ab1[2],ab2[2],ab3[2];

	LineFit(CtrlLpIO.last_Ang_command[0],RATE_SLOPE_NUM,ab1);
	LineFit(CtrlLpIO.last_Ang_command[1],RATE_SLOPE_NUM,ab2);
	LineFit(CtrlLpIO.last_Ang_command[2],RATE_SLOPE_NUM,ab3);

	CtrlLpIO.Ang_command_d[0] = pidRol->PID_Controller(R_err,CtrlDt) + 0.0*ab1[0]/CtrlDt;
	CtrlLpIO.Ang_command_d[1] = pidPit->PID_Controller(P_err,CtrlDt) + 0.0*ab2[0]/CtrlDt;
	CtrlLpIO.Ang_command_d[2] = pidYaw->PID_Controller(Y_err,CtrlDt) + 0.0*ab3[0]/CtrlDt+CtrlLpIO.AngVel_command[2];

	//----------------欧拉角变化率到pqr----------------
	CtrlLpIO.pqr_command[0] = CtrlLpIO.Ang_command_d[0] - CtrlLpIO.Ang_command_d[2]*sin(CtrlFbck.Ang[1]);
	CtrlLpIO.pqr_command[1] = CtrlLpIO.Ang_command_d[1]*cos(CtrlFbck.Ang[0]) + CtrlLpIO.Ang_command_d[2]*cos(CtrlFbck.Ang[1])*sin(CtrlFbck.Ang[0]);
	CtrlLpIO.pqr_command[2] =-CtrlLpIO.Ang_command_d[1]*sin(CtrlFbck.Ang[0]) + CtrlLpIO.Ang_command_d[2]*cos(CtrlFbck.Ang[1])*cos(CtrlFbck.Ang[0]);
}

void CONTROL::Gyro_Control()
{
	float fan_dir = 1.0;//转动方向，1为顺时针，-1为逆时针
	float Ifan = -0.000037; //风扇转动惯量
	float k_T0 = 9.9796018325697625989171178675552e-6;//风扇拉力系数
//	CtrlLpIO.thrust_command[0] = OneG;
	if(CtrlLpIO.thrust_command[0] < 0.5f*OneG) return;
	float T = CtrlLpIO.thrust_command[0] * GROSSMASS;
	int speed = sqrt(T /k_T0);//转速
	float d_cs = 0.007328636;
	float den = 1.225;//空气密度kg/m^3
	float S = 0.040828138126052952;//风扇桨盘面积
	float sd = 0.7; //涵道扩压比
	Vector3f Ve(CtrlFbck.X[1], CtrlFbck.Y[1], CtrlFbck.Z[1]);
	Vector3f Vb = Rbe.transpose() * Ve;
	float Vci= -Vb(2)/2 + sqrt( pow(Vb(2)/2, 2) + T/(sd*den*S)); //涵道出口风速
	Matrix<float,4,3> B_pseudo;
	B_pseudo<< -2.9276,         0,    3.7606,
				     0,   -2.9276,    3.7606,
				2.9276,         0,    3.7606,
				     0,    2.9276,    3.7606;
	Matrix<float,4,3> Dcs_pseudo = 1/(Vci*Vci*d_cs)*B_pseudo;
	Vector3f omega(-fan_dir*CtrlFbck.pqr[1], fan_dir*CtrlFbck.pqr[0], 0);
	gyroTorqueCompensation = Ifan * speed * Dcs_pseudo * omega;
	//------------------------陀螺力矩补偿量计算（力矩单位）----------------------
//	CtrlLpIO.gyro_output[0] = gyro_output(0);
//	CtrlLpIO.gyro_output[1] = gyro_output(1);
//	CtrlLpIO.gyro_output[2] = 0.0f;
}

void CONTROL::Control_Output_INDI()
{
	xQueuePeek(queuePID, &pid_msg, 0);
	xQueuePeek(queueAccDatFil, &acc_fil, 0);
	float c_m=(CS_LIMIT_MAX/RAD_TO_PWM);
	//-----------------------获取角加速度----------------------//

	for (u8 i=0;i<3;i++)
	{
		CtrlINDI.pqr_d_raw[i] = (CtrlFbck.pqr[i] - CtrlINDI.last_pqr[i])/CtrlDt;//差分得到角加速度
		CtrlINDI.pqr_d0[i]    = (CtrlINDI.pqr_d0[i]*(CtrlINDI.Filt_AngularAcc[i]-1) + CtrlINDI.pqr_d_raw[i])/CtrlINDI.Filt_AngularAcc[i];//滤波
	}

	//-----------------------放大系数-------------------------//

	float k_ve;
//	k_rss = 1.225*0.040828138126052952*0.7;
//	V_e   = -CtrlFbck.W[0]/2 + sqrt(SQR((CtrlFbck.W[0]/2))+GROSSMASS*fConstrain(-acc_fil.acc_filter[2],0,400)/k_rss);
//	k_ve  = SQR(V_e)/(GROSSMASS*OneG/k_rss);
	k_ve = fConstrain(-acc_fil.acc_filter[2]/OneG,0.5,1.5);
//	k_ve = 1.0f;

    //----------------------计算控制效率矩阵-----------------//

	float k_ad[3];//用于微调控制效率矩阵
	k_ad[0] = 1.0f;
	k_ad[1] = 1.0f;
	k_ad[2] = 1.0f;
	CtrlINDI.G_cv[0] = (O2MX/INERTIA_X)*k_ve*k_ad[0];
	CtrlINDI.G_cv[1] = (O2MY/INERTIA_Y)*k_ve*k_ad[1];
	CtrlINDI.G_cv[2] = (O2MZ/INERTIA_Z)*k_ve*k_ad[2];


	//---------------------控制输出INDI----------------------//

	float Indi_enb[3];//用于单通道调试
	Indi_enb[0] = 1.0f;
	Indi_enb[1] = 1.0f;
	Indi_enb[2] = 1.0f;
	for (u8 i=0;i<3;i++)
	{
		CtrlINDI.G_cv[i]     = fConstrain(CtrlINDI.G_cv[i],10,200);//限幅为了防止其为0
		CtrlINDI.output_i[i] = -CtrlINDI.pqr_d0[i]/CtrlINDI.G_cv[i] + CtrlINDI.output_0[i];
		CtrlINDI.output_i[i] = fConstrain(Indi_enb[i]*CtrlINDI.output_i[i],-c_m,c_m);
	}

	//---------------------控制输出PID----------------------//

	for (u8 i=0;i<3;i++)
	{
		CtrlLpIO.pqr_err[i] = CtrlLpIO.pqr_command[i] - CtrlFbck.pqr[i];
		float ab[2];
		LineFit(CtrlLpIO.last_pqr_command[i],RATE_SLOPE_NUM,ab);
		CtrlLpIO.pqr_command_d[i] = ab[0]/CtrlDt;
	}
	float k_aa=0.0;
	CtrlINDI.output_f[0] = pidRolRate->PID_Controller(CtrlLpIO.pqr_err[0],CtrlDt)/k_ve + k_aa*CtrlLpIO.pqr_command_d[0]/CtrlINDI.G_cv[0];
	CtrlINDI.output_f[1] = pidPitRate->PID_Controller(CtrlLpIO.pqr_err[1],CtrlDt)/k_ve + k_aa*CtrlLpIO.pqr_command_d[1]/CtrlINDI.G_cv[1];
	CtrlINDI.output_f[2] = pidYawRate->PID_Controller(CtrlLpIO.pqr_err[2],CtrlDt)/k_ve + k_aa*CtrlLpIO.pqr_command_d[2]/CtrlINDI.G_cv[2];


	//---------------------陀螺力矩补偿---------------------//

	Gyro_Control();

	//---------------------输出分配-------------------------//

	for (u8 i=0;i<3;i++)
	{
		CtrlIO.output1[i] = CtrlINDI.output_i[i] + (CtrlIO.mid_trim[i]/((double)RAD_TO_PWM));//+ CtrlLpIO.gyro_output[i]/CtrlINDI.G_cv[i];
//		CtrlIO.output1[i] = pid[i].iout;
//		CtrlIO.output2[i] = CtrlINDI.output_f[i] - CtrlIO.output1[i];
		CtrlIO.output2[i] = CtrlINDI.output_f[i];
//		CtrlIO.output1[i] = CtrlIO.output1[i] + CtrlIO.output2[i];


		CtrlIO.output [i] =   CtrlIO.output1[i] + CtrlIO.output2[i];
		CtrlIO.output [i] = fConstrain(CtrlIO.output[i],-c_m,c_m);
	}

	//---------------------输出滤波-------------------------//

	for (u8 i=0;i<3;i++)
	{
		CtrlINDI.output_0[i] = (CtrlINDI.output_0[i]*(CtrlINDI.Filt_Output[i]-1) + CtrlIO.output[i])/CtrlINDI.Filt_Output[i];
	}

	//------------------------拉力输出-----------------------------------------------
	CtrlIO.output[3] = CtrlLpIO.thrust_command[0];
}

void CONTROL::Output_To_Motor()
{
//	dir.p_limits= (CS_LIMIT_MAX/RAD_TO_PWM)*180.0f/3.1415926535897931f;//单位：度
	//--------------------舵机输出----------------------------------
//	double p_limits= CS_LIMIT_MAX/RAD_TO_PWM;//单位：弧度
	dir_alloc.two_dir_alloc_mch(CtrlIO.output1, CtrlIO.output2, dir_alloc.dir.u);

	// for( int i = 0; i < 4; ++i) { // 陀螺力矩补偿
	// 	dir_alloc.dir.u[i] += gyroTorqueCompensation(i);
	// 	dir_alloc.dir.u[i] = fConstrain(dir_alloc.dir.u[i],-40*D2R,40*D2R);
	// }
	CtrlIO.cs_output[0] = (int) ((dir_alloc.dir.u[0]) * RAD_TO_PWM);
	CtrlIO.cs_output[1] = (int) ((dir_alloc.dir.u[1]) * RAD_TO_PWM);
	CtrlIO.cs_output[2] = (int) ((dir_alloc.dir.u[2]) * RAD_TO_PWM);
	CtrlIO.cs_output[3] = (int) ((dir_alloc.dir.u[3]) * RAD_TO_PWM);
	//--------------------电机输出----------------------------------
	double K_pwm = OneG/((HOVER_PWM-1090)*(HOVER_PWM-1090));
	//-----------------------地面状态积分清零-----------------------------------------
  	if (CtrlIO.FlightStatus == IDLING || CtrlIO.FlightStatus == GROUND || CtrlIO.FlightStatus == GE)
  	{
  		for (u8 i=0;i<12;i++)
  		{
  			pid[i].integral      = 0.0f;
  		}
		for (u8 j=0;j<3;j++)
		{
			CtrlINDI.output_0[j] = 0.0f;
		}
  		CtrlLpIO.u1_Tilt[0] = 0.0f;
  	}

	if(CtrlIO.rc_status == NORMAL || CtrlIO.rc_status == LOST) //已加入未接受到遥控器指令（有Gps）下的失控返航
	{
		CtrlIO.mt_output[0] = (int)(sqrt((CtrlIO.output[3])/K_pwm))+1090;
	}
	else if (CtrlIO.rc_status==CLOSE)
	{
		CtrlIO.mt_output[0] = 1000;
	}
	if(isexceedLimit)CtrlIO.mt_output[0] = (int)(sqrt((0.95*OneG)/K_pwm))+1090;
	if(!isGpsNormal && CtrlIO.control_mode==4)
	{
		CtrlIO.mt_output[0] = (int)(sqrt((0.95*OneG)/K_pwm))+1090;
	}
	if(CtrlIO.FlightStatus==IDLING)
	{
		CtrlIO.mt_output[0] = IDLING_PWM;
	}
	if(CtrlIO.FlightStatus==GE)
	{
		CtrlIO.mt_output[0] = 1090;
	}
	if(CtrlIO.FlightStatus == LAUNCH)
	{
		CtrlIO.mt_output[0] = (int)(sqrt((1.35*OneG)/K_pwm))+1090;
	}
	control_output.mt_output[0] = CtrlIO.mt_output[0];
	control_output.cs_output[0] = CtrlIO.cs_output[0];
	control_output.cs_output[1] = CtrlIO.cs_output[1];
	control_output.cs_output[2] = CtrlIO.cs_output[2];
	control_output.cs_output[3] = CtrlIO.cs_output[3];
	xQueueOverwrite(queueControlOutputDat,&control_output);
}
