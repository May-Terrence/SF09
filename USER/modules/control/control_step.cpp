/*
 * control_step.cpp
 *
 *  Created on: 2020年9月2日
 *      Author: 17900
 */
#include "control_step.hpp"
#include <tran_and_rc/transfer_and_rc.hpp>
#include "path_follow/pathFollow.hpp"
#include "claw_tran/claw_tran.hpp"

CONTROL_STEP control_step;



void CONTROL_STEP::Control_Step()
{
	xQueuePeek(queueRCCommand, &rcCommand, 0);					//从队列中获取遥控器数据
	if(xQueueReceive(queueFlyPoint, &fly_point_overwrite, 0) == pdPASS)	//从队列中获取目标点并删除
	{
//		if(fly_point_overwrite.num != 0)//默认起始点为第一个航点，即数组索引0
		{
			point_list[fly_point_overwrite.num].SetPosition(&fly_point_overwrite.NED_position_m);
			point_list[fly_point_overwrite.num].enable = fly_point_overwrite.enable;
			point_list[fly_point_overwrite.num].SetStayTime(&fly_point_overwrite.stay_time_s);
			point_list[fly_point_overwrite.num].SetSpeed(&fly_point_overwrite.vel_mps);
			point_list[fly_point_overwrite.num].SetYaw(&fly_point_overwrite.yaw_degree);
		}
	}

	//--------------保存上次信息---------//

	CtrlIO.last_control_mode = CtrlIO.control_mode;
	CtrlIO.last_yaw_mode     = CtrlIO.yaw_mode    ;
	CtrlIO.last_fly_mode     = CtrlIO.fly_mode    ;

	for (u8 i=0;i<3;i++)
	{
		for(u8 j=1;j<RATE_SLOPE_NUM;j++)
		{
			CtrlLpIO.last_Pos_command[i][j-1] = CtrlLpIO.last_Pos_command[i][j];
			CtrlLpIO.last_Vel_command[i][j-1] = CtrlLpIO.last_Vel_command[i][j];
			CtrlLpIO.last_Ang_command[i][j-1] = CtrlLpIO.last_Ang_command[i][j];
			CtrlLpIO.last_pqr_command[i][j-1] = CtrlLpIO.last_pqr_command[i][j];
		}
		CtrlLpIO.last_Pos_command[i][RATE_SLOPE_NUM-1] = CtrlLpIO.Pos_command[i];
		CtrlLpIO.last_Vel_command[i][RATE_SLOPE_NUM-1] = CtrlLpIO.Vel_command[i];
		CtrlLpIO.last_Ang_command[i][RATE_SLOPE_NUM-1] = CtrlLpIO.Ang_command[i];
		CtrlLpIO.last_pqr_command[i][RATE_SLOPE_NUM-1] = CtrlLpIO.pqr_command[i];

		CtrlINDI.last_pqr[i] = CtrlFbck.pqr[i];
	}

	//------------------------------------//

	Control_Feedback();       //控制测量反馈
	ControlRC_Check();          //检查遥控器信号
//	Integral_Reset();       //起降重置积分

	//----------------摇杆信号---------------//

	CtrlIO.input[0] = -rcCommand.Val[0];
	CtrlIO.input[1] = -rcCommand.Val[1];
	CtrlIO.input[2] =  rcCommand.Val[2];
	CtrlIO.input[3] =  rcCommand.Val[3];


	//----------------飞行模式---------------//

	if(rcCommand.Key[3]==0 )
	{
		CtrlIO.fly_mode = 0;		//直飞模式
	}
	else if (rcCommand.Key[3]==2 )
	{
//		CtrlIO.control_mode=3;		//平飞模式
	}

	if ( CtrlIO.input[2]>-50 && CtrlIO.input[2]<50 && rcCommand.Key[2]!=0) CtrlIO.yaw_mode = 1;//杆量小于阈值时，开启锁尾，锁定偏航角
	else  	                                                              CtrlIO.yaw_mode = 0;//超出阈值不锁尾


	//----------------控制模式---------------//
	if (CtrlIO.rc_status == NORMAL)
	{
		if(CtrlIO.last_control_mode == 4 && isGpsNormal){ //失控后又可控
			StatusClear();
			rcCommand.Clr_flypoint = true;
		}
		switch(rcCommand.Key[2])
		{
			case 0:
				CtrlIO.control_mode = 0;	//手控
				if(CtrlIO.last_control_mode != 0)StatusClear();
				break;
			case 1:
				CtrlIO.control_mode=1;	//半自控
				if(CtrlIO.last_control_mode != 1)StatusClear();
				break;
			case 2:
				isGpsNormal=true;
				if(rcCommand.Clr_flypoint == true) //清除飞控航点
				{
					for(u8 i=0;i<=fly_point_overwrite.num;i++)
					{
						point_list[i].enable = 0;
					}
					Ctrltrack.flypoint_start  = false;
					Ctrltrack.flycircle_start = false;
				}
				if(rcCommand.Key[3] == 0 )
				{
					CtrlIO.control_mode = 2;//自控模式
					if(claw.isOpen) claw.Close_Request_Tran();
				}
				else if (rcCommand.Key[3] == 2 )
				{
					CtrlIO.control_mode = 3;		//轨迹模式

					if(CtrlIO.last_control_mode != 3){
						xQueuePeek(queuelaserFlow,&laserFlow,0);
						CtrlLpIO.X_pos0 = CtrlFbck.X[0];//仅第一次记录xy当前位置
						CtrlLpIO.Y_pos0 = CtrlFbck.Y[0];
						CtrlLpIO.Z_pos0 = CtrlFbck.Z[0];
						CtrlLpIO.Yaw0   = CtrlFbck.Ang[2];
						CtrlLpIO.end_yaw= CtrlLpIO.Yaw0;
						CtrlLpIO.Z_laser_pos0 = laserFlow.heightFil;

						if(claw.isClose) claw.Open_Request_Tran();
						xQueuePeek(queueClaw, &claw_msg,0);
						CtrlLpIO.X_claw_pos0 = claw_msg.Pos[0];//仅第一次记录xy当前位置
						CtrlLpIO.Y_claw_pos0 = claw_msg.Pos[1];
						CtrlLpIO.Z_claw_pos0 = claw_msg.Pos[2];
					}
					CtrlLpIO.Pos_estimate[0] = CtrlFbck.X[0] + 0.62*CtrlFbck.X[1];// + 0.5*AX*0.62*0.62;//位置预测
					CtrlLpIO.Pos_estimate[1] = CtrlFbck.Y[0] + 0.62*CtrlFbck.Y[1];

//					xQueuePeek(queuelaserFlow,&laserFlow,0);
//					CtrlLpIO.Z_diff_gps = CtrlFbck.Z[0] - CtrlLpIO.Z_pos0;
//					CtrlLpIO.Z_diff_laser = laserFlow.heightFil - CtrlLpIO.Z_laser_pos0;
//
//					Z_laser_err = (laserFlow.heightFil-CtrlLpIO.Z_laser_pos0)-(CtrlFbck.Z[0]-CtrlLpIO.Z_pos0+Z_err_cor);
//					Z_err_cor += pidLaser->PID_Controller(Z_laser_err,CtrlDt);
//					CtrlLpIO.Z_diff = CtrlFbck.Z[0] - CtrlLpIO.Z_pos0 + Z_err_cor;

	// 				if(CtrlIO.last_control_mode != 3)
	// 				{
	// 					CtrlLpIO.Ang_command[2] = CtrlFbck.Ang[2];
	// 					if(point_list[fly_point_overwrite.num].enable) //TODO 不同情况下的判断
	// 					{
	// 						int num = fly_point_overwrite.num+2;
	// 						std::vector<std::vector<float>> waypoints(num, std::vector<float>(3, 0));
	// 						waypoints[0][0] = CtrlFbck.X[0];
	// 						waypoints[0][1] = CtrlFbck.Y[0];
	// 						waypoints[0][2] = CtrlFbck.Z[0];
	// 						for(int i = 0; i < num-1; ++i)
	// 						{
	// 							point_list[i].GetPosition(&position);//获得航点位置信息
	// 							waypoints[i+1][0] = position.data.x;
	// 							waypoints[i+1][1] = position.data.y;
	// //							waypoints[i][2] = position.data.z;
	// 							waypoints[i+1][2] = CtrlFbck.Z[0];
	// 						}
	// 						PATHFOLLOW::path_planing(waypoints, PATHFOLLOW::Dubins);
	// 						//根据航路点和规划方案规划对应路径paths
	// 						TRAN::set_plot_path();//plot_path = true
	// 					}
	// 					else if(return_to_base==false)
	// 					{
	// 						PATHFOLLOW::restart();//state_transition = Start;
	// 					}
	// 				}

				}


//				if(rcCommand.Key[1]==2)
//				{
//					CtrlIO.control_mode=5;		//轨迹模式
//					if(CtrlIO.last_control_mode != 5)
//					{
//						CtrlLpIO.Ang_command[2] = CtrlFbck.Ang[2];
//						if(point_list[fly_point_overwrite.num].enable) //TODO 不同情况下的判断
//						{
//							int num = fly_point_overwrite.num+2;
//							std::vector<std::vector<float>> waypoints(num, std::vector<float>(3, 0));
//							//创建了一个名为 waypoints 的二维浮点型向量，其大小为 num 行，每行包含三个元素，并初始化所有元素为0
//							waypoints[0][0] = CtrlFbck.X[0];
//							waypoints[0][1] = CtrlFbck.Y[0];
//							waypoints[0][2] = CtrlFbck.Z[0];
//							for(int i = 0; i < num-1; ++i)
//							{
//								point_list[i].GetPosition(&position);//获得航点位置信息
//								waypoints[i+1][0] = position.data.x;
//								waypoints[i+1][1] = position.data.y;
//	//							waypoints[i][2] = position.data.z;
//								waypoints[i+1][2] = CtrlFbck.Z[0];
//							}
//							PATHFOLLOW::path_planing(waypoints, PATHFOLLOW::Dubins);
//							//根据航路点和规划方案规划对应路径paths
//							TRAN::set_plot_path();//plot_path = true
//						}
//						else if(return_to_base==false)
//						{
//							PATHFOLLOW::restart();//state_transition = Start;
//						}
//					}
//				}
				break;
			default:break;
		}
	}
	else if(CtrlIO.rc_status == LOST)
	{
		CtrlIO.control_mode = 4;	//失去遥控器控制
	}

	if(CtrlIO.rc_status == CLOSE && slam.Ready_Take_off){
		CtrlIO.control_mode = 2;
		rcCommand.Key[0]    = 2;
		static int take_off_cnt = 0;

		if(++take_off_cnt>600){
			CtrlIO.control_mode = 3;
			slam.Ready_Take_off = false;
			slam.status = Taking_off;
			take_off_cnt = 0;
		}

		if(CtrlIO.last_control_mode != 3){
			xQueuePeek(queuelaserFlow,&laserFlow,0);
			CtrlLpIO.X_pos0 = CtrlFbck.X[0];//仅第一次记录xy当前位置
			CtrlLpIO.Y_pos0 = CtrlFbck.Y[0];
			CtrlLpIO.Z_pos0 = CtrlFbck.Z[0];
			CtrlLpIO.Yaw0   = CtrlFbck.Ang[2];
			CtrlLpIO.end_yaw= CtrlLpIO.Yaw0;
			CtrlLpIO.Z_laser_pos0 = laserFlow.heightFil;

			if(claw.isClose) claw.Open_Request_Tran();
			xQueuePeek(queueClaw, &claw_msg,0);
			CtrlLpIO.X_claw_pos0 = claw_msg.Pos[0];//仅第一次记录xy当前位置
			CtrlLpIO.Y_claw_pos0 = claw_msg.Pos[1];
			CtrlLpIO.Z_claw_pos0 = claw_msg.Pos[2];
		}
	}

	static uint32_t lastTime = 0, gpsHaltCnt;
	xQueuePeek(queueGps, &gps, 0);
	if( gps.timestamp == lastTime ) gpsHaltCnt++;
	else gpsHaltCnt = 0;
	lastTime = gps.timestamp;
	if( ( gpsHaltCnt > 80 && ( CtrlIO.FlightStatus == AIR || CtrlIO.FlightStatus == TAKEOFF || CtrlIO.FlightStatus == LANDING
	|| CtrlIO.FlightStatus == RETURNING ) ) || gps.star >= 40 || gps.status >= 7 ) {  //Gps板子卡死   正常情况下star在0到三十多；status在0到6
		isGpsNormal=false;
		CtrlIO.control_mode = 4;
	}

	if( !isGpsNormal && CtrlIO.rc_status != LOST ) { // 如果Gps信号丢失但是遥控器信号没有丢失，那么就切回手控模式
		if( rcCommand.Key[2] == 0 ) CtrlIO.control_mode = 0;	//手控
		if( rcCommand.Key[2] == 1 ) CtrlIO.control_mode = 1;	//手控
	}
	//----------------控制逻辑----------------//
	switch(CtrlIO.control_mode)
	{
		case 0://手控
			if ((CtrlIO.FlightStatus != AIR)    && (CtrlIO.output[3]>(0.8f*OneG)))
			{
				CtrlIO.FlightStatus = AIR;
			}
			if ((CtrlIO.FlightStatus != GROUND) && (CtrlIO.output[3]<(0.1f*OneG)))
			{
				CtrlIO.FlightStatus = GROUND;
			}
			  CtrlLpIO.pqr_command[0] =  CtrlIO.input[0]*0.225f*D2R;//滚转角速度输入0.225对应90°/s
			  CtrlLpIO.pqr_command[1] =  CtrlIO.input[1]*0.225f*D2R;//俯仰角速度输入0.225对应90°/s
			  CtrlLpIO.pqr_command[2] =  CtrlIO.input[2]*0.225f*D2R;//不锁尾，遥控器输入为航向角速度指令
			  CtrlLpIO.thrust_command[0] = (CtrlIO.input[3]-1500)/(1500-1090)*OneG + OneG;//拉力输入，遥控器1500pwm对应1g拉力

		  break;
		case 1://半自控
			if ((CtrlIO.FlightStatus != AIR)    && (CtrlIO.output[3]>(0.8f*OneG)))
			{
				CtrlIO.FlightStatus = AIR;
			}
			if ((CtrlIO.FlightStatus != GROUND) && (CtrlIO.output[3]<(0.1f*OneG)))
			{
				CtrlIO.FlightStatus = GROUND;
			}
			  CtrlLpIO.Ang_command[0] =  CtrlIO.input[0]*0.05f*D2R;//滚转角度输入0.05对应20°
			  CtrlLpIO.Ang_command[1] =  CtrlIO.input[1]*0.05f*D2R;//俯仰角度输入0.05对应20°
			  CtrlLpIO.thrust_command[0] = (CtrlIO.input[3]-1500)/(1500-1090)*OneG + OneG;//拉力输入，遥控器1500pwm对应1g拉力
			  CtrlLpIO.AngleLimitR       = 20*D2R;//滚转角限幅
			  CtrlLpIO.AngleLimitP       = 20*D2R;//俯仰角限幅
			  In_Loop_Step2();              //角度环
		  break;
		case 2://速度操纵
//			if( rcCommand.Key[3] == 2) {
//				  xQueuePeek(queuelaserFlow, &laserFlow, 0);
//				  CtrlFbck.Z[0] = -laserFlow.heightFil;
//			}
			if(CtrlIO.last_control_mode == 3) {
				CtrlIO.FlightStatus = GROUND;
				rcCommand.OneKeyTakeoff = false;
				rcCommand.OneKeyLanding = false;
				isReady = true;
			}

			Out_Loop_XY_Pre2();           //水平速度给定，判断定点模式
			Out_Loop_Z_Pre2();            //高度给定
			CtrlLpIO.AngleLimitR       = 30*D2R;//滚转角限幅
			CtrlLpIO.AngleLimitP       = 30*D2R;//俯仰角限幅

			Out_Loop_Step2();
			if (CtrlIO.FlightStatus == GE)
			{
				CtrlLpIO.Ang_command[0] = 0.0f;
				CtrlLpIO.Ang_command[1] = 0.0f;
			}

			In_Loop_Step2();              //角度环
		  	break;
		case 3://全自控，航点

			//原模式三
//			  if(slam.Ready_Take_off && claw.isClose) claw.Open_Request_Tran();
//			  if(slam.Ready_Take_off && claw.isOpen && !rcCommand.TakeOffFinish) OneKeyTakeOff();
			  if(rcCommand.OneKeyTakeoff == true)  OneKeyTakeOff();
			  if((slam.status == Taking_off || slam.status == Air) && CtrlIO.FlightStatus != AIR) OneKeyTakeOff();
			  if(CtrlIO.FlightStatus == AIR && rcCommand.OneKeyLanding == false) Auto_flypoint();
//			  if(CtrlIO.FlightStatus == AIR && rcCommand.OneKeyLanding == false) Auto_flycircle();//飞圆，测试前请确保磁力计数据正常
//			  if(rcCommand.OneKeyLanding == true)  OneKeyLanding(0,0,false,true);
			  if(rcCommand.OneKeyLanding == true){
				  if(rcCommand.Key[1]==2){
					  if(isReady){
						  xQueuePeek(queueClaw, &claw_msg,0);
						  if(claw.isUpdate && claw.noMove){
							  CtrlLpIO.enable_Grab_flag = true;
							  CtrlLpIO.end_command[0] = CtrlLpIO.X_pos0 + claw_msg.Pos[0] - CtrlLpIO.X_claw_pos0;
							  CtrlLpIO.end_command[1] = CtrlLpIO.Y_pos0 + claw_msg.Pos[1] - CtrlLpIO.Y_claw_pos0;
							  CtrlLpIO.end_command[2] = CtrlLpIO.Z_pos0 + claw_msg.Pos[2] - CtrlLpIO.Z_claw_pos0;
							  CtrlLpIO.end_yaw = claw_msg.Yaw*D2R;
							  isReady = false;
						  }
					  }
					  if(!isReady) OneKeyLanding(CtrlLpIO.end_command[0],CtrlLpIO.end_command[1],true,true);
				  }
				  else
					  OneKeyLanding(0,0,false,true);
			  }
			  if(slam.Ready_Land && CtrlIO.FlightStatus != GE){
//				  if(isReady){
//					  xQueuePeek(queueClaw, &claw_msg,0);
//					  if(claw.isUpdate && claw.noMove){
//						  CtrlLpIO.enable_Grab_flag = true;
//						  CtrlLpIO.end_command[0] = CtrlLpIO.X_pos0 + claw_msg.Pos[0] - CtrlLpIO.X_claw_pos0;
//						  CtrlLpIO.end_command[1] = CtrlLpIO.Y_pos0 + claw_msg.Pos[1] - CtrlLpIO.Y_claw_pos0;
//						  CtrlLpIO.end_command[2] = CtrlLpIO.Z_pos0 + claw_msg.Pos[2] - CtrlLpIO.Z_claw_pos0;
//						  CtrlLpIO.end_yaw = claw_msg.Yaw*D2R;
//						  isReady = false;
//					  }
//				  }
//				  if(!isReady) OneKeyLanding(CtrlLpIO.end_command[0],CtrlLpIO.end_command[1],true,true);
				  OneKeyLanding(0,0,false,true);
			  }

//			  trackPath();
//			  frog_jump();

			  CtrlLpIO.AngleLimitR       = 30*D2R;//滚转角限幅
			  CtrlLpIO.AngleLimitP       = 30*D2R;//俯仰角限幅
			  Out_Loop_Step2();
			  if (CtrlIO.FlightStatus == GE){
				CtrlLpIO.Ang_command[0] = 0.0f;
				CtrlLpIO.Ang_command[1] = 0.0f;
			  }
		      In_Loop_Step2();              //角度环
		      if(CtrlIO.rc_status == CLOSE && slam.status != NONE) rcCommand.Key[0] = 2;

		  break;
		case 4://失控保护
			if(!isGpsNormal){ //没有Gps信号则原地降落
				  CtrlLpIO.Ang_command[0] = 0.0f;      //滚转角度输入0
				  CtrlLpIO.Ang_command[1] = 0.0f;      //俯仰角度输入0
				  CtrlLpIO.AngVel_command[2] =0.0f;
			}
			if(isGpsNormal){ //若有Gps则自动返航
			  OneKeyLanding(0,0,false,false);
			  CtrlLpIO.AngleLimitR       = 20*D2R;//滚转角限幅
			  CtrlLpIO.AngleLimitP       = 20*D2R;//俯仰角限幅
			  Out_Loop_Step2();
			  if (CtrlIO.FlightStatus == GE){
				CtrlLpIO.Ang_command[0] = 0.0f;
				CtrlLpIO.Ang_command[1] = 0.0f;
			  }
			}
	      In_Loop_Step2();              //角度环
		  break;
		case 5://dubins
			CtrlLpIO.AngleLimitR       = 20*D2R;//滚转角限幅
			CtrlLpIO.AngleLimitP       = 20*D2R;//俯仰角限幅
//			trackPath(); //
			frog_jump();
			Out_Loop_Step2();
			if (CtrlIO.FlightStatus == GE){
				CtrlLpIO.Ang_command[0] = 0.0f;
				CtrlLpIO.Ang_command[1] = 0.0f;
			}
		    In_Loop_Step2();              //角度环
			break;
		case 6:
		break;
		case 7: //gps板子卡死，没有位置信息
		break;
		default:break;
	}
	virtualFence(); //限高
	rcCommand.Clr_flypoint   = false;
  	xQueueOverwrite(queueRCCommand,&rcCommand);
}

void CONTROL_STEP::Control_Step2()
{
	switch(CtrlIO.control_mode)
	{
		case 0://手控
//		  Control_Output();           //角速度环（PID）
		  Control_Output_INDI();
		  break;
		case 1://半自控
//		  Control_Output();              //角速度环（PID）
		  Control_Output_INDI();
		  break;
		case 2://速度操纵
//		  Control_Output();
		  Control_Output_INDI();
		  break;
		case 3://轨迹
//		  Control_Output();
		  Control_Output_INDI();
		  break;
		case 4://失控保护
//		  Control_Output();              //角速度环（PID）
		  Control_Output_INDI();
		  break;
		case 5://
		  Control_Output_INDI();
		  break;
		case 6://
		  Control_Output_INDI();
		  break;
		case 7://
		  Control_Output_INDI();
		  break;
		default:break;
	}

	Output_To_Motor();          //控制输出到执行器
//	Integral_Restore();

}
void CONTROL_STEP::PID_Para_Update(void)
{
	xQueuePeek(queuePID, &pid_msg, 0);
	for(u8 i=0;i<18;i++)
		pid[i].Pid_Set_Para(pid_msg.kp[i], pid_msg.ki[i], pid_msg.kd[i]);
	//调试信号
	CtrlIO.mid_trim[0] = pid[14].Kp;
	CtrlIO.mid_trim[1] = pid[14].Ki;
	CtrlIO.mid_trim[2] = pid[14].Kd;

//	CtrlLpIO.ko1 = pid[15].Kp;
//	CtrlLpIO.VelSet = pid[15].Ki;


	CtrlLpIO.ko1	 = pid[17].Kp;
	CtrlLpIO.VelSet 		 = pid[17].Ki;
	CtrlLpIO.relative_height = pid[17].Kd;

}

void CONTROL_STEP::Tranfer_Data_Updata(void)
{
	control_data.Ang[0] = CtrlFbck.Ang[0];
	control_data.Ang[1] = CtrlFbck.Ang[1];
	control_data.Ang[2] = CtrlFbck.Ang[2];

	control_data.pqr[0] = CtrlFbck.pqr[0];
	control_data.pqr[1] = CtrlFbck.pqr[1];
	control_data.pqr[2] = CtrlFbck.pqr[2];

	control_data.X[0] = CtrlFbck.X[0];
	control_data.X[1] = CtrlFbck.X[1];
	control_data.Y[0] = CtrlFbck.Y[0];
	control_data.Y[1] = CtrlFbck.Y[1];
	control_data.Z[0] = CtrlFbck.Z[0];
	control_data.Z[1] = CtrlFbck.Z[1];

	control_data.XH[0] = CtrlFbck.XH[0];
	control_data.XH[1] = CtrlFbck.XH[1];
	control_data.YH[0] = CtrlFbck.YH[0];
	control_data.YH[1] = CtrlFbck.YH[1];

	control_data.X_command[0] = CtrlLpIO.Pos_command[0];
	control_data.X_command[1] = CtrlLpIO.Vel_command[0];
	control_data.Y_command[0] = CtrlLpIO.Pos_command[1];
	control_data.Y_command[1] = CtrlLpIO.Vel_command[1];
	control_data.Z_command[0] = CtrlLpIO.Pos_command[2];
	control_data.Z_command[1] = CtrlLpIO.Vel_command[2];

	control_data.AngVel_command[0] = CtrlLpIO.AngVel_command[0];
	control_data.AngVel_command[1] = CtrlLpIO.AngVel_command[1];
	control_data.AngVel_command[2] = CtrlLpIO.AngVel_command[2];

	control_data.Acc_command[0] = CtrlLpIO.Acc_command[0];
	control_data.Acc_command[1] = CtrlLpIO.Acc_command[1];
	control_data.Acc_command[2] = CtrlLpIO.Acc_command[2];
	control_data.rc_status = CtrlIO.rc_status;
	control_data.brake_mode = CtrlLpIO.brake_mode;

	control_data.mt_output[0] = CtrlIO.mt_output[0];
	control_data.mt_output[1] = CtrlLpIO.u1_Tilt[0];

	control_data.output[0] = CtrlIO.output[0];
	control_data.output[1] = CtrlIO.output[1];
	control_data.output[2] = CtrlIO.output[2];
	control_data.output[3] = CtrlIO.output[3];
	control_data.output[4] = CtrlIO.output[4];

	control_data.output1[0] = CtrlIO.output1[0];
	control_data.output1[1] = CtrlIO.output1[1];
	control_data.output1[2] = CtrlIO.output1[2];

	control_data.pitch_command[0] = CtrlLpIO.Ang_command[1];
	control_data.pitch_command[1] = CtrlLpIO.Ang_command_d[1];
	control_data.roll_command[0] = CtrlLpIO.Ang_command[0];
	control_data.roll_command[1] = CtrlLpIO.Ang_command_d[0];

	control_data.yaw_command[0] = CtrlLpIO.Ang_command[2];
	control_data.yaw_command[1] = CtrlLpIO.Ang_command_d[2];

	control_data.pqr_command[0] = CtrlLpIO.pqr_command[0];
	control_data.pqr_command[1] = CtrlLpIO.pqr_command[1];
	control_data.pqr_command[2] = CtrlLpIO.pqr_command[2];

	control_data.pqr_d0[0] = CtrlINDI.pqr_d0[0];
	control_data.pqr_d0[1] = CtrlINDI.pqr_d0[1];
	control_data.pqr_d0[2] = CtrlINDI.pqr_d0[2];

	control_data.pqr_d_raw[0] = CtrlINDI.pqr_d_raw[0];
	control_data.pqr_d_raw[1] = CtrlINDI.pqr_d_raw[1];
	control_data.pqr_d_raw[2] = CtrlINDI.pqr_d_raw[2];

	control_data.control_mode = CtrlIO.control_mode;
	control_data.FlightStatus = CtrlIO.FlightStatus;
	control_data.TranStatus   = Ctrltrack.PTP_Status;

	control_data.XY_phase=CtrlLpIO.XY_phase;
	control_data.Z_phase=CtrlLpIO.Z_phase;
	control_data.XYZ_phase=CtrlLpIO.XYZ_phase;
	control_data.Jerk_command[0]=CtrlLpIO.Jerk_command[0];
	control_data.Jerk_command[1]=CtrlLpIO.Jerk_command[1];

	control_data.Pos_estimate[0] = CtrlLpIO.Pos_estimate[0];
	control_data.Pos_estimate[1] = CtrlLpIO.Pos_estimate[1];
	control_data.enable_Grab_flag = CtrlLpIO.enable_Grab_flag;
	control_data.X_err_estimate = CtrlLpIO.X_err_estimate;
	control_data.Y_err_estimate = CtrlLpIO.Y_err_estimate;
	control_data.XY_err_estimate = CtrlLpIO.XY_err_estimate;

	control_data.X_pos0 = CtrlLpIO.X_pos0;
	control_data.Y_pos0 = CtrlLpIO.Y_pos0;
	control_data.Z_pos0 = CtrlLpIO.Z_pos0;

//	control_data.Z_diff_gps = CtrlLpIO.Z_diff_gps;
//	control_data.Z_diff_laser = CtrlLpIO.Z_diff_laser;
//	control_data.Z_diff = CtrlLpIO.Z_diff;

	control_data.end_command[0] = CtrlLpIO.end_command[0];
	control_data.end_command[1] = CtrlLpIO.end_command[1];
	control_data.end_yaw = CtrlLpIO.end_yaw;

	xQueueOverwrite(queueControlTransfer,&control_data);
}

extern "C" void control_main(void *argument)
{
	static u32 cnt = 0;
	control_step.Control_Init();
	control_step.return_to_base = false;

	osDelay(1500);
	for(;;)
	{
		osSemaphoreAcquire(semControl,0xffffffff);

		control_step.startTimerLast = control_step.startTimer;
		getTimer_us(&control_step.startTimer);
		control_step.cycleTime_us = control_step.startTimer - control_step.startTimerLast;

		control_step.Control_Step();
		control_step.Control_Step2();

		if(cnt%200)
			control_step.PID_Para_Update();
		if(cnt%10)
			slam.Position_Transfer();
		if(cnt%4)
			control_step.Tranfer_Data_Updata();
		cnt++;

		getTimer_us(&control_step.stopTimer);
		control_step.executionTime_us = control_step.stopTimer - control_step.startTimer;
	}
}
