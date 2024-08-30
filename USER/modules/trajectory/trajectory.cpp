/*
 * trajectory.cpp
 *
 *  Created on: 2020年12月2日
 *      Author: 刘成吉
 */
#include "trajectory.hpp"

TRAJECTORY trajectory;

float circle_radius_test = 10.0f;
float brake_distance = 0.0f;
float circle_roll_angle = 0.0f;
int num = 0;
bool Mixture_to_Pointtopoint_flag = false;

//-------------2015/6/3-------------------
unsigned char auto_pilot11 = 0;
//----------------------------------------
void TRAJECTORY::Trajectory_Step(int trajectory_mode)
{
	switch (trajectory_mode)
	{
		case Hover : Trajectory_Hover_Mode_Step();//
		break;

		case Point_to_point : Trajectory_Point_to_point_Mode_Step();//
		break;

		case Circle : Trajectory_Circle_Mode_Step();//
		break;

		case Mixture : Trajectory_Mixture_Mode_Step();//
		break;

		case Test : Trajectory_Test_Mode_Step();//
		break;

		case RTL : Trajectory_RTL_Mode_Step(); //
		break;

		default : Trajectory_Brake_Down_Step();	//
		break;
	}
}
void TRAJECTORY::Trajectory_Target_Receive()
{

	xQueuePeek(queueTargetBuffer, &target_buffer, 0);
//	xQueuePeek(queueRCCommand, &rcCommand, 0);

//	if(target_buffer.receive_flag || rcCommand.Frist_Entry_TakeOff)//用轨迹规划实现一键起降

	if(target_buffer.receive_flag)
	{
		xQueuePeek(queueAhrsEuler, &ahrsEuler, 0);

		if (Target_Queue_Push_Target() == false)
			Trajectory_Brake_Down_Step();
		first_receive_target ++;
		target_buffer.count --;
		target_buffer.receive_flag = false;
		xQueueOverwrite(queueTargetBuffer,&target_buffer);
	}
}

void TRAJECTORY::Pos_Hold_Init()
{

	xQueuePeek(queueTargetBuffer, &target_buffer, 0);
	xQueuePeek(queueESKF,&eskf,0);
	xQueuePeek(queueHeight,&height,0);

//	yaw_command = ahrsEuler.Ang[2];
	yaw_command = eskf.Attitude[2];
	target_buffer.receive_flag = false;

	Target_Command_Reset();//清除TargetBuffer

	Target_Queue_Reset();

	Trajectory_Reset();

	xQueueOverwrite(queueTargetBuffer,&target_buffer);
}

//----------------------------------------

void TRAJECTORY::Trajectory_Reset()
{
	trajectory_mod = Point_to_point;
	heli_step = 1;
	RTL_Orignal_flag = 0; //RTL
	RTL_Orignal_update_flag = 1;//RTL
	RTL_First_switch = 1;//RTL
	end_heading = ahrsEuler.Ang[2];
	hover_time = FLY_TIME;
	Isturn = 0;
	first_receive_target = 0;
	turn_accel = 0;
	turn_accel_flag = false;
	first_entry_brake = 1;
	first_entry_circle = 1;
	circle_velocity = 1.0f;//圆弧轨线速度
	circle_inner_flag = 0;
	Horizontal_tracing_state = 0;//水平跟踪标志位：0未进入目标点半径 1已进入
	Vertical_tracing_state = 0;//竖直跟踪标志位
	PointFinish = 0;

	hover_ned[0] = eskf.Pos[0];//用的是NED
	hover_ned[1] = eskf.Pos[1];//用的是NED
	hover_ned[2] = eskf.Pos[2];
//	hover_ned[2] = -height.height*0.01;//用的是NED

	for(int i =0 ; i!= 3 ;  ++i)
	{
//		hover_ned[i] = eskf.Pos[i];//用的是NED
		next_target_ned[i] = hover_ned[i];
		cur_target_pos_ned[i] = hover_ned[i];
		cur_target_vel_ned[i] = 0.0f;
		com_pqr[i]      = 0.0f;
		com_theta[i]    = 0.0f;
		top_speed_body[i] = 0.0f;
		how_long_body[i] = 0.0f;
	}

	target_buffer.receive_flag = false;
}
void TRAJECTORY::Trajectory_Current_Target_Update(float *next_target_ned)
{
	for(int i=0 ; i!=3 ; ++i)
		cur_target_pos_ned[i] = next_target_ned[i];
}

//void ahrs_adjust(float *theta,float *angle_offset)
//{
//	for (int i = 0;i < 3;i++)
//	{
//		angle_offset[i] = angle_offset[i]*C_DEG2RAD;
//	}
//	for (int j = 0;j < 3;j++)
//	{
//		theta[j] -= angle_offset[j];
//	}
//}

void TRAJECTORY::Target_Plus_Hover_Point()
{

//匿名V7上位机用的LLH坐标系
	for(int i=0;i<3;i++)
	{
		next_target_ned[i] = next_target_ned[i] + hover_ned[i];
	}

}


bool TRAJECTORY::Trajectory_Target_Update(int trajectory_mod)//轨迹目标更新
{
	if (trajectory_mod == Point_to_point)
	{
		bool queue_status = Target_Queue_Is_Queue_Empty();//队列不空queue_status=0
//		if ((hover_time <= 1 && heli_step == 1
//				&& (!queue_status)) || first_receive_target == 1)
//		if ((hover_time <= 1 && PointFinish == 1
//					&& (!queue_status)) || first_receive_target == 1)
		if (((queue_status == 0) && (PointFinish == 1)) || first_receive_target == 1)
		{
			Horizontal_tracing_state = 0;
			Vertical_tracing_state = 0;
			PointFinish = 0;
			if(!Target_Queue_Pop_Target(next_target_ned,&hover_time))//如果没有下一个目标点
				return false;
			Target_Plus_Hover_Point();
			first_receive_target++;
			return true;
		}
		if (queue_status && hover_time <= -2000)
		{
			next_target_ned[0] = hover_ned[0] ;
			next_target_ned[1] = hover_ned[1] ;
			next_target_ned[2] = hover_ned[2] ;
			hover_time = DEFAULT_HOVER_TIME ;
			return true;
		}
		PointFinish = 0;
		xQueuePeek(queueTargetBuffer, &target_buffer, 0);
		for(int i = 0; i<3; i++)
		{
			target_buffer.next_target_ned[i] = next_target_ned[i];//观测值发回地面站
		}
		xQueueOverwrite(queueTargetBuffer,&target_buffer);
	}

	if (trajectory_mod == Circle)
	{
		if ((!Target_Queue_Is_Queue_Empty()) && first_receive_target == 1)
		{
			if(!Target_Queue_Pop_Target(next_target_ned,&hover_time))
			return false;
			Target_Plus_Hover_Point();
			first_receive_target++;
			return true;
		}
		if ((!Target_Queue_Is_Queue_Empty()) && circle_finish_flag == true && hover_time <=-1 )
		{
			if(!Target_Queue_Pop_Target(next_target_ned,&hover_time))
			return false;
			Target_Plus_Hover_Point();
			circle_finish_flag = false;
			return true;
		}
/*
if (target_queue_t->is_queue_empty() && hover_time <= -2000)
//,1
{
	ele->next_target_ned[0] = ele->hover_ned[0] ;
	ele->next_target_ned[1] = ele->hover_ned[1] ;
	ele->next_target_ned[2] = ele->hover_ned[2] ;
	ele->hover_time = DEFAULT_HOVER_TIME ;
	return true;
}
*/
	}

	if (trajectory_mod == Mixture)
	{
		if (circle_finish_flag == true)
		{
//			for (int i=0; i!=5; i++)
			for (int i=0; i!=4; i++)
			{
				next_target_ned[i] = next_next_target[i];
			}
			if(!Target_Queue_Pop_Target(next_next_target,&hover_time))
			{
				heli_step = 8;
				return false;
			}
			Target_Plus_Hover_Point();
			return true;
		}
		if ((!Target_Queue_Is_Queue_Empty()) && first_receive_target == 1)
		{
			if(!Target_Queue_Pop_Target(next_target_ned,&hover_time))
				return false;
			Target_Plus_Hover_Point();
			if(!Target_Queue_Pop_Target(next_next_target,&hover_time))
				return false;
			Target_Plus_Hover_Point();
			first_receive_target++;
			return true;
		}
	}
	return false;
}

void TRAJECTORY::Trajectory_Distance_Calculate(int trajectory_mod)
{
	if (trajectory_mod == Point_to_point || trajectory_mod == Circle || trajectory_mod == RTL)
	{
		how_long_body[0] = sqrt(SQR(cur_target_pos_ned[0]-next_target_ned[0])+ SQR(cur_target_pos_ned[1]-next_target_ned[1]));
//		how_long_body[0] = cur_target_pos_ned[0]-next_target_ned[0];//
//		how_long_body[1] = cur_target_pos_ned[1]-next_target_ned[1];//
		how_long_body[2] = cur_target_pos_ned[2]-next_target_ned[2];
		vertical_speed_direction = how_long_body[2] >= 0?-1:1;//-1  1
		distance_line = how_long_body[0];
		distance_vertical = fabs(how_long_body[2]);
		return;
	}
	if (trajectory_mod == Mixture)
	{
	/*v1(x1, y1)v2(x2, y2)v1譾2=x1y2-y1x2
	v1v2,,
	0()*/
		float cross_product = (next_target_ned[0]-cur_target_pos_ned[0])*(next_next_target[1]-cur_target_pos_ned[1])
			- (next_target_ned[1]-cur_target_pos_ned[1])*(next_next_target[0]-cur_target_pos_ned[0]);
		if (cross_product == 0)
		{
			for (int i=0; i<4; i++)
			{
				next_target_ned[i] = next_next_target[i];
			}
			if(!Target_Queue_Pop_Target(next_next_target,&hover_time))
			{
				heli_step = 8;
				return;
			}
			Target_Plus_Hover_Point();
		}
		else
		{
			if (cross_product < 0)
			{
				circle_turn = 2;	//,
			}
			else
			{
				circle_turn = 1;	//,
			}
		}

		how_long_body[0] = sqrt((cur_target_pos_ned[0]-next_target_ned[0])*(cur_target_pos_ned[0]-next_target_ned[0])
					+ (cur_target_pos_ned[1]-next_target_ned[1])*(cur_target_pos_ned[1]-next_target_ned[1]));
		how_long_body[2] = hover_ned[2]-next_target_ned[2];

		float b = sqrt((next_target_ned[0]-next_next_target[0])*(next_target_ned[0]-next_next_target[0])
						+ (next_target_ned[1]-next_next_target[1])*(next_target_ned[1]-next_next_target[1]));
		float c = sqrt((cur_target_pos_ned[0]-next_next_target[0])*(cur_target_pos_ned[0]-next_next_target[0])
						+ (cur_target_pos_ned[1]-next_next_target[1])*(cur_target_pos_ned[1]-next_next_target[1]));

		sector_angle = acos((pow(how_long_body[0],2)+pow(b,2)-pow(c,2)) / 2*fabs(how_long_body[0]*b));
		circle_radius = 2*circle_velocity;
		if (sector_angle >= 0.785f && sector_angle <= 2.356f)//60-120
		{
			float r;
			sector_angle = PI - sector_angle;
			r = circle_radius*tan(sector_angle/2);
			how_long_body[0] = how_long_body[0] - r;
			next_target_ned[0] -= r*cos(ahrsEuler.Ang[2]);
			next_target_ned[1] -= r*sin(ahrsEuler.Ang[2]);
			return;
		}
		else
		{
			trajectory_mod = Point_to_point;
			Mixture_to_Pointtopoint_flag = true;
			return;
		}
	}
}


void TRAJECTORY::Trajectory_Speed_Calculate(float max_com_speed, int trajectory_mod)
/*
 * 功能	计算速度
 *
 * 输入	how_long_body
 *
 * 输出	top_speed_body[0]	机身水平速度矢量大小，XY方向的矢量和
 * 		top_speed_body[1]	机身垂直方向矢量大小
 */
{
	if (trajectory_mod == Point_to_point || trajectory_mod == RTL )
	{
		if (2.0f*(SQR(max_com_speed))/ACCEL > how_long_body[0]) //1  大于5m
		{
			top_speed_body[0] = sqrt(ACCEL * how_long_body[0]/2.0f );// 1/4
		}
		else
		{
			top_speed_body[0] = max_com_speed;//0.5
		}
		if (2.0f*SQR(MAX_COM_SPEED_VERTICAL)/ACCEL_VERTICAL > distance_vertical) //1 大于5m
		{
			top_speed_body[2] = sqrt(ACCEL_VERTICAL * distance_vertical/2.0f);// 1/4
		}
		else
		{
			top_speed_body[2] = MAX_COM_SPEED_VERTICAL;
		}
		return;
	}
	if (trajectory_mod == Circle)
	{
		if (0.5f*(max_com_speed*max_com_speed)/ACCEL > how_long_body[0])
		{
			top_speed_body[0] = 0;
		}
		else
		{
			top_speed_body[0] = max_com_speed;
		}
		return;
	}
//	if (ele->trajectory_mod == Mixture)
//	{
//		if (ele->circle_finish_flag == false)
//		{
//			if (0.5f*(max_com_speed*max_com_speed)/ACCEL > ele->how_long_body[0])
//			{
//				ele->top_speed_body[0] = 0;
//			}
//			else
//			{
//				ele->top_speed_body[0] = max_com_speed;
//			}
//			return;
//		}
//		else
//		{
//			ele->top_speed_body[0] = max_com_speed;
//		}

//	}
}


void TRAJECTORY::Trajectory_Schedule(float max_speed, int mode)  //next_targe
{
	Trajectory_Distance_Calculate(mode);              //ele->how_long_body
	Trajectory_Speed_Calculate(max_speed, mode);                //ele->top_speed_body
}

//  ,--ele->end_heading--ele->Isturn
// end_heading_point ;
//n_target ;current_heading ;current_xyz
void TRAJECTORY::Trajectory_Turn_Heading(float * end_heading_point)
{
	float temp = atan2f(next_target_ned[0]-cur_target_pos_ned[0],
								 next_target_ned[1]-cur_target_pos_ned[1]);
	// atan2X,end_headingNED
	if ( temp > -PI /2.0f )//计算目标点与当前点的方位角（-PI to PI）
		*end_heading_point = PI /2.0f - temp ;
	else
		*end_heading_point = - 3.0f * PI /2.0f - temp ;

	if ( *end_heading_point - yaw_command > PI )//eskf.Attitude[2]
	{
		Isturn = 1;     //
		return;
	}
	if ( *end_heading_point - yaw_command < -PI )
	{
		Isturn = 2;     //
		return;
	}
	if (*end_heading_point - yaw_command > 0.00f && *end_heading_point - yaw_command < PI)
	{
		Isturn = 3;     //
		return;
	}
	if (*end_heading_point - yaw_command < 0.00f && *end_heading_point - yaw_command > -PI)
	{
	Isturn = 4;    //
	return;
	}
}
//-------------------------------syc--2012.4.13--------------------//
void TRAJECTORY::Trajectory_Turning_Head_Step()//转头
{
	if ((fabs(end_heading - yaw_command)) < 0.04f )  //0.04rad 2.3deg
	{
		Isturn = 0 ;
		yaw_command = end_heading;
		turn_accel_flag=true;
		return;
	}
	if ( Isturn == 1 || Isturn ==4 )//end_heading-yaw_command大于PI或大于-PI小于0
	{ //
		yaw_command -= 0.003f; //by syc
		if (yaw_command< - PI )
			yaw_command +=2* PI;
		return;
	}
	if ( Isturn == 2 || Isturn ==3 )//end_heading-yaw_command小于-PI或大于0小于PI
	{ //
		yaw_command += 0.003f;
		if (yaw_command > PI )
			yaw_command -=2* PI;
		//position_t->heading += 0.025 ;
		return;
	}
}

void TRAJECTORY::Trajectory_Speed_Up_Step()
{
	if (cur_target_vel_body[0] < top_speed_body[0])
	{
		cur_target_vel_body[0] +=  CONTROL_DT * ACCEL ;
	}
	else
	{
		cur_target_vel_body[0] =  top_speed_body[0] ;
	}

	how_long_body[0] -= cur_target_vel_body[0] * CONTROL_DT * 1.0f;
	cur_target_vel_ned[0] = cur_target_vel_body[0] * cos(yaw_command);
	cur_target_vel_ned[1] = cur_target_vel_body[0] * sin(yaw_command);

	cur_target_pos_ned[0]+= cur_target_vel_ned[0] * CONTROL_DT;
	cur_target_pos_ned[1]+= cur_target_vel_ned[1] * CONTROL_DT;

//	#ifdef circle_test
//	if(ele->cur_target_vel_body[0] >= 0.5)
//	{
//		ele->heli_step = 9;
//	  return ;
//	}
//	#endif
}


void TRAJECTORY::Trajectory_Speed_Down_Step()
{
	if ( cur_target_vel_body[0] > 0.1f )
	{
		cur_target_vel_body[0] -=  CONTROL_DT *ACCEL ;
	}
	else
	{
		cur_target_vel_body[0] = 0.1f;
	}
	how_long_body[0] -= cur_target_vel_body[0] * CONTROL_DT;
	cur_target_vel_ned[0] = cur_target_vel_body[0] * cos(yaw_command);
	cur_target_vel_ned[1] = cur_target_vel_body[0] * sin(yaw_command);

	cur_target_pos_ned[0]+= cur_target_vel_ned[0] * CONTROL_DT;
	cur_target_pos_ned[1]+= cur_target_vel_ned[1] * CONTROL_DT;

}

void TRAJECTORY::Trajectory_Speed_Up_Step_Vertical()
{
	if(how_long_body[2] <0 )//要下降
	{
		if (cur_target_vel_body[2] < top_speed_body[2])
		{
			cur_target_vel_body[2] +=  CONTROL_DT * ACCEL_VERTICAL ;
		}
		else
		{
			cur_target_vel_body[2] =  top_speed_body[2] ;
		}
	}
	else//要上升
	{
		if (cur_target_vel_body[2] > -1.0f * top_speed_body[2])//当前速度如果大于最大速度（速度向下为正）
		{
			cur_target_vel_body[2] -=  CONTROL_DT * ACCEL_VERTICAL ;//继续增大向上速度
		}
		else
		{
			cur_target_vel_body[2] = -1.0f * top_speed_body[2] ;
		}
	}

		how_long_body[2] += cur_target_vel_body[2]*CONTROL_DT;
		cur_target_vel_ned[2] = cur_target_vel_body[2];
		cur_target_pos_ned[2] += cur_target_vel_ned[2]*CONTROL_DT;
//	if(vertical_speed_direction == 1)//-1  1    要上升时值为-1
//	{
//		if (cur_target_vel_body[2] < top_speed_body[2])
//		{
//			cur_target_vel_body[2] +=  CONTROL_DT * ACCEL_VERTICAL ;
//		}
//		else
//		{
//			cur_target_vel_body[2] =  top_speed_body[2] ;
//		}
//
//		how_long_body[2] += cur_target_vel_body[2]*CONTROL_DT;
//		cur_target_vel_ned[2] = cur_target_vel_body[2];
//		cur_target_pos_ned[2] -= cur_target_vel_ned[2]*CONTROL_DT;
//	}
//	else if(vertical_speed_direction == -1)//-1  1
//	{
//		if (cur_target_vel_body[2] > -1.0f * top_speed_body[2])//当前速度如果大于最大速度（方向向上，负数）
//		{
//			cur_target_vel_body[2] -=  CONTROL_DT * ACCEL_VERTICAL ;//继续增大向上速度
//		}
//		else
//		{
//			cur_target_vel_body[2] =  -1.0f * top_speed_body[2] ;
//		}
//
//		how_long_body[2] += cur_target_vel_body[2]*CONTROL_DT;//更新剩余距离（计算）
//		cur_target_vel_ned[2] = cur_target_vel_body[2];//机身速度就是北东地速度
////		cur_target_pos_ned[2] -= cur_target_vel_ned[2]*CONTROL_DT;
//		cur_target_pos_ned[2] += cur_target_vel_ned[2]*CONTROL_DT;//更新北东地期望值
//	}
}

void TRAJECTORY::Trajectory_Speed_Down_Step_Vertical()
{
	if(how_long_body[2] <0 )//在下降 速度为正 要减速为零
	{
		if ( cur_target_vel_body[2] > 0.1f)
		{
			cur_target_vel_body[2] -=  CONTROL_DT *ACCEL_VERTICAL ;
		}
		else
		{
			cur_target_vel_body[2] = 0.1f;
		}

	}
	else//在上升 速度为负 要增加到零
	{
		if (cur_target_vel_body[2] < -0.1f)
		{
			cur_target_vel_body[2] +=  CONTROL_DT *ACCEL_VERTICAL ;
		}
		else
		{
			cur_target_vel_body[2] = -0.1f;
		}
	}

		how_long_body[2] += cur_target_vel_body[2]*CONTROL_DT;
		cur_target_vel_ned[2] = cur_target_vel_body[2];
		cur_target_pos_ned[2] += cur_target_vel_ned[2]*CONTROL_DT;
}


void TRAJECTORY::Trajectory_Land_Step()
{
	cur_target_vel_body[2] = 0.1f;
	how_long_body[2] += cur_target_vel_body[2]*CONTROL_DT;
	cur_target_vel_ned[2] = cur_target_vel_body[2];
	cur_target_pos_ned[2] -= cur_target_vel_ned[2];
}
bool TRAJECTORY::Trajectory_Climbing_Step()
{

	return false;
}

void TRAJECTORY::Trajectory_Circle_Step(float _circle_velocity, float _radius,float _helix_velocity, float _angle)
{
//	static float init_theta,init_position[3];
//	float delta_dis2 = 0;

	if (first_entry_circle == 1)
	{

		circle_acc_kp = 0.1f;
		float sinY,cosY;
		sinY = sinf(ahrsEuler.Ang[2]);
		cosY = cosf(ahrsEuler.Ang[2]);


//		circle_entry_vel = cplfil.Ned_spd[0]*cosY + cplfil.Ned_spd[1]*sinY;
		circle_entry_vel = eskf.Ned_spd[0]*cosY + eskf.Ned_spd[1]*sinY;
		circle_entry_acc = circle_entry_vel*circle_entry_vel/_radius;

		circle_body_vel[0] = circle_entry_vel;
		circle_body_acc[1] = circle_entry_acc;

		circle_center_pos[0] = eskf.Pos[0] - _radius*sinY;
		circle_center_pos[1] = eskf.Pos[1] + _radius*cosY;
		circle_center_pos[2] = eskf.Pos[2];

//		circle_center_pos[0] = cplfil.Ned[0] - _radius*sinY;
//		circle_center_pos[1] = cplfil.Ned[1] + _radius*cosY;
//		circle_center_pos[2] = cplfil.Ned[2];

		circle_inner_flag = 1;
		circle_body_vel[1] = 0;


		cur_target_vel_body[2] = -_helix_velocity;
		cushion_flag = 1;

		cur_target_vel_body[0] = _circle_velocity;//limit(ele->cur_target_vel_body[0],-init_velocity_on_circle,init_velocity_on_circle ); //
		com_pqr_ned[2] = cur_target_vel_body[0]/_radius;       //

		circle_angle = 0.0f;                    //,


	}

	if(first_entry_circle==1)
	{
//		now_circle_pos[0] = cplfil.Ned[0];
//		now_circle_pos[1] = cplfil.Ned[1];
//		now_circle_pos[2] = cplfil.Ned[2];
//
//		last_circle_pos[0] = cplfil.Ned[0];
//		last_circle_pos[1] = cplfil.Ned[1];
//		last_circle_pos[2] = cplfil.Ned[2];

		now_circle_pos[0] = eskf.Pos[0];
		now_circle_pos[1] = eskf.Pos[1];
		now_circle_pos[2] = eskf.Pos[2];

		last_circle_pos[0] = eskf.Pos[0];
		last_circle_pos[1] = eskf.Pos[1];
		last_circle_pos[2] = eskf.Pos[2];

		c_dis = 0;
		a_dis = sqrt((now_circle_pos[0] - circle_center_pos[0])*(now_circle_pos[0] - circle_center_pos[0])
						+(now_circle_pos[1] - circle_center_pos[1])*(now_circle_pos[1] - circle_center_pos[1]));
		b_dis = a_dis;

		dtheta = com_pqr_ned[2]*CONTROL_DT;

	}
	else
	{
//		now_circle_pos[0] eskf.Posed[0];
//		now_circle_pos[1] = cplfil.Ned[1];
//		now_circle_pos[2] = cplfil.Ned[2];

		now_circle_pos[0] = eskf.Pos[0];
		now_circle_pos[1] = eskf.Pos[1];
		now_circle_pos[2] = eskf.Pos[2];

		a_dis = sqrt((now_circle_pos[0] - circle_center_pos[0])*(now_circle_pos[0] - circle_center_pos[0])
				+(now_circle_pos[1] - circle_center_pos[1])*(now_circle_pos[1] - circle_center_pos[1]));
		c_dis = sqrt((now_circle_pos[0] - last_circle_pos[0])*(now_circle_pos[0] - last_circle_pos[0])
				+(now_circle_pos[1] - last_circle_pos[1])*(now_circle_pos[1] - last_circle_pos[1]));

		dtheta = acos((a_dis*a_dis + b_dis*b_dis - c_dis*c_dis)/2*a_dis*b_dis);	//余弦定理


		b_dis = a_dis;
		last_circle_pos[0] = now_circle_pos[0];
		last_circle_pos[1] = now_circle_pos[1];
		last_circle_pos[2] = now_circle_pos[2];
	}


//	ele->dtheta = ele->com_pqr_ned[2]*CONTROL_DT;
	circle_angle += dtheta;
	if (circle_angle >= _angle)
	{
		for(int i =0 ; i!= 3 ;  ++i)
		{
			com_pqr[i]      = 0.0;
			com_theta[i]    = 0.0;
			top_speed_body[i] = 0.0;
			how_long_body[i] = 0.0;
		}
		circle_inner_flag = 0;

		cur_target_vel_body[2] = 0.0;
		circle_finish_flag = true;
		first_entry_circle = 1;
		first_entry_brake = 1;
		heli_step = 8;//,,
		return;
	}
	else
	{
		com_pqr[2] = com_pqr_ned[2] * cos(ahrsEuler.Ang[0]) * cos(ahrsEuler.Ang[1]);    //????
//		ele->dyaw = ele->com_pqr[2]*CONTROL_DT;
		dyaw = dtheta* cos(ahrsEuler.Ang[0]) * cos(ahrsEuler.Ang[1]);
	}

//	if (ele->circle_turn == 1)//
//	{
		yaw_command += dyaw;
		if (yaw_command > PI)
		{
		  yaw_command -= 2*PI;
		}


		float delta_distance = a_dis;


		circle_body_vel[0] = circle_entry_vel;
//		ele->circle_body_acc[1] = ele->circle_entry_acc +  ele->circle_acc_kp*(_radius - delta_distance);
		circle_body_acc[1] = -circle_acc_kp*(_radius - delta_distance);
		circle_delta_distance = _radius - delta_distance;
		circle_body_vel[1] += circle_body_acc[1] * CONTROL_DT;

//		circle_roll_angle = atan2((ele->cur_target_vel_body[0]*ele->com_pqr[2]),G_zinit);//roll;
//		if((fabs(ele->com_theta[0]) < f/abs(circle_roll_angle)) & (ele->cushion_flag ==1))
//		{
//			ele->com_theta[0] = ele->com_theta[0] + circle_roll_angle *0.02f;
//		}
//		else
//		{
//			ele->com_theta[0] = circle_roll_angle;
//			ele->cushion_flag = 0;
//		}
//	}
//	if (ele->circle_turn == 2)//
//	{
//		ele->yaw_command -= ele->com_pqr[2]*CONTROL_DT;
//		if (ele->yaw_command < -PI)
//		{
//			ele->yaw_command += 2*PI;
//		}
//		ele->cur_target_pos_ned[0] = init_position[0]+_radius*(sin(init_theta)-sin(ele->yaw_command));
//		ele->cur_target_pos_ned[1] = init_position[1]+_radius*(-cos(init_theta)+cos(ele->yaw_command));
//		ele->cur_target_vel_ned[0] = ele->cur_target_vel_body[0] * cos(ele->yaw_command);
//		ele->cur_target_vel_ned[1] = ele->cur_target_vel_body[0] * sin(ele->yaw_command);
//
////		circle_roll_angle = -atan2((ele->cur_target_vel_body[0]*ele->com_pqr[2]),G_zinit);//roll
////		if((fabs(ele->com_theta[0]) < fabs(circle_roll_angle)) & (ele->cushion_flag ==1))
////		{
////			ele->com_theta[0] = ele->com_theta[0] + circle_roll_angle *0.04;
////		}
////		else
////		{
////			ele->com_theta[0] = circle_roll_angle;
////			ele->cushion_flag = 0;
////		}
//	}

	cur_target_pos_ned[2] += cur_target_vel_body[2]*CONTROL_DT ;
	com_pqr[1] = com_pqr[2]*cos(com_theta[0]);//pitch
	first_entry_circle ++;
	if(first_entry_circle >= 10000)
		first_entry_circle = 200;
}


void TRAJECTORY::Trajectory_Hover_Mode_Step()//悬停
{
	if (heli_step == 1)
		Trajectory_Hover_Step();
	if (hover_time <= 0)
		hover_time = FLY_TIME;
}
void TRAJECTORY::Trajectory_Hover_Step()
{
	for(int i =0 ; i!= 3 ;  ++i)
	{
		com_pqr[i] = 0.0f;
		com_theta[i] = 0.0f;   //!
		cur_target_vel_ned[i] = 0.0f;
	}
//	cur_target_vel_ned[0] = 0.0f;
//	cur_target_vel_ned[1] = 0.0f;
//	cur_target_vel_ned[2] = 0.0f;
	cur_target_vel_body[0] = 0.0f;

	top_speed_body[0] = 0.0f;
	how_long_body[0] = 0.0f;
	top_speed_body[1] = 0.0f;
	how_long_body[1] = 0.0f;
	top_speed_body[2] = 0.0f;
	how_long_body[2] = 0.0f;

	hover_time--;
	if(hover_time < -500)
		hover_time = -200;

	xQueuePeek(queueTargetBuffer, &target_buffer, 0);

	target_buffer.hover_time_Trajectory = hover_time;

	xQueueOverwrite(queueTargetBuffer,&target_buffer);
}
void TRAJECTORY::Trajectory_Point_to_point_Mode_Step()
{
//	xQueuePeek(queueRCCommand, &rcCommand, 0);
//	if (Trajectory_Target_Update(Point_to_point))		// ele->next_target_ned
//	{
//	if(Horizontal_tracing_state == 1 && Vertical_tracing_state = 1)
//	{
//		PointFinish = 1;
//	}
//	else
//	{
//		PointFinish = 0;
//	}
	Trajectory_Target_Update(Point_to_point);//target_buffer.next_target_ned[i]
	Trajectory_Schedule(MAX_COM_SPEED, Point_to_point);	// ele->next_target_ned
/*how_long_body[0],top_speed_body[0]水平距离和速度,how_long_body[2],top_speed_body[2],垂直距离和速度*/
	if (distance_line > Hover_BOUNDARY_DISTANCE)//判断水平距离并规划
	{
		Trajectory_Turn_Heading(&end_heading);//ahrsEuler.Ang[2]
		heli_step = 2;
	}
	else
	{Horizontal_tracing_state = 2;}

	if(distance_vertical > Hover_BOUNDARY_DISTANCE)//判断垂直距离并规划
	{
		heli_step = 5;
	}
	else {Vertical_tracing_state = 2;}

	if(Horizontal_tracing_state == 2 && Vertical_tracing_state == 2)//已进入目标点
	{
		Trajectory_Current_Target_Update(next_target_ned);
		heli_step = 1;
		end_heading = eskf.Attitude[2];
		if(hover_time <0)
		{
			PointFinish = 1;
		}
	}
	if (heli_step == 1)//悬停
	{
		Trajectory_Hover_Step();
	}
	if (heli_step == 2)//调整朝向yaw
	{
		Trajectory_Turning_Head_Step();   //转头
		if(turn_accel_flag == true)//朝向已对准
		{
			turn_accel=0;
			turn_accel_flag=false;
			heli_step = 3;
		}
	}
	if (heli_step == 3)//朝向已对准
	{
		if (distance_line <=  Speed_Down_BOUNDARY)
		{
			heli_step = 4;//距离目标小于一定距离就进入减速段
			Horizontal_tracing_state = 1;
		}
		else
		{
			Trajectory_Speed_Up_Step();//一直加速
			Horizontal_tracing_state = 0;
		}
	}
	if (heli_step == 4)//小于临界距离
	{
		Trajectory_Speed_Down_Step(); //减速 直到小于Hover_BOUNDARY_DISTANCE后不再执行减速

		if (distance_line <= Hover_BOUNDARY_DISTANCE)
		{
			Horizontal_tracing_state = 2;
		}
	}
	if (heli_step == 5) //垂直距离未达到
	{
		if (distance_vertical <=  Speed_Down_BOUNDARY)
		{
			heli_step = 6;//距离目标小于一定距离就进入减速段
			Vertical_tracing_state = 1;
		}
		else
		{
			Trajectory_Speed_Up_Step_Vertical();//减速
			Vertical_tracing_state = 0;
		}
	}
	if (heli_step == 6) //减速或悬停
	{
		if (distance_vertical <= Hover_BOUNDARY_DISTANCE)
		{
			Vertical_tracing_state = 2;
		}
		else
		{
		  Trajectory_Speed_Down_Step_Vertical();        //加速
		}
	}
//	if (heli_step == 7)//
//	{
//		if(fabs(how_long_body[2]) <= 0.01f)
//		{
//			heli_step = 1;
//			auto_pilot11 = 7;//
//			Trajectory_Current_Target_Update(next_target_ned);
//		}
//		else
//			Trajectory_Land_Step();
//	}
//
//	if (heli_step == 8)
//	{
//		Trajectory_Brake_Down_Step();
//	}
}

void TRAJECTORY::Trajectory_Circle_Mode_Step()
{
	if (Trajectory_Target_Update(Circle))		//主要判断有没有接受到目标期望点，并且把期望点和当前的悬停位置相加得到GPS的具体位置
	{
		Trajectory_Schedule(circle_velocity, Circle);
		if (how_long_body[0] > 0.5f*circle_velocity*circle_velocity/ACCEL)
		{
			Trajectory_Turn_Heading(&end_heading);
		  heli_step = 2;
		}
		else
		{
			Trajectory_Current_Target_Update(next_target_ned);
			end_heading = ahrsEuler.Ang[2];
			heli_step = 1;
		}
	}
	if (heli_step == 1)
	{
		Trajectory_Hover_Step();
	}
	if (heli_step == 2)
	{
		Trajectory_Turning_Head_Step();
		if(turn_accel_flag==true)
		{
		  turn_accel++;
		}
		if(turn_accel==300)//3
		{
			turn_accel=0;
			turn_accel_flag=false;
			heli_step = 3;
		}
	}
	if (heli_step == 3)
	{
		Trajectory_Speed_Up_Step();
		if (abs(how_long_body[0] <= 0.2f))
		{
			heli_step = 9;
			circle_finish_flag = false;
			Trajectory_Current_Target_Update(next_target_ned);
		}
	}
	if (heli_step == 9 && circle_finish_flag == false)
	{
		circle_turn = 1;
		circle_radius_test = 3.0f;   //12.11
		Trajectory_Circle_Step(circle_velocity, circle_radius_test, 0.0f, 360.0f * D2R*5);//by wybb
		return;
	}
	if (heli_step == 8)
	{
		Trajectory_Brake_Down_Step();
	}
}

void TRAJECTORY::Trajectory_Mixture_Mode_Step()
{
//if (target_update(Mixture))
//{
//schedule(init_velocity_on_circle, Mixture);
//if (Mixture_to_Pointtopoint_flag == true)
//{
//Mixture_to_Pointtopoint_flag = false;
//if (ele->how_long_body[0] > BOUNDARY_DISTANCE)
//{
//		turn_heading(&ele->end_heading);
//		ele->heli_step = 2;
//	}
//if (ele->how_long_body[0] <= BOUNDARY_DISTANCE)
//	{
//		position_t->current_target_update(ele->next_target_ned);
//		position_t->heading = ahrs_theta[2];
//		ele->end_heading = ahrs_theta[2];
//		ele->heli_step = 1;
//	}
//return;
//}
//if (circle_finish_flag == true)
//{
//circle_finish_flag = false;
//position_t->current_target_update(ele->next_target_ned);
//position_t->heading = ahrs_theta[2];
//ele->heli_step = 3;
//}
//if (ele->how_long_body[0] > 0.5*init_velocity_on_circle*init_velocity_on_circle/ACCEL
//&& circle_finish_flag == false)
//{
//		turn_heading(&ele->end_heading);
//		ele->heli_step = 2;
//}
//else
//{
//		position_t->current_target_update(ele->hover_ned);
//		position_t->heading = ahrs_theta[2];
//		ele->end_heading = ahrs_theta[2];
//		ele->heli_step = 1;
//}
//}
//if (ele->heli_step == 1)
//{
//hover_step();
//}
//if (ele->heli_step == 2)
//{
//turning_head_step();   //

//if(ele->turn_accel_flag==true)
//{
//	ele->turn_accel++;
//}
//if(ele->turn_accel==90)//3
//{
//	ele->turn_accel=0;
//	ele->turn_accel_flag=false;
//	ele->heli_step = 3;
//	position_t->current_target_update(ele->next_target_ned);
//}
//}
//if (ele->heli_step == 3)
//{
//				speed_up_step();        //
//if (abs(input_xyz[0]) <= 3 ||input_xyz[0]*last_input_xyz0<0)
//{
//				 ele->how_long_body[0] = 0.0;
//ele->heli_step = 9;
//}
//}
//if (ele->heli_step == 9 && circle_finish_flag == false)
//{
//circle_step(ele->circle_velocity, circle_radius_test, helix_velocity, ele->sector_angle); //by wybb
//return;
//}
//if (ele->heli_step == 8)
//{
//brake_down_step();
//}
}

void TRAJECTORY::Trajectory_Test_Mode_Step()
{
//	if (ele->heli_step == 1)
//	{
//		hover_step();
//		if (num == 100)
//		{
//			com_theta[0]  = 0.052; //roll            3?
//			com_theta[1]  = -0.052;//pitch          -3?
//			yaw_com_theta = 0.174; //yaw            10?
//			G_zinit=G_zinit+0.5;   //
//		}
//		if (num == 160)
//		{
//			com_theta[0]  = -0.052; //roll            -3?
//			com_theta[1]  = 0.052;//pitch              3?
//			yaw_com_theta = -0.174; //yaw            -10?
//			G_zinit=G_zinit - 1;   //
//		}
//		if (num == 260)
//		{
//			com_theta[0]  = 0.052; //roll            3?
//			com_theta[1]  = -0.052;//pitch          -3?
//			yaw_com_theta = 0.174; //yaw            10?
//			G_zinit=G_zinit + 1;   //
//		}
//		if (num == 360)
//		{
//			com_theta[0]  = -0.052; //roll            -3?
//			com_theta[1]  =  0.052;//pitch              3?
//			yaw_com_theta = -0.174; //yaw            -10?
//			G_zinit=G_zinit - 1;   //
//		}
//		if (num == 460)
//		{
//			//
//			com_theta[0]  = 0; //roll
//			com_theta[1]  = 0;//pitch
//			yaw_com_theta = 0; //yaw
//			G_zinit=G_zinit +0.5; //
//		}
//		}
//}
}

void TRAJECTORY::Trajectory_RTL_Mode_Step()
{
	if((2 == auto_pilot11) && (hover_time <=-1)) //,
	{
		next_target_ned[0] = cur_target_pos_ned[0];
		next_target_ned[1] = cur_target_pos_ned[1];
		next_target_ned[2] = -1.0;   //,1
		hover_time = 200;//4
		Trajectory_Schedule(MAX_COM_SPEED, Point_to_point);
		if(fabs(how_long_body[2]) < 0.4f)
		{
			heli_step = 1;
			Trajectory_Current_Target_Update(next_target_ned);
		}
		else
		{
			heli_step = 5;
		}
		auto_pilot11 = 4;  //,    4
	}
	else if((4 == auto_pilot11) && (hover_time <=-1)) //,14
	{
		next_target_ned[0] = cur_target_pos_ned[0];
		next_target_ned[1] = cur_target_pos_ned[1];
		next_target_ned[2] = 0.0f;
		hover_time = 100;
		how_long_body[2] = -1;
		heli_step = 7;
		auto_pilot11 = 6;  //,
	}
	else if(RTL_Orignal_update_flag == 0)
	{   //
		next_target_ned[0] = hover_ned[0];
		next_target_ned[1] = hover_ned[1];
		next_target_ned[2] = hover_ned[2];
		hover_time = 200;//4
		Trajectory_Schedule(MAX_COM_SPEED, Point_to_point);	// next_target_ned
		RTL_Orignal_update_flag = 1;
		if (how_long_body[0] > BOUNDARY_DISTANCE)
		{
			Trajectory_Turn_Heading(&end_heading);
			heli_step = 2;
		}
		else if (how_long_body[0] <= BOUNDARY_DISTANCE)
		{
			Trajectory_Current_Target_Update(next_target_ned);
			end_heading = ahrsEuler.Ang[2];
			heli_step = 1;
			RTL_Orignal_flag = 1;
		}
	}
	if (heli_step == 1)
	{
		Trajectory_Hover_Step();
		if((hover_time < -1) && (RTL_Orignal_flag == 1))
		{
			auto_pilot11 = 2;
			RTL_Orignal_flag = 0;
		}
	}

	if (heli_step == 2)
	{
		Trajectory_Turning_Head_Step();  //
		if(turn_accel_flag==true)
		{
			turn_accel++;
		}
		if(turn_accel==90)//3
		{
			turn_accel=0;
			turn_accel_flag=false;
			heli_step = 3;
		}
	}

	if (heli_step == 3)
	{
		if (fabs(how_long_body[0]) <= 0.5f * top_speed_body[0] * top_speed_body[0])
			heli_step = 4;
		else
			Trajectory_Speed_Up_Step_Vertical();          //
	}

	if (heli_step == 4)
	{
		if ((how_long_body[0]) <= 0.05f || cur_target_vel_body[0] == 0.0f)//compass
		{
			how_long_body[0] = 0.0f;
			RTL_Orignal_flag = 1;//
			heli_step = 1;
			Trajectory_Current_Target_Update(next_target_ned);
		}
		else
		{
			Trajectory_Speed_Down_Step_Vertical();        //
		}
	}

	if (heli_step == 5) //
	{
		if (fabs(how_long_body[2]) <= 0.25f * distance_vertical)
			heli_step = 6;
		else
			Trajectory_Speed_Up_Step_Vertical();        //
	}

	if (heli_step == 6) //
	{
		if (fabs(how_long_body[2]) <= 0.05f || cur_target_vel_body[2] == 0.0f) //1/4
		{
			heli_step = 1;
			Trajectory_Current_Target_Update(next_target_ned);
		}
		else
			Trajectory_Speed_Down_Step_Vertical();         //
	}

	if (heli_step == 7)//
	{
		if(fabs(how_long_body[2]) <= 0.01f)
		{
			heli_step = 1;
			auto_pilot11 = 7;//
			Trajectory_Current_Target_Update(next_target_ned);
		}
		else
		{
			Trajectory_Land_Step();
		}
	}
}

void TRAJECTORY::Trajectory_Brake_Down_Step()
{
	hover_time = DEFAULT_HOVER_TIME;

	if((first_entry_brake))
	{
		brake_distance = 0.5f * cur_target_vel_body[0] * cur_target_vel_body[0] / ACCEL;
		first_entry_brake = 0;
		for(int i =0 ; i!= 3 ;  ++i)
		{
			top_speed_body[i] 	= 0.0f;
			how_long_body[i] 	= 0.0f;
		}
	}
	else
	{
		if (cur_target_vel_body[0] > 0)
		{
			cur_target_vel_body[0] -=  CONTROL_DT *ACCEL ;
			brake_distance = brake_distance - cur_target_vel_body[0]*CONTROL_DT;
			cur_target_vel_ned[0] = cur_target_vel_body[0] * cos(yaw_command);
			cur_target_vel_ned[1] = cur_target_vel_body[0] * sin(yaw_command);
			cur_target_pos_ned[0]+= cur_target_vel_ned[0] * CONTROL_DT;
			cur_target_pos_ned[1]+= cur_target_vel_ned[1] * CONTROL_DT;
		}
		else
		{
			heli_step = 1;
			first_entry_brake = 1;
		}
	}
}




