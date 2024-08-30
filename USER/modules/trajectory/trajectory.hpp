/*
 * trajectory.hpp
 *
 *  Created on: 2020年12月2日
 *      Author: 刘成吉
 */

#ifndef MODULES_TRAJECTORY_TRAJECTORY_HPP_
#define MODULES_TRAJECTORY_TRAJECTORY_HPP_

#include "system/system.hpp"
#include "userlib/userlib.hpp"
#include "target.hpp"

#ifdef __cplusplus

#include <Eigen>

using namespace std;
using namespace Eigen;


#define FLY_TIME            500
//#define ACCEL               1.0f
#define ACCEL               0.1f
//#define MAX_COM_SPEED       4.0f
#define MAX_COM_SPEED       0.5f
#define BOUNDARY_DISTANCE   2.0f

#define Hover_BOUNDARY_DISTANCE   0.1f
#define Speed_Down_BOUNDARY   0.5f
#define DEFAULT_HOVER_TIME  500

//#define ACCEL_VERTICAL               0.5f
//#define MAX_COM_SPEED_VERTICAL       1.0f
//#define BOUNDARY_DISTANCE_VERTICAL   1.0f

#define ACCEL_VERTICAL               0.1f
#define MAX_COM_SPEED_VERTICAL       0.5f
#define BOUNDARY_DISTANCE_VERTICAL   0.5f


#define PILOT_HEIGHT        1.0f
#define TAKE_OFF_HEIGHT     1.0f



#define CONTROL_HZ 200
#define CONTROL_DT 0.005f
#define G_zinit 9.788f


extern unsigned char auto_pilot11;
enum  Trajectory_mod
{
	Hover,                      //0
	Point_to_point,		    	//1
	Circle,			   			//2
	Mixture,		   			//3
	Test,			 		   	//4
	RTL,                        //5（Return to Launch？）
	False						//6
};

class TRAJECTORY:public TARGET
{
public:
	bool		ReceiveData;
	float        top_speed_body[3];
	float        how_long_body[3] ;

	Target_Queue* target_queue_t;

	float        next_target_ned[4];
	float        end_heading;
	float        hover_time;
//	float        hover_ned[3];      //,
	float        com_pqr[3];
	float        com_pqr_ned[3];
	float        com_theta[3];
	int          heli_step;

	int			 Horizontal_tracing_state;//水平跟踪标志位：0未进入目标点半径 1已进入
	int			 Vertical_tracing_state;//竖直跟踪标志位
	int			 PointFinish;

	float		TakeOffNED[3];
	float		LandingNED[3];

	int          Isturn;
	float        yaw_com_theta;

	float   		 sector_angle;    //
	float   		 circle_velocity; //
	float 		   helix_velocity;  //
	float				 circle_radius;	 //
	float        circle_angle;
	float 		   next_next_target[4];
	int          circle_turn;
	float        distance_line; //
	float        distance_vertical;//
	int 				 vertical_speed_direction;//
	int 				 RTL_Orignal_flag; //RTL
	int 				 RTL_Orignal_update_flag;//RTL
	int 				 RTL_First_switch;
	int 				 first_receive_target;
	int				receiveCount;
	int 				 turn_accel;
	bool         turn_accel_flag;
	int          first_entry_brake;
	int          first_entry_circle;
	int					 cushion_flag;
	bool         circle_finish_flag;

	int	test[10];

	float yaw_command;
	bool target_receive_flag;


	float circle_center_pos[3];
	float circle_entry_vel;
	float circle_entry_acc;
	float circle_body_vel[3];
	float circle_body_acc[3];
	float circle_acc_kp;
	bool circle_inner_flag;


	float circle_delta_distance;


	float now_circle_pos[3];
	float last_circle_pos[3];
	float a_dis;
	float b_dis;
	float c_dis;
	float dtheta;
	float dyaw;
	float        cur_target_vel_body[3];
	float        cur_target_vel_ned[3];
	float        cur_target_pos_ned[3];
	enum Trajectory_mod trajectory_mod;

	bool First_TakeOff_Flag;

	void Trajectory_Reset();
	void Trajectory_Step(int trajectory_mode);
	void Trajectory_Point_to_Point_Mode_Step();
	void Trajectory_Hover_Mode_Step();
	void Trajectory_Point_to_point_Mode_Step();
	void Trajectory_Circle_Mode_Step();
	void Trajectory_Mixture_Mode_Step();
	void Trajectory_Test_Mode_Step();
	void Trajectory_RTL_Mode_Step();
	void Trajectory_Brake_Down_Step();
	void Trajectory_Schedule(float max_speed, int mode);
	void Pos_Hold_Init();
	void Trajectory_Target_Receive();
	void Trajectory_Current_Target_Update(float *next_target_ned);
	void Target_Plus_Hover_Point();
	bool Trajectory_Target_Update(int trajectory_mod);
	void Trajectory_Turn_Heading(float * end_heading_point);
	void Trajectory_Distance_Calculate(int trajectory_mod);
	void Trajectory_Speed_Calculate(float max_com_speed, int trajectory_mod);
	void Trajectory_Hover_Step();
	void Trajectory_Turning_Head_Step();
	void Trajectory_Speed_Up_Step();
	void Trajectory_Speed_Down_Step();
	void Trajectory_Speed_Up_Step_Vertical();
	void Trajectory_Speed_Down_Step_Vertical();
	void Trajectory_Land_Step();
	bool Trajectory_Climbing_Step();
	void Trajectory_Circle_Step(float _circle_velocity, float _radius,float _helix_velocity, float _angle);
	ahrs_euler_msg ahrsEuler;

	ekf_cplfil_msg cplfil;




};

extern TRAJECTORY trajectory;

#endif
#endif /* MODULES_TRAJECTORY_TRAJECTORY_HPP_ */
