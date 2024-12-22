/*
 * control.hpp
 *
 *  Created on: 2020年9月2日
 *      Author: 17900
 */

#ifndef CONTROL_HPP_
#define CONTROL_HPP_

#ifdef __cplusplus
extern "C"
{
#endif

#include "math.h"
#ifdef __cplusplus
}
#endif




#ifdef __cplusplus

#include <Eigen>

using namespace Eigen;

#include "userlib/userlib.hpp"
#include "userlib/filters.hpp"

#include "pid/pid.hpp"
#include "alloc/wls_alloc.hpp"

#include "trajectory/trajectory.hpp"
#include "fly_point.h"
#include <path_follow/pathFollow.hpp>
#include "hardware.h"
#include <slam/slam.hpp>

#define dt 0.01
#define CTRLFB_NUM 1
#define CTRLIO_NUM 1
#define CTRLIL_NUM 1
#define CTRLKT_NUM 1
#define CtrlDt 0.01f
#define CTRLFB_SLOPE_NUM 15
#define CTRLPID_NUM 12
#define FOLD_WING_PWM_MIN 0.0f
#define FOLD_Wing_PWM_MAX 900.0f
#define RC_CHECK_NUM 400
#define RATE_SLOPE_NUM 20 //40 for 200Hz

//float  Vel_tol=0.4;
//float  Velz_end=0.6f;
//float  Acc_alt_manu=0.3f;
//--------------------模型--------------------
#ifdef MF09II02
#define HOVER_SPEED 1225.0f//
#define GE_SPEED   0.1f
#define GE_ACC     -0.3f
#define HOVER_PWM 1860//细桨1650 //
#define IDLING_PWM 1650
#define DRIFT_PWM HOVER_PWM-20//细桨1630
#define GROSSMASS 1.85f//2.1f
#endif

#ifdef MF09II01
#define HOVER_SPEED 1225.0f//
#define GE_SPEED   0.1f
#define GE_ACC     -0.3f
#define HOVER_PWM 1720//细桨1650 //宽桨1545
#define IDLING_PWM 1450
#define DRIFT_PWM 1630//细桨1630 //宽桨1525
#define GROSSMASS 1.85f//2.1f
#endif

#ifdef MF09II03
#define HOVER_SPEED 1225.0f//
#define GE_SPEED   0.1f
#define GE_ACC     -0.3f
#define HOVER_PWM 1710//细桨1650 //宽桨1545
#define IDLING_PWM 1450
#define DRIFT_PWM 1630//细桨1630 //宽桨1525
#define GROSSMASS 1.85f//2.1f
#endif


#define INERTIA_X 0.0149f
#define INERTIA_Y 0.0149f
#define INERTIA_Z 0.005516f
#define O2MX  0.6755//0.9865f //单位弧度输出产生的力矩 output 2 Mx (1弧度输出对应2个舵各1弧度)
#define O2MY  0.6755//0.9865f
#define O2MZ  0.4985//0.728f

#define OneKeyTakeOffHeight 3.0f
typedef enum
{
	MODE_MANUAL = 0x00U,          //手控
	MODE_ALTITUDE  = 0x01U,       //姿态
	MODE_HIGH_HOLD = 0x02U,       //定高
	MODE_POSITION_HOLD = 0x03U,   //定点
	MODE_TARGET = 0x04U,          //航线
}eMODE;

typedef enum
{
	GROUND    = 0x00U,	 //地面
	TAKEOFF   = 0x01U,	 //起飞
	LANDING	  = 0x02U,	 //降落
	AIR 	  = 0x03U,	 //空中
	IDLING    = 0x04U,   //怠速
	GE        = 0x05U,   //地面效应
	RETURNING = 0x06U,   //返航
	LAUNCH    = 0x07U,
	RELANDING = 0x08U,	 //重新降落
}eFlightStatus;
typedef struct
{
	float pqr[3];//0 for p, 1 for q, 2for r
	float Ang[3];//0 for phi, 1 for theta, 2 for psi
	//-----------------------------------------------------------------------------------
	//NED XYZ, 0 for positition, 1 for velocity, 2 for acceleration
	float X[3];
	float Y[3];
	float Z[3];
	//Headframe XY //0 for position, 1 for velocity
	float XH[2];
	float YH[2];
	//-----------------------------------------------------------------------------------
	//Bodyframe UVW,0 for UVW, 1 for UVW derivative
	float U[2];
	float V[2];
	float W[2];
	float  AirSpeed;
	float  AirSpeed_Last;
	//发动机转速
	float engine_speed;
	//飞行模态
	float Z1_temp[CTRLFB_SLOPE_NUM];
}sFEEDBACK;


typedef struct
{
	float input[4];//控制输入
	double output[5];//控制输出,Mx,My,Mz,T,β
	int cs_output[7];//舵面输出
	int mt_output[2];//电机输出
	int fly_mode;//飞行模式，1直升机，2固定翼
	int tran_mode;//1垂直转平飞，2平飞转垂直, 3状态保持
	int level_mode;//1加速，2巡航
	int yaw_mode;//航向锁定，0无锁尾，1锁尾
	int control_mode;		//0手控，1半自控，2自控，3速度控制，4失控保护
	int last_yaw_mode;
	int last_control_mode;
	bool wing_flag;//flase折叠，true展开
	int last_fly_mode;
	double mid_trim[3];//平衡点调整，roll,pitch yaw
	int pwm_temp;//测试用
	int pwm_temp1;
	int JUMP;

	//-----------------------------------------

	float output_pid[3];
	double output1[3];
	double output2[3];
	float output_ail;
	//-----------------------------------------
	u8 rc_buffer[RC_CHECK_NUM];
	u16 rc_cnt;
	u16 rc_check;
	bool ever_rc;
	eRC rc_status;
	//-----------------------------------------
	eFlightStatus FlightStatus;
}sIO;


typedef struct
{
	float Z_mid;
	int   Z_mid_pwm;
	//-----------------------------------------
	float Pos_command[3];//NED位置
	float Vel_command[3];//NEDH速度
	float Acc_command[3]{0};//NEDH加速度
	float Jerk_command[3]{0};//NEDH加加速度
	float Jerk_forward=0;//NEDH加加速度
	float Ang_command[3];//欧拉角
	float AngVel_command[3]{0};//欧拉角速度（body）

	float Ang_command_d[3];//欧拉角变化率
	float pqr_command[3];//角速度
	float pqr_command_d[3];//角加速度
	float thrust_command[2];//推力
	//-----------------------------------------
	float last_Pos_command[3][RATE_SLOPE_NUM];
	float last_Vel_command[3][RATE_SLOPE_NUM];
	float last_Ang_command[3][RATE_SLOPE_NUM];
	float last_pqr_command[3][RATE_SLOPE_NUM];
	//-----------------------------------------
	float gyro_gain;
	float gyro_output[3]{0};
	//-----------------------------------------
	int   XY_phase;
	int   XY_phase1;
	int   XY_phase2;

	int   XYZ_phase;
	float POS_thr;
	float X_pos0,Y_pos0,Z_pos0,Z_laser_pos0;
	float X_claw_pos0,Y_claw_pos0,Z_claw_pos0;
	float Z_diff_gps,Z_diff_laser,Z_diff;
	float Yaw0;

	int count;
	int count1;

//	float X_pos0,Y_pos0;



	int   Z_phase;
	int   Z_phase1;
    bool  brake_mode;
    bool  alt_brake_mode;
    bool  landing_acc_mode;
    bool  trajectory_start;

	bool  circle_start;
	bool  iniLanding=false;


    sCNT  brake_cnt;
    sCNT  alt_brake_cnt;
    sCNT  alt_landing_cnt;
	float yaw_mid;
	float AngleLimitR;
	float AngleLimitP;
	//-----------------------------------------
    double Pos_err[3];//NEDH
    double Vel_err[3];//NEDH
    double Ang_err[3];//欧拉角误差在角速度空间里的映射，欧拉角向量左乘Q^-1
	double pqr_err[3];
	//-----------------------------------------
	float u1_Bar[2];
	float u1_Tilt[2];
	float ko1;
	int filn;
	int filr;
	//-----------------------------------------
	float X_init,Y_init;
	float Yaw_init;

    int    Tran_cnt;
	int    Phase;
	float  Vydd;//加速度规划
    float  Vyd0;
	float  Vyc; //速度规划
	float  VelSet;//速度终值
	//-----------------------------------------
	float Circle_Ang;
	float Circle_Rad;
	float Circle_X_center,Circle_Y_center;
	float indi_fltr_band;
	float relative_height;

	float Pos_estimate[2]{0};//估计位置
	float X_err_estimate{0},Y_err_estimate{0},XY_err_estimate{0}; //估计误差
	bool  enable_Grab_flag{false};

	float end_command[3]{0};
	float end_yaw{0};
}sLOOPIO;


typedef struct
{
	float pid_Ki_temp[12];
	bool int_reset;
	bool test;
}sKITEMP;

typedef struct
{
	float  pqr_d_raw[3];
	float  pqr_d0[3];
	float  last_pqr[3];
	double output_0[3];
	double output_i[3];
	double output_f[3];
	float  G_cv[3];
	int    Filt_AngularAcc[3];
	int    Filt_Output[3];
	float Acc_d_filter[3]{0};
	float Acc_d[3]{0};
    float Acc_d_filter_param[3]{0};
    float Acc_disturb[3]{0};
}sINDI;
typedef struct
{
	float H_des; //期望高度
	float Va_preSet; //预设空速
	float chi_des; //期望航向角
	float phi_c_ff; //前馈滚转角
	float delta_Va;	// 空速增量  Va_des = Va_preSet + delta_Va;
	float Va_des; //期望空速
}sLevelAutopilot;
//sLOOPIO CtrlLpIO;
class CONTROL:public WLS_ALLOC,PID,TRAJECTORY
{
public:
	CONTROL(){}
	~CONTROL(){}

	sFEEDBACK CtrlFbck;
	sIO CtrlIO;
	sLOOPIO CtrlLpIO;
	sINDI   CtrlINDI;
	sKITEMP CtrlKiTemp;
	sLevelAutopilot CtrlLevelAutopilot;
	trajectory_msg Ctrltrack;


	void Control_Init(void);
	void Control_Feedback(void);
	void Out_Loop_XY_Pre2(void);
	void Out_Loop_Circle(void);
	void Out_Loop_XYZ_auto(void);
	void Out_Loop_Z_Pre2(void);
	void Pos_Loop_Step(void);
	void Pos_Loop_Step2(void);
	void Pos_Loop_Step_Vertical(void);
	void Pos_Loop_Step_Level(void);
	void Out_Loop_Step(void);
	void Out_Loop_Step2(void);
	void Out_Loop_Step_Vertical(void);
	void Out_Loop_Step_Level(void);
	void In_Loop_Step(void);
	void In_Loop_Step2(void);
	void Gyro_Control(void);
	void Control_Output(void);
	void Control_Output2(void);
	void Control_Output3(void);
	void Control_Output_INDI(void);
	void Output_To_Motor(void);
	void ControlRC_Check(void);
	void Integral_Reset(void);
	void Integral_Restore(void);
	void Transition_Init(void);
	void Transition_Calc(void);
	void Transition_V2L_Calc(void);
	void Transition_L2V_Calc(void);
	void Circle_Init(void);
	void Circle_Calc(void);
	void OneKeyTakeOff(void);
	void OneKeyLanding(float X,float Y,bool isAppoint,bool isUnderControl);
	void Auto_flypoint(void);
	void Auto_flycircle(void);
	bool Turn_Heading(float SetAng);
	void trackPath(void);

	void levelfly_init(void);
	bool followStraightPath(Vector2f *line_start,Vector2f *line_end,float vel,bool isContinuation);
	bool followCirclePath(float radius,float vel,Vector2f center,float goal_angle,bool isContinuation,int direction);
	bool frog_jump(void);
	void calcCircularAngle(float iniAng,float goalAng,int direction);
	void StatusClear(void);
	void virtualFence(void);
	void land_pid(void);
	void remove_land_pid(void);

	PID pid[PID_NUM];

	PID *pidRolRate=&pid[0],*pidPitRate=&pid[1],*pidYawRate=&pid[2];
	PID *pidRol=&pid[3],*pidPit=&pid[4],*pidYaw=&pid[5];
	PID *pidXRate=&pid[6],*pidYRate=&pid[7],*pidZRate=&pid[8];
	PID *pidX=&pid[9],*pidY=&pid[10],*pidZ=&pid[11];
	PID *pidLaser=&pid[12];
	PID *pidThrust=&pid[13];
	PID *pidTmp=&pid[15];

	PID_msg pid_msg;

	float TakeOffDeltaZ;
	float LandingDeltaZ;
	float kp_accy;
	float kp_reduced_att;
	float kp_indi_ang_acc;

	control_transfer_msg control_data;

	RC_command_msg rcCommand;
	fly_point fly_point_overwrite;
	laserFlow_msg laserFlow;
	SLAM_msg slam_msg;

	FlyPoint point_list[10];
	Data_3D<float> position;
	bool isGpsNormal=true;
	bool isexceedLimit=false;
	bool isReady{true};

	float Z_laser_err{0};
	float Z_err_inter{0};
	float Z_err_cor{0};
	float last_laser_height{0};

private:
	uint32_t startTimer;
	uint32_t stopTimer;
	uint32_t  executionTime_us;
	bool isSegmentComplete{true}; //Dubins局部轨迹完成标志位
	bool isComplete{true}; //航点轨迹完成标志位

	RC_Status_msg rc_status_msg;

	sensor_gyro_msg gyro;
	sensor_gyro_filter_msg gyro_filt;
	sensor_acc_msg acc;
	sensor_mag_msg mag;
	ahrs_euler_msg ahrsEuler;

//	height_msg height;
//	flow_msg flow;
	sensor_acc_filter_msg acc_fil;
	Control_output_msg control_output;
	ekf_cplfil_msg cplfil;		//9阶卡尔曼互补滤波输出
	eskf_msg eskf;
	height_msg height;

	Vector2f TempBodyFrame; //临时机体坐标系

	Matrix<float,3,3> Rb2n;
	Matrix<float,3,3> Rbe;
	Vector4f gyroTorqueCompensation; //陀螺力矩补偿

//	Data_3D<float> vel;
	float		 vel{1.0};
	float		 fly_yaw {0.0};
	uint16_t	 staytime{0};
	float		 fly_radius {0.0};
	uint16_t circleRemaining{0}; //圆弧运动时，将画多少个整圆，不足一个不算
	uint16_t circleTraveled{0}; //圆弧运动时，已经画过多少个整圆
};


#endif



#endif /* CONTROL_HPP_ */
