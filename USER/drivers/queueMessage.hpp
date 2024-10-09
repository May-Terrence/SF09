/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : queueMessage.h
  * Description        :
  ******************************************************************************
  * Function           : 定义队列传输的数据结构，如有新的数据传输需要，请在该文件中添加，然后在queueCreat()
  * 					 中创建队列来存放数据
  *
  ******************************************************************************
  * Author             : Zhiping Fu
  * Creation Date      : Oct 19, 2020
  ******************************************************************************
  */
/* USER CODE END Header */

#ifndef __QUEUEMESSAGE_HPP
#define __QUEUEMESSAGE_HPP


#include "main.h"
#include <stdbool.h>
#include "userlib/data_struct.h"

//#include <Eigen>
//
//using namespace std;
//using namespace Eigen;
#ifdef __cplusplus
extern "C" {
#endif


typedef enum
{
	RC_NONE=0xFFU,    //无行为
	RC_FUN0=0x00U,    //右下 右下
	RC_MOT_UNLOCK=0x01U,  //右下 左下  解锁
	RC_FUN2=0x02U,    //右上 右下
	RC_FUN3=0x03U,    //右上 左下
	RC_FUN4=0x04U,    //右下 右上
	RC_FUN5=0x05U,    //右下 左上
	RC_FUN6=0x06U,    //右上 右上
	RC_FUN7=0x07U,    //右上 左上
	RC_MOT_LOCK=0x08U,    //左下 右下  上锁
	RC_FUN9=0x09U,    //左下 左下
	RC_FUN10=0x0AU,   //左上 右下
	RC_FUN11=0x0BU,   //左上 左下
	RC_FUN12=0x0CU,   //左下 右上
	RC_FUN13=0x0DU,   //左下 左上
	RC_FUN14=0x0EU,   //左上 右上
	RC_FUN15=0x0FU,   //左上 左上
}eRC_MODE;

typedef enum
{
	HELICOPTOR = 0X00U,
	FIXWING	   = 0X01U,
}eBODY_FRAME;

typedef enum			//电量状态
{
	BATTARY_NORM = 0x00U,
	BATTARY_LOW = 0x01U,
	BATTARY_CRITICAL = 0x02U,
}eBATTARY_STA;
typedef enum
{
	PTP_hover   = 0x00U,	//定点
	PTP_TurnHead   =0x1U,	//转头标志
	PTP_XY_Fly    = 0x02U,	//水平飞行标志
	PTP_Z_Fly	 = 0x03U,	//垂直飞行标志
	PTP_XYZ_Fly 	 = 0x04U,	//三维飞行
	PTP_Arrive  = 0x05U,	//到达标志位
	NonPTP   = 0x06U,
}ePTP_Status;

typedef enum
{
	Circle_hover   = 0x00U,	//定点
	Circle_TurnHead   =0x1U, //转头标志，表明将要飞到圆弧轨迹上，面向圆心
	Circle_Z_Fly    = 0x02U, //垂直飞行标志
	Circle_Line	 = 0x03U,	//从当前点飞到圆轨迹上
	Circle_TurnHead1   =0x4U, //转头标志，转向圆弧切线方向

	Circle_ing 	 = 0x05U,	//正在飞圆弧
	Circle_brake = 0x06U,	//正在刹车
	Circle_Arrive   = 0x07U, //已经到达
	NonCircle = 0x08U,
}eCircle_Status;
typedef enum
{
	Dubins_hover   = 0x00U,	//定点
	Dubins_TurnHead   =0x1U, //转头
	Dubins_Straight = 0x02U,	//直线
	Dubins_Orbit  =0x3U, //圆弧

	Dubins_brake = 0x04U,	//刹车
	Dubins_Arrive   = 0x05U, //到达
	NonDubins = 0x06U,
}eDubins_Status;
typedef enum {NORMAL=0,CLOSE=1,LOST=2}eRC;

/* 陀螺仪传感器原始数据 */
typedef struct
{
	uint32_t timestamp;		//time since Tim2 start(unit:us)

	float dt;				//delta time between samples(unit:s)

	float gyro[3];			//angular velocity body axis in rad/s

	float temperature;		//temperature in degrees celsius


}sensor_gyro_msg;
/* 陀螺仪传感器原始数据 */

/* 陀螺仪传感器滤波后的数据 */
typedef struct
{
	float gyro_filter[3];			//angular velocity body axis in rad/s

}sensor_gyro_filter_msg;
/* 陀螺仪传感器滤波后的数据 */

/* 加速度计传感器原始数据 */
typedef struct
{
	uint32_t timestamp;		//time since Tim2 start(unit:us)

	float dt;				//delta time between samples(unit:s)

	float acc[3];			//Acceleration body axis in m/s2

}sensor_acc_msg;
/* 加速度计传感器原始数据 */

/* 加速度传感器滤波后的数据 */
typedef struct
{
	float acc_filter[3];			//angular velocity body axis in rad/s
	float jerk_filter[3];
}sensor_acc_filter_msg;
/* 加速度传感器滤波后的数据 */

/* imu向下采样，  For eskf only */
typedef struct
{
	uint32_t timestamp;		//time since Tim2 start(unit:us)
	float gyro[3];
	float acc[3];
	int* sample_cnt; // 下采样区间内imu的采样总数，eskf读取后需由eskf重置
}downsample_imu_for_eskf_msg;
/* imu向下采样 */

/* 磁力计传感器原始数据 */
typedef struct
{
	float timestamp;		//time since Tim2 start(unit:s)

	float dt;				//delta time between samples(unit:s)

	float mag[3];			//Magnetometer body axis in gauss

}sensor_mag_msg;
/* 磁力计传感器原始数据 */

/* 姿态角度 ESKF */
typedef struct
{
	eBODY_FRAME bodyFrame;

	float Ang[3];		//姿态角, 单位rad

	float pqr[3];		//卡尔曼滤波得到的角速度, 单位rad/s

//	Matrix<float,3,3> drotation;  //旋转矩阵
//
	float rotation[3][3];

}ahrs_euler_msg;
/* 姿态角度 ESKF */

/* 电池电压  */
typedef struct
{
	uint8_t battery_n_S;		//n S电池
	float battery_Voltage;		//电池电压，伏
}battery_msg;

/* 电池电压  */
typedef struct
{
	float kp[18];
	float ki[18];
	float kd[18];
}PID_msg;

typedef struct
{
	uint32_t timestamp;		//时间戳,单位us
	float Velx;			//x轴光流速度，单位cm/s
	float Vely;			//y轴光流速度，单位cm/s
	float Posx;			//x轴光流位置，单位cm
	float Posy;			//y轴光流位置，单位cm
	float height0;
	float height;
	float heightFil;
	float VelxFil;
	float VelyFil;
	float VelxRaw;
	float VelxRawFil;
	float VelyRaw;
	float VelyRawFil;

}laserFlow_msg;			//光流激光数据
typedef struct
{
	uint32_t timestamp;		//时间戳,单位us
	float flowx;			//x轴光流速度，单位cm/s
	float flowy;			//y轴光流速度，单位cm/s
	float flowx2;			//x轴光流位置，单位cm
	float flowy2;			//y轴光流位置，单位cm
}flow_msg;			//光流数据
typedef struct
{
	uint32_t timestamp;		//时间戳,单位us
	float height;			//高度,单位cm
	float speedZ;			//高度,单位cm/s
}height_msg;		//激光高度数据


/* 遥控器数据 */
typedef struct
{
	uint16_t  PPM[16];		//遥控器各通道PPM
}RCData;
/* 遥控器数据 */
typedef struct
{
	bool ever_rc;

	eRC rc_status;
	int rtcm_id[5];
	int rctm_id_cnt;
}RC_Status_msg;

/* 遥控器数据 */
typedef struct
{
	uint8_t    Key[4];     //4个开关
	bool OneKeyTakeoff{false};		//1为一键起降模式
	bool Frist_Entry_TakeOff{false};
	bool TakeOffFinish{false};//一键起降待机模式 该值为0时锁电机，为1时解锁

	bool OneKeyLanding{false};		//1为一键起降模式

	bool LandingFinish{false};//一键起降待机模式 该值为0时锁电机，为1时解锁

	bool Clr_flypoint{false};
	eRC_MODE  Mode;   //行为模式

	int TakeOff_Trajectory_Landing{false};//0为地面待机，1为一键起飞过程中，2为轨迹模式，3为一键降落过程中
	float Val[4];     //roll pitch yaw high

	float Ang[3];     //遥控器角度 roll pitch YAW
	float dAng;       //角速度
	float Thr;        //遥控器油门
	float Uvw[3];     //遥控器Uvw

}RC_command_msg;
/* 遥控器数据 */

typedef struct
{
	int cs_output[7];//舵面输出
	int mt_output[2];//电机输出
	double output[5];//控制输出,T,Mx,My,Mz,β

}Control_output_msg;

typedef struct
{
	uint16_t PWM_OBS[8];
}Motor_msg;


typedef struct
{
	uint32_t timestamp;		//时间戳,单位us
	float altitude;			//高度,单位m
	float altSlope;			//高度,单位m/s
	float bar_temp;
}sensor_baroAlt_msg;		//激光高度数据

typedef struct
{
	bool isReady;
	bool update;

	uint8_t status;
	uint8_t star;			//星数

	uint32_t timestamp;		//时间戳,单位us
	float NED[3];			//北东地坐标，单位m
	float gpsSpdAccuracy;
	float gpsPosAccuracy;
	float gpsHeiAccuracy;
	double	lat;			//纬度，单位°
	double	lng;			//经度，单位°
	double alti;			//高度,单位m
	double Zero_LLH[3];		//零点LLH，单位°或m
	double NED_spd[3];

}gps_msg;		//gps数据
typedef struct
{
	bool isReady;

	uint8_t status;
	uint8_t star;			//星数

	uint32_t timestamp;		//时间戳,单位us
	float NED[3];			//北东地坐标，单位m
	float gpsSpdAccuracy;
	float gpsPosAccuracy;
	float gpsHeiAccuracy;
	double	lat;			//纬度，单位°
	double	lng;			//经度，单位°
	double alti;			//高度,单位m
	double Zero_LLH[3];		//零点LLH，单位°或m
	double NED_spd[3];

}OrdinaryGps_msg;		//OrdinaryGps数据
typedef struct
{
	uint32_t timestamp;		//时间戳,单位us
	float MagRaw[3];
	float MagRel[3];  	//实际值
}mag_msg;		//磁力计数据

typedef struct
{
	int last_control_mode;
	int control_mode;
	int FlightStatus;

	int trajectory_status;//传递轨迹规划状态
	int last_trajectory_status;

	int XY_phase;
	int Z_phase;
	int XYZ_phase;
	float Jerk_command[3];
	float jerkFil[3];
	float X_command[2];
	float Y_command[2];
	float Z_command[2];
	float XH[2];
	float YH[2];
	float X[2];
	float Y[2];
	float Z[2];
	float Ang[3];
	float roll_command[2];
	float pitch_command[2];

	float yaw_command[2];

	float brake_mode;
	float rc_status;
	float output[5];
	float mt_output[2];

	float pqr[3];
	float pqr_d_raw[3];
	float pqr_d0[3];
	float pqr_command[3];
	float AngVel_command[3];
	float Acc_command[3];
	double output1[3];

	float Pos_estimate[2]{0};//估计位置
	float X_err_estimate{0},Y_err_estimate{0},XY_err_estimate{0}; //估计误差
	bool  enable_Grab_flag{false};

	float X_pos0,Y_pos0,Z_pos0;

	float Z_diff_gps{0},Z_diff_laser{0},Z_diff{0};

	float end_command[2]{0};
	float end_yaw{0};

}control_transfer_msg;
typedef struct
{

	bool path_type; //Dubins曲线类型


	bool brake_finish{false};  //飞航点刹车完成标志位，true表示刹车完成
	bool isBrake{false}; //刹车状态标志位，只在运动规划函数followStraightPath和followCirclePath中使用，ture表示正在刹车/减速
	bool isUniform{false}; //匀速状态标志位，只在运动规划函数followStraightPath和followCirclePath中使用，ture表示正在匀速运动
	bool  flypoint_start{false}; //true表示飞PTP(点对点)航点过程开始
	bool  flycircle_start{false};//true表示飞圆轨迹过程开始
	bool  dubins_start{false}; //true表示飞Dubins曲线过程开始
	ePTP_Status PTP_Status; //点对点运动的状态
	eCircle_Status Circle_Status; //圆弧运动的状态
	eDubins_Status Dubins_Status; //Dubins曲线运动的状态
	uint32_t  executionTime_us;

	float angeskf[3]; //eskf得到的姿态角
	float Vel_command_ref[3]; //参考速度指令
	float Pos_command_ref[3]; //参考位置指令
	float Pos_track[3]; //目前跟踪到的位置，此位置是以正式飞航点时（转头之后）的机体坐标系为参考系
	float azimuth {0}; //方位角 从起点到终点的连线与正北的夹角 计算方法azimuth = atan2f((end[1]-start[1]),(end[0]-start[0]));
	float desiredHeading; //飞圆弧时期望的航向角
	float headingAng; //当前的机头指向
	float YawAngle_traveled=0.0; //圆弧运动时 已经走过的角度
	float YawAngle_remaining=0.0;////圆弧运动时 还未走过的角度

	float iniAng; //转完头后 飞圆弧时的初始航向角
	float POS_err; //位置误差
	float distance_XY; //两个点之间的距离
	float accelerationAngle=0.0; //圆弧运动中 加速过程走过的角度(距离长度可以用 弧度*半径 计算)
	float accelerationDistance=0.0; //直线运动中 加速过程走过的距离
	float goalDistance=0; //Dubins曲线局部直线路径距离
	float goalAng=0; //Dubins曲线局部圆弧路径角度

	float Einv[3]{0}; //伪逆分配误差
	float Ei[3]{0}; //INDI分配误差
	float Ef[3]{0}; //反馈分配误差
	float Ti[3]{0}; //INDI输入
	float Tf[3]{0}; //反馈输入
}trajectory_msg;
typedef struct
{
	double Ned[3];
	double Ned_spd[3];
	double Acc_bias[3];
}ekf_cplfil_msg;		//EKF互补滤波输出

typedef struct
{
	eBODY_FRAME bodyFrame;
	bool update{false};
	float q[4];					//四元数
	float rotation[3][3];
	float Attitude[3];			//欧拉角,单位（弧度radian），roll,pitch,yaw
	float Pos[3];				//卡尔曼北东地坐标，单位（弧度radian）
	float Ned_spd[3];
	float Gyro_bias[3];
	float Acc_bias[3];

	float P[6]{0};

	float Attitude0[3];			//欧拉角,单位（弧度radian），roll,pitch,yaw
	float Pos0[3];				//卡尔曼北东地坐标，单位（弧度radian）
	float Ned_spd0[3];

	float Attitude1[3];			//欧拉角,单位（弧度radian），roll,pitch,yaw
	float Pos1[3];				//卡尔曼北东地坐标，单位（弧度radian）
	float Ned_spd1[3];

//	float pos_cor[3];
//	float vel_cor[3];
//	float ang_cor[3];
}eskf_msg;
typedef struct
{
	eBODY_FRAME bodyFrame;

//	float q[4];					//四元数
	float rotation[3][3];
	float Attitude[3];			//欧拉角,单位（弧度radian），roll,pitch,yaw
	float Pos[3];				//卡尔曼北东地坐标，单位（弧度radian）
	float Ned_spd[3];
//	float Gyro_bias[3];
//	float Acc_bias[3];

//	float pos_cor[3];
//	float vel_cor[3];
//	float ang_cor[3];
}eskf_baro_msg;

typedef struct
{
	bool   receive_flag;
	bool   NED_or_LLH;				//1forNED,0forLLH

	u16 hovertime[15];				//每个航点的计划悬停时间
	u16 PTPSpeed[15];				//Point_to_Point speed
	vs16 PTPHeading[15];				//Point_to_Point Heading
	int   index;                         //指针，指向队列中最后一个有效目标点
	int	  Last_index;
	int   cur_targert;

	int   Pointindex;                         //指针，指向队列中最后一个有效目标点
	int	  Last_Pointindex;

	int	   count;													//尚未被接受进队列的目标点数
	float TargetPointList[15][4];    //期望目标点数组，0，1，2为北东地位置，单位m，3为标志位：-1正常，-2不正常

	float    hover_time_Trajectory;				//当前航点将要悬停的时间，倒计时

	float   next_target_ned[4];

}Target_Buffer;

typedef struct
{
	uint8_t num;			// 航点序号
	uint8_t enable;			// 使能位
	Data_3D<float> NED_position_m;
	uint16_t stay_time_s;
	float vel_mps;
	float yaw_degree;		// degree
}fly_point;

typedef struct
{
	float Pos[3];//X,Y,Z
	float Yaw;
	int star;
	int model;
}CLAW_msg;

typedef struct
{
	float Pos[3];//X,Y,Z
}SLAM_msg;

#ifdef __cplusplus
}
#endif

#endif

/************************ (C) COPYRIGHT Longmen Drone Team *****END OF FILE****/
