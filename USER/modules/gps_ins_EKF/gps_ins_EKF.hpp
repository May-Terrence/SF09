/*
 * gps_ins_EKF.hpp
 *
 *  Created on: 2020年11月30日
 *      Author: 刘成吉
 */

#ifndef MODULES_GPS_INS_EKF_GPS_INS_EKF_HPP_
#define MODULES_GPS_INS_EKF_GPS_INS_EKF_HPP_


#include "system/system.hpp"
#include "userlib/userlib.hpp"


#ifdef __cplusplus

#include <Eigen>

using namespace std;
using namespace Eigen;

#define C_WGS84_a 6378137.0        //地球长半轴
#define C_WGS84_b 6356752.314245   //地球短半轴 f = (a-b)/a
#define C_WGS84_f 0.00335281066474748072//地球扁率
#define C_WGS84_e 0.081819790992   //第一偏心率

#define IMU_BUFFER_SIZE 42
//#define IMU_BUFFER_SIZE 13

#define GPS_BUFFER_SIZE 5
#define GPS_DELAYED_TIME 400000
//#define GPS_DELAYED_TIME 240000/2

#define g0 9.7883
typedef struct
{
	u32 update_time;
	double acc[3];
	double w_qpr[3];
	double T[9];
}IMU_RING_ELEMENT;

typedef struct
{
	u8  youngest;
	u8  oldest;
	bool is_filled;
  IMU_RING_ELEMENT imu_data[IMU_BUFFER_SIZE];
}STORE_IMU_BUFFER;

typedef struct
{
	u32 update_time;
	double Pos_obs[3];
	double Vel_obs[3];
}GPS_RING_ELEMENT;

typedef struct
{
	u8  youngest;
	u8  oldest;
	bool new_data_flag;
	u32 last_update_time;
	GPS_RING_ELEMENT gps_data[GPS_BUFFER_SIZE];
}STORE_GPS_BUFFER;

typedef struct
{
	double Pos[3];
	double Vel[3];
}OUTPUT_RING_ELEMENT;

typedef struct
{
	u8  youngest;
	u8  oldest;
	OUTPUT_RING_ELEMENT output_data[IMU_BUFFER_SIZE];
}STORE_OUTPUT_BUFFER;

typedef struct
{
	u32 update_time;
	double Height;
}BAR_RING_ELEMENT;


typedef struct
{
	u8  youngest;
	u8  oldest;
	bool new_data_flag;
	BAR_RING_ELEMENT bar_data[IMU_BUFFER_SIZE];
}STORE_BAR_BUFFER;


typedef struct
{
	bool GPS_INS_EKF_flag; //GPS_INS_EKF 初始化标志位
	bool GPS_INS_EKF_start_flag;
	double V_P[3];//速度观测值
	double P_p[3];//纬经度观测值
	double acc_bias_p[3];//加速度偏置预测值
	double Rm_p;
	double Rn_p;
	double V_EKF[3];//EKF 速度值
	double P_EKF[3];//EKF 位置
	double acc_bias_EKF[3];//
	double dV_o[3];
	double P_GI[81];//协方差矩阵
}EKF;



extern OUTPUT_RING_ELEMENT output_data_new;
extern bool ekf_update_flag;
extern double dV_inter[3];
void GPS_INS_EKF();
void LLH2NED();

ahrs_euler_msg ahrsEuler;
sensor_acc_msg accb;
gps_msg gps;
ekf_cplfil_msg cplfil;

#endif
#endif /* MODULES_GPS_INS_EKF_GPS_INS_EKF_HPP_ */
