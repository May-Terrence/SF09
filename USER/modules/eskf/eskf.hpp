/*
 * eskf.hpp
 *
 *  Created on: 2020年12月3日
 *      Author: 刘成吉
 */

#ifndef MODULES_ESKF_ESKF_HPP_
#define MODULES_ESKF_ESKF_HPP_

#include "system/system.hpp"
#include "userlib/userlib.hpp"
#include "openlog.h"

#ifdef __cplusplus
#include <Eigen>
#include <malloc.h>
#include "MySeqQueue.hpp"
#include "mahanyCPF.hpp"
using namespace std;
using namespace Eigen;

#define scalar_g	9.7883
#define dt	0.01f

class MAHANYCPF; //两个头文件相互包含，为避免循环包含，需要在此处前向声明

typedef struct
{
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW;//用于在使用 Eigen 库时为动态分配的对象提供对齐内存的支持
	Quaternion<float> q;
	Vector3f Pos;		//单位：弧度rad&米m
	Vector3f Vned;
	Vector3f Ba;
	Vector3f Bg;
}STATES;	//64字节长度

typedef struct
{
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
	Vector3f wm;		//imu测量值(角速度)
	Vector3f accm;		//加速度
	Vector3f magm;		//磁力计测量(假设磁力计与imu同步更新)
	uint32_t updtime;		//更新时间
}IMUMag;

typedef struct
{
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
	Vector3f Pos;			//单位：m
	Vector3f Vned;
	uint32_t updtime;		//更新时间
}GPS;

class ESKF
{
private:
/*系统状态x = [P V_ned q beta_a beta_g]',     Δx' = F·Δx + G·w
  *先把所有16Bytes倍数长度的变量放前面一块，每个变量相对于类首的相对地址又必须是16的整数倍(16字节倍数长度的成员变量按16字节对齐，包括32字节长度倍数)
 */
	Quaternion<float> q;
	Matrix<float,12,12> Q; //过程噪声协方差
	Matrix<float,15,12> G; //噪声状态对应的Jacobi矩阵
	STATES stateTempEle;

/*
 *4字节长度倍数（非16字节倍数）长度变脸按4字节对齐（包括8字节）
 */
	Matrix<float,15,15> P; //真值与预测值之间的协方差矩阵
	Matrix<float,15,15> F; //系统矩阵/状态转移矩阵
	Matrix<float,15,15> QQ;
	Matrix<float,7,15> H; //状态观测矩阵
	Matrix<float,15,7> K;
	Matrix<float,7,7> R; //测量噪声协方差
	Matrix<float,15,1> delta_x;	//Δx = x - x_estimate
	Matrix<float,7,1> z_sub_y;
	Matrix<float,3,3> A;	//旋转矩阵
	Matrix3f FS_matrix;		//本体坐标切换矩阵
	Matrix3f drotation;
	Matrix<float,3,3> I3;

	Vector3f maglevel;		//水平机头坐标系下的磁场
	Vector3f Pos;			//北东地位置
	Vector3f Vned;
	Vector3f Ba; //加速度计偏置
	Vector3f Bg; //陀螺仪偏置

	Vector3f g;
	Vector3f accm;			//加速度计测量值
	Vector3f wm;			//陀螺仪测量值
	Vector3f accb;			//accb = accm - Ba;
	Vector3f accn;			//accn = A*Accb;
	Vector3f wb;			//wb = wm - Bg;
    //观测数据：
	Vector3f magm;			//磁力计测量值
	Vector3f Ang; 		    //欧拉角，roll, pitch, yaw

	//互补滤波部分变量
	Vector3f pos_err;
	Vector3f pos_err_inter;
	Vector3f vel_err;
	Vector3f vel_err_inter;
	Vector3f ang_err;
	Vector3f ang_err_inter;
	Vector3f pos_cor;
	Vector3f vel_cor;
	Vector3f ang_cor;


	float psim;						//测量偏航角
	float dpsim;					//偏航角误差
	uint32_t  startTimer;			//计时器
	uint32_t  stopTimer;			//计时器
	uint32_t  executionTime_us;			//计时器
	uint32_t  startTimerLast;			//计时器
	uint32_t  cycleTime_us;

	//读取传感器数据
	gps_msg gps;
//	sensor_gyro_msg gyro;
//	sensor_acc_msg acc;
	downsample_imu_for_eskf_msg imu;
	mag_msg mag;
	eskf_msg output;

	IMUMag ImuMagTempEle;
	GPS GpsTempEle;

	MySeqQueue<STATES> states_Q;	//状态队列：四元数、纬经高、NED速度
	MySeqQueue<IMUMag> imu_Q;		//imu队列：角加速度、加速度、磁力计、更新时间
	GPS gps_Q;						//gps：北东地、NED速度、更新时间（始终用最新的GPS，假设延时时间固定420ms）

	PID_msg pid;//调试互补滤波写参数用

	/*
	 * 调试用到的变量临时存放到下面区域
	 */

public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
	ESKF(int stateQ_size = 28):states_Q(stateQ_size), imu_Q(stateQ_size){};
//	ESKF(){};
	~ESKF(){};

	void init(const MAHANYCPF& mahany);
	void Eskf_Calc();
	void propagate();
	void update();
	//互补滤波部分
	bool readSensors();
	void refreshStatesQ();	//互补滤波
	void outputStates();
	void switchFrame();

	RC_command_msg rcCommand;
};

//class ESKF_BARO
//{
//private:
///*系统状态x = [P V_ned q beta_a beta_g]',     Δx' = F·Δx + G·w
//  *先把所有16Bytes倍数长度的变量放前面一块，每个变量相对于类首的相对地址又必须是16的整数倍(16字节倍数长度的成员变量按16字节对齐，包括32字节长度倍数)
// */
//	Quaternion<float> q;
//	Matrix<float,12,12> Q; //过程噪声协方差
//	Matrix<float,15,12> G; //噪声状态对应的Jacobi矩阵
//	STATES stateTempEle;
//
///*
// *4字节长度倍数（非16字节倍数）长度变脸按4字节对齐（包括8字节）
// */
//	Matrix<float,15,15> P; //真值与预测值之间的协方差矩阵
//	Matrix<float,15,15> F; //系统矩阵/状态转移矩阵
//	Matrix<float,15,15> QQ;
//	Matrix<float,7,15> H; //状态观测矩阵
//	Matrix<float,15,7> K;
//	Matrix<float,7,7> R; //测量噪声协方差
//	Matrix<float,15,1> delta_x;	//Δx = x - x_estimate
//	Matrix<float,7,1> z_sub_y;
//	Matrix<float,3,3> A;	//旋转矩阵
//	Matrix3f FS_matrix;		//本体坐标切换矩阵
//	Matrix3f drotation;
//	Matrix<float,3,3> I3;
//
//	Vector3f maglevel;		//水平机头坐标系下的磁场
//	Vector3f Pos;			//北东地位置
//	Vector3f Vned;
//	Vector3f Ba; //加速度计偏置
//	Vector3f Bg; //陀螺仪偏置
//
//	Vector3f g;
//	Vector3f accm;			//加速度计测量值
//	Vector3f wm;			//陀螺仪测量值
//	Vector3f accb;			//accb = accm - Ba;
//	Vector3f accn;			//accn = A*Accb;
//	Vector3f wb;			//wb = wm - Bg;
//    //观测数据：
//	Vector3f magm;			//磁力计测量值
//	Vector3f Ang; 		    //欧拉角，roll, pitch, yaw
//
//	//互补滤波部分变量
//	Vector3f pos_err;
//	Vector3f pos_err_inter;
//	Vector3f vel_err;
//	Vector3f vel_err_inter;
//	Vector3f ang_err;
//	Vector3f ang_err_inter;
//	Vector3f pos_cor;
//	Vector3f vel_cor;
//	Vector3f ang_cor;
//
//
//	float psim;						//测量偏航角
//	float dpsim;					//偏航角误差
//	uint32_t  startTimer;			//计时器
//	uint32_t  stopTimer;			//计时器
//	uint32_t  executionTime_us;			//计时器
//	uint32_t  startTimerLast;			//计时器
//	uint32_t  cycleTime_us;
//
//	//读取传感器数据
//	OrdinaryGps_msg gps;
//	sensor_baroAlt_msg baroAlt;
////	sensor_gyro_msg gyro;
////	sensor_acc_msg acc;
//	downsample_imu_for_eskf_msg imu;
//	mag_msg mag;
//	eskf_baro_msg output;
//
//	IMUMag ImuMagTempEle;
//	GPS GpsTempEle;
//
//	MySeqQueue<STATES> states_Q;	//状态队列：四元数、纬经高、NED速度
//	MySeqQueue<IMUMag> imu_Q;		//imu队列：角加速度、加速度、磁力计、更新时间
//	GPS gps_Q;						//gps：北东地、NED速度、更新时间（始终用最新的GPS，假设延时时间固定420ms）
//
//	PID_msg pid;//调试互补滤波写参数用
//
//	/*
//	 * 调试用到的变量临时存放到下面区域
//	 */
//
//public:
//	EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
//	ESKF_BARO(int stateQ_size = 28):states_Q(stateQ_size), imu_Q(stateQ_size){};
////	ESKF(){};
//	~ESKF_BARO(){};
//
//	void init(const MAHANYCPF& mahany);
//	void Eskf_Calc();
//	void propagate();
//	void update();
//	//互补滤波部分
//	bool readSensors();
//	void refreshStatesQ();	//互补滤波
//	void outputStates();
//	void switchFrame();
//
//	RC_command_msg rcCommand;
//};


#endif
#endif /* MODULES_ESKF_ESKF_HPP_ */
