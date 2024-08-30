/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : ahrs.hpp
  * Description        :
  ******************************************************************************
  * Function           :
  *
  ******************************************************************************
  * Author             : Zhiping Fu
  * Creation Date      : Oct 11, 2020
  ******************************************************************************
  */
/* USER CODE END Header */

#ifndef __AHRS_HPP
#define __AHRS_HPP

#include "system/system.hpp"
#include "userlib/userlib.hpp"


#ifdef __cplusplus

#include <Eigen>

using namespace std;
using namespace Eigen;

class AHRS
{
public:
	AHRS(){};
	AHRS(char *n) : name(n){}
	~AHRS(){};
	bool Ahrs_Init();
	void Ahrs_Update();

private:
	char *name;
	sTIM Tim;
	float dt;
	float Cb2n[3][3];
	float DatGyr[3];
	float DatAcc[3];
	float DatMag[3];
	bool Update;
	uint32_t startTimer;
	uint32_t stopTimer;
	uint32_t  executionTime_us;
	float   AccUpdate_time;//更新时间
	float   AccDeltaT_Sample;
	float   GyroUpdate_time;//更新时间
	float   GyroDeltaT_Sample;
	float   MagUpdate_time;//更新时间
	float   MagDeltaT_Sample;

	float acc_sum;
	float acc_R_x;
	float acc_R_a;
	float acc_R_b;
	float R_ele;


	//矩阵运算变量定义（ESKF）
	Matrix<float,4,6> H;
	Matrix<float,3,4> H1;
	Matrix<float,4,3> H2;
	Matrix<float,6,4> K;
	Matrix<float,6,6> P;
	Matrix<float,4,4> R;
	Matrix<float,6,6> Q;
	Matrix<float,4,1> Z;
	Matrix<float,4,1> Z_p;
	Matrix<float,7,1> X_p;
	Matrix<float,4,1> Z_sub_H;
	Matrix<float,7,1> X_k;
	Matrix<float,6,1> X_k_error;
	Matrix<float,6,6> F;
	Matrix3f FS_matrix;
	Quaternion<float> frame_switch_quat;

	Vector3f MatAcc; //加速度计数据
	Vector3f MatGyr; //陀螺仪数据
	Vector3f MatMag; //磁力计数据
	Vector3f Magfiel;//当地磁场
	Vector3f last_Magfiel;

	Quaternion<float> q;  //四元数
	Quaternion<float> theq;
	Matrix<float,3,3> rotation;

	Quaternion<float> dq;


	Matrix<float,3,3> drotation;  //旋转矩阵

	Vector3f Euler;  //欧拉角


	sensor_gyro_msg gyro;
	sensor_acc_msg acc;
	mag_msg mag;
	ahrs_euler_msg ahrsEuler;

};

#endif

#endif

/************************ (C) COPYRIGHT Longmen Drone Team *****END OF FILE****/
