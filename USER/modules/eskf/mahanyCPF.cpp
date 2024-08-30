/*
 * mahanyCPF.cpp
 *
 *  Created on: 2020年12月10日
 *      Author: 刘成吉
 */
#include "mahanyCPF.hpp"

bool MAHANYCPF::init()
{
	mag_msg* mag= new mag_msg;
	downsample_imu_for_eskf_msg* imu = new downsample_imu_for_eskf_msg;

	xQueuePeek(queueMag, mag, 0);
	xQueuePeek(queueDownsampleIMU, imu, 0);
	*(imu->sample_cnt) = 0;

	Vector3f minus_accb;
	Vector3f magb;

	minus_accb << -imu->acc[0], -imu->acc[1], -imu->acc[2];
	magb << mag->MagRel[0], mag->MagRel[1], mag->MagRel[2];

	delete mag;
	delete imu;

	if(magb.norm() == 0)			//没有磁力计
	{
		return false;
	}

	minus_accb.normalize(); //归一化
	magb.normalize();

	this->Ang.y() = -asin(minus_accb.x()); //Pitch
	this->Ang.x() =  atan2(minus_accb.y(),minus_accb.z()); //Roll
	float sinR = sinf(this->Ang.x()), cosR = cosf(this->Ang.x());
	float sinP = sinf(this->Ang.y()), cosP = cosf(this->Ang.y());
	this->Ang.z() =-atan2f(magb.y()*cosR - magb.z()*sinR, magb.x()*cosP + magb.y()*sinP*sinR + magb.z()*sinP*cosR) -  3.1167*D2R;//Yaw
	this->q = eul2qua(this->Ang);
	q.normalize();

	last_Magfiel = Magfiel;
	static uint8_t convergent_CNT = 0;
	static uint8_t window = 0;
	if(convergent_CNT < 15)
	{
		Magfiel = (Magfiel*window + q.toRotationMatrix()*magb)/(window + 1);
		Magfiel.normalize();
		if(window < 200)
			window++;
		if ((last_Magfiel - Magfiel).norm() < 1e-5)//确保磁场稳定
			convergent_CNT++;
		last_Magfiel = Magfiel;
		return false;
	}

	this->g << 0,0,scalar_g;
	//本体坐标切换矩阵
	FS_matrix << 0.0f, 0.0f, 1.0f,
 	   	   	  	 0.0f, 1.0f, 0.0f,
				-1.0f, 0.0f, 0.0f;
	frame_switch_quat = FS_matrix;
	mahany.bodyFrame = HELICOPTOR;

	return true;
}


void MAHANYCPF::MahanyCPF()
{
	getTimer_us(&startTimer);

	mag_msg* mag= new mag_msg;
	downsample_imu_for_eskf_msg* imu = new downsample_imu_for_eskf_msg;

	xQueuePeek(queueMag, mag, 0);
	xQueuePeek(queueDownsampleIMU, imu, 0);
	xQueuePeek(queuePID, &pid, 0);

	*(imu->sample_cnt) = 0;

	Vector3f minus_accb;
	Vector3f wb;
	Vector3f magb;
	Vector3f mag_est;
	Vector3f gb_norm;
	Vector3f error;
	static Vector3f errInt = Vector3f::Zero();

	minus_accb << -imu->acc[0], -imu->acc[1], -imu->acc[2];

	wb << imu->gyro[0], imu->gyro[1], imu->gyro[2];

	magb << mag->MagRel[0], mag->MagRel[1], mag->MagRel[2];

	if(abs(this->Ang(1) + PI/2) < 20*D2R && mahany.bodyFrame == HELICOPTOR)
	{
		q = q*frame_switch_quat;
		q.normalize();
		mahany.bodyFrame = FIXWING;
	}
	else if(this->Ang(1) > 70*D2R && mahany.bodyFrame == FIXWING)
	{
		q = q*FS_matrix.transpose();
		q.normalize();
		mahany.bodyFrame = HELICOPTOR;
	}
	if(mahany.bodyFrame == FIXWING)
	{
		minus_accb = FS_matrix.transpose()*minus_accb.eval();
		wb 		   = FS_matrix.transpose()*wb.eval();
		magb 	   = FS_matrix.transpose()*magb.eval();
	}

	minus_accb.normalize();

	gb_norm = q.toRotationMatrix().bottomRows<1>().transpose(); //理论重力加速度方向

	magb.normalize();

	mag_est = q.toRotationMatrix().transpose()*Magfiel; //理论当地磁场方向

	error = -mag_est.cross(magb)*1.0 - gb_norm.cross(minus_accb)*1.0;
	errInt += error;



//	wb += error*pid.kp[12] + errInt*pid.ki[12] * 0.00001f;


	wb += error*0.2f + errInt*0.005f;


	q = q*Expqua(wb*dt/2); //旋转角度缩放
	q.normalize();

	//转欧拉角输出
	this->Ang = qua2eul(q);
	mahany.Attitude[0] = this->Ang.x();
	mahany.Attitude[1] = this->Ang.y();
	mahany.Attitude[2] = this->Ang.z();

	xQueueOverwrite(queueESKF, &mahany);

	getTimer_us(&stopTimer);
	executionTime_us = stopTimer - startTimer;

	delete mag;
	delete imu;
}

