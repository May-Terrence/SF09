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

#ifndef __QUEUEMESSAGE_H
#define __QUEUEMESSAGE_H

#include "main.h"
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif


/* 陀螺仪传感器原始数据 */
typedef struct
{
	float timestamp;		//time since Tim2 start(unit:s)

	float dt;				//delta time between samples(unit:s)

	float gyro[3];			//angular velocity body axis in rad/s

	float temperature;		//temperature in degrees celsius


}sensor_gyro_msg;
/* 陀螺仪传感器原始数据 */

/* 加速度计传感器原始数据 */
typedef struct
{
	float timestamp;		//time since Tim2 start(unit:s)

	float dt;				//delta time between samples(unit:s)

	float acc[3];			//Acceleration body axis in m/s2

}sensor_acc_msg;
/* 加速度计传感器原始数据 */

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
	float Ang[3];		//姿态角, 单位rad

	float pqr[3];		//卡尔曼滤波得到的角速度, 单位rad/s
}ahrs_euler_msg;
/* 姿态角度 ESKF */

/* 电池电压  */
typedef struct
{
	uint8_t battery_n_S;		//n S电池
	float battery_Voltage;		//电池电压，伏
}battery_msg;

/* 电池电压  */

/* 遥控器数据 */
typedef struct
{
	uint16_t  PPM[8];		//遥控器各通道PPM
}RCData;
/* 遥控器数据 */

/* 遥控器数据 */
typedef struct
{

	uint8_t    Key[4];     //4个开关
	float Ang[3];     //遥控器角度 roll pitch YAW
	float dAng[3];       //角速度
	float Thr;        //遥控器油门
	float Uvw[2];     //遥控器Uvw


}RC_command_msg;
/* 遥控器数据 */

/* 电机PWM */
typedef struct
{
	uint16_t PWM[4];
}Motor_PWM_msg;
/* 电机PWM */

typedef struct
{
	float kp[18];
	float ki[18];
	float kd[18];
}Pid_msg;
#ifdef __cplusplus
}
#endif

#endif


/************************ (C) COPYRIGHT Longmen Drone Team *****END OF FILE****/
