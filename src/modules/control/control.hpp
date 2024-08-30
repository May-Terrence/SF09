/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : control.hpp
  * Description        : 
  ******************************************************************************
  * Function           :
  *
  ******************************************************************************
  * Author             : Zhiping Fu
  * Creation Date      : Oct 23, 2020
  ******************************************************************************
  */
/* USER CODE END Header */

#ifndef __CONTROL_HPP
#define __CONTROL_HPP

#include "userlib/userlib.hpp"
#include "system/system.hpp"
#include "pid/pid.hpp"

#ifdef __cplusplus
extern "C" {
#endif

#define QUADX		//X字型四轴
//#define OCTAP		//十字型八轴
#define PWM_IDLE	1040

#define PID_NUM		12

typedef enum
{
	QAUD_MANUAL		= 0x00U,	//手控
	QAUD_ATTITUDE	= 0x01U,	//姿态
	QAUD_ALT_HOLD	= 0x02U,	//定高
	QAUD_POS_HOLD	= 0x03U,	//定点
	QAUD_TRAJECTORY	= 0x04U,	//航线
}eQUAD_MODE;

#ifdef __cplusplus
}
#endif

#ifdef __cplusplus

using namespace std;

class ATTITUDE_CTRL
{
public:
	ATTITUDE_CTRL();
	~ATTITUDE_CTRL();
	void flight_Mode_Update();
	void motor_Output();

	void Attitude_Ctrl_Init();

	void Attitude_ctrl_run();

	void pid_Para_Update();


private:
	eQUAD_MODE Mode, preMode;
	u8    TakeOff;		//起飞，0:不使能 1:第一次 2:多次 3:结束或无法启用
	u8    Land;			//降落，0:不使能 1:第一次 2:多次 3:结束
	u8    HomeWard;		//一键返航，0:不使能 1:第一次 2:多次 3:结束
//	float WindDir;		//风向
//	float WindAng;		//风力

	bool  LockHead;		//锁尾
	bool  LockHigh;		//锁高
	float HoldYaw;		//锁尾偏航角度
	float InitHigh;		//初始定高高度

	float SetXyz[3];	//位置环目标值
	float SetUvw[3];	//速度环目标值
	float SetAcc[3];	//加速度环目标值
	float SetAng[3];	//角度环目标值
	float SetDAng[3];	//角速度环目标值
	float FeedBackDAng[3];
	float FeedBackAng[3];
	float FeedBackXyz[3];
	float FeedBackUvw[3];

	float ROL_OUT;
	float PIT_OUT;
	float YAW_OUT;
	float Z_OUT;
	float FOREWARD;
	float THROTTLE;

	u8 isPosHold;
	bool LockHeadFirst;
	bool isFirstAltHold;


	uint32_t startTimer;
	uint32_t stopTimer;
	uint32_t  executionTime_us;

	sensor_gyro_msg gyro;
	sensor_acc_msg acc;
	ahrs_euler_msg ahrsEuler;
	RC_command_msg rcCommand;
	Motor_PWM_msg motorPWM;
	Pid_msg pid_para;

};


#endif



#endif


/************************ (C) COPYRIGHT Longmen Drone Team *****END OF FILE****/
