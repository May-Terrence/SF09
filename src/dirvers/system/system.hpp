/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : system.hpp
  * Description        : 
  ******************************************************************************
  * Function           : 创建任务、信号量、消息队列、软件定时器，详细请看system.cpp文件中的mySystemInit函数
  *
  *
  ******************************************************************************
  * Author             : Zhiping Fu
  * Creation Date      : 2020年10月19日
  ******************************************************************************
  */
/* USER CODE END Header */

#ifndef __SYSTEM_H
#define __SYSTEM_H

#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os.h"
#include "semphr.h"                     // ARM.FreeRTOS::RTOS:Core
#include "timers.h"
#include "queue.h"
#include "queueMessage.h"


#ifdef __cplusplus
extern "C"{
#endif


//自己创建的任务
/* Definitions for icm20602GyroTask */
extern TaskHandle_t icm20602GyroTaskHandle;
extern void Icm20602_main_gyro(void *argument);
extern SemaphoreHandle_t semIcm20602Gyr;

/* Definitions for icm20602AccTask */
extern TaskHandle_t icm20602AccTaskHandle;
extern void Icm20602_main_acc(void *argument);
extern SemaphoreHandle_t semIcm20602Acc;

/* Definitions for magnetometerTask */
extern TaskHandle_t magnetometerTaskHandle;
extern void Ak8975_main_mag(void *argument);
extern SemaphoreHandle_t semAk8975Mag;

/* Definitions for ahrsTask */
extern TaskHandle_t ahrsTaskHandle;
extern void ahrs_main(void *argument);
extern SemaphoreHandle_t semAhrs;

/* Definitions for tranTask */
extern TaskHandle_t tranTaskHandle;
extern void tran_main(void *argument);
extern SemaphoreHandle_t semTran;

/* Definitions for ledTask */
extern TaskHandle_t ledTaskHandle;
extern void led_main(void *argument);
extern SemaphoreHandle_t semLed;

/* Definitions for batteryTask */
extern TaskHandle_t batteryTaskHandle;
extern void battery_main(void *argument);
extern SemaphoreHandle_t semBattery;

/* Definitions for rcTask */
extern TaskHandle_t rcTaskHandle;
extern void rc_main(void *argument);

/* Definitions for motorTask */
extern TaskHandle_t motorTaskHandle;
extern void motor_main(void *argument);
extern SemaphoreHandle_t semMotor;

/* Definitions for attitudeCtrlTask */
extern TaskHandle_t attitudeCtrlTaskHandle;
extern void attitudeCtrl_main(void *argument);
extern SemaphoreHandle_t semAttitudeCtrl;

/* Definitions for tranReceiveTask */
extern TaskHandle_t tranReceiveTaskHandle;
extern void tranReceive_main(void *argument);

extern TaskHandle_t Openlog_send_Handle;
extern void tskOpenlog_send(void *argument);
extern SemaphoreHandle_t semOpenlog_send;

extern TaskHandle_t Log_Handle;
extern void tskLog(void *argument);
extern SemaphoreHandle_t semLog;
//自己创建的任务


extern QueueHandle_t queueGyrDat;
extern QueueHandle_t queueAccDat;
extern QueueHandle_t queueMagDat;
extern QueueHandle_t queueAhrsEuler;
extern QueueHandle_t queueBattery;
extern QueueHandle_t queueRCData;
extern QueueHandle_t queueRCCommand;
extern QueueHandle_t queueMotorPWM;
extern QueueHandle_t queuePID;

extern TimerHandle_t myTimer1ms;

void Timer1msCallback(void *argument);


void mySystemInit(void);

#ifdef __cplusplus
}
#endif

#ifdef __cplusplus
using namespace std;
class SYSTEM
{
public:
	SYSTEM();
	~SYSTEM();
	void taskCreat();		//任务创建
	void semCreat();		//信号量创建
	void timerCreat();		//软件定时器创建
	void queueCreat();		//消息队列创建


private:
	BaseType_t isGyroTaskCreatSuccess;		//陀螺仪数据采集任务创建成功标志位
	BaseType_t isAccTaskCreatSuccess;		//加速度计数据采集任务创建成功标志位
	BaseType_t isMagTaskCreatSuccess;		//磁力计数据采集任务创建成功标志位
	BaseType_t isAhrsTaskCreatSuccess;		//姿态解算任务创建成功标志位
	BaseType_t isTranTaskCreatSuccess;		//数据传输任务创建成功标志位
	BaseType_t isTranReceiveTaskCreatSuccess;		//数据传输任务创建成功标志位
	BaseType_t isLedTaskCreatSuccess;		//LED任务任务创建成功标志位
	BaseType_t isBatteryTaskCreatSuccess;		//电池电压测量任务创建成功标志位
	BaseType_t isRcTaskCreatSuccess;		//遥控器解析任务创建成功标志位
	BaseType_t isMotorTaskCreatSuccess;		//电机驱动任务创建成功标志位
	BaseType_t isAttitudeCtrlTaskCreatSuccess;  //姿态控制任务创建成功标志位


};

#endif

#endif

/************************ (C) COPYRIGHT Longmen Drone Team *****END OF FILE****/
