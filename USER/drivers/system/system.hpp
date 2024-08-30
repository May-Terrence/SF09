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

#ifndef __SYSTEM_HPP
#define __SYSTEM_HPP


#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os.h"
#include "semphr.h"                     // ARM.FreeRTOS::RTOS:Core
#include "timers.h"
#include "queue.h"

#include "userlib/userlib.hpp"

#ifdef __cplusplus
extern "C"{
#endif

//自己创建的任务

/* Definitions for tranTask */

/* Definitions for flowReceiveTask */
extern TaskHandle_t flowTaskHandle;
extern void flow_main(void *argument);

/* Definitions for ControlTask */
extern TaskHandle_t ControlTaskHandle;
extern void control_main(void *argument);
extern SemaphoreHandle_t semControl;

/* Definitions for batteryTask */
extern TaskHandle_t batteryTaskHandle;
extern void battery_main(void *argument);
extern SemaphoreHandle_t semBattery;

/* Definitions for bellTask */
extern TaskHandle_t bellTaskHandle;
extern void bell_main(void *argument);
extern SemaphoreHandle_t semBell;

/* Definitions for ahrsTask */
extern TaskHandle_t ahrsTaskHandle;
extern void ahrs_main(void *argument);
extern SemaphoreHandle_t semAhrs;

/* Definitions for icm20602Task */
extern TaskHandle_t icm20602TaskHandle;
extern void Icm20602_main(void *argument);
extern SemaphoreHandle_t semIcm20602;

/* Definitions for magnetometerTask */
extern TaskHandle_t lsm303dTaskHandle;
extern void lsm303d_main(void *argument);
extern SemaphoreHandle_t semLsm303d;

/* Definitions for rcTask */
extern TaskHandle_t rcTaskHandle;
extern void rc_main(void *argument);

/* Definitions for RcCheckTask */
extern TaskHandle_t RcCheckTaskHandle;
extern void RC_Check_main(void *argument);
extern SemaphoreHandle_t semRcCheck;

/* Definitions for motorTask */
extern TaskHandle_t motorTaskHandle;
extern void motor_main(void *argument);
extern SemaphoreHandle_t semMotor;

/* Definitions for motorTask */
//extern TaskHandle_t ms5611TaskHandle;
//extern void ms5611_main(void *argument);
//extern SemaphoreHandle_t semMs5611;
extern TaskHandle_t dps368TaskHandle;
extern void DPS368_main(void *argument);
extern SemaphoreHandle_t semDPS368;

extern TaskHandle_t tranTaskHandle;
extern void tran_main(void *argument);
extern SemaphoreHandle_t semTran;

/* Definitions for tranReceiveTask */
extern TaskHandle_t tranReceiveTaskHandle;
extern void tranReceive_main(void *argument);

/* Definitions for gpsmagTask */
extern TaskHandle_t gpsmagTaskHandle;
extern void gpsmag_main(void *argument);

///* Definitions for OrdinaryGpsTask */
//extern TaskHandle_t OrdinaryGpsTaskHandle;
//extern void OrdinaryGps_main(void *argument);

/* Definitions for ekfTask */
extern TaskHandle_t ekfTaskHandle;
extern void ekf_main(void *argument);
extern SemaphoreHandle_t semEkf;

/* Definitions for eskfTask */
extern TaskHandle_t eskfTaskHandle;
extern void eskf_main(void *argument);
extern SemaphoreHandle_t semEskf;

/* Definitions for eskf_baroTask */
extern TaskHandle_t eskf_baroTaskHandle;
extern void eskf_baro_main(void *argument);
extern SemaphoreHandle_t semEskf_baro;

/* Definitions for rm3100Task */
extern TaskHandle_t rm3100TaskHandle;
extern void rm3100_main(void *argument);
extern SemaphoreHandle_t semRm3100;

extern TaskHandle_t Openlog_send_Handle;
extern void tskOpenlog_send(void *argument);
extern SemaphoreHandle_t semOpenlog_send;

extern TaskHandle_t Log_Handle;
extern void tskLog(void *argument);
extern SemaphoreHandle_t semLog;

extern TaskHandle_t laserFlowTaskHandle;
extern void laserFlow_main(void *argument);
extern SemaphoreHandle_t semlaserFlow;

/* Definitions for mocapTask */
extern TaskHandle_t clawTaskHandle;
extern void claw_main(void *argument);

//自己创建的任务
extern QueueHandle_t queuePID;
extern QueueHandle_t queueInloopControl;


extern QueueHandle_t queueRC_Status;
extern QueueHandle_t queueBattery;
extern QueueHandle_t queueFlow;
extern QueueHandle_t queueHeight;
extern QueueHandle_t queueRCCommand;
extern QueueHandle_t queueAccDat;
extern QueueHandle_t queueAccDatFil;
extern QueueHandle_t queueGyrDat;
extern QueueHandle_t queueGyrDatFil;
extern QueueHandle_t queueAhrsEuler;
extern QueueHandle_t queueMagDat;
extern QueueHandle_t queueRCData;
extern QueueHandle_t queueMotorData;
extern QueueHandle_t queueBaroAlt;
extern QueueHandle_t queueControlTransfer;
extern QueueHandle_t queueControlOutputDat;
extern QueueHandle_t queueGps;
//extern QueueHandle_t queueOrdinaryGps;
extern QueueHandle_t queueMag;
extern QueueHandle_t queueEKF;
extern QueueHandle_t queueESKF;
extern QueueHandle_t queueESKF_baro;

extern QueueHandle_t queueDownsampleIMU;
extern QueueHandle_t queuetrajectoryData;

extern TimerHandle_t myTimer1ms;

extern QueueHandle_t queueFlyPoint;
extern QueueHandle_t queueTargetBuffer;
extern QueueHandle_t queuelaserFlow;
extern QueueHandle_t queueClaw;

void Timer1msCallback(void *argument);

void TIM5_IRQHandler(void);


void mySystemInit(void);

#ifdef __cplusplus
}
#endif


#ifdef __cplusplus

#include "queueMessage.hpp"

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

	uint32_t  startTimer;			//计时器
	uint32_t  stopTimer;			//计时器
	uint32_t  executionTime_us;			//计时器

	uint32_t  startTimerLast;			//计时器
	uint32_t  cycleTime_us;

private:

	BaseType_t isFlowTaskCreatSuccess;  //姿态控制任务创建成功标志位
	BaseType_t isControlTaskCreatSuccess;  //姿态控制任务创建成功标志位
	BaseType_t isBatteryTaskCreatSuccess;		//电池电压测量任务创建成功标志位
	BaseType_t isBellTaskCreatSuccess;		//电池电压测量任务创建成功标志位
	BaseType_t isAhrsTaskCreatSuccess;		//姿态解算任务创建成功标志位
	BaseType_t isIcm20602TaskCreatSuccess;		//陀螺仪数据采集任务创建成功标志位
	BaseType_t isMagTaskCreatSuccess;		//磁力计数据采集任务创建成功标志位
	BaseType_t isMotorTaskCreatSuccess;		//电机驱动任务创建成功标志位
	BaseType_t isRcTaskCreatSuccess;		//遥控器解析任务创建成功标志位
//	BaseType_t isMs5611TaskCreatSuccess;		//遥控器解析任务创建成功标志位
	BaseType_t isDps368TaskCreatSuccess;		//遥控器解析任务创建成功标志位
	BaseType_t isTranTaskCreatSuccess;		//数据传输任务创建成功标志位
	BaseType_t isTranReceiveTaskCreatSuccess;		//数据传输任务创建成功标志位
	BaseType_t isClawTaskCreatSuccess;		//爪子数据获取任务创建成功标志位

};

#endif

#endif

/************************ (C) COPYRIGHT Longmen Drone Team *****END OF FILE****/
