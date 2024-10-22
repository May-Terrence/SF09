/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : system.cpp
  * Description        :
  ******************************************************************************
  * Function           : 创建任务、信号量、消息队列、软件定时器，详细请看system.cpp文件中的mySystemInit函数
  *
  ******************************************************************************
  * Author             : Zhiping Fu
  * Creation Date      : 2020年10月19日
  ******************************************************************************
  */
/* USER CODE END Header */

#include "system.hpp"


//自己创建的任务


/* Definitions for icm20602Task */
TaskHandle_t icm20602TaskHandle;
void Icm20602_main(void *argument);
SemaphoreHandle_t semIcm20602;

/* Definitions for magnetometerTask */
TaskHandle_t lsm303dTaskHandle;
void lsm303d_main(void *argument);
SemaphoreHandle_t semLsm303d;

/* Definitions for motorTask */
//TaskHandle_t ms5611TaskHandle;
//void ms5611_main(void *argument);
//SemaphoreHandle_t semMs5611;
TaskHandle_t dps368TaskHandle;
void DPS368_main(void *argument);
SemaphoreHandle_t semDPS368;

/* Definitions for ahrsTask */
TaskHandle_t ahrsTaskHandle;
void ahrs_main(void *argument);
SemaphoreHandle_t semAhrs;

/* Definitions for batteryTask */
TaskHandle_t batteryTaskHandle;
void battery_main(void *argument);
SemaphoreHandle_t semBattery;

/* Definitions for bellTask */
TaskHandle_t bellTaskHandle;
void bell_main(void *argument);
SemaphoreHandle_t semBell;

/* Definitions for ControlTaskTask */
TaskHandle_t ControlTaskHandle;
void control_main(void *argument);
SemaphoreHandle_t semControl;

/* Definitions for motorTask */
TaskHandle_t motorTaskHandle;
void motor_main(void *argument);
SemaphoreHandle_t semMotor;

/* Definitions for tranTask */
TaskHandle_t tranTaskHandle;
void tran_main(void *argument);
SemaphoreHandle_t semTran;

/* Definitions for RcCheckTask */
TaskHandle_t RcCheckTaskHandle;
void RC_Check_main(void *argument);
SemaphoreHandle_t semRcCheck;

/* Definitions for tranReceiveTask */
TaskHandle_t tranReceiveTaskHandle;
void tranReceive_main(void *argument);

/* Definitions for rcTask */
TaskHandle_t rcTaskHandle;
void rc_main(void *argument);

/* Definitions for flowReceiveTask */
TaskHandle_t flowTaskHandle;
void flow_main(void *argument);

/* Definitions for gpsmagTask */
TaskHandle_t gpsmagTaskHandle;
void gpsmag_main(void *argument);

///* Definitions for OrdinaryGpsTask */
//TaskHandle_t OrdinaryGpsTaskHandle;
//void OrdinaryGps_main(void *argument);

/* Definitions for ekfTask */
TaskHandle_t ekfTaskHandle;
void ekf_main(void *argument);
SemaphoreHandle_t semEkf;

/* Definitions for eskfTask */
TaskHandle_t eskfTaskHandle;
void eskf_main(void *argument);
SemaphoreHandle_t semEskf;

/* Definitions for eskf_baroTask */
TaskHandle_t eskf_baroTaskHandle;
void eskf_baro_main(void *argument);
SemaphoreHandle_t semEskf_baro;

///* Definitions for rm3100Task */
//TaskHandle_t rm3100TaskHandle;
//void rm3100_main(void *argument);
//SemaphoreHandle_t semRm3100;

TaskHandle_t Openlog_send_Handle;
void tskOpenlog_send(void *argument);
SemaphoreHandle_t semOpenlog_send;

TaskHandle_t Log_Handle;
void tskLog(void *argument);
SemaphoreHandle_t semLog;


TaskHandle_t laserFlowTaskHandle;
void laserFlow_main(void *argument);
SemaphoreHandle_t semlaserFlow;

/* Definitions for mocapTask */
TaskHandle_t clawTaskHandle;
void claw_main(void *argument);

/* Definitions for slamTask */
TaskHandle_t slamTaskHandle;
void slam_main(void *argument);

QueueHandle_t queueInloopControl;
QueueHandle_t queueFlow;
QueueHandle_t queueHeight;
QueueHandle_t queueRC_Status;
QueueHandle_t queueBattery;
QueueHandle_t queueRCCommand;
QueueHandle_t queueGyrDat;
QueueHandle_t queueGyrDatFil;
QueueHandle_t queueAccDat;
QueueHandle_t queueAccDatFil;
QueueHandle_t queueAhrsEuler;
QueueHandle_t queueMagDat;
QueueHandle_t queueRCData;
QueueHandle_t queueMotorData;
QueueHandle_t queuePID;
QueueHandle_t queueBaroAlt;
QueueHandle_t queueControlTransfer;
QueueHandle_t queueControlOutputDat;
QueueHandle_t queueGps;
//QueueHandle_t queueOrdinaryGps;
QueueHandle_t queueMag;
QueueHandle_t queueEKF;
QueueHandle_t queueESKF;
QueueHandle_t queueESKF_baro;

QueueHandle_t queueDownsampleIMU;

QueueHandle_t queueFlyPoint;
QueueHandle_t queueTargetBuffer;
QueueHandle_t queuetrajectoryData;
QueueHandle_t queuelaserFlow;
TimerHandle_t myTimer1ms;

QueueHandle_t queueClaw;
QueueHandle_t queueSlam;


SYSTEM::SYSTEM()
{

}

SYSTEM::~SYSTEM()
{

}


/**********************************
 * function : 创建用户任务
 *
 * task list :	 任务句柄				任务功能					执行时间
 * 				icm20602Task		: 陀螺仪加速度计数据采集			31us
 * 				motorTask			: 电机驱动					24us
 * 				lsm303dTask			: 磁力计数据采集				15us
 * 				ahrsTask			: 姿态解算					195us
 * 				attitudeCtrlTask	: 内环控制（角度角速度串级PID控制）
 * 				tranTask			: 飞控数据发送到地面站
 * 				tranReceiveTask		: 地面站数据发送到飞控
 * 				ledTask				: LED报警
 * 				batteryTask			: 电池电压采集
 * 				rcTask				: 遥控器
 *
 * para			:void
 *
 * return		:void
 *
 * *************************************/
void SYSTEM::taskCreat()
{

	isIcm20602TaskCreatSuccess = xTaskCreate ((TaskFunction_t)Icm20602_main, "icm20602Task", (uint16_t)(128 * 4), NULL, (osPriority_t) osPriorityRealtime5, &icm20602TaskHandle);
	isMagTaskCreatSuccess = xTaskCreate ((TaskFunction_t)lsm303d_main, "lsm303dTask", (uint16_t)(128 * 4), NULL, (osPriority_t) osPriorityRealtime1, &lsm303dTaskHandle);
//	isMs5611TaskCreatSuccess = xTaskCreate ((TaskFunction_t)ms5611_main, "ms5611Task", (uint16_t)(128 * 4), NULL, (osPriority_t) osPriorityNormal1, &ms5611TaskHandle);
	isDps368TaskCreatSuccess = xTaskCreate ((TaskFunction_t)DPS368_main, "dps368Task", (uint16_t)(128 * 4), NULL, (osPriority_t) osPriorityRealtime3, &dps368TaskHandle);
	isAhrsTaskCreatSuccess = xTaskCreate ((TaskFunction_t)ahrs_main, "ahrsTask", (uint16_t)(256 * 4), NULL, (osPriority_t) osPriorityHigh7, &ahrsTaskHandle);
	isBatteryTaskCreatSuccess = xTaskCreate ((TaskFunction_t)battery_main, "batteryTask", (uint16_t)(128 * 4), NULL, (osPriority_t) osPriorityBelowNormal1, &batteryTaskHandle);
	isBellTaskCreatSuccess = xTaskCreate ((TaskFunction_t)bell_main, "bellTask", (uint16_t)(128 * 4), NULL, (osPriority_t) osPriorityBelowNormal1, &bellTaskHandle);
	isControlTaskCreatSuccess = xTaskCreate ((TaskFunction_t)control_main, "ControlTask", (uint16_t)(128 * 4), NULL, (osPriority_t) osPriorityHigh5, &ControlTaskHandle);
	isMotorTaskCreatSuccess = xTaskCreate ((TaskFunction_t)motor_main, "motorTask", (uint16_t)(128 * 4), NULL, (osPriority_t) osPriorityHigh3, &motorTaskHandle);
	isTranTaskCreatSuccess = xTaskCreate ((TaskFunction_t)tran_main, "tranTask", (uint16_t)(128 * 4), NULL, (osPriority_t) osPriorityAboveNormal3, &tranTaskHandle);
	isTranReceiveTaskCreatSuccess = xTaskCreate ((TaskFunction_t)tranReceive_main, "tranReceiveTask", (uint16_t)(128 * 4), NULL, (osPriority_t) osPriorityAboveNormal4, &tranReceiveTaskHandle);
	isTranReceiveTaskCreatSuccess = xTaskCreate ((TaskFunction_t)gpsmag_main, "gpsmagTask", (uint16_t)(128 * 4), NULL, (osPriority_t) osPriorityAboveNormal5, &gpsmagTaskHandle);
//	isTranReceiveTaskCreatSuccess = xTaskCreate ((TaskFunction_t)OrdinaryGps_main, "OrdinaryGpsTask", (uint16_t)(128 * 4), NULL, (osPriority_t) osPriorityAboveNormal4, &OrdinaryGpsTaskHandle);
	isTranReceiveTaskCreatSuccess = xTaskCreate ((TaskFunction_t)RC_Check_main, "RcCheckTask", (uint16_t)(128 * 4), NULL, (osPriority_t) osPriorityAboveNormal5, &RcCheckTaskHandle);
//	isTranReceiveTaskCreatSuccess = xTaskCreate ((TaskFunction_t)ekf_main, "EkfTask", (uint16_t)(512 * 4), NULL, (osPriority_t) osPriorityHigh7, &ekfTaskHandle);
	isTranReceiveTaskCreatSuccess = xTaskCreate ((TaskFunction_t)eskf_main, "EskfTask", (uint16_t)(1024 * 4), NULL, (osPriority_t) osPriorityHigh7, &eskfTaskHandle);
//	isTranReceiveTaskCreatSuccess = xTaskCreate ((TaskFunction_t)eskf_baro_main, "Eskf_baroTask", (uint16_t)(1024 * 4), NULL, (osPriority_t) osPriorityHigh7, &eskf_baroTaskHandle);

//	isTranReceiveTaskCreatSuccess = xTaskCreate ((TaskFunction_t)rm3100_main, "rm3100Task", (uint16_t)(128 * 4), NULL, (osPriority_t) osPriorityRealtime1, &rm3100TaskHandle);
//	isFlowTaskCreatSuccess = xTaskCreate((TaskFunction_t)flow_main, "flowTask", (uint16_t)(128 * 4), NULL, (osPriority_t) osPriorityAboveNormal7, &flowTaskHandle);

	/* 在这里添加你的任务 */
//	xTaskCreate((TaskFunction_t)tskLog, 			"App.Log",    (uint16_t)(128 * 4), NULL, (osPriority_t) osPriorityAboveNormal2, &Log_Handle);
	xTaskCreate((TaskFunction_t)laserFlow_main, 			"laserFlow",    (uint16_t)(128 * 4), NULL, (osPriority_t) osPriorityAboveNormal5, &laserFlowTaskHandle);
	isClawTaskCreatSuccess = xTaskCreate ((TaskFunction_t)claw_main, "clawTask", (uint16_t)(256 * 4), NULL, (osPriority_t) osPriorityRealtime, &clawTaskHandle);
	isSlamTaskCreatSuccess = xTaskCreate ((TaskFunction_t)slam_main, "slamTask", (uint16_t)(256 * 4), NULL, (osPriority_t) osPriorityRealtime, &slamTaskHandle);
	/* 在这里添加你的任务 */

}

void SYSTEM::semCreat()
{

	semControl = xSemaphoreCreateBinary();
	semAhrs = xSemaphoreCreateBinary();
	semBattery = xSemaphoreCreateBinary();
	semBell = xSemaphoreCreateBinary();
	semIcm20602 = xSemaphoreCreateBinary();
	semLsm303d = xSemaphoreCreateBinary();
	semMotor = xSemaphoreCreateBinary();
//	semMs5611 = xSemaphoreCreateBinary();
	semDPS368 = xSemaphoreCreateBinary();
	semTran = xSemaphoreCreateBinary();
	semRcCheck = xSemaphoreCreateBinary();
	semEkf = xSemaphoreCreateBinary();
	semEskf = xSemaphoreCreateBinary();
	semEskf_baro = xSemaphoreCreateBinary();

//	semRm3100 = xSemaphoreCreateBinary();
	/* 在这里添加你的信号量 */
	semLog = xSemaphoreCreateBinary();
	semlaserFlow = xSemaphoreCreateBinary();
	/* 在这里添加你的信号量 */
}

void SYSTEM::timerCreat()
{
	myTimer1ms = xTimerCreate ("myTimer", 1, pdTRUE, (void *)0, (TimerCallbackFunction_t)Timer1msCallback);
	xTimerStart(myTimer1ms,0);
}

void SYSTEM::queueCreat()
{
	queueRCCommand = xQueueCreate(1,sizeof(RC_command_msg));
	queueBattery = xQueueCreate(1,sizeof(battery_msg));
	queueFlow = xQueueCreate(1,sizeof(flow_msg));
	queueHeight = xQueueCreate(1,sizeof(height_msg));
	queueRC_Status = xQueueCreate(1,sizeof(RC_Status_msg));
	queueAccDat = xQueueCreate(1,sizeof(sensor_acc_msg));
	queueAccDatFil = xQueueCreate(1,sizeof(sensor_acc_filter_msg));
	queueGyrDat = xQueueCreate(1,sizeof(sensor_gyro_msg));
	queueGyrDatFil = xQueueCreate(1,sizeof(sensor_gyro_filter_msg));
	queueAhrsEuler = xQueueCreate(1,sizeof(ahrs_euler_msg));
	queueMagDat = xQueueCreate(1,sizeof(sensor_mag_msg));
	queueRCData = xQueueCreate(1,sizeof(RCData));
    queueMotorData = xQueueCreate(1,sizeof(RCData));
    queuePID = xQueueCreate(1,sizeof(PID_msg));
    queueBaroAlt = xQueueCreate(1,sizeof(sensor_baroAlt_msg));
    queueControlTransfer = xQueueCreate(1,sizeof(control_transfer_msg));
    queueControlOutputDat = xQueueCreate(1,sizeof(Control_output_msg));
    queueGps = xQueueCreate(1,sizeof(gps_msg));
//    queueOrdinaryGps = xQueueCreate(1,sizeof(OrdinaryGps_msg));

    queueMag =  xQueueCreate(1,sizeof(mag_msg));
    queueEKF = xQueueCreate(1,sizeof(ekf_cplfil_msg));
    queueESKF = xQueueCreate(1,sizeof(eskf_msg));
    queueESKF_baro = xQueueCreate(1,sizeof(eskf_baro_msg));

    queueDownsampleIMU = xQueueCreate(1,sizeof(downsample_imu_for_eskf_msg));
	/* 在这里添加你的消息队列 */
    queueTargetBuffer = xQueueCreate(1,sizeof(Target_Buffer));
    queueFlyPoint = xQueueCreate(10,sizeof(fly_point));//容纳10个航点
    queuetrajectoryData = xQueueCreate(1,sizeof(trajectory_msg));
    queuelaserFlow = xQueueCreate(1,sizeof(laserFlow_msg));
    queueClaw = xQueueCreate(1,sizeof(CLAW_msg));
    queueSlam = xQueueCreate(1,sizeof(SLAM_msg));
	/* 在这里添加你的消息队列 */

}

SYSTEM system0;
/* 在启动实时操作系统之前调用该函数 */
void mySystemInit(void)
{
	system0.taskCreat();				//任务的创建
	system0.semCreat();				//信号量的创建,用于任务的同步
//	system0.timerCreat();			//软件定时器的创建,用于任务的时间片调度
	system0.queueCreat();			//消息队列的创建,用于任务之间的通信
}
void TIM5_IRQHandler(void)
{
	if(LL_TIM_IsActiveFlag_UPDATE(TIM5))
	{
		LL_TIM_ClearFlag_UPDATE(TIM5);
		system0.startTimerLast = system0.startTimer;
		getTimer_us(&system0.startTimer);
		system0.cycleTime_us = system0.startTimer - system0.startTimerLast;

		//任务时间片调度,设定定时任务时间片长度,单位为ms
		static uint32_t _cnt = 0;
		const uint32_t taskPeriod[20] = {1,5,10,20,10};
		if(_cnt%taskPeriod[0] == taskPeriod[0]-1)
		{
			osSemaphoreRelease(semIcm20602);
//			osSemaphoreRelease(semBell);
		}
		if(_cnt%taskPeriod[1] == taskPeriod[1]-1)
		{
			osSemaphoreRelease(semAhrs);
//			osSemaphoreRelease(semControl);
//			osSemaphoreRelease(semMotor);
//			osSemaphoreRelease(semEskf);

		}
		if(_cnt%taskPeriod[2] == taskPeriod[2]-1)
		{
//			osSemaphoreRelease(semLog);
//			osSemaphoreRelease(semRm3100);
			osSemaphoreRelease(semEskf);
//			osSemaphoreRelease(semEskf_baro);

//			osSemaphoreRelease(semMs5611);
//			osSemaphoreRelease(semDPS368);
	//		osSemaphoreRelease(semTran);
//			osSemaphoreRelease(semEkf);
			osSemaphoreRelease(semControl);
			osSemaphoreRelease(semMotor);
			osSemaphoreRelease(semRcCheck);
		}
		if(_cnt%taskPeriod[3] == taskPeriod[3]-1)
		{
	//		osSemaphoreRelease(semLsm303d);
			osSemaphoreRelease(semTran);
			osSemaphoreRelease(semBattery);

		}
		if(_cnt%taskPeriod[4] == taskPeriod[4]-1)
		{
			osSemaphoreRelease(semDPS368);
		}
		_cnt++;

		getTimer_us(&system0.stopTimer);

		system0.executionTime_us = system0.stopTimer - system0.startTimer;
	}
}
//void Timer1msCallback(void *argument)getRearPtr
//{
//	//任务时间片调度,设定定时任务时间片长度,单位为ms
//	static uint32_t _cnt = 0;
//	const uint32_t taskPeriod[20] = {1,5,10,20,100};
//	if(_cnt%taskPeriod[0] == taskPeriod[0]-1)
//	{
//		osSemaphoreRelease(semIcm20602);
//		osSemaphoreRelease(semBell);
//	}
//	if(_cnt%taskPeriod[1] == taskPeriod[1]-1)
//	{
//		osSemaphoreRelease(semAhrs);
//	}
//	if(_cnt%taskPeriod[2] == taskPeriod[2]-1)
//	{
//		osSemaphoreRelease(semEskf);
////		osSemaphoreRelease(semMs5611);
//		osSemaphoreRelease(semTran);
//		osSemaphoreRelease(semEkf);
//		osSemaphoreRelease(semControl);
//		osSemaphoreRelease(semMotor);
//		osSemaphoreRelease(semRcCheck);
//	}
//	if(_cnt%taskPeriod[3] == taskPeriod[3]-1)
//	{
////		osSemaphoreRelease(semLsm303d);
//	}
//	if(_cnt%taskPeriod[3] == taskPeriod[3]-1)
//	{
//		osSemaphoreRelease(semBattery);
//	}
//	_cnt++;
//}


/************************ (C) COPYRIGHT Longmen Drone Team *****END OF FILE****/
