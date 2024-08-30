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
/* Definitions for icm20602GyroTask */
TaskHandle_t icm20602GyroTaskHandle;
void Icm20602_main_gyro(void *argument);
SemaphoreHandle_t semIcm20602Gyr;

/* Definitions for icm20602AccTask */
TaskHandle_t icm20602AccTaskHandle;
void Icm20602_main_acc(void *argument);
SemaphoreHandle_t semIcm20602Acc;

/* Definitions for magnetometerTask */
TaskHandle_t magnetometerTaskHandle;
void Ak8975_main_mag(void *argument);
SemaphoreHandle_t semAk8975Mag;

/* Definitions for ahrsTask */
TaskHandle_t ahrsTaskHandle;
void ahrs_main(void *argument);
SemaphoreHandle_t semAhrs;

/* Definitions for tranTask */
TaskHandle_t tranTaskHandle;
void tran_main(void *argument);
SemaphoreHandle_t semTran;

/* Definitions for ledTask */
TaskHandle_t ledTaskHandle;
void led_main(void *argument);
SemaphoreHandle_t semLed;

/* Definitions for batteryTask */
TaskHandle_t batteryTaskHandle;
void battery_main(void *argument);
SemaphoreHandle_t semBattery;

/* Definitions for rcTask */
TaskHandle_t rcTaskHandle;
void rc_main(void *argument);

/* Definitions for motorTask */
TaskHandle_t motorTaskHandle;
void motor_main(void *argument);
SemaphoreHandle_t semMotor;

/* Definitions for attitudeCtrlTask */
TaskHandle_t attitudeCtrlTaskHandle;
void attitudeCtrl_main(void *argument);
SemaphoreHandle_t semAttitudeCtrl;

/* Definitions for tranReceiveTask */
TaskHandle_t tranReceiveTaskHandle;
void tranReceive_main(void *argument);

//自己创建的任务


QueueHandle_t queueGyrDat;
QueueHandle_t queueAccDat;
QueueHandle_t queueMagDat;
QueueHandle_t queueAhrsEuler;
QueueHandle_t queueBattery;
QueueHandle_t queueRCData;
QueueHandle_t queueRCCommand;
QueueHandle_t queueMotorPWM;
QueueHandle_t queuePID;

TimerHandle_t myTimer1ms;


SYSTEM::SYSTEM()
{

}

SYSTEM::~SYSTEM()
{

}

//Ahrs任务耗时大概176us
void SYSTEM::taskCreat()
{
	isGyroTaskCreatSuccess = xTaskCreate ((TaskFunction_t)Icm20602_main_gyro, "icm20602GyrTask", (uint16_t)(128 * 4), NULL, (osPriority_t) osPriorityRealtime5, &icm20602GyroTaskHandle);
	isMotorTaskCreatSuccess = xTaskCreate ((TaskFunction_t)motor_main, "motorTask", (uint16_t)(128 * 4), NULL, (osPriority_t) osPriorityHigh3, &motorTaskHandle);
	isAccTaskCreatSuccess = xTaskCreate ((TaskFunction_t)Icm20602_main_acc, "icm20602AccTask", (uint16_t)(128 * 4), NULL, (osPriority_t) osPriorityRealtime3, &icm20602AccTaskHandle);
	isMagTaskCreatSuccess = xTaskCreate ((TaskFunction_t)Ak8975_main_mag, "ak8975MagTask", (uint16_t)(128 * 4), NULL, (osPriority_t) osPriorityRealtime1, &magnetometerTaskHandle);
	isAhrsTaskCreatSuccess = xTaskCreate ((TaskFunction_t)ahrs_main, "ahrsTask", (uint16_t)(256 * 4), NULL, (osPriority_t) osPriorityHigh7, &ahrsTaskHandle);
	isAttitudeCtrlTaskCreatSuccess = xTaskCreate ((TaskFunction_t)attitudeCtrl_main, "attitudeCtrlTask", (uint16_t)(128 * 4), NULL, (osPriority_t) osPriorityHigh5, &attitudeCtrlTaskHandle);
	isTranTaskCreatSuccess = xTaskCreate ((TaskFunction_t)tran_main, "tranTask", (uint16_t)(128 * 4), NULL, (osPriority_t) osPriorityAboveNormal3, &tranTaskHandle);
	isTranReceiveTaskCreatSuccess = xTaskCreate ((TaskFunction_t)tranReceive_main, "tranReceiveTask", (uint16_t)(128 * 4), NULL, (osPriority_t) osPriorityAboveNormal5, &tranReceiveTaskHandle);
	isLedTaskCreatSuccess = xTaskCreate ((TaskFunction_t)led_main, "ledTask", (uint16_t)(128 * 4), NULL, (osPriority_t) osPriorityBelowNormal, &ledTaskHandle);
	isBatteryTaskCreatSuccess = xTaskCreate ((TaskFunction_t)battery_main, "batteryTask", (uint16_t)(128 * 4), NULL, (osPriority_t) osPriorityBelowNormal1, &batteryTaskHandle);
	isRcTaskCreatSuccess = xTaskCreate ((TaskFunction_t)rc_main, "rcTask", (uint16_t)(128 * 4), NULL, (osPriority_t) osPriorityAboveNormal5, &rcTaskHandle);
	/* 在这里添加你的任务 */

	/* 在这里添加你的任务 */

}

void SYSTEM::semCreat()
{
	semIcm20602Gyr = xSemaphoreCreateBinary();
	semMotor = xSemaphoreCreateBinary();
	semIcm20602Acc = xSemaphoreCreateBinary();
	semAk8975Mag = xSemaphoreCreateBinary();
	semAhrs = xSemaphoreCreateBinary();
	semAttitudeCtrl = xSemaphoreCreateBinary();
	semTran = xSemaphoreCreateBinary();
	semLed = xSemaphoreCreateBinary();
	semBattery = xSemaphoreCreateBinary();
	/* 在这里添加你的信号量 */

	/* 在这里添加你的信号量 */
}

void SYSTEM::timerCreat()
{
	myTimer1ms = xTimerCreate ("myTimer", 1, pdTRUE, (void *)0, (TimerCallbackFunction_t)Timer1msCallback);
	xTimerStart(myTimer1ms,0);
}

void SYSTEM::queueCreat()
{
	queueGyrDat = xQueueCreate(1,sizeof(sensor_gyro_msg));
	queueAccDat = xQueueCreate(1,sizeof(sensor_acc_msg));
	queueMagDat = xQueueCreate(1,sizeof(sensor_mag_msg));
	queueAhrsEuler = xQueueCreate(1,sizeof(ahrs_euler_msg));
	queueBattery = xQueueCreate(1,sizeof(battery_msg));
	queueRCData = xQueueCreate(1,sizeof(RCData));
	queueRCCommand = xQueueCreate(1,sizeof(RC_command_msg));
	queueMotorPWM = xQueueCreate(1,sizeof(Motor_PWM_msg));
	queuePID = xQueueCreate(1,sizeof(Pid_msg));
	/* 在这里添加你的消息队列 */

	/* 在这里添加你的消息队列 */

}

SYSTEM system;
/* 在启动实时操作系统之前调用该函数 */
void mySystemInit(void)
{
	system.taskCreat();				//任务的创建
	system.semCreat();				//信号量的创建,用于任务的同步
	system.timerCreat();			//软件定时器的创建,用于任务的时间片调度
	system.queueCreat();			//消息队列的创建,用于任务之间的通信

}
void Timer1msCallback(void *argument)
{
	//任务时间片调度,设定定时任务时间片长度,单位为ms
	static uint32_t _cnt = 0;
	const uint32_t taskPeriod[20] = {5,10,200};
	if(_cnt%taskPeriod[0] == taskPeriod[0]-1)
	{
		osSemaphoreRelease(semIcm20602Gyr);
		osSemaphoreRelease(semMotor);
		osSemaphoreRelease(semIcm20602Acc);
		osSemaphoreRelease(semAhrs);
		osSemaphoreRelease(semAttitudeCtrl);

	}
	if(_cnt%taskPeriod[1] == taskPeriod[1]-1)
	{
		osSemaphoreRelease(semAk8975Mag);
		osSemaphoreRelease(semTran);
	}
	if(_cnt%taskPeriod[2] == taskPeriod[2]-1)
	{
		osSemaphoreRelease(semLed);
		osSemaphoreRelease(semBattery);
	}
	_cnt++;
}


/************************ (C) COPYRIGHT Longmen Drone Team *****END OF FILE****/
