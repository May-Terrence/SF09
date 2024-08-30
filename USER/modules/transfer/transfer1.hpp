/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : transfer.hpp
  * Description        :
  ******************************************************************************
  * Function           :
  *
  ******************************************************************************
  * Author             : Zhiping Fu
  * Creation Date      : 2020年10月21日
  ******************************************************************************
  */
/* USER CODE END Header */

#ifndef __TRANSFER_HPP
#define __TRANSFER_HPP

#include "system/system.hpp"
#include "userlib/userlib.hpp"


#ifdef __cplusplus
extern "C" {
#endif

//#include "string.h"

#define TRAN_RX_LEN		60
#define TRAN_TX_LEN		60

void USART1_IRQHandler(void);
void DMA2_Stream2_IRQHandler(void);  //接收DMA中断
void DMA2_Stream7_IRQHandler(void);

#ifdef __cplusplus
}
#endif

#ifdef __cplusplus
using namespace std;

class TRAN
{
public:
	TRAN(){}
	TRAN(USART_TypeDef * h, char * n) : huart(h),name(n){}
	~TRAN(){}

	void tran_Init(void);

	bool uart_Send_DMA(uint8_t * pData,uint16_t Size);

	void uart_Send_DMA_loop(void);

	bool uart_Send_Sensor(void);
	bool uart_Send_Sensor2(void);
	bool uart_Send_Status(void);
	bool uart_Send_MotorPWM(void);
	bool uart_Send_rcData(void);
	bool uart_Send_Power(void);
	bool uart_Send_Version(void);
	bool uart_Send_GPS(void);
	bool uart_Send_Check(void);
	bool uart_Send_PID(uint8_t group);
	bool uart_Send_User(void);


	void uart_Receive_Update(void);
	void uart_Set_Pid_Para(uint8_t group);

	HAL_LockTypeDef LockRx;
	bool  RxFlag;		//接收标志位
	uint16_t   RxDataSize;
	uint8_t    RxRawDat[TRAN_RX_LEN];	//数据
	uint8_t    RxDat[TRAN_RX_LEN];
	bool  TxFlag;		//接收标志位
	USART_TypeDef * huart;
private:


	char * name;
	uint8_t    TxHead;
	uint8_t    TxSum;
	uint8_t    TxDat[TRAN_TX_LEN];

	bool send_version;
	bool send_pid;
	uint8_t send_pid_group;

	uint32_t  index;		//发送数据序号,每发完一帧数据加一


	uint32_t  startTimer;			//计时器
	uint32_t  stopTimer;			//计时器
	uint32_t  executionTime_us;			//计时器
	uint32_t  startTimerLast;
	uint32_t  cycleTime_us;
	//用户数据
	bool  Update;		//更新  --
	eSTA  Sta;			//状态  --
	eERR  Err;			//错误信息  --


	sensor_gyro_msg gyro;
	sensor_acc_msg acc;
	sensor_mag_msg mag;
	ahrs_euler_msg ahrsEuler;
	eskf_msg eskf;
	ekf_cplfil_msg ekfcpf;
	mag_msg magb;
	gps_msg gps;
	battery_msg battery;
	RCData rcPPM;
	PID_msg pid;
	Motor_msg motor_msg;
	RC_command_msg rcCommand;
	sensor_gyro_filter_msg gyro_filter;
	control_transfer_msg control_data;
	height_msg height;
	flow_msg flow;
	RC_Status_msg rc_status_msg;

	downsample_imu_for_eskf_msg imu;
};




#endif



#endif

/************************ (C) COPYRIGHT Longmen Drone Team *****END OF FILE****/
