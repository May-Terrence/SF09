/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : rc.hpp
  * Description        :
  ******************************************************************************
  * Function           :
  *
  ******************************************************************************
  * Author             : Zhiping Fu
  * Creation Date      : 2020年10月23日
  ******************************************************************************
  */
/* USER CODE END Header */

#ifndef __RC_HPP
#define __RC_HPP
//#define tran_U6
#define tran_U3
#include "system/system.hpp"
#include "userlib/userlib.hpp"
#include "string.h"
#include "gpsMag/gpsMag.hpp"

#ifdef __cplusplus
extern "C" {
#endif


#define TRAN_RX_LEN		300
#define TRAN_TX_LEN		100

#define RC_RX_LEN 300
#define RC_TX_LEN 30
#define PPM_NUM 27   //开头只有1个$，两个$的是28个
#define PWM_RC_NUM 8


#define RC_FUN_MIN			1200   //判别用，不是实际的
#define RC_FUN_MAX			1800
#define RC_PPM_MIN          1000   //实际
#define RC_PPM_MID          1500   //实际
#define RC_PPM_MAX          2000   //实际

#define RC_CHECK_NUM 400

#ifdef tran_U3
void USART3_IRQHandler(void);
void DMA1_Stream1_IRQHandler(void);  //接收DMA中断
void DMA1_Stream3_IRQHandler(void);
#endif

#ifdef tran_U6
void USART6_IRQHandler(void);
void DMA2_Stream1_IRQHandler(void);  //接收DMA中断
void DMA2_Stream6_IRQHandler(void);
#endif

#pragma pack(1)
struct s_cur_NED_position
{
    float x;
    float y;
    float z;
    float x_command;
    float y_command;
    float z_command;
    float u;
    float v;
    float w;
};
#pragma pack()

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
	bool uart_Send_NED(void);
	bool uart_Send_Paths(void);
	bool uart_Send_Base_Station_Data(uint16_t len,uint8_t *buffer,uint16_t index);


	void uart_Receive_Update(void);
	void uart_Set_Pid_Para(uint8_t group);
	void delete_fly_point();
	void write_fly_point();

	HAL_LockTypeDef LockRx;
	bool  RxFlag;		//接收标志位
	uint16_t   RxDataSize;
	uint8_t    RxRawDat[TRAN_RX_LEN];	//数据
	uint8_t    RxDat[TRAN_RX_LEN];
	bool  TxFlag;		//接收标志位
	USART_TypeDef * huart;
	static void set_plot_path();

	bool send_log;
	uint16_t space;
	uint8_t buffer[2000];
	uint16_t read_index;

	uint8_t Rdata[3]={0,1,2};
	uint16_t Rsize=3;

private:
	static bool plot_path;

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

	uint32_t testCounter;

	sensor_gyro_msg gyro;
	sensor_acc_msg acc;
	sensor_mag_msg mag;
	ahrs_euler_msg ahrsEuler;
	eskf_msg eskf;
	eskf_baro_msg eskf_baro;
	sensor_baroAlt_msg baroAlt;

	ekf_cplfil_msg ekfcpf;
	mag_msg magb;
	gps_msg gps;
	OrdinaryGps_msg ordinaryGps;
	trajectory_msg trajectoryData;
	laserFlow_msg laserFlow;
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
	eskf_msg mahany;

	Target_Buffer target_buffer;
	Control_output_msg control_output_msg;
	sensor_acc_filter_msg acc_filter;
	sensor_acc_filter_msg acc_fil;
	int path_size;

};

class RC
{
public:
	RC(){}
	RC(USART_TypeDef * h,char * n) : huart(h),name(n){}
	~RC(){}

	void rc_Init(void);
	void rc_Update(void);
	void ControlRC_Check(void);

	HAL_LockTypeDef LockRx;
	bool  RxFlag;		//接收标志位
	uint16_t   RxDataSize;
	uint8_t RxRawDat[RC_RX_LEN];   //数据
	char RxDat[RC_RX_LEN];

	eRC rc_status;
	bool ever_rc;
	RC_Status_msg rc_status_msg;
private:
	USART_TypeDef * huart;
	char *name;

	uint32_t  startTimer;			//计时器
	uint32_t  stopTimer;			//计时器
	uint32_t  executionTime_us;			//计时器

	uint32_t  startTimerLast;			//计时器
	uint32_t  cycleTime_us;

	//用户数据
	bool  Update;		//更新  --
	eSTA  Sta;			//状态  --
	eERR  Err;			//错误信息  --
	u8    Pack;       //收包个数，1s
	eRC_MODE  Mode;   //行为模式
	sCNT ModeChk;

	u16  PPM[PWM_RC_NUM];
	float Val[4];     //roll pitch yaw high
	float Ang[2];     //遥控器角度 roll pitch
	float dAng;       //yaw角速度
	float Thr;        //遥控器油门
	float Uvw[3];     //遥控器Uvw
	float a,b,c;      //油门2次曲线
	s16   HIG_THR;    //油门高位点
	s16   MID_THR;    //油门中位点
	s16   LOW_THR;    //油门低位点
	u8    Key[4];     //4个开关

	bool rc_buffer[RC_CHECK_NUM];
	u16 rc_cnt;
	u16 rc_check;

	RCData rc_ppm;
	RC_command_msg rcCommand;
};

extern RC rc;

extern TRAN tran;

#endif

#endif


/************************ (C) COPYRIGHT Longmen Drone Team *****END OF FILE****/
