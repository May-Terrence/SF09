/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : motor.hpp
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

#ifndef __MOTOR_HPP
#define __MOTOR_HPP

#include "system/system.hpp"
#include "userlib/userlib.hpp"
#include "hardware.h"
//#define motor_U6
#define motor_U8

#ifdef __cplusplus
extern "C" {
#endif

#define Servo_num_8

#define PWM_OUT_NUM 10
#define INI_PWM 1500
#define Min_PWM_Out  800 //800    //us
#define Max_PWM_Out  2500//2200   //us
#define teest 10

#define RAD_TO_PWM 501.338f			//
#define Fy_TO_PWM 29.17f            //(175/6)
//#define Mx_Motor_Factor 1.2f       //(43/36)用于抵消在平衡点时20°舵角（对应力是6，取平方36）带来的X轴力矩 两个电机的PWM需要分别加减43
#define Mx_Motor_Factor 7.16f       //(43/6)用于抵消在平衡点时20°舵角（对应力是6）带来的X轴力矩 两个电机的PWM需要分别加减43


/*------------------TALON GT Rebel---------------------------*/
//#define RAD_TO_PWM 960.51f//			//
#define AILERON_LIMIT_MIN -400
#define AILERON_LIMIT_MAX 400     //400对应23.87度
#define RUDDERVATOR_LIMIT_MIN -500
#define RUDDERVATOR_LIMIT_MAX 500


#ifdef MF09II02
#define CS_LIMIT_MIN -350
#define CS_LIMIT_MAX 350    //175对应20度(0.349rad)


#define PWM_MID_0 1090


#define PWM_MID_1 1515//1556

#define PWM_MID_2 1595//1526
#define PWM_MID_3 1530//1586
#define PWM_MID_4 1500//1527


#define PWM_MID_5 1540
#define PWM_MID_6 1413


#define PWM_MID_7 1466
#define PWM_MID_8 1250
#define PWM_MID_9 1453
#endif

#ifdef MF09II01
#define CS_LIMIT_MIN -340
#define CS_LIMIT_MAX 340     //175对应20度(0.349rad)

#define PWM_MID_0 1090

#define PWM_MID_1 1517//1621//1573motor2
#define PWM_MID_2 1500//1437//1581
#define PWM_MID_3 1372//1420//1607motor4
#define PWM_MID_4 1568//1615//1583

#define PWM_MID_5 1540
#define PWM_MID_6 1413

#define PWM_MID_7 1466
#define PWM_MID_8 1250
#define PWM_MID_9 1453
#endif

#ifdef MF09II03
#define CS_LIMIT_MIN -340
#define CS_LIMIT_MAX 340     //175对应20度(0.349rad)

#define PWM_MID_0 1090

#define PWM_MID_1 1500
#define PWM_MID_2 1585
#define PWM_MID_3 1500
#define PWM_MID_4 1515

#define PWM_MID_5 1540
#define PWM_MID_6 1413

#define PWM_MID_7 1466
#define PWM_MID_8 1250
#define PWM_MID_9 1453
#endif

#define TRAN_TX_LEN		PWM_OUT_NUM*2
#define TRAN_RX_LEN		11

#ifdef motor_U8
#define Motor_DMA			DMA1
#define Motor_DMA_Stream	LL_DMA_STREAM_0
#define Motor_DMA_RX		LL_DMA_STREAM_6
#define Motor_DMA_UART		UART8

void UART8_IRQHandler(void);
void DMA1_Stream6_IRQHandler(void);//接收DMA中断
void DMA1_Stream0_IRQHandler(void);//发送DMA中断
#endif

#ifdef motor_U6
#define Motor_DMA			DMA2
#define Motor_DMA_Stream	LL_DMA_STREAM_6
#define Motor_DMA_RX		LL_DMA_STREAM_1
#define Motor_DMA_UART		USART6

void USART6_IRQHandler(void);
void DMA2_Stream1_IRQHandler(void);  //接收DMA中断
void DMA2_Stream6_IRQHandler(void);	//发送DMA中断
#endif



#ifdef __cplusplus
}
#endif


#ifdef __cplusplus

using namespace std;

class MOTORMAG
{
public:

	MOTORMAG(USART_TypeDef *h,char *n,char *s) : huart(h),name(n),DirChr(s){};

	USART_TypeDef * huart;


	void motor_Init(void);
	void motor_run(void);

	bool uart_Send_DMA(uint8_t * pData,uint16_t Size);

	bool  TxFlag;		//发送标志位
	bool  RxFlag;		//接收标志位
	int32_t MagRaw[3];
	int32_t LastMagRaw[3];
	uint8_t rm_dat[20];
	uint8_t RxDat[20];
	uint16_t   RxDataSize;
	HAL_LockTypeDef LockRx;

	uint16_t   PID_MID[8] = {PWM_MID_2,PWM_MID_3,PWM_MID_4,PWM_MID_5,PWM_MID_6,PWM_MID_7,PWM_MID_8,PWM_MID_9};
private:

//	TIM_TypeDef *htimA;
//	TIM_TypeDef *htimB;
	char *name;
	char *DirChr;


	bool  Update;     //更新  --
	eSTA  Sta;        //状态  --
	eERR  Err;        //错误信息  --
	bool  UnLock;     //解锁
	uint8_t requestTelemetry;




	u16 PWM[PWM_OUT_NUM+1];//最后一位为通信校验位FFFF
	u16 PWM_OBS[PWM_OUT_NUM];
	s16 PwmOff[PWM_OUT_NUM];
	double p_limits;//控制舵幅值（度）

	uint32_t  startTimer;			//计时器
	uint32_t  stopTimer;			//计时器
	uint32_t  executionTime_us;			//计时器

	uint32_t  startTimerLast;			//计时器
	uint32_t  cycleTime_us;

	uint32_t  InitCounter;

	RC_command_msg rcCommand;
	Control_output_msg control_output;
	Motor_msg motor_msg;
	PID_msg pid;

	mag_msg mag;
	u32   mag_update_time_us;


	int8_t  Dir[6];     //方向  --
	bool MagUpdate;
	bool MagFlag;
	eSTA MagSta;		//状态  --

	float   MagOff[3];  //偏置
//	s16     MagRaw[3];  //原始值
	float   MagRot[3];  //旋转值
	float MagRel[3];  //实际值
	float MagFil[3];

	uint8_t    TxSum;

};

#endif


#endif


/************************ (C) COPYRIGHT Longmen Drone Team *****END OF FILE****/
