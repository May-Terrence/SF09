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

#include "userlib/userlib.hpp"
#include "system/system.hpp"

#ifdef __cplusplus
extern "C" {
#endif

#define RC_SBUS

#ifdef RC_SBUS
	#define RC_RX_LEN 30
	#define PPM_NUM 25   //SBUS协议一个完整的数据包有25个字节
	#define PWM_RC_NUM 16

#else
	#define RC_RX_LEN 30
	#define PPM_NUM 27   //开头只有1个$，两个$的是28个
	#define PWM_RC_NUM 8

#endif

	#define RC_FUN_MIN			1220   //判别用，不是实际的
	#define RC_FUN_MAX			1800
	#define RC_PPM_MIN          1000   //实际
	#define RC_PPM_MID          1500   //实际
	#define RC_PPM_MAX          2000   //实际

	#define RC_LOW_THR			1000
	#define RC_MID_THR			1500
	#define RC_HIG_THR			2000


void USART1_IRQHandler(void);
void DMA2_Stream2_IRQHandler(void);
void DMA2_Stream7_IRQHandler(void);

#ifdef __cplusplus
}
#endif


#ifdef __cplusplus

using namespace std;

class RC
{
public:
	RC(){}
	RC(USART_TypeDef * h,char * n) : huart(h),name(n){}
	~RC(){}

	void rc_Init(void);
	void rc_Update(void);

	HAL_LockTypeDef LockRx;
	bool  RxFlag;		//接收标志位
	uint16_t   RxDataSize;
	uint8_t RxRawDat[RC_RX_LEN];   //数据
	char RxDat[RC_RX_LEN];

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

	#ifdef RC_SBUS
		uint16_t channel[PWM_RC_NUM];

	#endif

	uint8_t    Pack;       //收包个数，1s
	float Val[4];     //roll pitch yaw high
	int16_t   HIG_THR;    //油门高位点
	int16_t   MID_THR;    //油门中位点
	int16_t   LOW_THR;    //油门低位点

	RCData rcPPM;
	RC_command_msg rcCommand;
};



#endif

#endif


/************************ (C) COPYRIGHT Longmen Drone Team *****END OF FILE****/
