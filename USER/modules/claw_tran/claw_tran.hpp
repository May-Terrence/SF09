/*
 * claw_tran.hpp
 *
 *  Created on: 2024年8月27日
 *      Author: mengc
 */

#ifndef MODULES_CLAW_TRAN_CLAW_TRAN_HPP_
#define MODULES_CLAW_TRAN_CLAW_TRAN_HPP_

#include "system/system.hpp"
#include "userlib/userlib.hpp"

#ifdef __cplusplus
extern "C" {
#endif

#define CLAW_RX_LEN 60
#define CLAW_TX_LEN 30

void USART6_IRQHandler(void);
void DMA2_Stream1_IRQHandler(void);
void DMA2_Stream6_IRQHandler(void);

#ifdef __cplusplus
}
#endif


#ifdef __cplusplus

using namespace std;

class CLAW
{
public:
	CLAW(){}
	CLAW(USART_TypeDef * h,char * n) : huart(h),name(n){}
	~CLAW(){}

	void claw_Init(void);
	void claw_Update(void);
	bool uart_Send_DMA(uint8_t * pData,uint16_t Size);
	void Open_Request_Tran(void);
	void Close_Request_Tran(void);
//	void Start_Request_Tran(void);
//	void End_Request_Tran(void);

	HAL_LockTypeDef LockRx;
	bool  TxFlag;		//标志位
	bool  RxFlag;		//接收标志位
	uint16_t   RxDataSize;
	uint8_t RxRawDat[CLAW_RX_LEN];   //数据
	uint8_t TxDat[CLAW_TX_LEN];
	char RxDat[CLAW_RX_LEN];

	bool isOpen{false};
	bool isClose{true};
	bool isConnect{false};
	bool isUpdate{false};
	bool noMove{false};
	CLAW_msg claw_msg;
	float last_Pos[2]{0};

private:
	USART_TypeDef * huart;
	char *name;
	uint8_t RxSum;
	BIT32 temp;

	uint32_t  startTimer;			//计时器
	uint32_t  stopTimer;			//计时器
	uint32_t  executionTime_us;		//计时器

	uint32_t  startTimerLast;		//计时器
	uint32_t  cycleTime_us;

	//用户数据
	bool  Update;		//更新  --
	eSTA  Sta;			//状态  --
	eERR  Err;			//错误信息  --
};

#endif

#pragma pack(1)
typedef struct
{
	float position_N;
	float position_E;
	float position_D;
	float yaw;

	uint8_t satellites_used;
	uint8_t work_mode;
}claw_msg_update;

extern CLAW claw;

#endif /* MODULES_CLAW_TRAN_CLAW_TRAN_HPP_ */
