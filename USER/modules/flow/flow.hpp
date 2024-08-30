
#ifndef __FLOW_HPP
#define __FLOW_HPP

#include "system/system.hpp"
#include "userlib/userlib.hpp"
#include "hspi/hspi.hpp"
#include "led/led.hpp"


#ifdef __cplusplus
 extern "C" {
#endif

#define FLOW_RX_LEN		60
#define FLOW_TX_LEN		60


 void USART2_IRQHandler(void);
 //void DMA1_Stream5_IRQHandler(void);  //接收DMA中断
 //void DMA1_Stream3_IRQHandler(void);


#ifdef __cplusplus
}
#endif

#ifdef __cplusplus
using namespace std;


class FLOW
{
public:
	FLOW(){}
	FLOW(USART_TypeDef *h,char *n,char *s) : huart(h),name(n),DirChr(s){};
	~FLOW(){}

	USART_TypeDef * huart;
	uint16_t RxDataSize;
	uint8_t  RxRawDat[FLOW_RX_LEN];	//数据
	uint8_t  RxDat[FLOW_RX_LEN];
	HAL_LockTypeDef LockRx;
	bool  RxFlag;		//接收标志位
	bool  TxFlag;		//发送标志位

	uint8_t    TxHead;
	//uint8_t    TxSum;
	uint8_t    TxAdd;

	void flow_Init_para(void);
	void flow_Init(void);
	void flow_Update(void);

private:
	int8_t  Dir[6];     //方向  --
	char *name;
	char *DirChr;
	double Re2t[3][3];
	uint8_t  TxDat[FLOW_TX_LEN];

	s16   MagRaw[3];  //原始值
	float MagOff[3];  //偏置值
	float MagRot[3];  //旋转值
	float MagRel[3];  //实际值
	float MagFil[3];  //滤波值

	//BIT32 temp;
	uint32_t startTimer;
	uint32_t stopTimer;
	uint32_t startTimerLast;
	uint32_t executionTime_us;
	uint32_t cycleTime_us;

	u32   flow_update_time_us;
	u32   height_update_time_us;
	u32   mag_update_time_us;

	eSTA  Sta;        //状态  --
	//eSTA  FlowCal;
	eERR  Err;        //错误信息  --
	u32   Time;       //计时

	//用户访问数据
	bool FlowUpdate;     //更新  --
	//bool FlowFlag;
	//bool MagFlag;
	bool MagUpdate;
	eERR MagErr;		 //错误信息  --
	eSTA MagSta;

	//bool HeightFlag;
	bool HeightUpdate;

	flow_msg flow;
	height_msg height;
	mag_msg mag;
};




#endif


#endif





