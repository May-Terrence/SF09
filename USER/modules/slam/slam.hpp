/*
 * SLAM.hpp
 *
 *  Created on: Sep 12, 2024
 *      Author: su
 */

#ifndef MODULES_SLAM_SLAM_HPP_
#define MODULES_SLAM_SLAM_HPP_

#include "system/system.hpp"
#include "userlib/userlib.hpp"

#ifdef __cplusplus
extern "C" {
#endif
#define slamDMA DMA2
#define slamRxStream LL_DMA_STREAM_2
#define slamTxStream LL_DMA_STREAM_7

#define SLAM_RX_LEN 30
#define SLAM_TX_LEN 30

void USART1_IRQHandler(void);
void DMA2_Stream2_IRQHandler(void); //接收DMA中断
void DMA2_Stream7_IRQHandler(void); //发送DMA中断
bool USART1_Send_DMA(uint8_t * pData,uint16_t Size);

#ifdef __cplusplus
}
#endif


#ifdef __cplusplus
using namespace std;

typedef enum
{
	Taking_off   		= 0x00U,	 //正在起飞
	Fail_to_take_off    = 0x01U,	 //坠毁
	Air	  				= 0x02U,	 //起飞成功
	Landing 	  		= 0x03U,	 //正在降落
	Fail_to_land   		= 0x04U,   	 //坠毁
	Ground        		= 0x05U,   	 //降落成功
	NONE				= 0x06U,	 //无效
}Status;

class SLAM
{
public:
	SLAM(){}
	SLAM(USART_TypeDef *h,char *n,char *s) : huart(h),name(n),DirChr(s){};
	~SLAM(){}

	void SLAM_Init();
	void Command_Receive(void);
	void Relative_Position_Transfer(void);
	void Status_Transfer(void);
	void Take_off_Request_Transfer(void);
	void Land_Request_Transfer(void);
	bool uart_Send_Check(void);

	uint16_t   RxDataSize;
	uint8_t    RxRawDat[SLAM_RX_LEN];	//数据
	uint8_t    RxDat[SLAM_RX_LEN];
	HAL_LockTypeDef LockRx;
	bool  RxFlag;		//接收标志位
	bool TxFlag;

	bool isCommunicating{false};
	bool Ready_Take_off{false};
	bool Ready_Land{false};

	Status status;
	uint8_t test_flag;

private:
	USART_TypeDef * huart;
	char *name;
	int8_t  Dir[6];     //方向  --
	char *DirChr;
	uint8_t  TxDat[SLAM_TX_LEN];

	eSTA  Sta;        //状态  --
	eERR  Err;        //错误信息  --

	SLAM_msg slam_msg;
	gps_msg gps;
};
#endif

extern SLAM slam;

#endif /* MODULES_SLAM_SLAM_HPP_ */
