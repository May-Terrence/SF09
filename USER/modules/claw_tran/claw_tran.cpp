/*
 * claw_tran.cpp
 *
 *  Created on: 2024年8月27日
 *      Author: mengc
 */

#include "claw_tran/claw_tran.hpp"
#include "string.h"

CLAW claw(USART6,(char *)"CLAW");

void USART6_IRQHandler(void)
{
	if(LL_USART_IsActiveFlag_IDLE(USART6))
	{
		LL_USART_ClearFlag_IDLE(USART6);
		LL_DMA_DisableStream(DMA2,LL_DMA_STREAM_1);


		LL_DMA_ClearFlag_DME1(DMA2);
		LL_DMA_ClearFlag_HT1(DMA2);
		LL_DMA_ClearFlag_TC1(DMA2);
		LL_DMA_ClearFlag_TE1(DMA2);
		LL_DMA_ClearFlag_FE1(DMA2);

		LL_USART_ClearFlag_CM(USART6);
		LL_USART_ClearFlag_EOB(USART6);
		LL_USART_ClearFlag_FE(USART6);
		LL_USART_ClearFlag_LBD(USART6);
		LL_USART_ClearFlag_NE(USART6);
		LL_USART_ClearFlag_ORE(USART6);
		LL_USART_ClearFlag_PE(USART6);
		LL_USART_ClearFlag_RTO(USART6);
		LL_USART_ClearFlag_TC(USART6);
		LL_USART_ClearFlag_WKUP(USART6);
		LL_USART_ClearFlag_nCTS(USART6);
		LL_USART_ClearFlag_IDLE(USART6);
		claw.RxDataSize = CLAW_RX_LEN - LL_DMA_GetDataLength(DMA2, LL_DMA_STREAM_1);

		do{
			if(claw.RxRawDat[0] != 0xCC || claw.RxRawDat[1] != 0xCC || claw.RxRawDat[2] != claw.RxDataSize-5) break;
			if(claw.LockRx == HAL_LOCKED)break;
			claw.LockRx = HAL_LOCKED;
			memcpy(claw.RxDat,claw.RxRawDat,claw.RxDataSize);
			claw.RxDat[claw.RxDataSize]=0;
			claw.LockRx = HAL_UNLOCKED;
			claw.RxFlag = true;   //收到完整一帧

			BaseType_t YieldRequired = xTaskResumeFromISR(clawTaskHandle);
			if(YieldRequired==pdTRUE)
			{
				/*如果函数xTaskResumeFromISR()返回值为pdTRUE，那么说明要恢复的这个
				任务的任务优先级等于或者高于正在运行的任务(被中断打断的任务),所以在
				退出中断的时候一定要进行上下文切换！*/
				portYIELD_FROM_ISR(YieldRequired);
			}

		}while(0);

		LL_DMA_SetDataLength(DMA2, LL_DMA_STREAM_1, CLAW_RX_LEN);
		LL_DMA_EnableStream(DMA2, LL_DMA_STREAM_1);
	}
}

void CLAW::claw_Init()
{
	LockRx = HAL_UNLOCKED;
	RxFlag = false;
	TxFlag = false;
	RxDataSize = 0;
	executionTime_us = 0;
	Sta = STA_INI;
	Err = ERR_NONE;

	/* 配置接收DMA */
	LL_DMA_DisableStream(DMA2,LL_DMA_STREAM_1);
	LL_DMA_SetPeriphAddress(DMA2, LL_DMA_STREAM_1, (uint32_t)&huart->RDR);
	LL_DMA_SetMemoryAddress(DMA2, LL_DMA_STREAM_1, (uint32_t)RxRawDat);
	LL_DMA_SetDataLength(DMA2, LL_DMA_STREAM_1, CLAW_RX_LEN);
	LL_DMA_EnableStream(DMA2, LL_DMA_STREAM_1);
	/* 配置接收DMA */

	LL_DMA_ClearFlag_DME1(DMA2);
	LL_DMA_ClearFlag_HT1(DMA2);
	LL_DMA_ClearFlag_TC1(DMA2);
	LL_DMA_ClearFlag_TE1(DMA2);
	LL_DMA_ClearFlag_FE1(DMA2);

	/* 配置发送DMA */
	LL_DMA_DisableStream(DMA2,LL_DMA_STREAM_6);
	LL_DMA_SetPeriphAddress(DMA2, LL_DMA_STREAM_6, (uint32_t)&huart->TDR);
	LL_DMA_SetMemoryAddress(DMA2, LL_DMA_STREAM_6, (uint32_t)TxDat);
	LL_DMA_SetDataLength(DMA2, LL_DMA_STREAM_6, CLAW_TX_LEN);
	LL_DMA_ClearFlag_TC6(DMA2);
	LL_DMA_EnableIT_TC(DMA2, LL_DMA_STREAM_6);
	LL_DMA_EnableStream(DMA2, LL_DMA_STREAM_6);

	LL_USART_ClearFlag_CM(huart);
	LL_USART_ClearFlag_EOB(huart);
	LL_USART_ClearFlag_FE(huart);
	LL_USART_ClearFlag_LBD(huart);
	LL_USART_ClearFlag_NE(huart);
	LL_USART_ClearFlag_ORE(huart);
	LL_USART_ClearFlag_PE(huart);
	LL_USART_ClearFlag_RTO(huart);
	LL_USART_ClearFlag_TC(huart);
	LL_USART_ClearFlag_WKUP(huart);
	LL_USART_ClearFlag_nCTS(huart);
	LL_USART_ClearFlag_IDLE(huart);

	LL_USART_EnableDMAReq_RX(huart);
	LL_USART_EnableDMAReq_TX(huart);
	LL_USART_ClearFlag_IDLE(huart);
	LL_USART_EnableIT_IDLE(huart);
	Sta = STA_RUN;
	osDelay(100);
}

void DMA2_Stream6_IRQHandler(void)  //发送DMA中断
{
	LL_DMA_DisableStream(DMA2,LL_DMA_STREAM_6);
	LL_DMA_ClearFlag_TC6(DMA2);
	claw.TxFlag = false;
}

bool CLAW::uart_Send_DMA(uint8_t * pData,uint16_t Size)	//DMA发送
{
	if(claw.TxFlag == true) return false;	//串口发送忙,放弃发送该帧数据
	LL_DMA_DisableStream(DMA2,LL_DMA_STREAM_6);
	LL_DMA_SetDataLength(DMA2, LL_DMA_STREAM_6, Size);
	LL_DMA_SetMemoryAddress(DMA2, LL_DMA_STREAM_6, (uint32_t)pData);
	LL_DMA_EnableStream(DMA2, LL_DMA_STREAM_6);
	claw.TxFlag = true;
	return true;
}

void CLAW::claw_Update(void)
{
	if(RxDat[2] == 0x01){
		for(int i=1; i<4; i++)
		{
			claw_msg.Pos[i-1] = ((RxDat[i*4+3]<<24)|(RxDat[i*4+2]<<16)|(RxDat[i*4+1]<<8)|(RxDat[i*4]));
		}

		claw_msg.Yaw = ((RxDat[19]<<24)|(RxDat[18]<<16)|(RxDat[17]<<8)|(RxDat[16]));
		claw_msg.star = RxDat[20];
		claw_msg.model = RxDat[21];

		isUpdate = true;
		xQueueOverwrite(queueClaw, &claw_msg);
	}
	else if(RxDat[2] == 0x00){
		switch(RxDat[4] == 0x00)
		{
		case 0x00:
			isOpen = true;
			isClose = false;
			break;
		case 0x01:
			isClose = true;
			isOpen = false;
			break;
		}
	}
}

void CLAW::Open_Request_Tran(void)
{
	claw.TxDat[0] = 0xCC;
	claw.TxDat[1] = 0xCC;
	claw.TxDat[2] = 0x01;
	claw.TxDat[3] = 1;
	claw.TxDat[4] = 0x00;
	u8 sum = 0;
	for(u8 i=0;i<5;i++)sum += claw.TxDat[i];
		claw.TxDat[5] = sum;
	claw.uart_Send_DMA((u8 *)claw.TxDat, 6);
}

void CLAW::Close_Request_Tran(void)
{
	claw.TxDat[0] = 0xCC;
	claw.TxDat[1] = 0xCC;
	claw.TxDat[2] = 0x01;
	claw.TxDat[3] = 1;
	claw.TxDat[4] = 0x01;
	u8 sum = 0;
	for(u8 i=0;i<5;i++)sum += claw.TxDat[i];
		claw.TxDat[5] = sum;
	claw.uart_Send_DMA((u8 *)claw.TxDat, 6);
}

extern "C" void claw_main(void *argument)
{
	claw.claw_Init();
	for(;;)
	{
		vTaskSuspend(clawTaskHandle);
		claw.claw_Update();
	}
}


