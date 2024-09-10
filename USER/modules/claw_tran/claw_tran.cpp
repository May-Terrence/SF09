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
			if(claw.RxRawDat[0] != 0xCC || claw.RxRawDat[1] != 0xCC || claw.RxRawDat[2] != claw.RxDataSize) break;
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

void CLAW::claw_Update()
{
	if(RxDat[3] == 0x01){
		for(int i=1; i<4; i++)
		{
			claw_msg.Pos[i-1] = ((RxDat[i*4+3]<<24)|(RxDat[i*4+2]<<16)|(RxDat[i*4+1]<<8)|(RxDat[i*4]))*0.0001;
		}

		claw_msg.Vel[0] = ((RxDat[19]<<24)|(RxDat[18]<<16)|(RxDat[17]<<8)|(RxDat[16]))*0.0001;
		claw_msg.Vel[1] = ((RxDat[23]<<24)|(RxDat[22]<<16)|(RxDat[21]<<8)|(RxDat[20]))*0.0001;
		claw_msg.Vel[2] = ((RxDat[27]<<24)|(RxDat[26]<<16)|(RxDat[25]<<8)|(RxDat[24]))*0.0001;

		claw_msg.Ang[0] = ((RxDat[31]<<24)|(RxDat[30]<<16)|(RxDat[29]<<8)|(RxDat[28]))*0.0001;
		claw_msg.Ang[1] = ((RxDat[35]<<24)|(RxDat[34]<<16)|(RxDat[33]<<8)|(RxDat[32]))*0.0001;
		claw_msg.Ang[2] = ((RxDat[39]<<24)|(RxDat[38]<<16)|(RxDat[37]<<8)|(RxDat[36]))*0.0001;
	}

	xQueueOverwrite(queueClaw, &claw_msg);
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


