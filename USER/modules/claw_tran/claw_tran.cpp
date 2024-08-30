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
			if(claw.RxRawDat[0] != 0xAU || claw.RxRawDat[1] != claw.RxDataSize) break;
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

void CLAW::claw_Update()
{
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


