/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : rc.cpp
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

#include "rc.hpp"
#include "string.h"


RC rc(USART1,(char *)"rc");


void USART1_IRQHandler(void)
{
	if(LL_USART_IsActiveFlag_IDLE(USART1))
	{
		LL_USART_ClearFlag_IDLE(USART1);
		LL_DMA_DisableStream(DMA2,LL_DMA_STREAM_2);
		LL_DMA_ClearFlag_DME2(DMA2);
		LL_DMA_ClearFlag_HT2(DMA2);
		LL_DMA_ClearFlag_TC2(DMA2);
		LL_DMA_ClearFlag_TE2(DMA2);
		LL_DMA_ClearFlag_FE2(DMA2);
		rc.RxDataSize = RC_RX_LEN - LL_DMA_GetDataLength(DMA2, LL_DMA_STREAM_2);

		do{
			if(rc.LockRx == HAL_LOCKED)break;
			rc.LockRx = HAL_LOCKED;
			memcpy(rc.RxDat,rc.RxRawDat,rc.RxDataSize);
			rc.RxDat[rc.RxDataSize]=0;
			rc.LockRx = HAL_UNLOCKED;
			rc.RxFlag = true;   //收到完整一帧

			BaseType_t YieldRequired = xTaskResumeFromISR(rcTaskHandle);
			if(YieldRequired==pdTRUE)
			{
				/*如果函数xTaskResumeFromISR()返回值为pdTRUE，那么说明要恢复的这个
				任务的任务优先级等于或者高于正在运行的任务(被中断打断的任务),所以在
				退出中断的时候一定要进行上下文切换！*/
				portYIELD_FROM_ISR(YieldRequired);
			}

		}while(0);



		LL_DMA_SetDataLength(DMA2, LL_DMA_STREAM_2, RC_RX_LEN);
		LL_DMA_EnableStream(DMA2, LL_DMA_STREAM_2);
	}
}

void RC::rc_Init(void)
{
	LockRx = HAL_UNLOCKED;
	RxFlag = false;
	RxDataSize = 0;
	executionTime_us = 0;
	Sta = STA_INI;
	Err = ERR_NONE;

	HIG_THR = RC_HIG_THR;    //油门高位点
	MID_THR = RC_MID_THR;    //油门中位点
	LOW_THR = RC_LOW_THR;    //油门低位点
	rcCommand.Key[0] = 0;     //4个开关
	rcCommand.Key[1] = 0;     //4个开关
	rcCommand.Key[2] = 1;     //4个开关
	rcCommand.Key[3] = 0;     //4个开关
	for(uint8_t i=0;i<4;i++)
		Val[i] = 0.0f;
	for(uint8_t i=0;i<3;i++)
	{
		rcCommand.Ang[i] = 0.0f;
		rcCommand.dAng[i] = 0.0f;
	}
	rcCommand.Thr = 0.0f;


	/* 配置接收DMA */
	LL_DMA_DisableStream(DMA2,LL_DMA_STREAM_2);
	LL_DMA_SetPeriphAddress(DMA2, LL_DMA_STREAM_2, (uint32_t)&huart->RDR);
	LL_DMA_SetMemoryAddress(DMA2, LL_DMA_STREAM_2, (uint32_t)RxRawDat);
	LL_DMA_SetDataLength(DMA2, LL_DMA_STREAM_2, RC_RX_LEN);
	LL_DMA_EnableStream(DMA2, LL_DMA_STREAM_2);
	/* 配置接收DMA */

	LL_USART_EnableDMAReq_RX(huart);
	LL_USART_ClearFlag_IDLE(huart);
	LL_USART_EnableIT_IDLE(huart);
	Sta = STA_RUN;
	osDelay(100);

}

void RC::rc_Update(void)
{

	startTimerLast = startTimer;
	getTimer_us(&startTimer);
	cycleTime_us = startTimer - startTimerLast;

	if(LockRx == HAL_LOCKED)return;
	LockRx = HAL_LOCKED;
	do{
		if(RxDat[0]==0x0f && RxDat[PPM_NUM-2]==0x00 && RxDat[PPM_NUM-1]==0x00)
		{
			channel[0] = RxDat[1] + ((RxDat[2]&0x07)<<8);
			channel[1] = ((RxDat[2]&0xf8)>>3) + ((RxDat[3]&0x3f)<<5);
			channel[2] = ((RxDat[3]&0xc0)>>6) + (RxDat[4]<<2) + ((RxDat[5]&0x01)<<10);
			channel[3] = ((RxDat[5]&0xfe)>>1) + ((RxDat[6]&0x0f)<<7);
			channel[4] = ((RxDat[6]&0xf0)>>4) + ((RxDat[7]&0x7f)<<4);
			channel[5] = ((RxDat[7]&0x80)>>7) + (RxDat[8]<<1) + ((RxDat[9]&0x03)<<9);
			channel[6] = ((RxDat[9]&0xfc)>>2) + ((RxDat[10]&0x1f)<<6);
			channel[7] = ((RxDat[10]&0xe0)>>5) + (RxDat[11]<<3);
			channel[8] = RxDat[12] + ((RxDat[13]&0x07)<<8);
			channel[9] = ((RxDat[13]&0xf8)>>3) + ((RxDat[14]&0x3f)<<5);
			channel[10] = ((RxDat[14]&0xc0)>>6) + (RxDat[15]<<2) + ((RxDat[16]&0x01)<<10);
			channel[11] = ((RxDat[16]&0xfe)>>1) + ((RxDat[17]&0x0f)<<7);
			channel[12] = ((RxDat[17]&0xf0)>>4) + ((RxDat[18]&0x7f)<<4);
			channel[13] = ((RxDat[18]&0x80)>>7) + (RxDat[19]<<1) + ((RxDat[20]&0x03)<<9);
			channel[14] = ((RxDat[20]&0xfc)>>2) + ((RxDat[21]&0x1f)<<6);
			channel[15] = ((RxDat[21]&0xe0)>>5) + (RxDat[22]<<3);

			for(uint8_t i=0;i<8;i++)
			{
				channel[i] += 476;
				rcPPM.PPM[i] = channel[i];
			}

			Val[0] = 1500.0f-rcPPM.PPM[3];
			Val[1] = 1500.0f-rcPPM.PPM[1];
			Val[2] = -(1500.0f-rcPPM.PPM[0]);
			Val[3] = (float)rcPPM.PPM[2];
			//角度设定
			rcCommand.Ang[0] = Val[0]*0.04f;   //-16°到16°
			rcCommand.Ang[1] = Val[1]*0.04f;   //-16°到16°
			rcCommand.Ang[2] = Val[2]*0.11f;   //-45°到45°

			rcCommand.dAng[0] = Val[0]*0.11f;		//-45°/s到45°/s
			rcCommand.dAng[1] = Val[1]*0.11f;		//-45°/s到45°/s
			rcCommand.dAng[2] = Val[2]*0.225f;   //-45°/s到45°/s

			rcCommand.Thr = Val[3];
//			if(rcCommand.Thr<RC_LOW_THR) rcCommand.Thr = RC_LOW_THR;
//			if(rcCommand.Thr>RC_HIG_THR) rcCommand.Thr = RC_HIG_THR;

			//位置设定
			rcCommand.Uvw[0] = Val[0]*0.0025f; //-1m/s~1m/s
			rcCommand.Uvw[1] = Val[1]*0.0025f; //-1m/s~1m/s

			rcCommand.Key[2] = rcPPM.PPM[4]<RC_FUN_MIN?0:(rcPPM.PPM[4]>RC_FUN_MAX?2:1);
			rcCommand.Key[1] = rcPPM.PPM[5]<RC_FUN_MIN?0:(rcPPM.PPM[5]>RC_FUN_MAX?2:1);
			rcCommand.Key[0] = rcPPM.PPM[6]<RC_FUN_MIN?0:(rcPPM.PPM[6]>RC_FUN_MAX?2:1);
			rcCommand.Key[3] = rcPPM.PPM[7]<RC_FUN_MIN?0:(rcPPM.PPM[7]>RC_FUN_MAX?2:1);
			Update = true;

			xQueueOverwrite(queueRCCommand,&rcCommand);

			xQueueOverwrite(queueRCData,&rcPPM);
		}
	}while(0);
	LockRx = HAL_UNLOCKED;
	getTimer_us(&stopTimer);

	executionTime_us = stopTimer - startTimer;
}

extern "C" void rc_main(void *argument)
{
	rc.rc_Init();
	osDelay(14);
	for(;;)
	{
		vTaskSuspend(rcTaskHandle);
		rc.rc_Update();


	}

}

/************************ (C) COPYRIGHT Longmen Drone Team *****END OF FILE****/
