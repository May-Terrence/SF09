
#include "flow.hpp"



FLOW flow(USART2, (char*) "FLOW", (char*) "+X+Y+Z");

void USART2_IRQHandler(void)
{
	if(LL_USART_IsActiveFlag_IDLE(flow.huart))
	{
		LL_USART_ClearFlag_IDLE(flow.huart);
		LL_DMA_DisableStream(DMA1,LL_DMA_STREAM_5);
		LL_DMA_ClearFlag_DME5(DMA1);
		LL_DMA_ClearFlag_HT5(DMA1);
		LL_DMA_ClearFlag_TC5(DMA1);
		LL_DMA_ClearFlag_TE5(DMA1);
		LL_DMA_ClearFlag_FE5(DMA1);
		flow.RxDataSize = FLOW_RX_LEN - LL_DMA_GetDataLength(DMA1, LL_DMA_STREAM_5);

		do{
			//if(flow.RxRawDat[0]!='$'|| flow.RxRawDat[flow.RxDataSize-1]!='\n') break;
			if(flow.RxRawDat[1] != 0xfd)break;//标志位
			if(flow.LockRx == HAL_LOCKED) break;
			flow.LockRx = HAL_LOCKED;
			memcpy(flow.RxDat, flow.RxRawDat, flow.RxDataSize);
			flow.RxDat[flow.RxDataSize] = 0;
			flow.LockRx = HAL_UNLOCKED;
			flow.RxFlag = true;   //收到完整一帧

			BaseType_t YieldRequired = xTaskResumeFromISR(flowTaskHandle);
			if(YieldRequired==pdTRUE)
			{
				/*如果函数xTaskResumeFromISR()返回值为pdTRUE，那么说明要恢复的这个
				任务的任务优先级等于或者高于正在运行的任务(被中断打断的任务),所以在
				退出中断的时候一定要进行上下文切换！*/
				portYIELD_FROM_ISR(YieldRequired);
			}
		}while(0);

		LL_DMA_SetDataLength(DMA1, LL_DMA_STREAM_5, FLOW_RX_LEN);
		LL_DMA_EnableStream(DMA1, LL_DMA_STREAM_5);
	}
}


void FLOW::flow_Init(void)
{
	LockRx = HAL_UNLOCKED;
	RxFlag = false;
	TxFlag = false;

	RxDataSize = 0;
	executionTime_us = 0;
	Sta = STA_INI;
	Err = ERR_NONE;

	//HeightFlag = false;
	//FlowFlag = false;
	FlowUpdate = false;
	//MagFlag = false;
//	MagUpdate = false;
//	MagErr = ERR_NONE;
//	MagSta = STA_INI;
//	if(Dir_Trans(Dir, DirChr)==false) Err = ERR_SOFT;
//
//	MagOff[0] = 0.0f;
//	MagOff[1] = 0.0f;
//	MagOff[2] = 0.0f;//
//	for (u8 i=0;i<3;i++)
//	{
//		MagRaw[i]=0;
//		MagFil[i]=0;
//		mag.MagRel[i] = MagRel[i] = 0;
//	}
//
//	mag_update_time_us = 0;
//	mag.timestamp = mag_update_time_us;
//	xQueueOverwrite(queueFlowmag, &mag);//光流上的磁力计 暂时没有用

	flow_update_time_us = 0;
	height_update_time_us = 0;
	flow.timestamp = flow_update_time_us;
	height.timestamp = height_update_time_us;

	xQueueOverwrite(queueFlow, &flow);
	xQueueOverwrite(queueHeight, &height);
	osDelay(250);


	/* 配置接收DMA */
	LL_DMA_DisableStream(DMA1,LL_DMA_STREAM_5);
	LL_DMA_SetPeriphAddress(DMA1, LL_DMA_STREAM_5, (uint32_t)&huart->RDR);
	LL_DMA_SetMemoryAddress(DMA1, LL_DMA_STREAM_5, (uint32_t)RxRawDat);
	LL_DMA_SetDataLength(DMA1, LL_DMA_STREAM_5, FLOW_RX_LEN);
	LL_DMA_EnableStream(DMA1, LL_DMA_STREAM_5);
	/* 配置接收DMA */

	LL_USART_EnableDMAReq_RX(huart);
	LL_USART_ClearFlag_IDLE(huart);
	LL_USART_EnableIT_IDLE(huart);

	Sta = STA_RUN;
	MagSta = STA_RUN;
	//FlowCal = true;


	osDelay(100);


}

void FLOW::flow_Update(void)
{
	if(Sta == STA_INI)return;

	startTimerLast = startTimer;
	getTimer_us(&startTimer);
	cycleTime_us = startTimer - startTimerLast;

	if(RxFlag == false) return;
		RxFlag = false;
	if(LockRx == HAL_LOCKED)return;
	LockRx = HAL_LOCKED;

	bool FlowFlag = false;
	bool MagFlag = false;
	bool HeightFlag = false;
	BIT32 temp;
	do{
		uint8_t num = RxDat[2];//长度
		uint8_t TxSum = 0;
		for(uint8_t i=0; i<num-1; i++) TxSum += RxDat[i];
		if(TxSum != RxDat[num-1])break;		//和校验 判断sum
		switch(RxDat[0])
		{
		case 0xf1:	//光流得到的数据
			memcpy(temp.uc, &RxDat[3], 4);
			height.height = temp.fp;
			memcpy(temp.uc, &RxDat[7], 4);
			height.speedZ = temp.fp;
			memcpy(temp.uc, &RxDat[11], 4);
			flow.flowx = temp.fp;
			memcpy(temp.uc, &RxDat[15], 4);
			flow.flowy = temp.fp;//单位: cm/s
			//memcpy(temp.uc,&RxDat[19],4);
			//flow.flowy2 = temp.fp;
			//memcpy(temp.uc,&RxDat[23],4);
			//flow.flowx2 = -temp.fp;
			memcpy(temp.uc, &RxDat[19], 4);
			MagRaw[0] = (temp.fp);
			memcpy(temp.uc, &RxDat[23], 4);
			MagRaw[1] = (temp.fp);
			memcpy(temp.uc, &RxDat[27], 4);
			MagRaw[2] = (temp.fp);

			FlowFlag = true;
			MagFlag = true;
			HeightFlag = true;
			break;
		case 0xf2:	//光流数据
			memcpy(temp.uc,&RxDat[3],4);
			flow.flowy = temp.fp;
			memcpy(temp.uc,&RxDat[7],4);
			flow.flowx = -temp.fp;
			memcpy(temp.uc,&RxDat[11],4);
			flow.timestamp = temp.ui;
			FlowFlag = true;
			//xQueueOverwrite(queueFlow, &flow);
			break;

		case 0xf3:	//高度数据
			memcpy(temp.uc,&RxDat[3],4);
			height.height = temp.fp;
			memcpy(temp.uc,&RxDat[7],4);
			height.timestamp = temp.ui;
			HeightFlag = true;
			xQueueOverwrite(queueHeight, &height);
			break;

		}
	}while(0);
	LockRx = HAL_UNLOCKED;

//
//	if(MagFlag==true)//光流上的磁力计 暂时没有用
//	{
//		MagUpdate = false;//
//		//方向变换
//		MagRot[0]=Dir[0]*(MagRaw[Dir[1]]-MagOff[Dir[1]]);
//		MagRot[1]=Dir[2]*(MagRaw[Dir[3]]-MagOff[Dir[3]]);
//		MagRot[2]=Dir[4]*(MagRaw[Dir[5]]-MagOff[Dir[5]]);
//		//实际值转换
//		MagFil[0]=MagRel[0]=MagRot[0];
//		MagFil[1]=MagRel[1]=MagRot[1];
//		MagFil[2]=MagRel[2]=MagRot[2];
//
//		for(uint8_t i=0; i<3; i++)
//		{
//			mag.MagRaw[i] = MagRaw[i];
//			mag.MagRel[i] = MagRel[i];
//		}
//
//		if(MagSta == STA_RUN)
//		{
//			xQueueOverwrite(queueFlowmag, &mag);
//			MagUpdate = true;
//		}
//	}

	if(FlowFlag == true)
	{
		xQueueOverwrite(queueFlow, &flow);
		FlowUpdate = true;
	}
	if(HeightFlag == true)
	{
		xQueueOverwrite(queueHeight, &height);
		HeightUpdate = true;
	}


	getTimer_us(&stopTimer);

	executionTime_us = stopTimer - startTimer;
}


extern "C" void flow_main(void *argument)
{
	//
	osDelay(2000);						//等待传感器数据
	flow.flow_Init();
	for(;;)
	{
		vTaskSuspend(flowTaskHandle);
		flow.flow_Update();
		//osDelay(10);//
	}
}












