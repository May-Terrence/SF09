/*
 * SLAM.cpp
 *
 *  Created on: Sep 12, 2024
 *      Author: su
 */
#include <slam/slam.hpp>

SLAM slam(USART1,(char*) "SLAM",(char*) "-X+Y+Z");	//slam坐标系南东地
/***********************************驱动程序*******************************************/

void USART1_IRQHandler(void)
{
	if(LL_USART_IsActiveFlag_IDLE(USART1))
	{
		LL_USART_ClearFlag_IDLE(USART1);
		LL_DMA_DisableStream(slamDMA,slamRxStream);

		LL_DMA_ClearFlag_DME2(slamDMA);
		LL_DMA_ClearFlag_HT2(slamDMA);
		LL_DMA_ClearFlag_TC2(slamDMA);
		LL_DMA_ClearFlag_TE2(slamDMA);
		LL_DMA_ClearFlag_FE2(slamDMA);

		LL_USART_ClearFlag_CM(USART1);
		LL_USART_ClearFlag_EOB(USART1);
		LL_USART_ClearFlag_FE(USART1);
		LL_USART_ClearFlag_LBD(USART1);
		LL_USART_ClearFlag_NE(USART1);
		LL_USART_ClearFlag_ORE(USART1);
		LL_USART_ClearFlag_PE(USART1);
		LL_USART_ClearFlag_RTO(USART1);
		LL_USART_ClearFlag_TC(USART1);
		LL_USART_ClearFlag_WKUP(USART1);
		LL_USART_ClearFlag_nCTS(USART1);
		LL_USART_ClearFlag_IDLE(USART1);

		slam.RxDataSize = SLAM_RX_LEN - LL_DMA_GetDataLength(slamDMA, slamRxStream);
		do{
			if(slam.RxRawDat[0] != 0xBB || slam.RxRawDat[2] != slam.RxDataSize-3) break;
			if(slam.LockRx == HAL_LOCKED) break;
			slam.LockRx = HAL_LOCKED;
			memcpy(slam.RxDat, slam.RxRawDat, slam.RxDataSize);
			slam.RxDat[slam.RxDataSize] = 0;
			slam.LockRx = HAL_UNLOCKED;
			slam.RxFlag = true;   //收到完整一帧

			BaseType_t YieldRequired = xTaskResumeFromISR(slamTaskHandle);
			if(YieldRequired==pdTRUE)
			{
				/*如果函数xTaskResumeFromISR()返回值为pdTRUE，那么说明要恢复的这个
				任务的任务优先级等于或者高于正在运行的任务(被中断打断的任务),所以在
				退出中断的时候一定要进行上下文切换！*/
				portYIELD_FROM_ISR(YieldRequired);
			}
		}while(0);

		LL_DMA_SetDataLength(slamDMA, slamRxStream, SLAM_RX_LEN);
		LL_DMA_EnableStream(slamDMA, slamRxStream);
	}
}

void DMA2_Stream7_IRQHandler(void)  //发送DMA中断
{
	LL_DMA_DisableStream(slamDMA,slamTxStream);
	LL_DMA_ClearFlag_TC7(slamDMA);
	slam.TxFlag = false;
}
void DMA2_Stream2_IRQHandler(void)  //接收DMA中断
{

}
bool USART1_Send_DMA(uint8_t * pData,uint16_t Size)	//DMA发送
{
	if(slam.TxFlag == true) return false;	//串口发送忙,放弃发送该帧数据
	LL_DMA_DisableStream(slamDMA,slamTxStream);
	LL_DMA_SetDataLength(slamDMA, slamTxStream, Size);
	LL_DMA_SetMemoryAddress(slamDMA, slamTxStream, (uint32_t)pData);
	LL_DMA_EnableStream(slamDMA, slamTxStream);
	slam.TxFlag = true;
	return true;
}

void SLAM::SLAM_Init(void)
{
	LockRx = HAL_UNLOCKED;
	RxFlag = false;
	TxFlag = false;
	RxDataSize = 0;
	Sta = STA_INI;
	Err = ERR_NONE;
	slam.status = NONE;

	if(Dir_Trans(Dir, DirChr)==false) Err = ERR_SOFT;

	osDelay(250);

	/* 配置接收DMA */
	LL_DMA_DisableStream(slamDMA,slamRxStream);
	LL_DMA_SetPeriphAddress(slamDMA, slamRxStream, (uint32_t)&huart->RDR);
	LL_DMA_SetMemoryAddress(slamDMA, slamRxStream, (uint32_t)RxRawDat);
	LL_DMA_SetDataLength(slamDMA, slamRxStream, SLAM_RX_LEN);
	LL_DMA_EnableStream(slamDMA, slamRxStream);
	/* 配置接收DMA */

	/* 配置发送DMA */
	LL_DMA_DisableStream(slamDMA,slamTxStream);
	LL_DMA_SetPeriphAddress(slamDMA, slamTxStream, (uint32_t)&huart->TDR);
	LL_DMA_SetMemoryAddress(slamDMA, slamTxStream, (uint32_t)TxDat);
	LL_DMA_SetDataLength(slamDMA, slamTxStream, SLAM_TX_LEN);
	LL_DMA_ClearFlag_TC7(slamDMA);
	LL_DMA_EnableIT_TC(slamDMA, slamTxStream);
	LL_DMA_EnableStream(slamDMA, slamTxStream);
	/* 配置发送DMA */

	LL_DMA_ClearFlag_DME2(slamDMA);
	LL_DMA_ClearFlag_HT2(slamDMA);
	LL_DMA_ClearFlag_TC2(slamDMA);
	LL_DMA_ClearFlag_TE2(slamDMA);
	LL_DMA_ClearFlag_FE2(slamDMA);

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

	osDelay(250);
}

void SLAM::Command_Receive(void)
{
	if(slam.RxRawDat[1] == 0x04){
		slam_msg.body_Pos[0] = ((s_slam_tran *)(&RxDat[3]))->n;
		slam_msg.body_Pos[1] = ((s_slam_tran *)(&RxDat[3]))->e;
		slam_msg.body_Pos[2] = ((s_slam_tran *)(&RxDat[3]))->d;
		slam_msg.body_q.x()  = ((s_slam_tran *)(&RxDat[3]))->x;
		slam_msg.body_q.y()  = ((s_slam_tran *)(&RxDat[3]))->y;
		slam_msg.body_q.z()  = ((s_slam_tran *)(&RxDat[3]))->z;
		slam_msg.body_q.w()  = ((s_slam_tran *)(&RxDat[3]))->w;
		slam_msg.body_Ang    = qua2eul(slam_msg.body_q);

		float sinY = sinf(slam_msg.body_Pos[2]);
		float cosY = cosf(slam_msg.body_Pos[2]);
		float body_YH = (slam_msg.body_Pos[0]*-sinY + slam_msg.body_Pos[1]*cosY);
		float body_XH = (slam_msg.body_Pos[0]* cosY + slam_msg.body_Pos[1]*sinY);
		switch(((s_slam_tran *)(&RxDat[3]))->quadrant){
		case 1:
			body_YH = (slam_msg.body_Pos[0]*-sinY + slam_msg.body_Pos[1]*cosY) - CORRECTION_DISTANCE;
			break;
		case 2:
			body_XH = (slam_msg.body_Pos[0]* cosY + slam_msg.body_Pos[1]*sinY) - CORRECTION_DISTANCE;
			break;
		case 3:
			body_YH = (slam_msg.body_Pos[0]*-sinY + slam_msg.body_Pos[1]*cosY) + CORRECTION_DISTANCE;
			break;
		case 4:
			body_XH = (slam_msg.body_Pos[0]* cosY + slam_msg.body_Pos[1]*sinY) + CORRECTION_DISTANCE;
			break;
		}
		slam_msg.body_Pos[0] = (body_XH* cosY - body_YH*sinY);
		slam_msg.body_Pos[1] = (body_XH* sinY + body_YH*cosY);
		xQueueOverwrite(queueSlam, &slam_msg);
		target = true;

		uart_Send_Check();
	}

//	if(slam.RxRawDat[2] == 0x00)
//	{
//		if(slam.RxRawDat[4] == 0x01)
//			slam.isCommunicating = true;
//	}
//	else if(slam.RxRawDat[2] == 0x01)
//	{
//		switch(slam.RxRawDat[4])
//		{
//		case 0x01:
//			Ready_Take_off = true;
//			break;
//		case 0x02:
//			Ready_Take_off = false;
//			break;
//		case 0x03:
//			Ready_Land = true;
//			break;
//		case 0x04:
//			Ready_Land = false;
//			break;
//		}
//	}
}

void SLAM::Relative_Position_Transfer(void)
{
	xQueuePeek(queueGps,&gps,0);

	float temp;
	slam.TxDat[0] = 0xBB;
	slam.TxDat[1] = 0x02;

	struct s_slam_tran2car *slam_tran2car;
	uint8_t len = sizeof(struct s_slam_tran2car);
	TxDat[2] = len;

	slam_tran2car = (struct s_slam_tran2car *)(TxDat+3);

	slam_tran2car->lng = gps.lng;
	slam_tran2car->lat = gps.lat;

	USART1_Send_DMA((u8 *)slam.TxDat, len+4);
}

void SLAM::Show_Msg_Transfer(void)
{
	xQueuePeek(queueGps,&gps,0);
	xQueuePeek(queueESKF,&eskf,0);
	xQueuePeek(queueAhrsEuler, &ahrsEuler, 0);

	float temp;
	slam.TxDat[0] = 0xBB;
	slam.TxDat[1] = 0x04;

	struct s_slam_tran *slam_tran;
	uint8_t len = sizeof(struct s_slam_tran);
	TxDat[2] = len;

	slam_tran = (struct s_slam_tran *)(TxDat+3);

	slam_tran->n = eskf.Pos[0];
	slam_tran->e = eskf.Pos[1];
	slam_tran->d = eskf.Pos[2];

	Quaternion<float> q;
	Vector3f Ang;
	for(int i=0;i<3;i++){
		Ang[i] = ahrsEuler.Ang[i];
	}
	q = eul2qua(Ang);
	slam_tran->x = q.x();
	slam_tran->y = q.y();
	slam_tran->z = q.z();
	slam_tran->w = q.w();
	slam_tran->quadrant = 0;

	USART1_Send_DMA((u8 *)slam.TxDat, len+4);
}

//void SLAM::Status_Transfer(void)
//{
//	slam.TxDat[0] = 0xBB;
//	slam.TxDat[1] = 0xAA;
//	slam.TxDat[2] = 0x01;
//	slam.TxDat[3] = 1;
//	slam.TxDat[4] = slam.status;
//
//	uint8_t sum = 0;
//	for(uint8_t i=0; i<5; i++) sum += TxDat[i];
//	TxDat[5] = sum;
//	USART1_Send_DMA((u8 *)slam.TxDat, 6);
//}
//
//void SLAM::Take_off_Request_Transfer(void)
//{
//	slam.TxDat[0] = 0xBB;
//	slam.TxDat[1] = 0xAA;
//	slam.TxDat[2] = 0x00;
//	slam.TxDat[3] = 1;
//	slam.TxDat[4] = 0x01;
//
//	uint8_t sum = 0;
//	for(uint8_t i=0; i<5; i++) sum += TxDat[i];
//	TxDat[5] = sum;
//	USART1_Send_DMA((u8 *)slam.TxDat, 6);
//}
//
//void SLAM::Land_Request_Transfer(void)
//{
//	slam.TxDat[0] = 0xBB;
//	slam.TxDat[1] = 0xAA;
//	slam.TxDat[2] = 0x00;
//	slam.TxDat[3] = 1;
//	slam.TxDat[4] = 0x02;
//
//	uint8_t sum = 0;
//	for(uint8_t i=0; i<5; i++) sum += TxDat[i];
//	TxDat[5] = sum;
//	USART1_Send_DMA((u8 *)slam.TxDat, 6);
//}
//
bool SLAM::uart_Send_Check(void)
{
	slam.TxDat[0] = 0xBB;
	TxDat[1] = 0x03;
	TxDat[2] = 1;
	TxDat[3] = 0x01;

	return USART1_Send_DMA((u8 *)TxDat, 4);
}

extern "C" void slam_main(void *argument)
{
	osDelay(500);//等待系统完成初始化
	slam.SLAM_Init();	//参数初始化
	for(;;)
	{
		vTaskSuspend(slamTaskHandle);
		slam.Command_Receive();
	}
}
