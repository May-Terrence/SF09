/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : transfer.cpp
  * Description        : 
  ******************************************************************************
  * Function           :
  *
  ******************************************************************************
  * Author             : Zhiping Fu
  * Creation Date      : Oct 21, 2020
  ******************************************************************************
  */
/* USER CODE END Header */

#include "transfer.hpp"
#include "string.h"




TRAN tran(USART3,(char *)"tran");

void USART3_IRQHandler(void)
{
	if(LL_USART_IsActiveFlag_IDLE(USART3))
	{
		LL_USART_ClearFlag_IDLE(USART3);
		LL_DMA_DisableStream(DMA1,LL_DMA_STREAM_1);
		LL_DMA_ClearFlag_DME1(DMA1);
		LL_DMA_ClearFlag_HT1(DMA1);
		LL_DMA_ClearFlag_TC1(DMA1);
		LL_DMA_ClearFlag_TE1(DMA1);
		LL_DMA_ClearFlag_FE1(DMA1);

		tran.RxDataSize = TRAN_RX_LEN - LL_DMA_GetDataLength(DMA1, LL_DMA_STREAM_1);

		do{//0xAA 0xAF [功能字] [字节数] [内容...] [校验和]
			if(tran.RxRawDat[0]!=0xAA || tran.RxRawDat[1]!=0xAF || tran.RxRawDat[3]!=tran.RxDataSize-5) break;
			if(tran.LockRx == HAL_LOCKED) break;
			tran.LockRx = HAL_LOCKED;
			memcpy(tran.RxDat, tran.RxRawDat, tran.RxDataSize);
			tran.RxDat[tran.RxDataSize] = 0;
			tran.LockRx = HAL_UNLOCKED;
			tran.RxFlag = true;   //收到完整一帧

			BaseType_t YieldRequired = xTaskResumeFromISR(tranReceiveTaskHandle);
			if(YieldRequired==pdTRUE)
			{
				/*如果函数xTaskResumeFromISR()返回值为pdTRUE，那么说明要恢复的这个
				任务的任务优先级等于或者高于正在运行的任务(被中断打断的任务),所以在
				退出中断的时候一定要进行上下文切换！*/
				portYIELD_FROM_ISR(YieldRequired);
			}
		}while(0);



		LL_DMA_SetDataLength(DMA1, LL_DMA_STREAM_1, TRAN_RX_LEN);
		LL_DMA_EnableStream(DMA1, LL_DMA_STREAM_1);
	}

}

void DMA1_Stream1_IRQHandler(void)  //接收DMA中断
{

}

void DMA1_Stream3_IRQHandler(void)  //发送DMA中断
{
	LL_DMA_DisableStream(DMA1,LL_DMA_STREAM_3);
	LL_DMA_ClearFlag_TC3(DMA1);
	tran.TxFlag = false;
}

void TRAN::tran_Init(void)
{
	LockRx = HAL_UNLOCKED;
	RxFlag = false;
	TxFlag = false;
	RxDataSize = 0;
	executionTime_us = 0;
	Sta = STA_INI;
	Err = ERR_NONE;
	index = 0;
	send_pid = false;
	send_pid_group = 0;
	send_version = false;

	osDelay(500);

	TxDat[0] = 0xAA;
	TxDat[1] = 0xAA;

	/* 配置接收DMA */
	LL_DMA_DisableStream(DMA1,LL_DMA_STREAM_1);
	LL_DMA_SetPeriphAddress(DMA1, LL_DMA_STREAM_1, (uint32_t)&huart->RDR);
	LL_DMA_SetMemoryAddress(DMA1, LL_DMA_STREAM_1, (uint32_t)RxRawDat);
	LL_DMA_SetDataLength(DMA1, LL_DMA_STREAM_1, TRAN_RX_LEN);
	LL_DMA_EnableStream(DMA1, LL_DMA_STREAM_1);
	/* 配置接收DMA */

	/* 配置发送DMA */
	LL_DMA_DisableStream(DMA1,LL_DMA_STREAM_3);
	LL_DMA_SetPeriphAddress(DMA1, LL_DMA_STREAM_3, (uint32_t)&USART3->TDR);
	LL_DMA_SetMemoryAddress(DMA1, LL_DMA_STREAM_3, (uint32_t)TxDat);
	LL_DMA_SetDataLength(DMA1, LL_DMA_STREAM_3, TRAN_TX_LEN);
	LL_DMA_ClearFlag_TC3(DMA1);
	LL_DMA_EnableIT_TC(DMA1, LL_DMA_STREAM_3);
	LL_DMA_EnableStream(DMA1, LL_DMA_STREAM_3);
	/* 配置发送DMA */

	LL_USART_EnableDMAReq_RX(huart);
	LL_USART_EnableDMAReq_TX(huart);
	LL_USART_ClearFlag_IDLE(huart);
	LL_USART_EnableIT_IDLE(huart);
}

bool TRAN::uart_Send_DMA(uint8_t * pData,uint16_t Size)
{
	if(TxFlag == true) return false;	//串口发送忙,放弃发送该帧数据
	LL_DMA_DisableStream(DMA1,LL_DMA_STREAM_3);

	LL_DMA_SetDataLength(DMA1, LL_DMA_STREAM_3, Size);
	LL_DMA_SetMemoryAddress(DMA1, LL_DMA_STREAM_3, (uint32_t)pData);
	LL_DMA_EnableStream(DMA1, LL_DMA_STREAM_3);
	TxFlag = true;
	return true;
}

void TRAN::uart_Send_DMA_loop(void)
{
	startTimerLast = startTimer;
	getTimer_us(&startTimer);
	cycleTime_us = startTimer - startTimerLast;
	index++;
	switch(index%20)
	{
	case 1:
		uart_Send_Sensor();
		break;
	case 3:
		uart_Send_Sensor2();
		break;
	case 5:
		uart_Send_Status();
		break;
	case 7:
		uart_Send_MotorPWM();
		break;
	case 9:
		uart_Send_rcData();
		break;
	case 11:
		uart_Send_Power();
		break;
	case 13:
		if(send_version == true)
		{
			uart_Send_Version();
			send_version = false;
		}
		break;
	case 15:
		uart_Send_GPS();
		break;
	case 17:

		break;
	case 19:
		if(send_pid == true)
		{
			xQueuePeek(queuePID, &pid, 0);
			send_pid_group = 1;
			send_pid = false;
		}
		if(send_pid_group > 0&&send_pid_group<7)
		{
			uart_Send_PID(send_pid_group);
			send_pid_group++;
		}
		break;
	case 0:
	case 2:
	case 4:
	case 6:
	case 8:
	case 10:
	case 12:
	case 14:
	case 16:
	case 18:
		uart_Send_User();
		break;

	}

	getTimer_us(&stopTimer);

	executionTime_us = stopTimer - startTimer;

}

bool TRAN::uart_Send_Sensor(void)
{
	if(TxFlag == true) return false;
	xQueuePeek(queueGyrDat, &gyro, 0);

	xQueuePeek(queueAccDat, &acc, 0);

	xQueuePeek(queueMagDat, &mag, 0);

	vs16 _temp;
	TxDat[2] = 0x02;
	TxDat[3] = 18;
	_temp = acc.acc[0]*100;
	TxDat[4] = BYTE1(_temp);
	TxDat[5] = BYTE0(_temp);
	_temp = acc.acc[1]*100;
	TxDat[6] = BYTE1(_temp);
	TxDat[7] = BYTE0(_temp);
	_temp = acc.acc[2]*100;
	TxDat[8] = BYTE1(_temp);
	TxDat[9] = BYTE0(_temp);
	_temp = gyro.gyro[0]*R2D;
	TxDat[10] = BYTE1(_temp);
	TxDat[11] = BYTE0(_temp);
	_temp = gyro.gyro[1]*R2D;
	TxDat[12] = BYTE1(_temp);
	TxDat[13] = BYTE0(_temp);
	_temp = gyro.gyro[2]*R2D;
	TxDat[14] = BYTE1(_temp);
	TxDat[15] = BYTE0(_temp);
	_temp = mag.mag[0];
	TxDat[16] = BYTE1(_temp);
	TxDat[17] = BYTE0(_temp);
	_temp = mag.mag[1];
	TxDat[18] = BYTE1(_temp);
	TxDat[19] = BYTE0(_temp);
	_temp = mag.mag[2];
	TxDat[20] = BYTE1(_temp);
	TxDat[21] = BYTE0(_temp);

	uint8_t sum = 0;
	for(uint8_t i=0; i<22; i++) sum += TxDat[i];
	TxDat[22] = sum;
	return uart_Send_DMA((u8 *)TxDat, 23);

}
bool TRAN::uart_Send_Sensor2(void)
{
	if(TxFlag == true) return false;

	vs32 _temp;
	vs16 _temp2;
	TxDat[2] = 0x07;
	TxDat[3] = 6;
	_temp = 0;		// 曲线 ALT_BAR
	TxDat[4] = BYTE3(_temp);
	TxDat[5] = BYTE2(_temp);
	TxDat[6] = BYTE1(_temp);
	TxDat[7] = BYTE0(_temp);
	_temp2 = 0;		// 曲线 ALT_CSB
	TxDat[8] = BYTE1(_temp2);
	TxDat[9] = BYTE0(_temp2);

	uint8_t sum = 0;
	for(uint8_t i=0; i<10; i++) sum += TxDat[i];
	TxDat[10] = sum;
	return uart_Send_DMA((u8 *)TxDat, 11);
}

bool TRAN::uart_Send_Status(void)
{
	if(TxFlag == true) return false;

	xQueuePeek(queueAhrsEuler, &ahrsEuler, 0);

	vs16 _temp;
	vs32 _temp2;
	TxDat[2] = 0x01;
	TxDat[3] = 12;
	_temp = (s16)(ahrsEuler.Ang[0]*R2D*100.0f);
	TxDat[4] = BYTE1(_temp);
	TxDat[5] = BYTE0(_temp);
	_temp = (s16)(ahrsEuler.Ang[1]*R2D*100.0f);
	TxDat[6] = BYTE1(_temp);
	TxDat[7] = BYTE0(_temp);
	_temp = (s16)(ahrsEuler.Ang[2]*R2D*100.0f);
	TxDat[8] = BYTE1(_temp);
	TxDat[9] = BYTE0(_temp);
	_temp2 = (s32)0;
	TxDat[10] = BYTE3(_temp2);
	TxDat[11] = BYTE2(_temp2);
	TxDat[12] = BYTE1(_temp2);
	TxDat[13] = BYTE0(_temp2);
	TxDat[14] = 0;		//模式
	TxDat[15] = 0;		//

	uint8_t sum = 0;
	for(uint8_t i=0; i<16; i++) sum += TxDat[i];
	TxDat[16] = sum;
	return uart_Send_DMA((u8 *)TxDat, 17);
}

bool TRAN::uart_Send_MotorPWM(void)
{

	return 1;
}

bool TRAN::uart_Send_rcData(void)
{

	xQueuePeek(queueRCData, &rcPPM, 0);
	if(TxFlag == true) return false;
	TxDat[2] = 0x03;
	TxDat[3] = 20;
	TxDat[4] = BYTE1(rcPPM.PPM[2]);
	TxDat[5] = BYTE0(rcPPM.PPM[2]);
	TxDat[6] = BYTE1(rcPPM.PPM[0]);
	TxDat[7] = BYTE0(rcPPM.PPM[0]);
	TxDat[8] = BYTE1(rcPPM.PPM[3]);
	TxDat[9] = BYTE0(rcPPM.PPM[3]);
	TxDat[10]= BYTE1(rcPPM.PPM[1]);
	TxDat[11]= BYTE0(rcPPM.PPM[1]);
	TxDat[12]= BYTE1(rcPPM.PPM[4]);
	TxDat[13]= BYTE0(rcPPM.PPM[4]);
	TxDat[14]= BYTE1(rcPPM.PPM[5]);
	TxDat[15]= BYTE0(rcPPM.PPM[5]);
	TxDat[16]= BYTE1(rcPPM.PPM[6]);
	TxDat[17]= BYTE0(rcPPM.PPM[6]);
	TxDat[18]= BYTE1(rcPPM.PPM[7]);
	TxDat[19]= BYTE0(rcPPM.PPM[7]);
	TxDat[20]= 0x03;
	TxDat[21]= 0xE8;
	TxDat[22]= 0x03;
	TxDat[23]= 0xE8;

	uint8_t sum = 0;
	for(uint8_t i=0; i<24; i++) sum += TxDat[i];
	TxDat[24] = sum;
	return uart_Send_DMA((u8 *)TxDat, 25);

}

bool TRAN::uart_Send_Power(void)
{
	if(TxFlag == true) return false;

	xQueuePeek(queueBattery, &battery, 0);

	u16 temp;
	TxDat[2] = 0x05;
	TxDat[3] = 4;
	temp = battery.battery_Voltage*100.0f;
	TxDat[4] = BYTE1(temp);
	TxDat[5] = BYTE0(temp);
	temp = 0;
	TxDat[6] = BYTE1(temp);
	TxDat[7] = BYTE0(temp);

	uint8_t sum = 0;
	for(uint8_t i=0; i<8; i++) sum += TxDat[i];
	TxDat[8] = sum;
	return uart_Send_DMA((u8 *)TxDat, 9);
}

bool TRAN::uart_Send_Version(void)
{
	if(TxFlag == true) return false;

	u8 hardware_type = 4;
	u16 hardware_ver = 300, software_ver = 100, protocol_ver = 400, bootloader_ver = 0;
	TxDat[2] = 0x00;
	TxDat[3] = 9;
	TxDat[4] = hardware_type;
	TxDat[5] = BYTE0(hardware_ver);
	TxDat[6] = BYTE1(hardware_ver);
	TxDat[7] = BYTE0(software_ver);
	TxDat[8] = BYTE1(software_ver);
	TxDat[9] = BYTE0(protocol_ver);
	TxDat[10] = BYTE3(protocol_ver);
	TxDat[11] = BYTE2(bootloader_ver);
	TxDat[12] = BYTE1(bootloader_ver);

	uint8_t sum = 0;
	for(uint8_t i=0; i<13; i++) sum += TxDat[i];
	TxDat[13] = sum;
	return uart_Send_DMA((u8 *)TxDat, 14);
}

bool TRAN::uart_Send_GPS(void)
{

	return 1;
}

bool TRAN::uart_Send_Check(void)
{
	if(tran.TxFlag == true) return false;
	tran.TxDat[2] = 0xEF;
	tran.TxDat[3] = 2;
	tran.TxDat[4] = tran.TxHead;
	tran.TxDat[5] = tran.TxSum;

	uint8_t sum = 0;
	for(uint8_t i=0; i<6; i++) sum += TxDat[i];
	TxDat[6] = sum;
	return uart_Send_DMA((u8 *)TxDat, 7);
}

bool TRAN::uart_Send_PID(uint8_t group)
{

	if(tran.TxFlag == true) return false;
	vs16 temp;
	tran.TxDat[2] = 0x10+group-1;
	tran.TxDat[3] = 18;
	temp = pid.kp[group*3-3] * 100;
	tran.TxDat[4] = BYTE1(temp);
	tran.TxDat[5] = BYTE0(temp);
	temp = pid.ki[group*3-3] * 100;
	tran.TxDat[6] = BYTE1(temp);
	tran.TxDat[7] = BYTE0(temp);
	temp = pid.kd[group*3-3] * 100;
	tran.TxDat[8] = BYTE1(temp);
	tran.TxDat[9] = BYTE0(temp);
	temp = pid.kp[group*3-2] * 100;
	tran.TxDat[10] = BYTE1(temp);
	tran.TxDat[11] = BYTE0(temp);
	temp = pid.ki[group*3-2] * 100;
	tran.TxDat[12] = BYTE1(temp);
	tran.TxDat[13] = BYTE0(temp);
	temp = pid.kd[group*3-2] * 100;
	tran.TxDat[14] = BYTE1(temp);
	tran.TxDat[15] = BYTE0(temp);
	temp = pid.kp[group*3-1] * 100;
	tran.TxDat[16] = BYTE1(temp);
	tran.TxDat[17] = BYTE0(temp);
	temp = pid.ki[group*3-1] * 100;
	tran.TxDat[18] = BYTE1(temp);
	tran.TxDat[19] = BYTE0(temp);
	temp = pid.kd[group*3-1] * 100;
	tran.TxDat[20] = BYTE1(temp);
	tran.TxDat[21] = BYTE0(temp);

	uint8_t sum = 0;
	for(uint8_t i=0; i<22; i++) sum += TxDat[i];
	TxDat[22] = sum;
	return uart_Send_DMA((u8 *)TxDat, 23);
}

bool TRAN::uart_Send_User(void)
{

	return 1;
}



void TRAN::uart_Receive_Update(void)
{
	if(RxFlag == false) return;
	RxFlag = false;
	if(LockRx == HAL_LOCKED) return;
	LockRx = HAL_LOCKED;
	do{
		uint8_t num = RxDat[3] + 5;
		TxSum = 0;
		for(uint8_t i=0; i<num-1; i++) TxSum += RxDat[i];
		if(TxSum != RxDat[num-1])break;		//和校验 判断sum
		TxHead = RxDat[2];
		switch(TxHead)  //功能字
		{
			case 0x01U:
				switch(RxDat[4])	//校准
				{

				}
				break;
			case 0x02U:
				switch(RxDat[4])  //读取
				{
					case 0x01U://读取pid
						send_pid = true;
						break;
					case 0x02U://读取飞行模式
						break;
					case 0x21U://读取航点数量
						break;
					case 0xA0U://读取版本
						send_version = true;
						break;
					case 0xA1U://恢复默认参数
						break;
					default: break;
				}
				break;
			case 0x03U: //读取遥控数据
				break;
			case 0x0AU: //读取飞行模式
				break;
			case 0x10U: //写入第1组pid
				uart_Set_Pid_Para(1);
				break;
			case 0x11U: //写入第2组pid
				uart_Set_Pid_Para(2);
				break;
			case 0x12U: //写入第3组pid
				uart_Set_Pid_Para(3);
				break;
			case 0x13U: //写入第4组pid
				uart_Set_Pid_Para(4);
				break;
			case 0x14U: //写入第5组pid
				uart_Set_Pid_Para(5);
				break;
			case 0x15U: //写入第6组pid
				uart_Set_Pid_Para(6);
				break;
			case 0x20U: //读取第n个航点
				break;
			case 0x21U: //写入航点
				break;
		}
	}while(0);
	LockRx = HAL_UNLOCKED;

}

void TRAN::uart_Set_Pid_Para(uint8_t group)
{
	pid.kp[group*3-3] = 0.01f * ( (vs16)(*(tran.RxDat+4 )<<8)|*(tran.RxDat+5 ) );
	pid.ki[group*3-3] = 0.01f * ( (vs16)(*(tran.RxDat+6 )<<8)|*(tran.RxDat+7 ) );
	pid.kd[group*3-3] = 0.01f * ( (vs16)(*(tran.RxDat+8 )<<8)|*(tran.RxDat+9 ) );
	pid.kp[group*3-2] = 0.01f * ( (vs16)(*(tran.RxDat+10 )<<8)|*(tran.RxDat+11 ) );
	pid.ki[group*3-2] = 0.01f * ( (vs16)(*(tran.RxDat+12 )<<8)|*(tran.RxDat+13 ) );
	pid.kd[group*3-2] = 0.01f * ( (vs16)(*(tran.RxDat+14 )<<8)|*(tran.RxDat+15 ) );
	pid.kp[group*3-1] = 0.01f * ( (vs16)(*(tran.RxDat+16 )<<8)|*(tran.RxDat+17 ) );
	pid.ki[group*3-1] = 0.01f * ( (vs16)(*(tran.RxDat+18 )<<8)|*(tran.RxDat+19 ) );
	pid.kd[group*3-1] = 0.01f * ( (vs16)(*(tran.RxDat+20 )<<8)|*(tran.RxDat+21 ) );

	xQueueOverwrite(queuePID,&pid);
	uart_Send_Check();

}

extern "C" void tran_main(void *argument)
{
	tran.tran_Init();
	osDelay(10);
	for(;;)
	{
		osSemaphoreAcquire(semTran,0xffffffff);
		tran.uart_Send_DMA_loop();
	}
}


extern "C" void tranReceive_main(void *argument)
{

	osDelay(500);//等待系统完成初始化
	for(;;)
	{
		vTaskSuspend(tranReceiveTaskHandle);
		tran.uart_Receive_Update();

	}
}

/************************ (C) COPYRIGHT Longmen Drone Team *****END OF FILE****/
