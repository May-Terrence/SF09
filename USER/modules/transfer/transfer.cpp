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


uint8_t    RxRawDat[TRAN_RX_LEN];	//数据

TRAN tran(USART3,(char *)"tran");

void USART3_IRQHandler(void)
{
	if(LL_USART_IsActiveFlag_IDLE(tran.huart))
	{
		LL_USART_ClearFlag_IDLE(tran.huart);
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
	LL_DMA_SetPeriphAddress(DMA1, LL_DMA_STREAM_3, (uint32_t)&huart->TDR);
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

//	xQueuePeek(queueMagDat, &mag, 0);

	xQueuePeek(queueMag, &magb, 0);


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
	_temp = magb.MagRaw[0];
	TxDat[16] = BYTE1(_temp);
	TxDat[17] = BYTE0(_temp);
	_temp = magb.MagRaw[1];;
	TxDat[18] = BYTE1(_temp);
	TxDat[19] = BYTE0(_temp);
	_temp = magb.MagRaw[2];
	TxDat[20] = BYTE1(_temp);
	TxDat[21] = BYTE0(_temp);

	uint8_t sum = 0;
	for(uint8_t i=0; i<22; i++) sum += TxDat[i];
	TxDat[22] = sum;
	return uart_Send_DMA((u8 *)TxDat, 23);

}
bool TRAN::uart_Send_Sensor2(void)
{
	xQueuePeek(queueGps,&gps,0);
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
	_temp2 = gps.gpsPosAccuracy*100;	;		// 曲线 ALT_CSB
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
	xQueuePeek(queueESKF,&eskf,0);

	vs16 _temp;
	vs32 _temp2;
	TxDat[2] = 0x01;
	TxDat[3] = 12;
//	_temp = (s16)(ahrsEuler.Ang[0]*R2D*100.0f);
	_temp = (s16)(eskf.Attitude[0]*R2D*100.0f);
	TxDat[4] = BYTE1(_temp);
	TxDat[5] = BYTE0(_temp);
//	_temp = (s16)(ahrsEuler.Ang[1]*R2D*100.0f);
	_temp = (s16)(eskf.Attitude[1]*R2D*100.0f);
	TxDat[6] = BYTE1(_temp);
	TxDat[7] = BYTE0(_temp);
//	_temp = (s16)(ahrsEuler.Ang[2]*R2D*100.0f);
	_temp = (s16)(eskf.Attitude[2]*R2D*100.0f);
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

	xQueuePeek(queueMotorData,&motor_msg, 0);

	if(TxFlag == true) return false;
	TxDat[2] = 0x06;
	TxDat[3] = 16;
	TxDat[4] = BYTE1(motor_msg.PWM_OBS[0]);
	TxDat[5] = BYTE0(motor_msg.PWM_OBS[0]);
	TxDat[6] = BYTE1(motor_msg.PWM_OBS[1]);
	TxDat[7] = BYTE0(motor_msg.PWM_OBS[1]);
	TxDat[8] = BYTE1(motor_msg.PWM_OBS[2]);
	TxDat[9] = BYTE0(motor_msg.PWM_OBS[2]);
	TxDat[10]= BYTE1(motor_msg.PWM_OBS[3]);
	TxDat[11]= BYTE0(motor_msg.PWM_OBS[3]);

	uint8_t sum = 0;
	for(uint8_t i=0; i<20; i++) sum += TxDat[i];
	TxDat[20] = sum;
	return uart_Send_DMA((u8 *)TxDat, 21);
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
	if(TxFlag == true)return false;

	xQueuePeek(queueGps, &gps, 0);

	vs32 _temp;
	vs16 _temp2;

	TxDat[2]=0x04;
	TxDat[3]=12;

	TxDat[4]=gps.status;
	TxDat[5]=gps.star;
	_temp = gps.lng *10000000.0;
	TxDat[6]=BYTE3(_temp);
	TxDat[7]=BYTE2(_temp);
	TxDat[8]=BYTE1(_temp);
	TxDat[9]=BYTE0(_temp);
	_temp = gps.lat *10000000.0;
	TxDat[10]=BYTE3(_temp);
	TxDat[11]=BYTE2(_temp);
	TxDat[12]=BYTE1(_temp);
	TxDat[13]=BYTE0(_temp);
//	_temp2 = gps->vtg_dir * 10.0f;
	_temp2 = 0;
	TxDat[14]=BYTE1(_temp2);
	TxDat[15]=BYTE0(_temp2);

	u8 sum = 0;
	for(u8 i=0;i<16;i++)sum += TxDat[i];
	TxDat[16]=sum;
	return uart_Send_DMA((u8 *)TxDat, 17);
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
	temp = pid.kp[group*3-3] * 1000;
	tran.TxDat[4] = BYTE1(temp);
	tran.TxDat[5] = BYTE0(temp);
	temp = pid.ki[group*3-3] * 1000;
	tran.TxDat[6] = BYTE1(temp);
	tran.TxDat[7] = BYTE0(temp);
	temp = pid.kd[group*3-3] * 1000;
	tran.TxDat[8] = BYTE1(temp);
	tran.TxDat[9] = BYTE0(temp);
	temp = pid.kp[group*3-2] * 1000;
	tran.TxDat[10] = BYTE1(temp);
	tran.TxDat[11] = BYTE0(temp);
	temp = pid.ki[group*3-2] * 1000;
	tran.TxDat[12] = BYTE1(temp);
	tran.TxDat[13] = BYTE0(temp);
	temp = pid.kd[group*3-2] * 1000;
	tran.TxDat[14] = BYTE1(temp);
	tran.TxDat[15] = BYTE0(temp);
	temp = pid.kp[group*3-1] * 1000;
	tran.TxDat[16] = BYTE1(temp);
	tran.TxDat[17] = BYTE0(temp);
	temp = pid.ki[group*3-1] * 1000;
	tran.TxDat[18] = BYTE1(temp);
	tran.TxDat[19] = BYTE0(temp);
	temp = pid.kd[group*3-1] * 1000;
	tran.TxDat[20] = BYTE1(temp);
	tran.TxDat[21] = BYTE0(temp);

	uint8_t sum = 0;
	for(uint8_t i=0; i<22; i++) sum += TxDat[i];
	TxDat[22] = sum;
	return uart_Send_DMA((u8 *)TxDat, 23);
}

/*
bool TRAN::uart_Send_User_control_data(void)
{

	if(tran.TxFlag == true) return false;
	xQueuePeek(queueControlTransfer,&control_data,0);
	xQueuePeek(queueHeight,&height,0);
	xQueuePeek(queueFlow,&flow,0);

	vs16 temp;
	tran.TxDat[2] = 0xF1;
	tran.TxDat[3] = 40;


	temp = control_data.X_command[1]*100;						//user_data1
	tran.TxDat[4] = BYTE1(temp);
	tran.TxDat[5] = BYTE0(temp);

	temp = control_data.Y_command[1]*100;						//user_data2
	tran.TxDat[6] = BYTE1(temp);
	tran.TxDat[7] = BYTE0(temp);

	temp = control_data.Z_command[1]*100;						//user_data3
	tran.TxDat[8] = BYTE1(temp);
	tran.TxDat[9] = BYTE0(temp);

	temp = control_data.XH[1]*100;						//user_data4
	tran.TxDat[10] = BYTE1(temp);
	tran.TxDat[11] = BYTE0(temp);

	temp = control_data.YH[1]*100;						//user_data5
	tran.TxDat[12] = BYTE1(temp);
	tran.TxDat[13] = BYTE0(temp);

	temp = control_data.Z[1]*100;						//user_data6
	tran.TxDat[14] = BYTE1(temp);
	tran.TxDat[15] = BYTE0(temp);

	temp = control_data.Z_command[0]*100;						//user_data7
	tran.TxDat[16] = BYTE1(temp);
	tran.TxDat[17] = BYTE0(temp);

	temp = control_data.Z[0]*100;						//user_data8
	tran.TxDat[18] = BYTE1(temp);
	tran.TxDat[19] = BYTE0(temp);

	temp = control_data.X[0]*100;						//user_data9
	tran.TxDat[20] = BYTE1(temp);
	tran.TxDat[21] = BYTE0(temp);

	temp = control_data.Y[0]*100;						//user_data10
	tran.TxDat[22] = BYTE1(temp);
	tran.TxDat[23] = BYTE0(temp);

	temp = control_data.Ang[0]*R2D*100;						//user_data11
	tran.TxDat[24] = BYTE1(temp);
	tran.TxDat[25] = BYTE0(temp);

	temp = control_data.Ang[1]*R2D*100;						//user_data12
	tran.TxDat[26] = BYTE1(temp);
	tran.TxDat[27] = BYTE0(temp);

	temp = control_data.roll_command[0]*R2D*100;						//user_data13
	tran.TxDat[28] = BYTE1(temp);
	tran.TxDat[29] = BYTE0(temp);

	temp = control_data.pitch_command[0]*R2D*100;						//user_data14
	tran.TxDat[30] = BYTE1(temp);
	tran.TxDat[31] = BYTE0(temp);

	temp = control_data.brake_mode;						//user_data15
	tran.TxDat[32] = BYTE1(temp);
	tran.TxDat[33] = BYTE0(temp);

	temp = control_data.rc_status;						//user_data16
	tran.TxDat[34] = BYTE1(temp);
	tran.TxDat[35] = BYTE0(temp);

	temp = control_data.mt_output[0];						//user_data17
	tran.TxDat[36] = BYTE1(temp);
	tran.TxDat[37] = BYTE0(temp);

	temp = flow.flowx*100;						//user_data18
	tran.TxDat[38] = BYTE1(temp);
	tran.TxDat[39] = BYTE0(temp);
	temp = flow.flowy*100;						//user_data19
	tran.TxDat[40] = BYTE1(temp);
	tran.TxDat[41] = BYTE0(temp);
	temp = height.height*100;						//user_data20
	tran.TxDat[42] = BYTE1(temp);
	tran.TxDat[43] = BYTE0(temp);
	uint8_t sum = 0;
	for(uint8_t i=0; i<44; i++) sum += TxDat[i];
	TxDat[44] = sum;
	return uart_Send_DMA((u8 *)TxDat, 45);
}
*/

//bool TRAN::uart_Send_User(void)
//{
//
//	if(tran.TxFlag == true) return false;
//	xQueuePeek(queueEKF,&ekfcpf,0);
//	xQueuePeek(queueControlTransfer,&control_data,0);
//
//	vs16 temp;
//	tran.TxDat[2] = 0xF1;
//	tran.TxDat[3] = 40;
//
//
//	temp = ekfcpf.Ned[0]*100;						//user_data1
//	tran.TxDat[4] = BYTE1(temp);
//	tran.TxDat[5] = BYTE0(temp);
//
//	temp = ekfcpf.Ned[1]*100;						//user_data2
//	tran.TxDat[6] = BYTE1(temp);
//	tran.TxDat[7] = BYTE0(temp);
//
//	temp = ekfcpf.Ned[2]*100;						//user_data3
//	tran.TxDat[8] = BYTE1(temp);
//	tran.TxDat[9] = BYTE0(temp);
//
//	temp = ekfcpf.Ned_spd[0];						//user_data4
//	tran.TxDat[10] = BYTE1(temp);
//	tran.TxDat[11] = BYTE0(temp);
//
//	temp = ekfcpf.Ned_spd[1];						//user_data5
//	tran.TxDat[12] = BYTE1(temp);
//	tran.TxDat[13] = BYTE0(temp);
//
//	temp = ekfcpf.Ned_spd[2];						//user_data6
//	tran.TxDat[14] = BYTE1(temp);
//	tran.TxDat[15] = BYTE0(temp);
//
//	temp = control_data.Z_command[0]*100;						//user_data7
//	tran.TxDat[16] = BYTE1(temp);
//	tran.TxDat[17] = BYTE0(temp);
//
//	temp = control_data.Z[0]*100;						//user_data8
//	tran.TxDat[18] = BYTE1(temp);
//	tran.TxDat[19] = BYTE0(temp);
//
//	temp = control_data.X[0]*100;						//user_data9
//	tran.TxDat[20] = BYTE1(temp);
//	tran.TxDat[21] = BYTE0(temp);
//
//	temp = control_data.Y[0]*100;						//user_data10
//	tran.TxDat[22] = BYTE1(temp);
//	tran.TxDat[23] = BYTE0(temp);
//
//	temp = control_data.Ang[0]*R2D*100;						//user_data11
//	tran.TxDat[24] = BYTE1(temp);
//	tran.TxDat[25] = BYTE0(temp);
//
//	temp = control_data.Ang[1]*R2D*100;						//user_data12
//	tran.TxDat[26] = BYTE1(temp);
//	tran.TxDat[27] = BYTE0(temp);
//
//	temp = control_data.roll_command[0]*R2D*100;						//user_data13
//	tran.TxDat[28] = BYTE1(temp);
//	tran.TxDat[29] = BYTE0(temp);
//
//	temp = control_data.posion_mode;						//user_data14
//	tran.TxDat[30] = BYTE1(temp);
//	tran.TxDat[31] = BYTE0(temp);
//
//	temp = control_data.brake_mode;						//user_data15
//	tran.TxDat[32] = BYTE1(temp);
//	tran.TxDat[33] = BYTE0(temp);
//
//	temp = control_data.rc_status;						//user_data16
//	tran.TxDat[34] = BYTE1(temp);
//	tran.TxDat[35] = BYTE0(temp);
//
//	temp = control_data.mt_output[0];						//user_data17
//	tran.TxDat[36] = BYTE1(temp);
//	tran.TxDat[37] = BYTE0(temp);
//
//	temp = flow.flowx*100;						//user_data18
//	tran.TxDat[38] = BYTE1(temp);
//	tran.TxDat[39] = BYTE0(temp);
//	temp = flow.flowy*100;						//user_data19
//	tran.TxDat[40] = BYTE1(temp);
//	tran.TxDat[41] = BYTE0(temp);
//	temp = height.height*100;						//user_data20
//	tran.TxDat[42] = BYTE1(temp);
//	tran.TxDat[43] = BYTE0(temp);
//	uint8_t sum = 0;
//	for(uint8_t i=0; i<44; i++) sum += TxDat[i];
//	TxDat[44] = sum;
//	return uart_Send_DMA((u8 *)TxDat, 45);
//}
#define __outloop  //__sensorcali传感器校正,__outloop外环,__trajectory航点,__trajectory2航点模式调试

#ifdef __inloop
bool TRAN::uart_Send_User(void)		//内环
{

	if(tran.TxFlag == true) return false;
	xQueuePeek(queueAhrsEuler, &ahrsEuler, 0);
	xQueuePeek(queueESKF,&eskf,0);
	xQueuePeek(queueGps,&gps,0);
	xQueuePeek(queueControlTransfer,&control_data,0);

	vs16 temp;
	tran.TxDat[2] = 0xF1;
	tran.TxDat[3] = 40;


	temp = control_data.Ang[0]*R2D*100;						//user_data1
	tran.TxDat[4] = BYTE1(temp);
	tran.TxDat[5] = BYTE0(temp);

	temp = control_data.Ang[1]*R2D*100;					//user_data2
	tran.TxDat[6] = BYTE1(temp);
	tran.TxDat[7] = BYTE0(temp);

	temp = control_data.Ang[2]*R2D*100;						//user_data3
	tran.TxDat[8] = BYTE1(temp);
	tran.TxDat[9] = BYTE0(temp);


	temp = control_data.roll_command[0]*R2D*100;						//user_data4
	tran.TxDat[10] = BYTE1(temp);
	tran.TxDat[11] = BYTE0(temp);

	temp = control_data.pitch_command[0]*R2D*100;						//user_data5
	tran.TxDat[12] = BYTE1(temp);
	tran.TxDat[13] = BYTE0(temp);

	temp = control_data.yaw_command[0]*R2D*100;						//user_data6
	tran.TxDat[14] = BYTE1(temp);
	tran.TxDat[15] = BYTE0(temp);

	temp = control_data.pqr[0]*R2D*100;						//user_data7
	tran.TxDat[16] = BYTE1(temp);
	tran.TxDat[17] = BYTE0(temp);

	temp = control_data.pqr[1]*R2D*100;						//user_data8
	tran.TxDat[18] = BYTE1(temp);
	tran.TxDat[19] = BYTE0(temp);

	temp = control_data.pqr[2]*R2D*100;						//user_data9
	tran.TxDat[20] = BYTE1(temp);
	tran.TxDat[21] = BYTE0(temp);

	temp = control_data.pqr_command[0]*R2D*100;						//user_data10
	tran.TxDat[22] = BYTE1(temp);
	tran.TxDat[23] = BYTE0(temp);

	temp = control_data.pqr_command[1]*R2D*100;						//user_data11
	tran.TxDat[24] = BYTE1(temp);
	tran.TxDat[25] = BYTE0(temp);

	temp = control_data.pqr_command[2]*R2D*100;						//user_data12
	tran.TxDat[26] = BYTE1(temp);
	tran.TxDat[27] = BYTE0(temp);

	temp = control_data.pqr_d0[0]*R2D*100;						//user_data13
	tran.TxDat[28] = BYTE1(temp);
	tran.TxDat[29] = BYTE0(temp);

	temp = control_data.pqr_d0[1]*R2D*100;						//user_data14
	tran.TxDat[30] = BYTE1(temp);
	tran.TxDat[31] = BYTE0(temp);

	temp = control_data.pqr_d0[2]*R2D*100;						//user_data15
	tran.TxDat[32] = BYTE1(temp);
	tran.TxDat[33] = BYTE0(temp);

	temp = control_data.pqr_d_raw[0]*R2D*100;        		//user_data16
	tran.TxDat[34] = BYTE1(temp);
	tran.TxDat[35] = BYTE0(temp);

	temp = control_data.pqr_d_raw[1]*R2D*100; 						//user_data17
	tran.TxDat[36] = BYTE1(temp);
	tran.TxDat[37] = BYTE0(temp);

	temp = control_data.pqr_d_raw[2]*R2D*100; 						//user_data18
	tran.TxDat[38] = BYTE1(temp);
	tran.TxDat[39] = BYTE0(temp);
	temp = control_data.mt_output[0];						//user_data19
	tran.TxDat[40] = BYTE1(temp);
	tran.TxDat[41] = BYTE0(temp);
	temp = ahrsEuler.Ang[2]*R2D*100;						//user_data20
	tran.TxDat[42] = BYTE1(temp);
	tran.TxDat[43] = BYTE0(temp);
	uint8_t sum = 0;
	for(uint8_t i=0; i<44; i++) sum += TxDat[i];
	TxDat[44] = sum;
	return uart_Send_DMA((u8 *)TxDat, 45);
}
#endif

#ifdef __sensorcali
bool TRAN::uart_Send_User(void)		//传感器校正
{

	if(tran.TxFlag == true) return false;
	xQueuePeek(queueGyrDat, &gyro, 0);

	xQueuePeek(queueAccDat, &acc, 0);

	xQueuePeek(queueMag, &magb, 0);

	vs16 temp;
	tran.TxDat[2] = 0xF1;
	tran.TxDat[3] = 40;


	temp = acc.acc[0]*1000;						//user_data1
	tran.TxDat[4] = BYTE1(temp);
	tran.TxDat[5] = BYTE0(temp);

	temp = acc.acc[1]*1000;					//user_data2
	tran.TxDat[6] = BYTE1(temp);
	tran.TxDat[7] = BYTE0(temp);

	temp = acc.acc[2]*1000;						//user_data3
	tran.TxDat[8] = BYTE1(temp);
	tran.TxDat[9] = BYTE0(temp);

	temp = gyro.gyro[0]*1000;						//user_data4
	tran.TxDat[10] = BYTE1(temp);
	tran.TxDat[11] = BYTE0(temp);

	temp = gyro.gyro[1]*1000;						//user_data5
	tran.TxDat[12] = BYTE1(temp);
	tran.TxDat[13] = BYTE0(temp);

	temp = gyro.gyro[2]*1000;						//user_data6
	tran.TxDat[14] = BYTE1(temp);
	tran.TxDat[15] = BYTE0(temp);

	temp = magb.MagRaw[0];						//user_data7
	tran.TxDat[16] = BYTE1(temp);
	tran.TxDat[17] = BYTE0(temp);

	temp = magb.MagRaw[1];						//user_data8
	tran.TxDat[18] = BYTE1(temp);
	tran.TxDat[19] = BYTE0(temp);

	temp = magb.MagRaw[2];						//user_data9
	tran.TxDat[20] = BYTE1(temp);
	tran.TxDat[21] = BYTE0(temp);

	temp = 0;						//user_data10
	tran.TxDat[22] = BYTE1(temp);
	tran.TxDat[23] = BYTE0(temp);

	temp = 0;						//user_data11
	tran.TxDat[24] = BYTE1(temp);
	tran.TxDat[25] = BYTE0(temp);

	temp = 0;						//user_data12
	tran.TxDat[26] = BYTE1(temp);
	tran.TxDat[27] = BYTE0(temp);

	temp = 0;						//user_data13
	tran.TxDat[28] = BYTE1(temp);
	tran.TxDat[29] = BYTE0(temp);

	temp = 0;						//user_data14
	tran.TxDat[30] = BYTE1(temp);
	tran.TxDat[31] = BYTE0(temp);

	temp = 0;						//user_data15
	tran.TxDat[32] = BYTE1(temp);
	tran.TxDat[33] = BYTE0(temp);

	temp = 0;						//user_data16
	tran.TxDat[34] = BYTE1(temp);
	tran.TxDat[35] = BYTE0(temp);

	temp = 0;						//user_data17
	tran.TxDat[36] = BYTE1(temp);
	tran.TxDat[37] = BYTE0(temp);

	temp = 0;						//user_data18
	tran.TxDat[38] = BYTE1(temp);
	tran.TxDat[39] = BYTE0(temp);
	temp = 0;						//user_data19
	tran.TxDat[40] = BYTE1(temp);
	tran.TxDat[41] = BYTE0(temp);
	temp = 0;						//user_data20
	tran.TxDat[42] = BYTE1(temp);
	tran.TxDat[43] = BYTE0(temp);
	uint8_t sum = 0;
	for(uint8_t i=0; i<44; i++) sum += TxDat[i];
	TxDat[44] = sum;
	return uart_Send_DMA((u8 *)TxDat, 45);
}
#endif

#ifdef __outloop
bool TRAN::uart_Send_User(void)//外环
{

	if(tran.TxFlag == true) return false;
	xQueuePeek(queueControlTransfer,&control_data,0);
	xQueuePeek(queueGps,&gps,0);
	xQueuePeek(queueHeight,&height,0);
	xQueuePeek(queueFlow,&flow,0);
	xQueuePeek(queueAccDat, &acc, 0);
	xQueuePeek(queueAccDatFil, &acc_fil, 0);

	vs16 temp;
	tran.TxDat[2] = 0xF1;
	tran.TxDat[3] = 40;


	temp = control_data.X_command[1]*100;						//user_data1
	tran.TxDat[4] = BYTE1(temp);
	tran.TxDat[5] = BYTE0(temp);

	temp = control_data.Y_command[1]*100;						//user_data2
	tran.TxDat[6] = BYTE1(temp);
	tran.TxDat[7] = BYTE0(temp);

	temp = control_data.Z_command[1]*100;						//user_data3
	tran.TxDat[8] = BYTE1(temp);
	tran.TxDat[9] = BYTE0(temp);

	temp = control_data.XH[1]*100;						//user_data4
	tran.TxDat[10] = BYTE1(temp);
	tran.TxDat[11] = BYTE0(temp);

	temp = control_data.YH[1]*100;						//user_data5
	tran.TxDat[12] = BYTE1(temp);
	tran.TxDat[13] = BYTE0(temp);

	temp = control_data.Z[1]*100;						//user_data6
	tran.TxDat[14] = BYTE1(temp);
	tran.TxDat[15] = BYTE0(temp);

	temp = control_data.Z_command[0]*100;						//user_data7
	tran.TxDat[16] = BYTE1(temp);
	tran.TxDat[17] = BYTE0(temp);

	temp = control_data.Z[0]*100;						//user_data8
	tran.TxDat[18] = BYTE1(temp);
	tran.TxDat[19] = BYTE0(temp);

	temp = control_data.X[0]*100;						//user_data9
	tran.TxDat[20] = BYTE1(temp);
	tran.TxDat[21] = BYTE0(temp);

	temp = control_data.Y[0]*100;						//user_data10
	tran.TxDat[22] = BYTE1(temp);
	tran.TxDat[23] = BYTE0(temp);

	temp = control_data.Ang[0]*R2D*100;						//user_data11
	tran.TxDat[24] = BYTE1(temp);
	tran.TxDat[25] = BYTE0(temp);

	temp = control_data.Ang[1]*R2D*100;						//user_data12
	tran.TxDat[26] = BYTE1(temp);
	tran.TxDat[27] = BYTE0(temp);

	temp = control_data.roll_command[0]*R2D*100;						//user_data13
	tran.TxDat[28] = BYTE1(temp);
	tran.TxDat[29] = BYTE0(temp);

	temp = control_data.pitch_command[0]*R2D*100;						//user_data14
	tran.TxDat[30] = BYTE1(temp);
	tran.TxDat[31] = BYTE0(temp);

	temp = control_data.yaw_command[0]*R2D*100;						//user_data15
//	temp = acc.acc[1]*100;
	tran.TxDat[32] = BYTE1(temp);
	tran.TxDat[33] = BYTE0(temp);

	temp = control_data.Ang[2]*R2D*100;						//user_data16
//	temp = acc.acc[2]*100;
	tran.TxDat[34] = BYTE1(temp);
	tran.TxDat[35] = BYTE0(temp);

	temp = control_data.mt_output[0];						//user_data17
//	temp = acc_fil.acc_filter[1]*100;
	tran.TxDat[36] = BYTE1(temp);
	tran.TxDat[37] = BYTE0(temp);

	temp = acc_fil.acc_filter[2]*100;					//user_data18
//	temp = acc_fil.acc_filter[2]*100;
	tran.TxDat[38] = BYTE1(temp);
	tran.TxDat[39] = BYTE0(temp);
	temp = control_data.output1[0]*R2D*100;						//user_data19
	tran.TxDat[40] = BYTE1(temp);
	tran.TxDat[41] = BYTE0(temp);
	temp = control_data.output[0]*R2D*100;						//user_data20
//	temp = test_cnt;
	tran.TxDat[42] = BYTE1(temp);
	tran.TxDat[43] = BYTE0(temp);
	uint8_t sum = 0;
	for(uint8_t i=0; i<44; i++) sum += TxDat[i];
	TxDat[44] = sum;
	return uart_Send_DMA((u8 *)TxDat, 45);
}
#endif


//bool TRAN::uart_Send_User(void)
//{
//
//	if(tran.TxFlag == true) return false;
//	xQueuePeek(queueAhrsEuler, &ahrsEuler, 0);
//	xQueuePeek(queueGps,&gps,0);
//	xQueuePeek(queueGyrDat, &gyro, 0);
//	xQueuePeek(queueAccDat, &acc, 0);
//	xQueuePeek(queueMag, &magb, 0);
//
//	vs16 temp;
//	uint16_t utemp;
//	tran.TxDat[2] = 0xF1;
//	tran.TxDat[3] = 40;
//
//
//	temp = gyro.gyro[0]*1000;						//user_data1
//	tran.TxDat[4] = BYTE1(temp);
//	tran.TxDat[5] = BYTE0(temp);
//
//	temp = gyro.gyro[1]*1000;					//user_data2
//	tran.TxDat[6] = BYTE1(temp);
//	tran.TxDat[7] = BYTE0(temp);
//
//	temp = gyro.gyro[2]*1000;						//user_data3
//	tran.TxDat[8] = BYTE1(temp);
//	tran.TxDat[9] = BYTE0(temp);
//
//	temp = acc.acc[0]*1000;						//user_data4
//	tran.TxDat[10] = BYTE1(temp);
//	tran.TxDat[11] = BYTE0(temp);
//
//	temp = acc.acc[1]*1000;						//user_data5
//	tran.TxDat[12] = BYTE1(temp);
//	tran.TxDat[13] = BYTE0(temp);
//
//	temp = acc.acc[2]*1000;						//user_data6
//	tran.TxDat[14] = BYTE1(temp);
//	tran.TxDat[15] = BYTE0(temp);
//
//	temp = magb.MagRel[0];						//user_data7
//	tran.TxDat[16] = BYTE1(temp);
//	tran.TxDat[17] = BYTE0(temp);
//
//	temp = magb.MagRel[1];						//user_data8
//	tran.TxDat[18] = BYTE1(temp);
//	tran.TxDat[19] = BYTE0(temp);
//
//	temp = magb.MagRel[2];						//user_data9
//	tran.TxDat[20] = BYTE1(temp);
//	tran.TxDat[21] = BYTE0(temp);
//
//	temp = gps.NED[0]*1000;						//user_data10
//	tran.TxDat[22] = BYTE1(temp);
//	tran.TxDat[23] = BYTE0(temp);
//
//	temp = gps.NED[1]*1000;						//user_data11
//	tran.TxDat[24] = BYTE1(temp);
//	tran.TxDat[25] = BYTE0(temp);
//
//	temp = gps.NED[2]*1000;						//user_data12
//	tran.TxDat[26] = BYTE1(temp);
//	tran.TxDat[27] = BYTE0(temp);
//
//	temp = gps.NED_spd[0]*1000;						//user_data13
//	tran.TxDat[28] = BYTE1(temp);
//	tran.TxDat[29] = BYTE0(temp);
//
//	temp = gps.NED_spd[1]*1000;						//user_data14
//	tran.TxDat[30] = BYTE1(temp);
//	tran.TxDat[31] = BYTE0(temp);
//
//	temp = gps.NED_spd[2]*1000;						//user_data15
//	tran.TxDat[32] = BYTE1(temp);
//	tran.TxDat[33] = BYTE0(temp);
//
//	temp = acc.timestamp/10000;						//user_data16
//	tran.TxDat[34] = BYTE1(temp);
//	tran.TxDat[35] = BYTE0(temp);
//
//	temp = acc.timestamp%10000;						//user_data17
//	tran.TxDat[36] = BYTE1(temp);
//	tran.TxDat[37] = BYTE0(temp);
//
//	temp = ahrsEuler.Ang[0]*R2D*100;						//user_data18
//	tran.TxDat[38] = BYTE1(temp);
//	tran.TxDat[39] = BYTE0(temp);
//	temp = ahrsEuler.Ang[1]*R2D*100;						//user_data19
//	tran.TxDat[40] = BYTE1(temp);
//	tran.TxDat[41] = BYTE0(temp);
//	temp = ahrsEuler.Ang[2]*R2D*100;						//user_data20
//	tran.TxDat[42] = BYTE1(temp);
//	tran.TxDat[43] = BYTE0(temp);
//	uint8_t sum = 0;
//	for(uint8_t i=0; i<44; i++) sum += TxDat[i];
//	TxDat[44] = sum;
//	return uart_Send_DMA((u8 *)TxDat, 45);
//}

#ifdef __trajectory
bool TRAN::uart_Send_User(void)//航点
{

	if(tran.TxFlag == true) return false;
	xQueuePeek(queueControlTransfer,&control_data,0);
	xQueuePeek(queueControlOutputDat,&control_output_msg,0);
	xQueuePeek(queueGps,&gps,0);
	xQueuePeek(queueHeight,&height,0);
	xQueuePeek(queueFlow,&flow,0);
	xQueuePeek(queueAccDat, &acc, 0);


	vs16 temp;
	tran.TxDat[2] = 0xF1;
	tran.TxDat[3] = 40;

	temp = control_data.X_command[0]*100;					//user_data1	X位置期望
	tran.TxDat[4] = BYTE1(temp);
	tran.TxDat[5] = BYTE0(temp);

	temp = control_data.Y_command[0]*100;					//user_data2	Y位置期望
	tran.TxDat[6] = BYTE1(temp);
	tran.TxDat[7] = BYTE0(temp);

	temp = control_data.Z_command[0]*100;					//user_data3	Z位置期望
	tran.TxDat[8] = BYTE1(temp);
	tran.TxDat[9] = BYTE0(temp);

	temp = control_data.X[0]*100;							//user_data4	X位置反馈值
	tran.TxDat[10] = BYTE1(temp);
	tran.TxDat[11] = BYTE0(temp);

	temp = control_data.Y[0]*100;							//user_data5	Y位置反馈值
	tran.TxDat[12] = BYTE1(temp);
	tran.TxDat[13] = BYTE0(temp);

	temp = control_data.Z[0]*100;							//user_data6	Z位置反馈值
	tran.TxDat[14] = BYTE1(temp);
	tran.TxDat[15] = BYTE0(temp);


	temp = control_data.X_command[1]*100;					//user_data7	X速度期望值
	tran.TxDat[16] = BYTE1(temp);
	tran.TxDat[17] = BYTE0(temp);

	temp = control_data.Y_command[1]*100;					//user_data8	Y速度期望值
	tran.TxDat[18] = BYTE1(temp);
	tran.TxDat[19] = BYTE0(temp);

	temp = control_data.Z_command[1]*100;					//user_data9	Z速度期望值
	tran.TxDat[20] = BYTE1(temp);
	tran.TxDat[21] = BYTE0(temp);

	temp = control_data.XH[1]*100;							//user_data10	X速度反馈值
	tran.TxDat[22] = BYTE1(temp);
	tran.TxDat[23] = BYTE0(temp);

	temp = control_data.YH[1]*100;							//user_data11	Y速度反馈值
	tran.TxDat[24] = BYTE1(temp);
	tran.TxDat[25] = BYTE0(temp);

	temp = control_data.Z[1]*100;							//user_data12	Z速度反馈值
	tran.TxDat[26] = BYTE1(temp);
	tran.TxDat[27] = BYTE0(temp);

	temp = control_data.yaw_command[0]*R2D*100;				//user_data13	偏航角期望值
	tran.TxDat[28] = BYTE1(temp);
	tran.TxDat[29] = BYTE0(temp);

	temp = control_data.Ang[2]*R2D*100;						//user_data14	偏航角反馈值
	tran.TxDat[30] = BYTE1(temp);
	tran.TxDat[31] = BYTE0(temp);

	temp = control_output_msg.mt_output[0];					//user_data15	电机1PWM
	tran.TxDat[32] = BYTE1(temp);
	tran.TxDat[33] = BYTE0(temp);

	temp = control_output_msg.mt_output[1];					//user_data16	电机2PWM
	tran.TxDat[34] = BYTE1(temp);
	tran.TxDat[35] = BYTE0(temp);

	temp = control_output_msg.cs_output[0];					//user_data17	舵机1PWM
	tran.TxDat[36] = BYTE1(temp);
	tran.TxDat[37] = BYTE0(temp);

	temp = control_output_msg.cs_output[1];					//user_data18	舵机2PWM
	tran.TxDat[38] = BYTE1(temp);
	tran.TxDat[39] = BYTE0(temp);

	temp = gps.gpsPosAccuracy*100;							//user_data19	RTK精度
	tran.TxDat[40] = BYTE1(temp);
	tran.TxDat[41] = BYTE0(temp);

	temp = control_data.trajectory_status;					//user_data20	轨迹模式状态
	tran.TxDat[42] = BYTE1(temp);
	tran.TxDat[43] = BYTE0(temp);


	uint8_t sum = 0;
	for(uint8_t i=0; i<44; i++) sum += TxDat[i];
	TxDat[44] = sum;
	return uart_Send_DMA((u8 *)TxDat, 45);
}
#endif

#ifdef __trajectory2
bool TRAN::uart_Send_User(void)//航点模式调试
{

	if(tran.TxFlag == true) return false;
	xQueuePeek(queueControlTransfer,&control_data,0);
	xQueuePeek(queueGps,&gps,0);
	xQueuePeek(queueHeight,&height,0);
	xQueuePeek(queueFlow,&flow,0);
	xQueuePeek(queueAccDat, &acc, 0);
	xQueuePeek(queueESKF,&eskf,0);
	xQueuePeek(queueTargetBuffer, &target_buffer, 0);

	vs16 temp;
	tran.TxDat[2] = 0xF1;
	tran.TxDat[3] = 40;


	temp = control_data.X_command[0]*100;			//user_data1  X位置期望值
	tran.TxDat[4] = BYTE1(temp);
	tran.TxDat[5] = BYTE0(temp);

	temp = control_data.Y_command[0]*100;			//user_data2  Y位置期望值
	tran.TxDat[6] = BYTE1(temp);
	tran.TxDat[7] = BYTE0(temp);

	temp = control_data.Z_command[0]*100;			//user_data3  Z位置期望值
	tran.TxDat[8] = BYTE1(temp);
	tran.TxDat[9] = BYTE0(temp);

	temp = eskf.Pos[0]*100;							//user_data4  X位置反馈值
	tran.TxDat[10] = BYTE1(temp);
	tran.TxDat[11] = BYTE0(temp);

	temp = eskf.Pos[1]*100;							//user_data5  Y位置反馈值
	tran.TxDat[12] = BYTE1(temp);
	tran.TxDat[13] = BYTE0(temp);

	temp = eskf.Pos[2]*100;							//user_data6  Z位置反馈值
	tran.TxDat[14] = BYTE1(temp);
	tran.TxDat[15] = BYTE0(temp);

	temp = control_data.roll_command[0]*R2D*100;	//user_data7     滚转期望值
	tran.TxDat[16] = BYTE1(temp);
	tran.TxDat[17] = BYTE0(temp);

	temp = control_data.pitch_command[0]*R2D*100;	//user_data8     俯仰期望值
	tran.TxDat[18] = BYTE1(temp);
	tran.TxDat[19] = BYTE0(temp);

	temp = control_data.yaw_command[0]*R2D*100;		//user_data9     偏航期望值
	tran.TxDat[20] = BYTE1(temp);
	tran.TxDat[21] = BYTE0(temp);

	temp = eskf.Attitude[0]*R2D*100;				//user_data10   滚转反馈值
	tran.TxDat[22] = BYTE1(temp);
	tran.TxDat[23] = BYTE0(temp);

	temp = eskf.Attitude[1]*R2D*100;				//user_data11   俯仰反馈值
	tran.TxDat[24] = BYTE1(temp);
	tran.TxDat[25] = BYTE0(temp);

	temp = eskf.Attitude[2]*R2D*100;				//user_data12   偏航反馈值
	tran.TxDat[26] = BYTE1(temp);
	tran.TxDat[27] = BYTE0(temp);

	temp = control_data.mt_output[0];				//user_data13  电机1转速
	tran.TxDat[28] = BYTE1(temp);
	tran.TxDat[29] = BYTE0(temp);

	temp = control_data.mt_output[1];				//user_data14  电机2转速
	tran.TxDat[30] = BYTE1(temp);
	tran.TxDat[31] = BYTE0(temp);

	temp = gps.gpsPosAccuracy*100;					//user_data15  GPS精度
	tran.TxDat[32] = BYTE1(temp);
	tran.TxDat[33] = BYTE0(temp);

	temp = target_buffer.hover_time;				//user_data16  悬停时间倒计时
	tran.TxDat[34] = BYTE1(temp);
	tran.TxDat[35] = BYTE0(temp);

	uint8_t sum = 0;
	for(uint8_t i=0; i<44; i++) sum += TxDat[i];
	TxDat[44] = sum;
	return uart_Send_DMA((u8 *)TxDat, 45);
}
#endif
//
#ifdef __trajectory3
bool TRAN::uart_Send_User(void)//航点
{

	if(tran.TxFlag == true) return false;
	xQueuePeek(queueControlTransfer,&control_data,0);
//	xQueuePeek(queueControlOutputDat,&control_output_msg,0);
	xQueuePeek(queueGps,&gps,0);
	xQueuePeek(queueESKF,&eskf,0);
	xQueuePeek(queueTargetBuffer,&target_buffer,0);

	xQueuePeek(queueAccDatFil,&acc_filter,0);

	vs16 temp;
	tran.TxDat[2] = 0xF1;
	tran.TxDat[3] = 40;

	temp = control_data.X_command[0]*100;					//user_data1	X位置期望
	tran.TxDat[4] = BYTE1(temp);
	tran.TxDat[5] = BYTE0(temp);

//	temp = testCounter;					//user_data1	X位置期望
//	tran.TxDat[4] = BYTE1(temp);
//	tran.TxDat[5] = BYTE0(temp);
//	testCounter++;

	temp = control_data.Y_command[0]*100;					//user_data2	Y位置期望
	tran.TxDat[6] = BYTE1(temp);
	tran.TxDat[7] = BYTE0(temp);

	temp = control_data.Z_command[0]*100;					//user_data3	Z位置期望
	tran.TxDat[8] = BYTE1(temp);
	tran.TxDat[9] = BYTE0(temp);

	temp = control_data.X[0]*100;							//user_data4	X位置反馈值
	tran.TxDat[10] = BYTE1(temp);
	tran.TxDat[11] = BYTE0(temp);

	temp = control_data.Y[0]*100;							//user_data5	Y位置反馈值
	tran.TxDat[12] = BYTE1(temp);
	tran.TxDat[13] = BYTE0(temp);

	temp = control_data.Z[0]*100;							//user_data6	Z位置反馈值
	tran.TxDat[14] = BYTE1(temp);
	tran.TxDat[15] = BYTE0(temp);


	temp = control_data.X_command[1]*100;					//user_data7	X速度期望值
	tran.TxDat[16] = BYTE1(temp);
	tran.TxDat[17] = BYTE0(temp);

	temp = control_data.Y_command[1]*100;					//user_data8	Y速度期望值
	tran.TxDat[18] = BYTE1(temp);
	tran.TxDat[19] = BYTE0(temp);

	temp = control_data.Z_command[1]*100;					//user_data9	Z速度期望值
	tran.TxDat[20] = BYTE1(temp);
	tran.TxDat[21] = BYTE0(temp);

	temp = control_data.XH[1]*100;							//user_data10	X速度反馈值
	tran.TxDat[22] = BYTE1(temp);
	tran.TxDat[23] = BYTE0(temp);

	temp = control_data.YH[1]*100;							//user_data11	Y速度反馈值
	tran.TxDat[24] = BYTE1(temp);
	tran.TxDat[25] = BYTE0(temp);

	temp = control_data.Z[1]*100;							//user_data12	Z速度反馈值
	tran.TxDat[26] = BYTE1(temp);
	tran.TxDat[27] = BYTE0(temp);

//	temp = control_data.pqr_command[0]*R2D*100;			//user_data13	滚转角期望值
//	tran.TxDat[28] = BYTE1(temp);
//	tran.TxDat[29] = BYTE0(temp);
//
//	temp = control_data.pqr_command[1]*R2D*100;			//user_data14	俯仰角期望值
//	tran.TxDat[30] = BYTE1(temp);
//	tran.TxDat[31] = BYTE0(temp);

	temp = control_data.yaw_command[0]*R2D*100;				//user_data13	偏航角期望值
	tran.TxDat[28] = BYTE1(temp);
	tran.TxDat[29] = BYTE0(temp);

	temp = control_data.Ang[2]*R2D*100;						//user_data14	偏航角反馈值
	tran.TxDat[30] = BYTE1(temp);
	tran.TxDat[31] = BYTE0(temp);
//
//	temp = control_output_msg.mt_output[0];					//user_data15	电机1PWM
//	tran.TxDat[32] = BYTE1(temp);
//	tran.TxDat[33] = BYTE0(temp);
//
//	temp = control_output_msg.mt_output[1];					//user_data16	电机2PWM
//	tran.TxDat[34] = BYTE1(temp);
//	tran.TxDat[35] = BYTE0(temp);
//
//	temp = motor_msg.PWM_OBS[2];					//user_data17	舵机1PWM
//	tran.TxDat[36] = BYTE1(temp);
//	tran.TxDat[37] = BYTE0(temp);
//
//	temp = motor_msg.PWM_OBS[3];					//user_data18	舵机2PWM
//	tran.TxDat[38] = BYTE1(temp);
//	tran.TxDat[39] = BYTE0(temp);

	temp = target_buffer.next_target_ned[0]*100;				//user_data15	下一个目标航点X位置
	tran.TxDat[32] = BYTE1(temp);
	tran.TxDat[33] = BYTE0(temp);

	temp = target_buffer.next_target_ned[1]*100;				//user_data16	下一个目标航点Y
	tran.TxDat[34] = BYTE1(temp);
	tran.TxDat[35] = BYTE0(temp);

	temp = target_buffer.next_target_ned[2]*100;				//user_data17	下一个目标航点Z
	tran.TxDat[36] = BYTE1(temp);
	tran.TxDat[37] = BYTE0(temp);

	temp = target_buffer.hover_time_Trajectory*100;					//user_data18	悬停时间
	tran.TxDat[38] = BYTE1(temp);
	tran.TxDat[39] = BYTE0(temp);

	temp = gps.gpsPosAccuracy*100;							//user_data19	RTK精度
	tran.TxDat[40] = BYTE1(temp);
	tran.TxDat[41] = BYTE0(temp);

//	temp = eskf.Pos[2]*100;							//user_data20	RTK高度
//	tran.TxDat[42] = BYTE1(temp);
//	tran.TxDat[43] = BYTE0(temp);

	temp = acc_filter.acc_filter[2]*100;
	tran.TxDat[42] = BYTE1(temp);
	tran.TxDat[43] = BYTE0(temp);

//	temp = control_data.trajectory_status;					//user_data19	轨迹模式状态
//	tran.TxDat[42] = BYTE1(temp);
//	tran.TxDat[43] = BYTE0(temp);


	uint8_t sum = 0;
	for(uint8_t i=0; i<44; i++) sum += TxDat[i];
	TxDat[44] = sum;
	return uart_Send_DMA((u8 *)TxDat, 45);
}
#endif

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

			case 0x04U:
				switch(RxDat[4])  //读取
				{
					case 0x01U://一键起飞
						xQueuePeek(queueRCCommand, &rcCommand, 0);

						rcCommand.OneKeyTakeoff = true;
						rcCommand.Frist_Entry_TakeOff = true;

						xQueueOverwrite(queueRCCommand,&rcCommand);

						uart_Send_Check();

						break;
					case 0x02U://一键降落
						xQueuePeek(queueRCCommand, &rcCommand, 0);

						rcCommand.OneKeyLanding = true;

						xQueueOverwrite(queueRCCommand,&rcCommand);

						uart_Send_Check();

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
			case 0x21U: //写LLH航点
				break;
			case 0x22U: //写NED航点

				xQueuePeek(queueTargetBuffer, &target_buffer, 0);

				target_buffer.index = *(tran.RxDat+4);//序号

				target_buffer.NED_or_LLH = 1;

				//写入北东地
				target_buffer.TargetPointList[target_buffer.index][0] = ((vs32) (*(tran.RxDat+5)<<24) | (*(tran.RxDat+6)<<16) | (*(tran.RxDat+7)<<8) | (*(tran.RxDat+8))) / 100.0f;//N
				target_buffer.TargetPointList[target_buffer.index][1] = ((vs32) (*(tran.RxDat+9)<<24) | (*(tran.RxDat+10)<<16) | (*(tran.RxDat+11)<<8) | (*(tran.RxDat+12))) / 100.0f;//E
				target_buffer.TargetPointList[target_buffer.index][2] = ((vs32) (*(tran.RxDat+13)<<24) | (*(tran.RxDat+14)<<16) | (*(tran.RxDat+15)<<8) | (*(tran.RxDat+16))) / 100.0f;

				target_buffer.hovertime[target_buffer.index] = ((u16) (*(tran.RxDat+17)<<8) | (*(tran.RxDat+18)));//悬停时间

				target_buffer.PTPSpeed[target_buffer.index] = ((u16) (*(tran.RxDat+19)<<8) | (*(tran.RxDat+20))) / 100.0f;//速度
				target_buffer.PTPHeading[target_buffer.index] = ((vs16) (*(tran.RxDat+21)<<8) | (*(tran.RxDat+22))) / 10.0f;//朝向


				if(target_buffer.Last_index != target_buffer.index)//每一帧发送一个航点，航点发完之后还会有两帧
				{
					target_buffer.receive_flag = true;
					target_buffer.count++;
				}
				else
				{
					target_buffer.receive_flag = false;
				}

				target_buffer.Last_index = target_buffer.index;

				xQueueOverwrite(queueTargetBuffer,&target_buffer);

				uart_Send_Check();
				break;
		}
	}while(0);
	LockRx = HAL_UNLOCKED;
}

void TRAN::uart_Set_Pid_Para(uint8_t group)
{
	pid.kp[group*3-3] = 0.001f * ( (vs16)(*(tran.RxDat+4 )<<8)|*(tran.RxDat+5 ) );
	pid.ki[group*3-3] = 0.001f * ( (vs16)(*(tran.RxDat+6 )<<8)|*(tran.RxDat+7 ) );
	pid.kd[group*3-3] = 0.001f * ( (vs16)(*(tran.RxDat+8 )<<8)|*(tran.RxDat+9 ) );
	pid.kp[group*3-2] = 0.001f * ( (vs16)(*(tran.RxDat+10 )<<8)|*(tran.RxDat+11 ) );
	pid.ki[group*3-2] = 0.001f * ( (vs16)(*(tran.RxDat+12 )<<8)|*(tran.RxDat+13 ) );
	pid.kd[group*3-2] = 0.001f * ( (vs16)(*(tran.RxDat+14 )<<8)|*(tran.RxDat+15 ) );
	pid.kp[group*3-1] = 0.001f * ( (vs16)(*(tran.RxDat+16 )<<8)|*(tran.RxDat+17 ) );
	pid.ki[group*3-1] = 0.001f * ( (vs16)(*(tran.RxDat+18 )<<8)|*(tran.RxDat+19 ) );
	pid.kd[group*3-1] = 0.001f * ( (vs16)(*(tran.RxDat+20 )<<8)|*(tran.RxDat+21 ) );

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
