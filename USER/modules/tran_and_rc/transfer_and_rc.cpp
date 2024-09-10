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

#include <tran_and_rc/rc_rtcm_process.hpp>
#include <tran_and_rc/ringbuffer.hpp>
#include <tran_and_rc/transfer_and_rc.hpp>
#include "path_follow/pathFollow.hpp"

#include "string.h"

uint8_t rtcm_cnt=0;
uint8_t last_rtcm_cnt=0;
uint8_t rtcm_data[1000];
uint8_t    RxRawDat[TRAN_RX_LEN];	//数据

#ifdef tran_U6
TRAN tran(USART6,(char *)"tran");
RC rc(USART6,(char *)"rc");

void USART6_IRQHandler(void)
{
	if(LL_USART_IsActiveFlag_IDLE(tran.huart))
	{
		LL_USART_ClearFlag_IDLE(tran.huart);
		LL_DMA_DisableStream(DMA2,LL_DMA_STREAM_1);
		LL_DMA_ClearFlag_DME1(DMA2);
		LL_DMA_ClearFlag_HT1(DMA2);
		LL_DMA_ClearFlag_TC1(DMA2);
		LL_DMA_ClearFlag_TE1(DMA2);
		LL_DMA_ClearFlag_FE1(DMA2);
		tran.RxDataSize = TRAN_RX_LEN - LL_DMA_GetDataLength(DMA2, LL_DMA_STREAM_1);

		do{//0xAA 0xAF [功能字] [字节数] [内容...] [校验和]
/*			if(tran.RxRawDat[0]!=0xAA || tran.RxRawDat[1]!=0xAF || tran.RxRawDat[3]!=tran.RxDataSize-5) break;
			if(tran.LockRx == HAL_LOCKED) break;
			tran.LockRx = HAL_LOCKED;
			memcpy(tran.RxDat, tran.RxRawDat, tran.RxDataSize);
			tran.RxDat[tran.RxDataSize] = 0;
			tran.LockRx = HAL_UNLOCKED;
			tran.RxFlag = true;   //收到完整一帧

			BaseType_t YieldRequired = xTaskResumeFromISR(tranReceiveTaskHandle);
			if(YieldRequired==pdTRUE)
			{
				如果函数xTaskResumeFromISR()返回值为pdTRUE，那么说明要恢复的这个
				任务的任务优先级等于或者高于正在运行的任务(被中断打断的任务),所以在
				退出中断的时候一定要进行上下文切换！
				portYIELD_FROM_ISR(YieldRequired);
			}
*/
		Write_to_ringbuffer((uint8_t *)tran.RxRawDat, tran.RxDataSize);
		}while(0);



		LL_DMA_SetDataLength(DMA2, LL_DMA_STREAM_1, TRAN_RX_LEN);
		LL_DMA_EnableStream(DMA2, LL_DMA_STREAM_1);
	}

}

void DMA2_Stream1_IRQHandler(void)  //接收DMA中断
{

}

void DMA2_Stream6_IRQHandler(void)  //发送DMA中断
{
	LL_DMA_DisableStream(DMA2,LL_DMA_STREAM_6);
	LL_DMA_ClearFlag_TC6(DMA2);
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
	LL_DMA_DisableStream(DMA2,LL_DMA_STREAM_1);
	LL_DMA_SetPeriphAddress(DMA2, LL_DMA_STREAM_1, (uint32_t)&huart->RDR);
	LL_DMA_SetMemoryAddress(DMA2, LL_DMA_STREAM_1, (uint32_t)RxRawDat);
	LL_DMA_SetDataLength(DMA2, LL_DMA_STREAM_1, TRAN_RX_LEN);
	LL_DMA_EnableStream(DMA2, LL_DMA_STREAM_1);
	/* 配置接收DMA */

	/* 配置发送DMA */
	LL_DMA_DisableStream(DMA2,LL_DMA_STREAM_6);
	LL_DMA_SetPeriphAddress(DMA2, LL_DMA_STREAM_6, (uint32_t)&huart->TDR);
	LL_DMA_SetMemoryAddress(DMA2, LL_DMA_STREAM_6, (uint32_t)TxDat);
	LL_DMA_SetDataLength(DMA2, LL_DMA_STREAM_6, TRAN_TX_LEN);
	LL_DMA_ClearFlag_TC6(DMA2);
	LL_DMA_EnableIT_TC(DMA2, LL_DMA_STREAM_6);
	LL_DMA_EnableStream(DMA2, LL_DMA_STREAM_6);
	/* 配置发送DMA */



	LL_USART_EnableDMAReq_RX(huart);
	LL_USART_EnableDMAReq_TX(huart);
	LL_USART_ClearFlag_IDLE(huart);
	LL_USART_EnableIT_IDLE(huart);
}
bool TRAN::uart_Send_DMA(uint8_t * pData,uint16_t Size)
{
	if(TxFlag == true) return false;	//串口发送忙,放弃发送该帧数据
	LL_DMA_DisableStream(DMA2,LL_DMA_STREAM_6);

	LL_DMA_SetDataLength(DMA2, LL_DMA_STREAM_6, Size);
	LL_DMA_SetMemoryAddress(DMA2, LL_DMA_STREAM_6, (uint32_t)pData);
	LL_DMA_EnableStream(DMA2, LL_DMA_STREAM_6);
	TxFlag = true;
	return true;
}
#endif


#ifdef tran_U3
TRAN tran(USART3,(char *)"tran");
RC rc(USART3,(char *)"rc");

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
/*			if(tran.RxRawDat[0]!=0xAA || tran.RxRawDat[1]!=0xAF || tran.RxRawDat[3]!=tran.RxDataSize-5) break;
			if(tran.LockRx == HAL_LOCKED) break;
			tran.LockRx = HAL_LOCKED;
			memcpy(tran.RxDat, tran.RxRawDat, tran.RxDataSize);
			tran.RxDat[tran.RxDataSize] = 0;
			tran.LockRx = HAL_UNLOCKED;
			tran.RxFlag = true;   //收到完整一帧

			BaseType_t YieldRequired = xTaskResumeFromISR(tranReceiveTaskHandle);
			if(YieldRequired==pdTRUE)
			{
				如果函数xTaskResumeFromISR()返回值为pdTRUE，那么说明要恢复的这个
				任务的任务优先级等于或者高于正在运行的任务(被中断打断的任务),所以在
				退出中断的时候一定要进行上下文切换！
				portYIELD_FROM_ISR(YieldRequired);
			}
*/
		Write_to_ringbuffer((uint8_t *)tran.RxRawDat, tran.RxDataSize);
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


	LL_USART_EnableDMAReq_RX(USART3);
	LL_USART_EnableDMAReq_TX(USART3);
	LL_USART_ClearFlag_IDLE(USART3);
	LL_USART_EnableIT_IDLE(USART3);
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
#endif


void RC::rc_Init(void)
{
	LockRx = HAL_UNLOCKED;
	RxFlag = false;
	RxDataSize = 0;
	executionTime_us = 0;
	Sta = STA_INI;
	Err = ERR_NONE;

	HIG_THR = 1912;    //油门高位点
	MID_THR = 1420;    //油门中位点
	LOW_THR = 1088;    //油门低位点

	Key[0] = 0;     //4个开关
	Key[1] = 0;     //4个开关
	Key[2] = 0;     //4个开关
	Key[3] = 0;     //4个开关


	ModeChk.CNT = 0;
	ModeChk.CCR = 6;

	dAng = 0.0f;
	Thr = 0.0f;
	for(uint8_t i=0;i<4;i++)
		Val[i] = 0.0f;
	for(uint8_t i=0;i<2;i++)
		Ang[i] = 0.0f;

	float fa,fb,fc;
	fa = LOW_THR/1000.0f;
	fb = MID_THR/1000.0f;
	fc = HIG_THR/1000.0f;
	a = -(4*(5*fc - 9*fa + 5*fa*fb - 5*fb*fc + 6))/(5*(2*fa - 3)*(3*fa - 3*fc - 2*fa*fc + 2*fc*fc));
	b = (4*(5*fa*fa*fb - 5*fb*fc*fc - 9*fa*fa + 5*fc*fc + 9))/(5*(2*fa - 3)*(3*fa - 3*fc - 2*fa*fc + 2*fc*fc));
	c = -(20*fb*fa*fa*fc - 54*fa*fa - 20*fb*fa*fc*fc + 81*fa + 30*fc*fc - 45*fc)/(5*(2*fa - 3)*(3*fa - 3*fc - 2*fa*fc + 2*fc*fc));

	Sta = STA_RUN;
	osDelay(100);

}

void RC::rc_Update(void)
{

	startTimerLast = startTimer;
	getTimer_us(&startTimer);
	cycleTime_us = startTimer - startTimerLast;
	xQueuePeek(queueRCCommand, &rcCommand, 0);

//	if(RxFlag == false) return;	//未更新
//
//	RxFlag = false;

	if(LockRx == HAL_LOCKED)return;
	LockRx = HAL_LOCKED;
	do{
		if(RxDat[0]=='$' && RxDat[PPM_NUM-2]=='\r' && RxDat[PPM_NUM-1]=='\n')
		{
			char CHN[4]="000";
			char *pStr;
			u16 tmp[PWM_RC_NUM];
			for(u8 i=0;i<PWM_RC_NUM;i++)
			{
				strncpy(CHN,RxDat+i*3+1,3);
				tmp[i]=strtol(CHN,&pStr,16) + 420;   //十六进制转换 +420偏置
			}
			//遥控器行为定义 油门2
			static u8 preFlag = 0xFF;
			u8 Chn_Flag_L = 0x00,Chn_Flag_H = 0x00;
			if(tmp[0] < RC_FUN_MIN)Chn_Flag_L|=0x01;
			else if(tmp[0] > RC_FUN_MAX)Chn_Flag_H|=0x01;
			if(tmp[1] < RC_FUN_MIN)Chn_Flag_L|=0x02;
			else if(tmp[1] > RC_FUN_MAX)Chn_Flag_H|=0x02;
			if(tmp[2] < RC_FUN_MIN)Chn_Flag_L|=0x04;
			else if(tmp[2] > RC_FUN_MAX)Chn_Flag_H|=0x04;
			if(tmp[3] < RC_FUN_MIN)Chn_Flag_L|=0x08;
			else if(tmp[3] > RC_FUN_MAX)Chn_Flag_H|=0x08;
			if((Chn_Flag_L|Chn_Flag_H)==0x0F)  //行为持续一定时间视为有效
			{
				if(preFlag == Chn_Flag_H)
				{
					if(++ModeChk.CNT>ModeChk.CCR)
						Mode = (eRC_MODE)Chn_Flag_H;
				}
				else
				{
					preFlag = Chn_Flag_H;
					ModeChk.CNT = 0;
				}
			}
			else preFlag = 0xFF;

			for(u8 i=0;i<PWM_RC_NUM;i++)
			{
				PPM[i] = tmp[i];
				rc_ppm.PPM[i] = PPM[i];
			}
			rcCommand.Val[0] = RC_PPM_MID-PPM[3];
			rcCommand.Val[1] = RC_PPM_MID-PPM[1];
			rcCommand.Val[2] = -(RC_PPM_MID-PPM[0]);
			rcCommand.Val[3] = PPM[2];
			//角度设定
			rcCommand.Ang[0] = rcCommand.Val[0]*0.1f;   //-50°到50°
			rcCommand.Ang[1] = rcCommand.Val[1]*0.1f;   //-50°到50°
			rcCommand.dAng   = rcCommand.Val[2]*0.5f;   //-250°/s到250°/s
			rcCommand.Thr    = a*SQR(PPM[2])/1000.0f + b*PPM[2] + c*1000.0f;//油门换算
			//位置设定
			rcCommand.Uvw[0] = rcCommand.Val[0]*0.0008f; //40m/s
			rcCommand.Uvw[1] = rcCommand.Val[1]*0.0008f; //40m/s
			rcCommand.Uvw[2] = rcCommand.Val[3]*0.0008f; //40m/s

			rcCommand.Key[0] = PPM[4]<RC_FUN_MIN?0:(PPM[4]>RC_FUN_MAX?2:1);
			rcCommand.Key[1] = PPM[5]<RC_FUN_MIN?0:(PPM[5]>RC_FUN_MAX?2:1);
			rcCommand.Key[2] = PPM[6]<RC_FUN_MIN?0:(PPM[6]>RC_FUN_MAX?2:1);
			rcCommand.Key[3] = PPM[7]<RC_FUN_MIN?0:(PPM[7]>RC_FUN_MAX?2:1);

			rcCommand.Mode = Mode;
			Update = true;

			xQueueOverwrite(queueRCData,&rc_ppm);
			xQueueOverwrite(queueRCCommand,&rcCommand);
		}
	}while(0);
	LockRx = HAL_UNLOCKED;
	getTimer_us(&stopTimer);

	executionTime_us = stopTimer - startTimer;
}


void RC::ControlRC_Check(void)
{
	static u16 new_index=0,old_index=1;
	rc_buffer[new_index] = RxFlag;
	rc_check = rc_check + rc_buffer[new_index] - rc_buffer[old_index];
	new_index = (new_index + 1)%RC_CHECK_NUM;
	old_index = (old_index + 1)%RC_CHECK_NUM;
	if(rc_check != 0)
	{
		rc_status=NORMAL;
		ever_rc = 1;
	}
	else if(ever_rc == 0 || rcCommand.Val[3] <= 1200)
	{
		rc_status=CLOSE;
	}
	else
	{
		rc_status=LOST;
	}

	rc_status_msg.rc_status = rc_status;
	rc_status_msg.ever_rc= ever_rc;
	RxFlag = false;
	xQueueOverwrite(queueRC_Status,&rc_status_msg);
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
//		if(send_log == true)
//		{
//			if(uart_Send_Base_Station_Data(space,buffer,read_index))
//				send_log = false;
//		}
//		uart_Send_MotorPWM();
		break;
	case 9:
//		uart_Send_rcData();
		break;
	case 11:
//		uart_Send_Power();
		break;
	case 13:
//		if(send_version == true)
//		{
//			uart_Send_Version();
//			send_version = false;
//		}
		break;
	case 15:
		uart_Send_GPS();
		break;
	case 17:
		uart_Send_NED();
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
		break;
	case 2:
		break;
	case 4:
		uart_Send_Paths();
		break;
	case 6:
	case 8:
		uart_Send_User();
		break;
	case 10:
	case 12:
	case 14:
	case 16:
	case 18:
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
//	xQueuePeek(queueOrdinaryGps,&ordinaryGps,0);

	if(TxFlag == true) return false;

	vs32 _temp;
	vs16 _temp2;
	TxDat[2] = 0x07;
	TxDat[3] = 6;
	_temp = gps.gpsPosAccuracy*100;		// 曲线 ALT_BAR
	TxDat[4] = BYTE3(_temp);
	TxDat[5] = BYTE2(_temp);
	TxDat[6] = BYTE1(_temp);
	TxDat[7] = BYTE0(_temp);
	_temp2 = gps.gpsPosAccuracy*100;		// 曲线 ALT_CSB
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
	_temp = (s16)(ahrsEuler.Ang[0]*R2D*100.0f);
//	_temp = (s16)(eskf.Attitude[0]*R2D*100.0f);
	TxDat[4] = BYTE1(_temp);
	TxDat[5] = BYTE0(_temp);
	_temp = (s16)(ahrsEuler.Ang[1]*R2D*100.0f);
//	_temp = (s16)(eskf.Attitude[1]*R2D*100.0f);
	TxDat[6] = BYTE1(_temp);
	TxDat[7] = BYTE0(_temp);
	_temp = (s16)(ahrsEuler.Ang[2]*R2D*100.0f);
//	_temp = (s16)(eskf.Attitude[2]*R2D*100.0f);
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
	TxDat[4] = BYTE1(motor_msg.PWM_OBS[1]);
	TxDat[5] = BYTE0(motor_msg.PWM_OBS[1]);
	TxDat[6] = BYTE1(motor_msg.PWM_OBS[2]);
	TxDat[7] = BYTE0(motor_msg.PWM_OBS[2]);
	TxDat[8] = BYTE1(motor_msg.PWM_OBS[3]);
	TxDat[9] = BYTE0(motor_msg.PWM_OBS[3]);
	TxDat[10]= BYTE1(motor_msg.PWM_OBS[4]);
	TxDat[11]= BYTE0(motor_msg.PWM_OBS[4]);

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
	TxDat[3]=16;

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
	_temp = gps.alti ;
	TxDat[14]=BYTE3(_temp);
	TxDat[15]=BYTE2(_temp);
	TxDat[16]=BYTE1(_temp);
	TxDat[17]=BYTE0(_temp);
	_temp2 = gps.update*100;
	TxDat[18]=BYTE1(_temp2);
	TxDat[19]=BYTE0(_temp2);

	u8 sum = 0;
	for(u8 i=0;i<20;i++)sum += TxDat[i];
	TxDat[20]=sum;
	return uart_Send_DMA((u8 *)TxDat, 21);
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
bool TRAN::uart_Send_NED(void)
{
	if(tran.TxFlag == true) return false;
	xQueuePeek(queueControlTransfer,&control_data,0);
	vs32 temp;
	vs16 temp1;

	uint8_t len=30;
//	struct s_cur_NED_position *NED_position;
//	uint8_t len = sizeof(struct s_cur_NED_position);

	TxDat[2] = 0x08;
	TxDat[3] = len;

	temp = vs32(control_data.X[0]*100);
	tran.TxDat[4] = BYTE3(temp);
	tran.TxDat[5] = BYTE2(temp);
	tran.TxDat[6] = BYTE1(temp);
	tran.TxDat[7] = BYTE0(temp);

	temp = vs32(control_data.Y[0]*100);
	tran.TxDat[8] = BYTE3(temp);
	tran.TxDat[9] = BYTE2(temp);
	tran.TxDat[10] = BYTE1(temp);
	tran.TxDat[11] = BYTE0(temp);

	temp = vs32(control_data.Z[0]*100);
	tran.TxDat[12] = BYTE3(temp);
	tran.TxDat[13] = BYTE2(temp);
	tran.TxDat[14] = BYTE1(temp);
	tran.TxDat[15] = BYTE0(temp);

	temp = vs32(control_data.X_command[0]*100);
	tran.TxDat[16] = BYTE3(temp);
	tran.TxDat[17] = BYTE2(temp);
	tran.TxDat[18] = BYTE1(temp);
	tran.TxDat[19] = BYTE0(temp);
	temp = vs32(control_data.Y_command[0]*100);
	tran.TxDat[20] = BYTE3(temp);
	tran.TxDat[21] = BYTE2(temp);
	tran.TxDat[22] = BYTE1(temp);
	tran.TxDat[23] = BYTE0(temp);
	temp = vs32(control_data.Z_command[0]*100);
	tran.TxDat[24] = BYTE3(temp);
	tran.TxDat[25] = BYTE2(temp);
	tran.TxDat[26] = BYTE1(temp);
	tran.TxDat[27] = BYTE0(temp);

	temp1 = 0;
	tran.TxDat[28] = BYTE1(temp1);
	tran.TxDat[29] = BYTE0(temp1);
	temp1 = 0;
	tran.TxDat[30] = BYTE1(temp1);
	tran.TxDat[31] = BYTE0(temp1);
	temp1 = 0;
	tran.TxDat[32] = BYTE1(temp1);
	tran.TxDat[33] = BYTE0(temp1);
//	NED_position = (struct s_cur_NED_position *)(TxDat+4);
//
//	NED_position->x = control_data.X[0];
//	NED_position->y = control_data.Y[0];
//	NED_position->z = control_data.Z[0];
//	NED_position->x_command = control_data.X_command[0];
//	NED_position->y_command = control_data.Y_command[0];
//	NED_position->z_command = control_data.Z_command[0];
//	NED_position->u = 0;
//	NED_position->v = 0;
//	NED_position->w = 0;

	uint8_t sum = 0;
	for(uint8_t i = 0; i < len+4; i ++) sum += TxDat[i];
	TxDat[len+4] = sum;
	return uart_Send_DMA((u8 *)TxDat, len+5);
}

//bool TRAN::uart_Send_Base_Station_Data(uint16_t len,uint8_t *buffer,uint16_t index)
//{
//	if(tran.TxFlag == true) return false;
//	tran.TxDat[2] = 0xED;
//	tran.TxDat[3] = (u8)(len+1);
//	tran.TxDat[4] = 0x00;
//
//	for(int j=0;j<len;j++)
//	{
//		tran.TxDat[j+5] = buffer[index+j];
//	}
//
//	uint8_t sum = 0;
//	for(uint8_t i=0; i<(len+5); i++) sum += TxDat[i];
//	TxDat[len+5] = sum;
//	return uart_Send_DMA((u8 *)TxDat, len+6);
//}
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
#define __trajectory3  //__sensorcali传感器校正,__outloop外环,__trajectory航点,__trajectory2航点模式调试

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
	xQueuePeek(queueHeight,&height,0);
	xQueuePeek(queueAccDat, &acc, 0);
	xQueuePeek(queueAccDatFil, &acc_fil, 0);
	xQueuePeek(queueESKF,&eskf,0);
	xQueuePeek(queueESKF_baro,&eskf_baro,0);
	xQueuePeek(queueBaroAlt,&baroAlt,0);
	xQueuePeek(queuetrajectoryData, &trajectoryData, 0);
	xQueuePeek(queuelaserFlow, &laserFlow, 0);



/*eskf XYZ Z_speed;eskf_baro XYZ Z_speed;baro altitude altslope */
	vs16 temp;
	tran.TxDat[2] = 0xF1;
	tran.TxDat[3] = 52;


	temp = control_data.X_command[1]*100;						//user_data1  XYZ body速度指令
	tran.TxDat[4] = BYTE1(temp);
	tran.TxDat[5] = BYTE0(temp);

	temp = control_data.Y_command[1]*100;						//user_data2
	tran.TxDat[6] = BYTE1(temp);
	tran.TxDat[7] = BYTE0(temp);

	temp = control_data.Z_command[1]*100;						//user_data3
	tran.TxDat[8] = BYTE1(temp);
	tran.TxDat[9] = BYTE0(temp);

	temp = control_data.XH[1]*100;						//user_data4	XYZ body速度反馈
	tran.TxDat[10] = BYTE1(temp);
	tran.TxDat[11] = BYTE0(temp);

	temp = control_data.YH[1]*100;						//user_data5
	tran.TxDat[12] = BYTE1(temp);
	tran.TxDat[13] = BYTE0(temp);

//	temp = control_data.Z[1]*100;						//user_data6
	temp = eskf.Ned_spd[2]*100;
	tran.TxDat[14] = BYTE1(temp);
	tran.TxDat[15] = BYTE0(temp);

	temp = control_data.Z_command[0]*100;						//user_data7	Z位置指令和反馈
	tran.TxDat[16] = BYTE1(temp);
	tran.TxDat[17] = BYTE0(temp);

//	temp = control_data.Z[0]*100;						//user_data8
	temp = eskf.Pos[2]*100;
	tran.TxDat[18] = BYTE1(temp);
	tran.TxDat[19] = BYTE0(temp);

//	temp = control_data.X[0]*100;						//user_data9	XY位置反馈
	temp = eskf.Pos[0]*100;
	tran.TxDat[20] = BYTE1(temp);
	tran.TxDat[21] = BYTE0(temp);

//	temp = control_data.Y[0]*100;						//user_data10
	temp = eskf.Pos[1]*100;
	tran.TxDat[22] = BYTE1(temp);
	tran.TxDat[23] = BYTE0(temp);

	temp = control_data.Ang[0]*R2D*100;						//user_data11	Euler角度反馈
//	temp = eskf_baro.Pos[0]*100;
	tran.TxDat[24] = BYTE1(temp);
	tran.TxDat[25] = BYTE0(temp);

	temp = control_data.Ang[1]*R2D*100;						//user_data12
//	temp = eskf_baro.Pos[1]*100;
	tran.TxDat[26] = BYTE1(temp);
	tran.TxDat[27] = BYTE0(temp);

	temp = control_data.Ang[2]*R2D*100;						//user_data13
//	temp = eskf_baro.Pos[2]*100;
	tran.TxDat[28] = BYTE1(temp);
	tran.TxDat[29] = BYTE0(temp);

	temp = control_data.roll_command[0]*R2D*100;			//user_data14	Euler角度指令
//	temp = control_data.brake_finish*100;
//	temp = trajectoryData.Dubins_Status*100;
	tran.TxDat[30] = BYTE1(temp);
	tran.TxDat[31] = BYTE0(temp);

	temp = control_data.pitch_command[0]*R2D*100;			//user_data15
//	temp = trajectoryData.distance_XY*100;
//	temp = acc.acc[1]*100;
	tran.TxDat[32] = BYTE1(temp);
	tran.TxDat[33] = BYTE0(temp);

	temp = control_data.yaw_command[0]*R2D*100;				//user_data16
//	temp = acc.acc[2]*100;
//	temp = control_data.XYZ_phase*100;
	tran.TxDat[34] = BYTE1(temp);
	tran.TxDat[35] = BYTE0(temp);

	temp = control_data.mt_output[0];						//user_data17	电机输出
//	temp = acc_fil.acc_filter[1]*100;
	tran.TxDat[36] = BYTE1(temp);
	tran.TxDat[37] = BYTE0(temp);

	temp = control_data.FlightStatus*100;					//user_data18
//	temp = acc_fil.acc_filter[2]*100;
	tran.TxDat[38] = BYTE1(temp);
	tran.TxDat[39] = BYTE0(temp);

	temp = trajectoryData.YawAngle_traveled*R2D;					//user_data19
	tran.TxDat[40] = BYTE1(temp);
	tran.TxDat[41] = BYTE0(temp);

//	temp = control_data.output[0]*R2D*100;						//user_data20
	temp = (acc_fil.acc_filter[2]+OneG)*100;
	tran.TxDat[42] = BYTE1(temp);
	tran.TxDat[43] = BYTE0(temp);

	temp = trajectoryData.PTP_Status*100;					//user_data21
	tran.TxDat[44] = BYTE1(temp);
	tran.TxDat[45] = BYTE0(temp);

	temp = trajectoryData.Circle_Status*100;					//user_data22
	tran.TxDat[46] = BYTE1(temp);
	tran.TxDat[47] = BYTE0(temp);

//	temp = trajectoryData.azimuth*R2D*100;					//user_data23
	temp = laserFlow.heightFil*100;
	tran.TxDat[48] = BYTE1(temp);
	tran.TxDat[49] = BYTE0(temp);

//	temp = control_data.XY_phase*100;					//user_data24
//	temp = control_data.Z_phase*100;
//	temp = control_data.Pos_track[0]*100;
//	temp = gps.NED[2]*100;
//	temp = eskf_baro.Ned_spd[2]*100;
	temp = laserFlow.VelxFil*100;
	tran.TxDat[50] = BYTE1(temp);
	tran.TxDat[51] = BYTE0(temp);

//	temp = 	control_data.X_command_ref[0]*100;					//user_data25
	temp = 	laserFlow.height*100;
	tran.TxDat[52] = BYTE1(temp);
	tran.TxDat[53] = BYTE0(temp);

//	temp = 	control_data.X_command_ref[1]*100;					//user_data26
	temp = 	laserFlow.Velx*100;
	tran.TxDat[54] = BYTE1(temp);
	tran.TxDat[55] = BYTE0(temp);
	uint8_t sum = 0;
	for(uint8_t i=0; i<56; i++) sum += TxDat[i];
	TxDat[56] = sum;
	return uart_Send_DMA((u8 *)TxDat, 57);
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
	xQueuePeek(queuelaserFlow,&laserFlow,0);

	vs16 temp;
	tran.TxDat[2] = 0xF1;
	tran.TxDat[3] = 42;

	temp = control_data.X_command[0]*100;					//user_data1	X位置期望
	tran.TxDat[4] = BYTE1(temp);
	tran.TxDat[5] = BYTE0(temp);

	temp = control_data.Y_command[0]*100;					//user_data2	Y位置期望
	tran.TxDat[6] = BYTE1(temp);
	tran.TxDat[7] = BYTE0(temp);

	temp = control_data.Z_command[0]*100;					//user_data3	Z位置期望
	tran.TxDat[8] = BYTE1(temp);
	tran.TxDat[9] = BYTE0(temp);

	temp = eskf.Pos[0]*100;							//user_data4	X位置反馈值
	tran.TxDat[10] = BYTE1(temp);
	tran.TxDat[11] = BYTE0(temp);

	temp = eskf.Pos[1]*100;							//user_data5	Y位置反馈值
	tran.TxDat[12] = BYTE1(temp);
	tran.TxDat[13] = BYTE0(temp);

	temp = eskf.Pos[2]*100;							//user_data6	Z位置反馈值
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

	temp = control_data.FlightStatus;
	tran.TxDat[32] = BYTE1(temp);
	tran.TxDat[33] = BYTE0(temp);

	temp = control_data.Pos_estimate[0];
	tran.TxDat[34] = BYTE1(temp);
	tran.TxDat[35] = BYTE0(temp);

	temp = control_data.Pos_estimate[1];
	tran.TxDat[36] = BYTE1(temp);
	tran.TxDat[37] = BYTE0(temp);

	temp = control_data.enable_Grab_flag;
	tran.TxDat[38] = BYTE1(temp);
	tran.TxDat[39] = BYTE0(temp);

	temp = laserFlow.VelxFil;
	tran.TxDat[40] = BYTE1(temp);
	tran.TxDat[41] = BYTE0(temp);

	temp = laserFlow.VelxFil;
	tran.TxDat[42] = BYTE1(temp);
	tran.TxDat[43] = BYTE0(temp);

	temp = laserFlow.heightFil;
	tran.TxDat[44] = BYTE1(temp);
	tran.TxDat[45] = BYTE0(temp);
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

//	temp = target_buffer.next_target_ned[0]*100;				//user_data15	下一个目标航点X位置
//	tran.TxDat[32] = BYTE1(temp);
//	tran.TxDat[33] = BYTE0(temp);

//	temp = target_buffer.next_target_ned[1]*100;				//user_data16	下一个目标航点Y
//	tran.TxDat[34] = BYTE1(temp);
//	tran.TxDat[35] = BYTE0(temp);

//	temp = target_buffer.next_target_ned[2]*100;				//user_data17	下一个目标航点Z
//	tran.TxDat[36] = BYTE1(temp);
//	tran.TxDat[37] = BYTE0(temp);

//	temp = target_buffer.hover_time_Trajectory*100;					//user_data18	悬停时间
//	tran.TxDat[38] = BYTE1(temp);
//	tran.TxDat[39] = BYTE0(temp);

//	temp = gps.gpsPosAccuracy*100;							//user_data19	RTK精度
//	tran.TxDat[40] = BYTE1(temp);
//	tran.TxDat[41] = BYTE0(temp);

//	temp = eskf.Pos[2]*100;							//user_data20	RTK高度
//	tran.TxDat[42] = BYTE1(temp);
//	tran.TxDat[43] = BYTE0(temp);

//	temp = acc_filter.acc_filter[2]*100;
//	tran.TxDat[42] = BYTE1(temp);
//	tran.TxDat[43] = BYTE0(temp);

//	temp = control_data.trajectory_status;					//user_data19	轨迹模式状态
//	tran.TxDat[42] = BYTE1(temp);
//	tran.TxDat[43] = BYTE0(temp);


	uint8_t sum = 0;
	for(uint8_t i=0; i<46; i++) sum += TxDat[i];
	TxDat[46] = sum;
	return uart_Send_DMA((u8 *)TxDat, 47);
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
						rcCommand.TakeOffFinish = false;

						xQueueOverwrite(queueRCCommand,&rcCommand);

						uart_Send_Check();

						break;
					case 0x02U://一键降落
						xQueuePeek(queueRCCommand, &rcCommand, 0);

						rcCommand.OneKeyLanding = true;

						if(rcCommand.IsAir == 1){
							rcCommand.ReLanding = true;
						}

						xQueueOverwrite(queueRCCommand,&rcCommand);

						uart_Send_Check();

						break;
					case 0x03U://删除航点
						delete_fly_point();
					case 0x07U://清空飞控航点
						xQueuePeek(queueRCCommand, &rcCommand, 0);

						rcCommand.Clr_flypoint = true;

						xQueueOverwrite(queueRCCommand,&rcCommand);
						uart_Send_Check();
					case 0x21U://读取航点数量
						break;
					case 0xA0U://读取版本
						send_version = true;
						uart_Send_Check();
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
				write_fly_point();
//				xQueuePeek(queueTargetBuffer, &target_buffer, 0);
//
//				target_buffer.index = *(tran.RxDat+4);//序号
//
//				target_buffer.NED_or_LLH = 1;
//
//				//写入北东地
//				target_buffer.TargetPointList[target_buffer.index][0] = ((vs32) (*(tran.RxDat+5)<<24) | (*(tran.RxDat+6)<<16) | (*(tran.RxDat+7)<<8) | (*(tran.RxDat+8))) / 100.0f;//N
//				target_buffer.TargetPointList[target_buffer.index][1] = ((vs32) (*(tran.RxDat+9)<<24) | (*(tran.RxDat+10)<<16) | (*(tran.RxDat+11)<<8) | (*(tran.RxDat+12))) / 100.0f;//E
//				target_buffer.TargetPointList[target_buffer.index][2] = ((vs32) (*(tran.RxDat+13)<<24) | (*(tran.RxDat+14)<<16) | (*(tran.RxDat+15)<<8) | (*(tran.RxDat+16))) / 100.0f;
//
//				target_buffer.hovertime[target_buffer.index] = ((u16) (*(tran.RxDat+17)<<8) | (*(tran.RxDat+18)));//悬停时间
//
//				target_buffer.PTPSpeed[target_buffer.index] = ((u16) (*(tran.RxDat+19)<<8) | (*(tran.RxDat+20))) / 100.0f;//速度
//				target_buffer.PTPHeading[target_buffer.index] = ((vs16) (*(tran.RxDat+21)<<8) | (*(tran.RxDat+22))) / 10.0f;//朝向
//
//
//				if(target_buffer.Last_index != target_buffer.index)//每一帧发送一个航点，航点发完之后还会有两帧
//				{
//					target_buffer.receive_flag = true;
//					target_buffer.count++;
//				}
//				else
//				{
//					target_buffer.receive_flag = false;
//				}
//
//				target_buffer.Last_index = target_buffer.index;
//
//				xQueueOverwrite(queueTargetBuffer,&target_buffer);
//
//				uart_Send_Check();
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
bool TRAN::plot_path = false;
void TRAN::set_plot_path() {plot_path = true;}

bool TRAN::uart_Send_Paths()
{
	static std::list<PATHFOLLOW::Path> paths;
	static std::list<PATHFOLLOW::Path>::iterator it = paths.begin();
	static uint8_t seq = 0;
	if(plot_path){
		PATHFOLLOW::get_paths(paths);
		it = paths.begin();
		plot_path = false;
		seq = 0;
		path_size = paths.size();
	}
	if(it == paths.end() || paths.empty()) return true;
	int32_t temp;
	tran.TxDat[2] = 0x0B;
	tran.TxDat[3] = 22;

	tran.TxDat[4] = seq;

	int8_t lam = it->lambda; // direction of turning (+1 for CW, -1 for CCW, 0 for straight line)
	if(lam == 0)
	{
		temp = it->r[0]*1000; //起点X
		tran.TxDat[5] = BYTE3(temp);
		tran.TxDat[6] = BYTE2(temp);
		tran.TxDat[7] = BYTE1(temp);
		tran.TxDat[8] = BYTE0(temp);

		temp = it->r[1]*1000; //起点Y
		tran.TxDat[9] = BYTE3(temp);
		tran.TxDat[10] = BYTE2(temp);
		tran.TxDat[11] = BYTE1(temp);
		tran.TxDat[12] = BYTE0(temp);

		temp = it->q[0]*1000; //方向X
		tran.TxDat[13] = BYTE3(temp);
		tran.TxDat[14] = BYTE2(temp);
		tran.TxDat[15] = BYTE1(temp);
		tran.TxDat[16] = BYTE0(temp);

		temp = it->q[1]*1000;
		tran.TxDat[17] = BYTE3(temp);
		tran.TxDat[18] = BYTE2(temp);
		tran.TxDat[19] = BYTE1(temp);
		tran.TxDat[20] = BYTE0(temp);

		temp = it->len*1000;//长度
		tran.TxDat[21] = BYTE3(temp);
		tran.TxDat[22] = BYTE2(temp);
		tran.TxDat[23] = BYTE1(temp);
		tran.TxDat[24] = BYTE0(temp);

		tran.TxDat[25] = lam;
	}
	else
	{
		temp = it->c[0]*1000;
		tran.TxDat[5] = BYTE3(temp);
		tran.TxDat[6] = BYTE2(temp);
		tran.TxDat[7] = BYTE1(temp);
		tran.TxDat[8] = BYTE0(temp);

		temp = it->c[1]*1000;
		tran.TxDat[9] = BYTE3(temp);
		tran.TxDat[10] = BYTE2(temp);
		tran.TxDat[11] = BYTE1(temp);
		tran.TxDat[12] = BYTE0(temp);

		temp = (it->w[0] - it->c[0])*1000;
		tran.TxDat[13] = BYTE3(temp);
		tran.TxDat[14] = BYTE2(temp);
		tran.TxDat[15] = BYTE1(temp);
		tran.TxDat[16] = BYTE0(temp);

		temp = (it->w[1] - it->c[1])*1000;
		tran.TxDat[17] = BYTE3(temp);
		tran.TxDat[18] = BYTE2(temp);
		tran.TxDat[19] = BYTE1(temp);
		tran.TxDat[20] = BYTE0(temp);

		temp = float(it->len*R2D*1000/it->rho);
		tran.TxDat[21] = BYTE3(temp);
		tran.TxDat[22] = BYTE2(temp);
		tran.TxDat[23] = BYTE1(temp);
		tran.TxDat[24] = BYTE0(temp);

		lam *= -1;
		tran.TxDat[25] = lam;
	}
	++seq;
	++it;
	uint8_t sum = 0;
	for(uint8_t i=0; i<26; ++i) sum += TxDat[i];
	TxDat[26] = sum;
	return uart_Send_DMA((u8 *)TxDat, 27);
}

void TRAN::delete_fly_point(void)
{
	uint8_t *pdata = RxDat+5;
	fly_point cmd_point = {0};

	cmd_point.num = pdata[0]+1;
	cmd_point.enable = 0;
	xQueueSend(queueFlyPoint, &cmd_point, portMAX_DELAY);//若队列满，将一直阻塞
	uart_Send_Check();
}

void TRAN::write_fly_point(void)
{
	uint8_t *pdata = RxDat+4;
	fly_point cmd_point;

	cmd_point.num = pdata[0];
	cmd_point.enable = 1;
	cmd_point.NED_position_m.data.x = 0.01f * ((int32_t)(pdata[1]<<24)|(pdata[2]<<16)|(pdata[3]<<8)|pdata[4]);
	cmd_point.NED_position_m.data.y = 0.01f * ((int32_t)(pdata[5]<<24)|(pdata[6]<<16)|(pdata[7]<<8)|pdata[8]);
	cmd_point.NED_position_m.data.z = 0.01f * ((int32_t)(pdata[9]<<24)|(pdata[10]<<16)|(pdata[11]<<8)|pdata[12]);
	cmd_point.stay_time_s = (uint16_t)(pdata[13]<<8)|pdata[14];
	cmd_point.vel_mps = ((uint16_t)(pdata[15]<<8)|pdata[16]) * 0.01f;
	cmd_point.yaw_degree = ((int16_t)(pdata[17]<<8)|pdata[18]) * 0.1f;

	xQueueSend(queueFlyPoint, &cmd_point, portMAX_DELAY);
	uart_Send_Check();
}
extern "C" void tran_main(void *argument)
{
	tran.tran_Init();
	osDelay(10);
	for(;;)
	{
		osSemaphoreAcquire(semTran,0xffffffff);
		tran.uart_Send_DMA_loop(); //循环发送数据到地面站
	}
}

extern "C" void tranReceive_main(void *argument)
{

	osDelay(500);//等待系统完成初始化
	for(;;)
	{
		vTaskSuspend(tranReceiveTaskHandle);
		tran.uart_Receive_Update(); //从地面站接受数据
	}
}

extern "C" void RC_Check_main(void *argument)
{
	rc.rc_Init();
	osDelay(14);
	for(;;)
	{
		osSemaphoreAcquire(semRcCheck,0xffffffff);
		Rc_And_Rtcm_Task();
		rc.rc_Update();
		rc.ControlRC_Check();		//
	}
}

/************************ (C) COPYRIGHT Longmen Drone Team *****END OF FILE****/
