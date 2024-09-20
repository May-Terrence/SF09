#include "USART_SD_Log.hpp"

openlog_classdef<16> openlog(uart_Send_DMA);

static bool TxFlag=false;
uint8_t    TxDat[60];

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
//		tran.RxDataSize = TRAN_RX_LEN - LL_DMA_GetDataLength(DMA2, LL_DMA_STREAM_2);
//		do{//0xAA 0xAF [功能字] [字节数] [内容...] [校验和]
//			if(tran.RxRawDat[0]!=0xAA || tran.RxRawDat[1]!=0xAF || tran.RxRawDat[3]!=tran.RxDataSize-5) break;
//			if(tran.LockRx == HAL_LOCKED) break;
//			tran.LockRx = HAL_LOCKED;
//			memcpy(tran.RxDat, tran.RxRawDat, tran.RxDataSize);
//			tran.RxDat[tran.RxDataSize] = 0;
//			tran.LockRx = HAL_UNLOCKED;
//			tran.RxFlag = true;   //收到完整一帧
//
//			BaseType_t YieldRequired = xTaskResumeFromISR(tranReceiveTaskHandle);
//			if(YieldRequired==pdTRUE)
//			{
//				/*如果函数xTaskResumeFromISR()返回值为pdTRUE，那么说明要恢复的这个
//				任务的任务优先级等于或者高于正在运行的任务(被中断打断的任务),所以在
//				退出中断的时候一定要进行上下文切换！*/
//				portYIELD_FROM_ISR(YieldRequired);
//			}
//		}while(0);
		LL_DMA_SetDataLength(DMA2, LL_DMA_STREAM_2, 60);
		LL_DMA_EnableStream(DMA2, LL_DMA_STREAM_2);
	}
}
void DMA2_Stream7_IRQHandler(void)  //发送DMA中断
{
	LL_DMA_DisableStream(DMA2,LL_DMA_STREAM_7);
	LL_DMA_ClearFlag_TC7(DMA2);
	TxFlag = false;
}
bool uart_Send_DMA(uint8_t * pData,uint16_t Size)
{
	if(TxFlag == true) return false;	//串口发送忙,放弃发送该帧数据
	LL_DMA_DisableStream(DMA2,LL_DMA_STREAM_7);
	LL_DMA_SetDataLength(DMA2, LL_DMA_STREAM_7, Size);
	LL_DMA_SetMemoryAddress(DMA2, LL_DMA_STREAM_7, (uint32_t)pData);
	LL_DMA_EnableStream(DMA2, LL_DMA_STREAM_7);
	TxFlag = true;
	return true;
}
void TRAN_Init(void)
{
	/* 配置发送DMA */
	LL_DMA_DisableStream(DMA2,LL_DMA_STREAM_7);
	LL_DMA_SetPeriphAddress(DMA2, LL_DMA_STREAM_7, (uint32_t)&USART1->TDR);
	LL_DMA_SetMemoryAddress(DMA2, LL_DMA_STREAM_7, (uint32_t)TxDat);
	LL_DMA_SetDataLength(DMA2, LL_DMA_STREAM_7, 60);
	LL_DMA_ClearFlag_TC7(DMA2);
	LL_DMA_EnableIT_TC(DMA2, LL_DMA_STREAM_7);
	LL_DMA_EnableStream(DMA2, LL_DMA_STREAM_7);
	/* 配置发送DMA */

	LL_USART_EnableDMAReq_RX(USART1);
	LL_USART_EnableDMAReq_TX(USART1);
	LL_USART_ClearFlag_IDLE(USART1);
	LL_USART_ClearFlag_CM(USART1);
	LL_USART_EnableIT_IDLE(USART1);
}
void EskfDataStorage(void)
{
	getTimer_us(&startTimer);
	xQueuePeek(queueESKF,&EskfLog,0);
	xQueuePeek(queueESKF_baro,&Eskf_baroLog,0);
	xQueuePeek(queueGps,&GpsLog,0);
	xQueuePeek(queueBaroAlt,&baroAlt,0);
	xQueuePeek(queueMag, &mag, 0);
	xQueuePeek(queueDownsampleIMU, &imu, 0);
	xQueuePeek(queueRCCommand, &rcCommand, 0);
	xQueuePeek(queueAccDat, &acc, 0);
	xQueuePeek(queueGyrDat, &gyro, 0);
	vs32 temp[60];
	temp[0] = startTimer/100;
	/*eskf data*/
	temp[1] = EskfLog.Pos[0]*1000;
	temp[2] = EskfLog.Pos[1]*1000;
	temp[3] = EskfLog.Pos[2]*1000;
	temp[4] = EskfLog.Ned_spd[0]*1000;
	temp[5] = EskfLog.Ned_spd[1]*1000;
	temp[6] = EskfLog.Ned_spd[2]*1000;
	temp[7] = EskfLog.Attitude[0]*1000;
	temp[8] = EskfLog.Attitude[1]*1000;
	temp[9] = EskfLog.Attitude[2]*1000;

	/*OrdinaryGPS data*/
	temp[10] = GpsLog.timestamp;
	temp[11] = GpsLog.NED[0]*1000;
	temp[12] = GpsLog.NED[1]*1000;
	temp[13] = GpsLog.NED[2]*1000;
	temp[14] = GpsLog.NED_spd[0]*1000;
	temp[15] = GpsLog.NED_spd[1]*1000;
	temp[16] = GpsLog.NED_spd[2]*1000;
	temp[17] = GpsLog.gpsPosAccuracy*100;
	temp[18] = GpsLog.gpsSpdAccuracy*100;

	/*baro data*/
	temp[19] = baroAlt.timestamp;
	temp[20] = baroAlt.altitude*1000;
	temp[21] = baroAlt.altSlope*1000;

	/*MAG data*/
	temp[22] = 0;
	temp[23] = mag.MagRel[0];
	temp[24] = mag.MagRel[1];
	temp[25] = mag.MagRel[2];

	/*IMU data*/
	temp[26] = imu.timestamp;
	temp[27] = acc.acc[0]*1000;
	temp[28] = acc.acc[1]*1000;
	temp[29] = acc.acc[2]*1000;
	temp[30] = gyro.gyro[0]*1000;
	temp[31] = gyro.gyro[1]*1000;
	temp[32] = gyro.gyro[2]*1000;

	/*mahany-q*/
	temp[33] = 0;
	temp[34] = 0;
	temp[35] = 0;
	temp[36] = 0;

	/*eskf_baro data*/
	temp[37] = Eskf_baroLog.Pos[0]*1000;
	temp[38] = Eskf_baroLog.Pos[1]*1000;
	temp[39] = Eskf_baroLog.Pos[2]*1000;
	temp[40] = Eskf_baroLog.Ned_spd[0]*1000;
	temp[41] = Eskf_baroLog.Ned_spd[1]*1000;
	temp[42] = Eskf_baroLog.Ned_spd[2]*1000;
	temp[43] = Eskf_baroLog.Attitude[0]*1000;
	temp[44] = Eskf_baroLog.Attitude[1]*1000;
	temp[45] = Eskf_baroLog.Attitude[2]*1000;
	for(int i=0;i<45;++i)
	{
		openlog.record("%d,",temp[i]);
	}
	openlog.record("%d\r",temp[45]);
	openlog.push_buff();
//	vTaskResume(Openlog_send_Handle);
}
void OutloopDataStorage(void)
{
	xQueuePeek(queueControlTransfer,&control_data,0);
	xQueuePeek(queueESKF,&EskfLog,0);
	xQueuePeek(queueGps, &gps, 0);
	xQueuePeek(queuetrajectoryData, &trajectoryData, 0);
	xQueuePeek(queuelaserFlow, &laserFlow, 0);



	getTimer_us(&startTimer);
	volatile vs32 temp[60];
	temp[0] = startTimer/100;

	/*body速度和反馈*/
	temp[1] = control_data.X_command[1]*1000;
	temp[2] = control_data.Y_command[1]*1000;
	temp[3] = control_data.Z_command[1]*1000;
	temp[4] = control_data.XH[1]*1000;
	temp[5] = control_data.YH[1]*1000;
	temp[6] = control_data.Z[1]*1000;

	/*ned位置指令和反馈*/
	temp[7] = control_data.X_command[0]*1000;
	temp[8] = control_data.Y_command[0]*1000;
	temp[9] = control_data.Z_command[0]*1000;
	temp[10] = control_data.X[0]*1000;
	temp[11] = control_data.Y[0]*1000;
	temp[12] = control_data.Z[0]*1000;

	/*Euler指令和反馈*/
	temp[13] = control_data.roll_command[0]*1000;
	temp[14] = control_data.pitch_command[0]*1000;
	temp[15] = control_data.yaw_command[0]*1000;
	temp[16] = control_data.Ang[0]*1000;
	temp[17] = control_data.Ang[1]*1000;
	temp[18] = control_data.Ang[2]*1000;

	/*油门*/
	temp[19] = control_data.mt_output[0];
	temp[20] = control_data.mt_output[1];

	/*飞行状态*/
	temp[21] = control_data.FlightStatus;
	temp[22] = trajectoryData.angeskf[0]*1000;//PTP_Status;
	temp[23] = trajectoryData.angeskf[1]*1000;//Circle_Status;
	temp[24] = trajectoryData.angeskf[2]*1000;//Dubins_Status;

	/* 全自控模式下相关变量 */
	temp[25] = control_data.XY_phase;
	temp[26] = trajectoryData.YawAngle_traveled*1000;
	temp[27] = trajectoryData.POS_err*1000;
	temp[28] = trajectoryData.azimuth*1000;
	temp[29] = trajectoryData.Vel_command_ref[0]*1000;
	temp[30] = trajectoryData.Pos_command_ref[0]*1000;
	temp[31] = trajectoryData.distance_XY*1000;
	temp[32] = trajectoryData.brake_finish;
	temp[33] = trajectoryData.Pos_track[0]*1000;
	temp[34] = trajectoryData.Pos_track[1]*1000;

	temp[35] = trajectoryData.accelerationAngle*1000;
	temp[36] = trajectoryData.accelerationDistance*1000;
	temp[37] = control_data.Jerk_command[0]*1000;


	temp[38] = trajectoryData.Ti[0]*1000;;//control_data.AngVel_command[0]*1000;
	temp[39] = trajectoryData.Ti[1]*1000;//control_data.AngVel_command[1]*1000;
	temp[40] = trajectoryData.Ti[2]*1000;//control_data.AngVel_command[2]*1000;
	temp[41] = control_data.pqr_command[0]*1000;//control_data.control_mode;
	temp[42] = control_data.pqr_command[1]*1000;//control_data.XYZ_phase;
	temp[43] = control_data.pqr_command[2]*1000;//control_data.Acc_command[0]*1000;
	temp[44] = control_data.pqr[0]*1000;//control_data.Acc_command[1]*1000;
	temp[45] = control_data.pqr[1]*1000;//control_data.Acc_command[2]*1000;
	temp[46] = control_data.pqr[2]*1000;//control_data.brake_mode;

	temp[47] = trajectoryData.executionTime_us;//control_data.Jerk_command[1]*1000;
	temp[48] = trajectoryData.Tf[0]*1000;//control_data.pqr_command[1]*1000;
	temp[49] = trajectoryData.Tf[1]*1000;//control_data.pqr_command[2]*1000;
	temp[50] = trajectoryData.Tf[2]*1000;//control_data.pqr[0]*1000;
	temp[51] = laserFlow.height*1000;//control_data.pqr[1]*1000;
	temp[52] = laserFlow.heightFil*1000;//control_data.pqr[2]*1000;
	temp[53] = laserFlow.Velx*1000;//control_data.roll_command[1]*1000;
	temp[54] = laserFlow.Vely*1000;//control_data.pitch_command[1]*1000;
	temp[55] = laserFlow.VelxFil*1000;//control_data.Jerk_command[1]*1000;
	temp[56] = laserFlow.VelyFil*1000;//trajectoryData.goalDistance*1000;;
	temp[57] = trajectoryData.Einv[0]*1000;//trajectoryData.goalAng*1000;;
	temp[58] = trajectoryData.Einv[1]*1000;;
	temp[59] = trajectoryData.Einv[2]*1000;;
	for(int i=0;i<59;++i)
	{
		openlog.record("%d,",temp[i]);
	}
	openlog.record("%d\r",temp[59]); // 换行
	openlog.push_buff();
}
extern "C" void tskLog(void *argument)
{
	/* Pre-Load for task */
	osDelay(6000);
	TRAN_Init();
	u8 num = 0;
	openlog.new_file("Data0420_%d.csv",num);
	openlog.append_file("Data0420_%d.csv",num);
	openlog.Send();
//	openlog.record("timestamp,KFpos0,KFpos1,KFpos2,KFspd0,KFspd1,KFspd2,KFroll,KFpitch,KFyaw\r");
//	openlog.record("GPStimestamp,GPSpos0,GPSpos1,GPSpos2,GPSspd0,GPSspd1,GPSspd2,gpsPosAccuracy,gpsSpdAccuracy,");
//	openlog.record("baro_timestamp,baro_altitude,baro_altSlope,MAGtimestamp,MAGdat0,MAGdat1,MAGdat2,imu_timestamp,acc0,acc1,acc2,gyro0,gyro1,gyro2,qx,qy,qz,qw\r");
//	openlog.push_buff();
	/* Infinite loop */
	for(;;)
	{
		/* wait for next circle */
		osSemaphoreAcquire(semLog,0xffffffff);
//		vTaskSuspend(Log_Handle);
		xQueuePeek(queueRCCommand, &rcCommand, 0);					//从队列中获取遥控器数据
		if(rcCommand.Key[1]==2){
			EskfDataStorage();
			openlog.Send();
		}
	}
}
