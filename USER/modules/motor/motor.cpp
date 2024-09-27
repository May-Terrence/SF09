/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : motor.cpp
  * Description        :
  ******************************************************************************
  * Function           :
  *
  ******************************************************************************
  * Author             : Zhiping Fu
  * Creation Date      : Oct 23, 2020
  ******************************************************************************
  */
/* USER CODE END Header */

#include "motor.hpp"
#include "hardware.h"

//MOTOR motor(TIM4,TIM3,(char *)"MOTOR");

//MOTORMAG motormag(UART8,(char*) "MOTORMAG",(char*) "-Y-X-Z");

void MOTORMAG::motor_Init(void)
{

	p_limits=(CS_LIMIT_MAX/RAD_TO_PWM)*180.0f/3.1415926535897931f;//单位：度
	//--------------------------------------------
	Update = true;
	Sta = STA_INI;
	Err = ERR_NONE;

	if(Dir_Trans(Dir, DirChr)==false) Err = ERR_SOFT;

	for(u8 i=0;i<PWM_OUT_NUM;i++)
	{
		PWM[i] = INI_PWM;
		PwmOff[i] = 0;
	}

	osDelay(100);

	LL_TIM_EnableCounter(TIM4);
	LL_TIM_OC_SetCompareCH1(TIM4,PWM[0]);
//	LL_TIM_OC_SetCompareCH2(TIM4,PWM[1]);
	LL_TIM_OC_SetCompareCH3(TIM4,PWM[5]);
	LL_TIM_OC_SetCompareCH4(TIM4,PWM[6]);
	LL_TIM_CC_EnableChannel(TIM4, LL_TIM_CHANNEL_CH1);
	LL_TIM_CC_EnableChannel(TIM4, LL_TIM_CHANNEL_CH2);
	LL_TIM_CC_EnableChannel(TIM4, LL_TIM_CHANNEL_CH3);
	LL_TIM_CC_EnableChannel(TIM4, LL_TIM_CHANNEL_CH4);
	LL_TIM_EnableAllOutputs(TIM4);

	TxFlag = false;
	/* 配置发送DMA */
	LL_DMA_DisableStream(Motor_DMA,Motor_DMA_Stream);
	LL_DMA_SetPeriphAddress(Motor_DMA, Motor_DMA_Stream, (uint32_t)&Motor_DMA_UART->TDR);
	LL_DMA_SetMemoryAddress(Motor_DMA, Motor_DMA_Stream, (uint32_t)PWM);
	LL_DMA_SetDataLength(Motor_DMA, Motor_DMA_Stream, TRAN_TX_LEN);
#ifdef motor_U8
	LL_DMA_ClearFlag_TC0(Motor_DMA);
#endif
#ifdef motor_U6
	LL_DMA_ClearFlag_TC6(Motor_DMA);
#endif

	LL_DMA_EnableIT_TC(Motor_DMA, Motor_DMA_Stream);
	LL_DMA_EnableStream(Motor_DMA, Motor_DMA_Stream);

	/* 配置接收DMA */
	LL_DMA_DisableStream(Motor_DMA,Motor_DMA_RX);
	LL_DMA_SetPeriphAddress(Motor_DMA, Motor_DMA_RX, (uint32_t)&Motor_DMA_UART->RDR);
	LL_DMA_SetMemoryAddress(Motor_DMA, Motor_DMA_RX, (uint32_t)rm_dat);
	LL_DMA_SetDataLength(Motor_DMA, Motor_DMA_RX, TRAN_RX_LEN);

//	LL_DMA_EnableIT_TC(Motor_DMA, Motor_DMA_RX);
	LL_DMA_EnableStream(Motor_DMA, Motor_DMA_RX);

	/*启用DMA*/
	LL_USART_EnableDMAReq_RX(Motor_DMA_UART);
	LL_USART_EnableDMAReq_TX(Motor_DMA_UART);

	LL_USART_ClearFlag_IDLE(Motor_DMA_UART);
	LL_USART_EnableIT_IDLE(Motor_DMA_UART);

#ifdef MF09II02
			MagOff[0] = -5078;
			MagOff[1] = 29;
			MagOff[2] = 12819;
#endif
#ifdef MF09II01
			MagOff[0] = -1944;
			MagOff[1] =  4493;
			MagOff[2] =  9196;
#endif


	mag_update_time_us = 0;
	mag.timestamp = mag_update_time_us;

	for(uint8_t i=0;i<3;i++)
	{
		mag.MagRel[i] = MagRel[i] = 0;
	}

	xQueueOverwrite(queueMag,&mag);

	InitCounter = 100;

}

bool MOTORMAG::uart_Send_DMA(uint8_t * pData,uint16_t Size)
{
	if(TxFlag == true) return false;	//串口发送忙,放弃发送该帧数据
	LL_DMA_DisableStream(Motor_DMA,Motor_DMA_Stream);
	LL_DMA_SetDataLength(Motor_DMA, Motor_DMA_Stream, Size);
	LL_DMA_SetPeriphAddress(Motor_DMA, Motor_DMA_Stream, (uint32_t)&Motor_DMA_UART->TDR);
	LL_DMA_SetMemoryAddress(Motor_DMA, Motor_DMA_Stream, (uint32_t)pData);
	LL_DMA_EnableStream(Motor_DMA, Motor_DMA_Stream);
	TxFlag = true;
	return true;
}

#ifdef motor_U8
MOTORMAG motormag(UART8,(char*) "MOTORMAG",(char*) "+Y-X+Z");//"-Y-X-Z"

void UART8_IRQHandler(void)
{
	if(LL_USART_IsActiveFlag_IDLE(Motor_DMA_UART))
	{
		LL_USART_ClearFlag_IDLE(Motor_DMA_UART);
		LL_DMA_DisableStream(Motor_DMA,Motor_DMA_RX);

		LL_DMA_ClearFlag_DME6(Motor_DMA);
		LL_DMA_ClearFlag_HT6(Motor_DMA);
		LL_DMA_ClearFlag_TC6(Motor_DMA);
		LL_DMA_ClearFlag_TE6(Motor_DMA);
		LL_DMA_ClearFlag_FE6(Motor_DMA);

		LL_USART_ClearFlag_CM(UART8);
		LL_USART_ClearFlag_EOB(UART8);
		LL_USART_ClearFlag_FE(UART8);
		LL_USART_ClearFlag_LBD(UART8);
		LL_USART_ClearFlag_NE(UART8);
		LL_USART_ClearFlag_ORE(UART8);
		LL_USART_ClearFlag_PE(UART8);
		LL_USART_ClearFlag_RTO(UART8);
		LL_USART_ClearFlag_TC(UART8);
		LL_USART_ClearFlag_WKUP(UART8);
		LL_USART_ClearFlag_nCTS(UART8);
		LL_USART_ClearFlag_IDLE(UART8);

		motormag.RxDataSize = TRAN_RX_LEN - LL_DMA_GetDataLength(Motor_DMA, Motor_DMA_RX);


		do{//0xAA 0xAF [功能字] [字节数] [内容...] [校验和]
//			if(motormag.rm_dat[9] != 0X24) break;
			if(motormag.rm_dat[10] != '$') break;//小板的最后一位飞控上会读成第一位
			if(motormag.LockRx == HAL_LOCKED) break;
			motormag.LockRx = HAL_LOCKED;
			memcpy(motormag.RxDat, motormag.rm_dat, motormag.RxDataSize);
			motormag.RxDat[motormag.RxDataSize] = 0;
			motormag.LockRx = HAL_UNLOCKED;
			motormag.RxFlag = true;   //收到完整一帧

			BaseType_t YieldRequired = xTaskResumeFromISR(tranReceiveTaskHandle);
			if(YieldRequired==pdTRUE)
			{
				/*如果函数xTaskResumeFromISR()返回值为pdTRUE，那么说明要恢复的这个
				任务的任务优先级等于或者高于正在运行的任务(被中断打断的任务),所以在
				退出中断的时候一定要进行上下文切换！*/
				portYIELD_FROM_ISR(YieldRequired);
			}
		}while(0);

		LL_DMA_SetDataLength(Motor_DMA, Motor_DMA_RX, TRAN_RX_LEN);
		LL_DMA_EnableStream(Motor_DMA, Motor_DMA_RX);

	}

}
void DMA1_Stream0_IRQHandler(void)  //发送DMA中断
{
	LL_DMA_DisableStream(Motor_DMA,Motor_DMA_Stream);
	LL_DMA_ClearFlag_TC0(Motor_DMA);
	motormag.TxFlag = false;
}

void DMA1_Stream6_IRQHandler(void)  //接收DMA中断
{


}
#endif

#ifdef motor_U6
MOTORMAG motormag(USART6,(char*) "MOTORMAG",(char*) "-Y-X-Z");

void USART6_IRQHandler(void)
{
	if(LL_USART_IsActiveFlag_IDLE(Motor_DMA_UART))
	{
		LL_USART_ClearFlag_IDLE(Motor_DMA_UART);
		LL_DMA_DisableStream(Motor_DMA,Motor_DMA_RX);

		LL_DMA_ClearFlag_DME1(Motor_DMA);
		LL_DMA_ClearFlag_HT1(Motor_DMA);
		LL_DMA_ClearFlag_TC1(Motor_DMA);
		LL_DMA_ClearFlag_TE1(Motor_DMA);
		LL_DMA_ClearFlag_FE1(Motor_DMA);

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

		motormag.RxDataSize = TRAN_RX_LEN - LL_DMA_GetDataLength(Motor_DMA, Motor_DMA_RX);


		do{//0xAA 0xAF [功能字] [字节数] [内容...] [校验和]
//			if(motormag.rm_dat[9] != 0X24) break;
			if(motormag.rm_dat[10] != '$') break;//小板的最后一位飞控上会读成第一位
			if(motormag.LockRx == HAL_LOCKED) break;
			motormag.LockRx = HAL_LOCKED;
			memcpy(motormag.RxDat, motormag.rm_dat, motormag.RxDataSize);
			motormag.RxDat[motormag.RxDataSize] = 0;
			motormag.LockRx = HAL_UNLOCKED;
			motormag.RxFlag = true;   //收到完整一帧

			BaseType_t YieldRequired = xTaskResumeFromISR(tranReceiveTaskHandle);
			if(YieldRequired==pdTRUE)
			{
				/*如果函数xTaskResumeFromISR()返回值为pdTRUE，那么说明要恢复的这个
				任务的任务优先级等于或者高于正在运行的任务(被中断打断的任务),所以在
				退出中断的时候一定要进行上下文切换！*/
				portYIELD_FROM_ISR(YieldRequired);
			}
		}while(0);

		LL_DMA_SetDataLength(Motor_DMA, Motor_DMA_RX, TRAN_RX_LEN);
		LL_DMA_EnableStream(Motor_DMA, Motor_DMA_RX);

	}

}
void DMA2_Stream6_IRQHandler(void)//发送DMA中断
{
	LL_DMA_DisableStream(Motor_DMA,Motor_DMA_Stream);
	LL_DMA_ClearFlag_TC6(Motor_DMA);
	motormag.TxFlag = false;
}
void DMA2_Stream1_IRQHandler(void)  //接收DMA中断
{

}
#endif

void MOTORMAG::motor_run(void)
{
	xQueuePeek(queueRCCommand, &rcCommand, 0);					//从队列中获取遥控器数据
	xQueuePeek(queueControlOutputDat, &control_output, 0);		//从队列中获取控制器输出数据
	xQueuePeek(queuePID, &pid, 0);								//获取PID数据（调试）

	if(rcCommand.Mode == RC_MOT_UNLOCK)UnLock = true;
	else if(rcCommand.Mode == RC_MOT_LOCK)UnLock = false;
	if(UnLock == false)    //电机锁定
	{
		for(u8 i=0;i<PWM_OUT_NUM;i++)
			PWM[i]=INI_PWM;
	}

	if(rcCommand.Key[0]==2) //电机锁
	{
		PWM[0] = iConstrain(control_output.mt_output[0],1090,1910);
//		PWM[0] =  rcCommand.Val[3];//电调校准行程用
	}
	else
	{
		PWM[0] = 1090;
	}

	PWM[1] = iConstrain(PWM_MID_1+control_output.cs_output[0],PWM_MID_1+CS_LIMIT_MIN,PWM_MID_1+CS_LIMIT_MAX);
	PWM[2] = iConstrain(PWM_MID_2+control_output.cs_output[1],PWM_MID_2+CS_LIMIT_MIN,PWM_MID_2+CS_LIMIT_MAX);
	PWM[3] = iConstrain(PWM_MID_3+control_output.cs_output[2],PWM_MID_3+CS_LIMIT_MIN,PWM_MID_3+CS_LIMIT_MAX);
	PWM[4] = iConstrain(PWM_MID_4+control_output.cs_output[3],PWM_MID_4+CS_LIMIT_MIN,PWM_MID_4+CS_LIMIT_MAX);
	PWM[5] = 1500;
	PWM[6] = 1500;

	PWM[7] = 1500;
	PWM[8] = 1500;

	PWM[9] = 1500;
	PWM[10] = (0xFF00 | rcCommand.Key[0] );//舵机锁（可能）



	LL_GPIO_ResetOutputPin(GPIOB,GPIO_PIN_0);

	for(u8 i=0;i<PWM_OUT_NUM;i++)  //限幅
	{
		PWM[i]=iConstrain(PWM[i]-PwmOff[i],Min_PWM_Out,Max_PWM_Out);
		motor_msg.PWM_OBS[i]=PWM[i];
	}

	xQueueOverwrite(queueMotorData,&motor_msg);

	uart_Send_DMA((u8 *)PWM, 22);//10×16bit PWM + 1×16bit 校验位


	LL_TIM_OC_SetCompareCH1(TIM4,PWM[0]);
//	LL_TIM_OC_SetCompareCH2(TIM4,PWM[1]);
	LL_TIM_OC_SetCompareCH3(TIM4,PWM[5]);
	LL_TIM_OC_SetCompareCH4(TIM4,PWM[6]);


	if(RxFlag == false) return;
//	if(LockRx == HAL_LOCKED) return;
	else
		{
		RxFlag = false;
//	LockRx = HAL_LOCKED;


		uint8_t num = motormag.RxDataSize;
		TxSum = 0;
		for(uint8_t i=1; i<=num-2; i++) TxSum += RxDat[i];
		if(TxSum != RxDat[0])return;		//和校验 判断sum
		else
		{
			motormag.LockRx = HAL_LOCKED;


			MagRaw[0] = 0;
			MagRaw[1] = 0;
			MagRaw[2] = 0;

			MagRaw[0] = RxDat[1]<<16 | RxDat[2]<<8 | RxDat[3];
			if(MagRaw[0] >= 0x00800000)
			{
				MagRaw[0] |= 0xff000000;
			}

			MagRaw[1] = RxDat[4]<<16 | RxDat[5]<<8 | RxDat[6];
			if(MagRaw[1] >= 0x00800000)
			{
				MagRaw[1] |= 0xff000000;
			}

			MagRaw[2] = RxDat[7]<<16 | RxDat[8]<<8 | RxDat[9];
			if(MagRaw[2] >= 0x00800000)
			{
				MagRaw[2] |= 0xff000000;
			}
			motormag.LockRx = HAL_UNLOCKED;

//		MagRaw[0] = rm_dat[0]<<16 | rm_dat[1]<<8 | rm_dat[2];
//		if(MagRaw[0] >= 0x00800000)
//		{
//			MagRaw[0] |= 0xff000000;
//		}
//
//		MagRaw[1] = rm_dat[3]<<16 | rm_dat[4]<<8 | rm_dat[5];
//		if(MagRaw[1] >= 0x00800000)
//		{
//			MagRaw[1] |= 0xff000000;
//		}
//
//		MagRaw[2] = rm_dat[6]<<16 | rm_dat[7]<<8 | rm_dat[8];
//		if(MagRaw[2] >= 0x00800000)
//		{
//			MagRaw[2] |= 0xff000000;
//		}

//		LockRx = HAL_UNLOCKED;

//	for(uint8_t i=0;i<3;i++)
//	{
//		if(InitCounter > 0)
//		{
//			InitCounter--;
//		}
//		else
//		{
//			if(abs(MagRaw[i] - LastMagRaw[i]) > 1000)
//			{
//				MagRaw[i] = LastMagRaw[i];
//			}
//		}
//		LastMagRaw[i] = MagRaw[i];
//	}

			MagFlag = true;

			getTimer_us(&mag_update_time_us);
		}
	}

	mag.timestamp = mag_update_time_us;

	if(MagFlag == true)
	{
		//方向变换
		MagRot[0]=Dir[0]*(MagRaw[Dir[1]]-MagOff[Dir[1]]);
		MagRot[1]=Dir[2]*(MagRaw[Dir[3]]-MagOff[Dir[3]]);
		MagRot[2]=Dir[4]*(MagRaw[Dir[5]]-MagOff[Dir[5]]);
		//实际值转换
		MagFil[0]=MagRel[0]=MagRot[0];
		MagFil[1]=MagRel[1]=MagRot[1];
		MagFil[2]=MagRel[2]=MagRot[2];

		if(MagSta == STA_RUN)MagUpdate = true;

		for(uint8_t i=0;i<3;i++)
		{
			mag.MagRaw[i] = MagRaw[i];
			mag.MagRel[i] = MagRel[i];
		}

		xQueueOverwrite(queueMag,&mag);
	}


}
//UART8

extern "C" void motor_main(void *argument)
{
	motormag.motor_Init();
	osDelay(1);
	for(;;)
	{
		osSemaphoreAcquire(semMotor,0xffffffff);
		motormag.motor_run();
	}
}


/************************ (C) COPYRIGHT Longmen Drone Team *****END OF FILE****/
