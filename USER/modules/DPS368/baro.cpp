/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : DPS368.cpp
  * Description        :
  ******************************************************************************
  * Function           :
  *
  ******************************************************************************
  * Author             : Zhiping Fu
  * Creation Date      : 2020年11月8日
  ******************************************************************************
  */
/* USER CODE END Header */
#include "../DPS368/baro.hpp"

DPS368 DPS368(&hspi2,(char *)"DPS368");

void DPS368::DPS368_Init()
{
	Update = false;
	Sta = STA_INI;
	Err = ERR_NONE;
	Cal = false;

	Filt1.CNT = 0;
	Filt1.CCR = 5;
	Filt2.CNT = 0;
	Filt2.CCR = 5;
	Filt3.CNT = 0;
	Filt3.CCR = 5;
	Delay = 128;


	Time = 0;
	StaCnt = 0;
	AltOff = 0.0f;
	for(u8 i=0;i<MS_SLOPE_NUM;i++)
		AltTmp[i]=0;

	osDelay(3000);

	DPS368_Spi_Tran(hspi,DPS368_CMD_RESET,0x89);
	osDelay(200);


	bool ready=0;
	while(ready!=1){
		DPS368_Spi_TranRecv(hspi,DPS368_CMD_MEAS_CFG_R,rxbuf);
		if(rxbuf[0]&0x80U)ready=1;
		osDelay(200);
	}
	DPS368_Spi_TranRecv(hspi,DPS368_CMD_ID,rxbuf);
	ID = rxbuf[0];
	osDelay(10);
	DPS368_Spi_TranRecv(hspi,DPS368_CMD_COEF,rxbuf);
	c0 = rxbuf[0] << (4U);
	osDelay(10);
	DPS368_Spi_TranRecv(hspi,DPS368_CMD_COEF+1,rxbuf);
	c0 += rxbuf[0] >> (4U);
	c1 = (rxbuf[0] & 0x0F) << (8U);
	osDelay(10);
	DPS368_Spi_TranRecv(hspi,DPS368_CMD_COEF+2,rxbuf);
	c1 += rxbuf[0];
	osDelay(10);
	DPS368_Spi_TranRecv(hspi,DPS368_CMD_COEF+3,rxbuf);
	c00 = rxbuf[0] << (12U);
	osDelay(10);
	DPS368_Spi_TranRecv(hspi,DPS368_CMD_COEF+4,rxbuf);
	c00 += rxbuf[0] << (4U);
	osDelay(10);
	DPS368_Spi_TranRecv(hspi,DPS368_CMD_COEF+5,rxbuf);
	c00 += rxbuf[0] >> (4U);
	c10 = (rxbuf[0] & 0x0F) << (16U);
	osDelay(10);
	DPS368_Spi_TranRecv(hspi,DPS368_CMD_COEF+6,rxbuf);
	c10 += rxbuf[0] << (8U);
	osDelay(10);
	DPS368_Spi_TranRecv(hspi,DPS368_CMD_COEF+7,rxbuf);
	c10 += rxbuf[0];
	osDelay(10);
	DPS368_Spi_TranRecv(hspi,DPS368_CMD_COEF+8,rxbuf);
	c01 = rxbuf[0] << (8U);
	osDelay(10);
	DPS368_Spi_TranRecv(hspi,DPS368_CMD_COEF+9,rxbuf);
	c01 += rxbuf[0];
	osDelay(10);
	DPS368_Spi_TranRecv(hspi,DPS368_CMD_COEF+10,rxbuf);
	c11 = rxbuf[0] << (8U);
	osDelay(10);
	DPS368_Spi_TranRecv(hspi,DPS368_CMD_COEF+11,rxbuf);
	c11 += rxbuf[0];
	osDelay(10);
	DPS368_Spi_TranRecv(hspi,DPS368_CMD_COEF+12,rxbuf);
	c20 = rxbuf[0] << (8U);
	osDelay(10);
	DPS368_Spi_TranRecv(hspi,DPS368_CMD_COEF+13,rxbuf);
	c20 += rxbuf[0];
	osDelay(10);
	DPS368_Spi_TranRecv(hspi,DPS368_CMD_COEF+14,rxbuf);
	c21 = rxbuf[0] << (8U);
	osDelay(10);
	DPS368_Spi_TranRecv(hspi,DPS368_CMD_COEF+15,rxbuf);
	c21 += rxbuf[0];
	osDelay(10);
	DPS368_Spi_TranRecv(hspi,DPS368_CMD_COEF+16,rxbuf);
	c30 = rxbuf[0] <<(8U);
	osDelay(10);
	DPS368_Spi_TranRecv(hspi,DPS368_CMD_COEF+17,rxbuf);
	c30 += rxbuf[0];
	osDelay(10);

	getTwosComplement(&c0,12);
	getTwosComplement(&c1,12);
	getTwosComplement(&c00,20);
	getTwosComplement(&c10,20);
	getTwosComplement(&c01,16);
	getTwosComplement(&c11,16);
	getTwosComplement(&c20,16);
	getTwosComplement(&c21,16);
	getTwosComplement(&c30,16);

	DPS368_Spi_Tran(hspi,DPS368_CMD_PRS_CFG,0x54);
	osDelay(10);
	DPS368_Spi_Tran(hspi,DPS368_CMD_TMP_CFG,0x82);
	osDelay(10);
	DPS368_Spi_Tran(hspi,DPS368_CMD_CFG_REG,0x04);
	osDelay(10);
	while(ready!=1){
		DPS368_Spi_TranRecv(hspi,DPS368_CMD_MEAS_CFG_R,rxbuf);
		if(rxbuf[0]&0x40U)ready=1;
		osDelay(200);
	}
	DPS368_Spi_Tran(hspi,DPS368_CMD_MEAS_CFG_W,0x07);
	osDelay(10);

	Sta = STA_CHK;

	PRS=0;
	TMP=0;

	if(ID!=0x10)  //0x3FED
	{
		Err=ERR_LOST;
		return;
	}

	LL_TIM_EnableCounter(TIM1);
	LL_TIM_OC_SetCompareCH1(TIM1,0);
	LL_TIM_CC_EnableChannel(TIM1, LL_TIM_CHANNEL_CH1);
	LL_TIM_EnableAllOutputs(TIM1);
	//初始化完毕
	Sta = STA_RUN;
	Cal = true;

}

bool DPS368::DPS368_Cali()
{
	return 0;
}

void DPS368::DPS368_Update()
{
	if(Sta != STA_CAL && Sta != STA_RUN)return;

	DPS368_Spi_TranRecv(hspi,DPS368_CMD_MEAS_CFG_R,rxbuf);
	TMP_RDY = (rxbuf[0] & 0x20);
	PRS_RDY = (rxbuf[0] & 0x10);

	if(TMP_RDY!=0){
		DPS368_Spi_TranRecv(hspi,DPS368_CMD_TMP_READ2,rxbuf);
		osDelay(1);
		DPS368_Spi_TranRecv(hspi,DPS368_CMD_TMP_READ1,rxbuf+1);
		osDelay(1);
		DPS368_Spi_TranRecv(hspi,DPS368_CMD_TMP_READ0,rxbuf+2);
		osDelay(1);
		TMP = (rxbuf[0] << 16)|(rxbuf[1] << 8)|(rxbuf[2] << 0);
	}

	if(PRS_RDY==0){
		return;
	}

	PRS_LST = PRS;

	DPS368_Spi_TranRecv(hspi,DPS368_CMD_PSR_READ2,rxbuf);
	osDelay(1);
	DPS368_Spi_TranRecv(hspi,DPS368_CMD_PSR_READ1,rxbuf+1);
	osDelay(1);
	DPS368_Spi_TranRecv(hspi,DPS368_CMD_PSR_READ0,rxbuf+2);
	osDelay(1);
	PRS = (rxbuf[0] << 16)|(rxbuf[1] << 8)|(rxbuf[2] << 0);

	startTimerLast = startTimer;
	getTimer_us(&startTimer);
	cycleTime_us = startTimer - startTimerLast;

	getTwosComplement(&PRS,24);
	getTwosComplement(&TMP,24);

	if(cycleTime_us<100){
		PrsRaw = PRS / 1040384.0f;
	}

	//PrsRaw = PRS / 1040384.0f;
	PrsRaw = PRS / 253952.0f;
	TmpRaw = TMP / 3670016.0f;

	PrsRel = c00 + PrsRaw * (c10 + PrsRaw * (c20 + PrsRaw * c30) ) + TmpRaw * c01 + TmpRaw * PrsRaw * (c11 + PrsRaw * c21);
	TmpRel = c0 * 0.5f + c1 * TmpRaw;

	if(Delay){
		Delay--;
		AltOff = (1.0f - pow(PrsRel/MSLP, 0.190295f))*44300.0f;
		return;
	}

	AltRel = (1.0f - pow(PrsRel/MSLP, 0.190295f))*44300.0f - AltOff;  //海拔，单位：m

	/*
	//实际值解算
	TmpRel = (float)TEMP/100.0f;
	PrsRel = (float)P/100.0f;
	AltRel = (float)ALT/100.0f;
	DPS368_Cali();
	AltRel -= AltOff;
	//滤波
	SlideFilt(&TmpFil,&TmpRel,1,&Filt1,1);
	SlideFilt(&PrsFil,&PrsRel,1,&Filt2,2);
	SlideFilt(&AltFil,&AltRel,1,&Filt3,2);
	 */
	for(u8 i=1;i<MS_SLOPE_NUM;i++) AltTmp[i-1]=AltTmp[i];
	AltTmp[MS_SLOPE_NUM-1]=AltRel;
	float ab[2];
	LineFit(AltTmp,MS_SLOPE_NUM,ab);  //最小二乘法拟合速率
	AltSlope = ab[0]*32.0f;           //32Hz

	TmpFil = TmpRel;

	baroAlt.altitude = AltRel;
	baroAlt.altSlope = AltSlope;
	baroAlt.timestamp = startTimer;
	baroAlt.bar_temp=TmpRel;
	xQueueOverwrite(queueBaroAlt,&baroAlt);

	getTimer_us(&stopTimer);
	executionTime_us = stopTimer - startTimer;

	if(Sta==STA_RUN)Update = true;
	return;

}

void DPS368::getTwosComplement(int32_t *raw, uint8_t length)
{
	if (*raw & ((uint32_t)1 << (length - 1)))
	{
		*raw -= (uint32_t)1 << length;
	}
}

void DPS368::PID_Temp()
{
	if(TmpFil==0){
		pid_tempe=0;
	}
	else{
		pid_tempe=msT_T-TmpFil;
	}

	if(pid_tempout>950&&pid_tempe>0)
	{
		pid_tempi=pid_tempi;
	}
	else if(pid_tempout<10&&pid_tempe<0)
	{
		pid_tempi=pid_tempi;
	}
	else
	{
		pid_tempi+=pid_tempe;
	}
	pid_tempd = pid_tempe-pid_tempdi;
	pid_tempdi += pid_tempd*0.5f;
	pid_tempout=(u16)(msT_P*pid_tempe+msT_I*pid_tempi*1.0f+msT_D*pid_tempd);
	LL_TIM_OC_SetCompareCH1(TIM1, pid_tempout);
}


//气压计状态机，一毫秒调用一次
void DPS368::DPS368_Loop()
{
	DPS368_Update();

	xQueuePeek(queueBattery, &battery, 0);
	if(battery.battery_Voltage>10.0f && TMP_RDY!=0)
	{
		PID_Temp();
	}
}

extern "C" void DPS368_main(void *argument)
{
	DPS368.DPS368_Init();
	for(;;)
	{
		osSemaphoreAcquire(semDPS368,0xffffffff);
		DPS368.DPS368_Loop();
	}
}

void DPS368::DPS368_Spi_Tran(SPI_HandleTypeDef *hspi,u8 Cmd,u8 data)
{
	DPS368_SPI2_CS_CLR;
	CMD_RW[0] = Cmd;
	CMD_RW[1] = data;
	HAL_SPI_Transmit(hspi,CMD_RW,2,400);
	DPS368_SPI2_CS_SET;
	return ;
}

void DPS368::DPS368_Spi_TranRecv(SPI_HandleTypeDef *hspi,u8 Cmd,uint8_t* ReadData)
{
	DPS368_SPI2_CS_CLR;
	HAL_SPI_Transmit(hspi, &Cmd, 1, 400);
	HAL_SPI_Receive(hspi, ReadData, 1, 400);
	DPS368_SPI2_CS_SET;
	return ;
}

//#pragma GCC optimize ("O0")
/************************ (C) COPYRIGHT Longmen Drone Team *****END OF FILE****/
