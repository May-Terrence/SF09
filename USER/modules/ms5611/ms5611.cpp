/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : ms5611.cpp
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
#include "ms5611.hpp"

MS5611 ms5611(SPI2,(char *)"ms5611");



void MS5611::Ms5611_Init()
{
	Update = false;
	Sta = STA_INI;
	Err = ERR_NONE;
	Cal = false;

	Filt.CNT = 0;
	Filt.CCR = 3;
	Delay = 0;

	Time = 0;
	StaCnt = 0;
	AltOff = 0.0f;
	for(u8 i=0;i<MS_SLOPE_NUM;i++)
		AltTmp[i]=0;


	LL_SPI_Enable(SPI2);
	LL_SPI_SetRxFIFOThreshold(SPI2, LL_SPI_RX_FIFO_TH_QUARTER);
	osDelay(100);

	Ms5611_Spi_Tran(hspi,MS5611_CMD_RESET);
	osDelay(100);
	Sta = STA_CHK;
	u8 rxbuf[2] = { 0, 0 };
	for(u8 i=0;i<7;i++)
	{
		Ms5611_Spi_TranRecv(hspi,MS5611_CMD_PROM_RD+i*2,rxbuf,2);
		PROM[i] = ((u16)rxbuf[0]<<8) | rxbuf[1];
		if((PROM[i]==0x00||PROM[i]==0xFFFF) && (i!=0))Err=ERR_LOST;

	}
	if(Err==ERR_LOST)  //0x3FED
	{
		Err=ERR_LOST;
		return;
	}

	Ms5611_Spi_Tran(hspi,MS5611_CMD_D2_4096);              //开始采集温度
    Ms5611_Spi_Tran(hspi,MS5611_CMD_D1_4096);              //开始采集气压
	osDelay(10);
	//初始化完毕
	Sta = STA_RUN;
	Cal = true;

}

void MS5611::Ms5611_Spi_Tran(SPI_TypeDef *hspi,u8 Cmd)
{
	MS5611_SPI2_CS_CLR;
	SPI_ReadWriteByte(hspi,Cmd);
	MS5611_SPI2_CS_SET;
	return;
}

void MS5611::Ms5611_Spi_TranRecv(SPI_TypeDef *hspi,u8 Cmd,u8* ReadData,u8 num)
{
	MS5611_SPI2_CS_CLR;
	SPI_ReadWriteByte(hspi,Cmd);
	SPI_Recesive(hspi,ReadData,num);
	MS5611_SPI2_CS_SET;
	return ;
}

bool MS5611::Ms5611_Cali()
{
	static bool Ms5611IsCali = false;
	if(Cal==true)
	{
		Cal = false;
		Sta = STA_CAL;
		Ms5611IsCali = true;
	}
	if(Ms5611IsCali==false)return false;

	static u16 Cali=0;   //下次进来就不用等待了
	if(Cali<50){Cali++;return false;}     //忽略前面500次
	#define SAMPLE_NUM 500
	static u16 SmpCnt=0;
	static float AltSum=0.0f;
	if(SmpCnt++<SAMPLE_NUM)
	{
		AltSum+=AltRel;
		return false;
	}
	SmpCnt=0;
	AltOff = AltSum/SAMPLE_NUM;
	AltSum=0.0f;
	Sta=STA_RUN;
	Ms5611IsCali = false;
	return true;
}


void MS5611::Ms5611_Update()
{
	if(Sta != STA_CAL && Sta != STA_RUN)return;

	s32 dT, TEMP, P, ALT;
	s64 OFF1,SENS,T2=0,OFF2=0,SENS2=0,Delt;
	u32 ADC_TMP;  // ADC result of temperature measurement
	u32 ADC_PRS;  // ADC result of pressure measurement

	ADC_TMP = ((u32)TmpRaw[0] << 16) | ((u32)TmpRaw[1] << 8) | TmpRaw[2];
	ADC_PRS = ((u32)PrsRaw[0] << 16) | ((u32)PrsRaw[1] << 8) | PrsRaw[2];

	if(ADC_TMP==0||ADC_TMP==0xFFFFFF||ADC_PRS==0||ADC_PRS==0xFFFFFF)
	{
		Err=ERR_LOST;
		return;
	}
	//计算温度
	dT   = (s32)ADC_TMP - ((s32)PROM[5] << 8);
	TEMP = 2000 + (((int64_t)dT * PROM[6]) >> 23);  //0.01°C
	//计算气压的温度补偿
	OFF1 = ((s64)PROM[2] << 16) + (((s64)dT * PROM[4]) >> 7);
	SENS = ((s64)PROM[1] << 15) + (((s64)dT * PROM[3]) >> 8);
	//温度低于20度的二次补偿
	if(TEMP < 2000)	// 温度低于20度
	{
		T2    = (((s64)dT)*dT) >> 31;
		Delt  = TEMP - 2000;
		Delt  = Delt * Delt;
		OFF2  = (5 * Delt) >> 1;
		SENS2 = (5 * Delt) >> 2;
		if(TEMP < -1500) // 温度低于-15度
		{
			Delt = TEMP + 1500;
			Delt = Delt * Delt;
			OFF2  += 7 * Delt;
			SENS2 += (11 * Delt) >> 1;
		}
	}
	OFF1 -= OFF2;
	SENS -= SENS2;
	//最终的温度和气压值
	TEMP -= T2;
	P = (((ADC_PRS * SENS) >> 21) - OFF1) >> 15;    //0.01mbar = Pa
	//计算高度
	ALT = (s32)((1.0f - pow(P/MSLP, 0.190295f))*4433000.0f);  //海拔，单位：cm

	//实际值解算
	TmpRel = (float)TEMP/100.0f;
	PrsRel = (float)P/100.0f;
	AltRel = (float)ALT/100.0f;
	Ms5611_Cali();
	AltRel -= AltOff;
	//滤波
	SlideFilt(&TmpFil,&TmpRel,1,&Filt,1);
	SlideFilt(&PrsFil,&PrsRel,1,&Filt,2);
	SlideFilt(&AltFil,&AltRel,1,&Filt,2);

	for(u8 i=1;i<MS_SLOPE_NUM;i++) AltTmp[i-1]=AltTmp[i];
	AltTmp[MS_SLOPE_NUM-1]=AltRel;
	float ab[2];
	LineFit(AltTmp,MS_SLOPE_NUM,ab);  //最小二乘法拟合速率
	AltSlope = ab[0]*50.0f;           //0.02s


	baroAlt.altitude = AltRel;
	baroAlt.altSlope = AltSlope;
	baroAlt.timestamp = startTimer;
	xQueueOverwrite(queueBaroAlt,&baroAlt);

	if(Sta==STA_RUN)Update = true;
	return;

}

//气压计状态机，一毫秒调用一次
void MS5611::Ms5611_Loop_10ms()
{
	if(Delay>0){Delay--;return;}
	startTimerLast = startTimer;
	getTimer_us(&startTimer);
	cycleTime_us = startTimer - startTimerLast;
	if(StaCnt<2)StaCnt++;
	else StaCnt=0;
	switch(StaCnt)
	{
		case 0:
			Ms5611_Spi_TranRecv(hspi,MS5611_CMD_ADC_READ,PrsRaw,3); //读取气压
			Ms5611_Spi_Tran(hspi,MS5611_CMD_D2_4096);              //开始采集温度
			Ms5611_Update();
			break;
		case 1:
			Ms5611_Spi_TranRecv(hspi,MS5611_CMD_ADC_READ,TmpRaw,3);//读取温度
		    Ms5611_Spi_Tran(hspi,MS5611_CMD_D1_4096);break;             //开始采集气压
		default:break;
	}
	getTimer_us(&stopTimer);

	executionTime_us = stopTimer - startTimer;
}

extern "C" void ms5611_main(void *argument)
{
	ms5611.Ms5611_Init();
	for(;;)
	{
		osSemaphoreAcquire(semMs5611,0xffffffff);
		ms5611.Ms5611_Loop_10ms();
	}
}


/************************ (C) COPYRIGHT Longmen Drone Team *****END OF FILE****/
