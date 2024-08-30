/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : icm20602.cpp
  * Description        :
  ******************************************************************************
  * Function           :
  *
  ******************************************************************************
  * Author             : Zhiping Fu
  * Creation Date      : 2020年10月10日
  ******************************************************************************
  */
/* USER CODE END Header */

#include "rm3100.hpp"

#define xyz_test		0x70

bool RM3100::RM3100_Init()
{
	MagUpdate = false;
	RM3100_SPI_CS_SET;
	Sta = STA_INI;
	Err = ERR_NONE;

	led.LED_Clr();
	BIST = 0;
	ID = 0;
	TMRC = 0;
	Filt.CNT = 0;
	Filt.CCR = 50;

	if(Dir_Trans(Dir, DirChr) == false) Err = ERR_SOFT; //DirChr="-Y-X-Z"

	MagOff[0] =  221.3623;
   	MagOff[1] =   -177.7987;
	MagOff[2] =   -130.0504;

	for(u8 i=0;i<3;i++)
	{
		MagFil[i] = 0;
	}
	for(u8 i=0;i<6;i++)
	{
		test[i] = 1;
	}
	Sta = STA_CHK;
//	设置循环计数寄存器  XYZ高低位周期值
//	uint8_t CCR[6] = {0,200,0,200,0,200};
//	RM3100_Spi_TranBytes(hspi,RM_Cycle_Count_XM,6,CCR);
//	osDelay(10);
//	RM3100_Spi_Tran(hspi,RM_Single_OUT,0x00);
//	osDelay(10);
	RM3100_Spi_TranRecv(hspi,RM_Cycle_Count_XM,6,test);
	osDelay(10);

	//Read chip ID 读取传感器的版本标识
	RM3100_Spi_TranRecv(hspi,0x36,1,&ID);
	osDelay(10);
	if(ID != MPU_WHOAMI_RM3100)
	{
		Err=ERR_LOST;
		led.LED_Set();
		return false;
	}
//	RM3100_Spi_TranRecv(hspi,RM_HSHAKE,1,&BIST);
//	osDelay(10);
//	//自检
//	RM3100_Spi_TranRecv(hspi,RM_XOUT2,9,rm_dat);//清空DRDY位
//	RM3100_Spi_Tran(hspi,RM_BIST,0x8F);
//	osDelay(10);
//	RM3100_Spi_Tran(hspi,RM_Single_OUT,0x70);
//	osDelay(1000);
//	while(!RM3100_Check_MagDataReady()){Err = ERR_UPDT;};
//	RM3100_Spi_TranRecv(hspi,RM_BIST,1,&BIST);
//	osDelay(10);


//  设置连续测量
//	RM3100_Spi_Tran(hspi,RM_Continuous_OUT,0x55);
//	osDelay(10);
//	RM3100_Spi_Tran(hspi,RM_TMRC,0x9B);	//设置连续测量时间间隔 92:1.7ms,600hz
//	osDelay(10);
//	RM3100_Spi_TranRecv(hspi,RM_TMRC,1,&TMRC);	//设置连续测量时间间隔 1.7ms,600hz
//	osDelay(10);

	RM3100_Spi_Tran(hspi,RM_Single_OUT,xyz_test); 	//发起单个测量(0x00) 0x70=0111 0000,测量Z、Y和X轴。
	osDelay(10);
	Sta = STA_RUN;
	return true;
}

void RM3100::RM3100_Spi_Tran(SPI_TypeDef *hspi,uint8_t add,uint8_t dat)
{
	RM3100_SPI_CS_CLR;
	SPI_ReadWriteByte(hspi,add);
	SPI_ReadWriteByte(hspi,dat);
	RM3100_SPI_CS_SET;
	return;
}

void RM3100::RM3100_Spi_TranBytes(SPI_TypeDef *hspi,uint8_t add,uint8_t length,uint8_t *dat)
{
	RM3100_SPI_CS_CLR;
	SPI_ReadWriteByte(hspi,add);
	for(uint8_t i=0; i<length; i++)
	{
		SPI_ReadWriteByte(hspi,dat[i]);
	}
	RM3100_SPI_CS_SET;
	return;
}

void RM3100::RM3100_Spi_TranRecv(SPI_TypeDef *hspi,uint8_t add,uint8_t length,uint8_t *TxData)
{
	RM3100_SPI_CS_CLR;
	SPI_ReadWriteByte(hspi,add|0x80);//|0x80
	SPI_Recesive(hspi,TxData,length);
	RM3100_SPI_CS_SET;
	return ;
}


bool RM3100::RM3100_Check_MagDataReady()
{
	bool cResult;
	//u8 temp=0;
	temp=0;
	RM3100_Spi_TranRecv(hspi,RM_STATUS,1,&temp); //读取状态寄存器 如果数据可用，第7位是HIGH，如果数据不可用，第7位是LOW。
	cResult = temp&0x80;
	if(cResult == 0x00)return false;
	return true;
}


bool RM3100::RM3100_Read_MagData()
{

	static char cNoDataCnt = 0;

	if (RM3100_Check_MagDataReady() == true)
	{
//		osDelay(10);
		RM3100_Spi_TranRecv(hspi,RM_XOUT2,9,rm_dat);
//		RM3100_Spi_TranRecv(hspi,RM_XOUT2,3,rm_datx);
//		RM3100_Spi_TranRecv(hspi,RM_YOUT2,3,rm_daty);
//		RM3100_Spi_TranRecv(hspi,RM_ZOUT2,3,rm_datz);
		//MagRaw[0]=(long)rm_datx[0]<<16 | (long)rm_datx[1]<<8 | rm_datx[2];//MagRaw[0]=((long)rm_dat[0]<<16) + ((long)rm_dat[1]<<8) + rm_dat[2];
		MagRaw[0]=rm_dat[0]<<16 | rm_dat[1]<<8 | rm_dat[2];//MagRaw[0]=(long)rm_dat[0]<<16 | (long)rm_dat[1]<<8 | rm_dat[2];
		if(MagRaw[0] >= 0x00800000)
		{
			MagRaw[0] |= 0xff000000;
		}
		//MagRaw[1]=(long)rm_daty[0]<<16 | (long)rm_daty[1]<<8 | rm_daty[2];
		MagRaw[1]=rm_dat[3]<<16 | rm_dat[4]<<8 | rm_dat[5];
		if(MagRaw[1] >= 0x00800000)
		{
			MagRaw[1] |= 0xff000000;
		}
		//MagRaw[2]=(long)rm_datz[0]<<16 | (long)rm_datz[1]<<8 | rm_datz[2];
		MagRaw[2]=rm_dat[6]<<16 | rm_dat[7]<<8 | rm_dat[8];
		if(MagRaw[2] >= 0x00800000)
		{
			MagRaw[2] |= 0xff000000;
		}
//		osDelay(10);
		Err = ERR_NONE;
		//rm_dat[0] = 0x70;
		RM3100_Spi_Tran(hspi,RM_Single_OUT,xyz_test);
	}
	else
	{
		cNoDataCnt+=5;
		if (cNoDataCnt>=20)
		{
			cNoDataCnt=0;
			RM3100_Spi_Tran(hspi,RM_Single_OUT,xyz_test);
			Err = ERR_UPDT;
		}

		return false;
	}
	return true;
}

void RM3100::RM3100_Update_Mag()
{
	startTimerLast = startTimer;
	getTimer_us(&startTimer);
	cycleTime_us = startTimer - startTimerLast;

	if(RM3100_Read_MagData() == false) return;
	MagRot[0] =  Dir[0]*(MagRaw[Dir[1]]-MagOff[Dir[1]]);
	MagRot[1] =  Dir[2]*(MagRaw[Dir[3]]-MagOff[Dir[3]]);
	MagRot[2] =  Dir[4]*(MagRaw[Dir[5]]-MagOff[Dir[5]]);

	//实际值MagRel
//	MagRel[0] = (float)Dir[0]*(MagRaw[Dir[1]]-MagOff[Dir[1]])*RM_MAG_FCT;
//	MagRel[1] = (float)Dir[2]*(MagRaw[Dir[3]]-MagOff[Dir[3]])*RM_MAG_FCT;
//	MagRel[2] = (float)Dir[4]*(MagRaw[Dir[5]]-MagOff[Dir[5]])*RM_MAG_FCT;
	MagRel[0] = (MagRot[0]/RM_MAG_GAIN); //-52  53 -1.0046
	MagRel[1] = (MagRot[1]/RM_MAG_GAIN); //-41 55   5.7339	 50~60uT
	MagRel[2] = (MagRot[2]/RM_MAG_GAIN); //-51 48  -4.3005  1uT=0.01G=10mG
//	MagRel[0] = (float)MagRot[0];
//	MagRel[1] = (float)MagRot[1];
//	MagRel[2] = (float)MagRot[2];
	//滤波值


	mag.timestamp = startTimer;
	for(uint8_t i=0;i<3;i++)
	{
		mag.MagRaw[i] = MagRaw[i];		// 猜是原始值？ 要来干啥？
		mag.MagRel[i] = MagRel[i];	//实际值
	}
	xQueueOverwrite(queueMag,&mag);
	MagUpdate=true;

	getTimer_us(&stopTimer);

	executionTime_us = stopTimer - startTimer;

	return;
}


RM3100 rm3100(SPI4,(char *)"RM3100",(char *) "-Y-X-Z",GPIOE,LL_GPIO_PIN_2);

extern "C" void rm3100_main(void *argument)
{
	rm3100.RM3100_Init();
	osDelay(100);
	for(;;)  //无限循环
	{
		osSemaphoreAcquire(semRm3100,0xffffffff);
		rm3100.RM3100_Update_Mag();
	}
}
/************************ (C) COPYRIGHT Longmen Drone Team *****END OF FILE****/
