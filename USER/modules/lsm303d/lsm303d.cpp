/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : lsm303d.cpp
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
#include "lsm303d.hpp"

void LSM303D::lsm_Init()
{
	Update = false;
	Sta = STA_INI;
	Err = ERR_NONE;

	Filt.CNT = 0;
	Filt.CCR = 5;

	if(Dir_Trans(Dir, DirChr) == false) Err = ERR_SOFT;

	AccOff[0] = 0;
	AccOff[1] = 0;
	AccOff[2] = 0;

	MagOff[0] = -411;
	MagOff[1] = -250;
	MagOff[2] = 1814;

	MagFil[0] = 0;
	MagFil[1] = 0;
	MagFil[2] = 0;


	osDelay(100);

	lsm303d_Spi_Tran(hspi, LSM_CTRL1, 0x67);//持续模式，100HZ
	lsm303d_Spi_Tran(hspi, LSM_CTRL2, 0xC8);//50Hz带宽，+-4g
	lsm303d_Spi_Tran(hspi, LSM_CTRL5, 0xF0);//50Hz带宽
	lsm303d_Spi_Tran(hspi, LSM_CTRL6, 0x00);//+-2gauss
	lsm303d_Spi_Tran(hspi, LSM_CTRL7, 0x80);

	Sta = STA_CHK;
	u8 ID;
	lsm303d_Spi_TranRecv(hspi, LSM_WHO_AM_I, 1, &ID);
	if(ID != 0x49)
	{
		Err = ERR_LOST;
		return;
	}

	Sta = STA_RUN;


}

void LSM303D::lsm303d_Spi_Tran(SPI_TypeDef *hspi,uint8_t add,uint8_t dat)
{
	LSM303D_SPI_CS_CLR;
	SPI_ReadWriteByte(hspi,add);
	SPI_ReadWriteByte(hspi,dat);
	LSM303D_SPI_CS_SET;
	return;
}

void LSM303D::lsm303d_Spi_TranRecv(SPI_TypeDef *hspi,uint8_t add,uint8_t length,uint8_t *TxData)
{
	LSM303D_SPI_CS_CLR;
	SPI_ReadWriteByte(hspi,add|0xC0);
	SPI_Recesive(hspi,TxData,length);
	LSM303D_SPI_CS_SET;
	return ;
}


bool LSM303D::lsm_Read()
{
	u8 temp;
	lsm303d_Spi_TranRecv(hspi, LSM_STATUS_M, 1, &temp);

	temp = temp&0x08;
	if(temp == 0x00)return false;
	u8 lsm_dat[6];

	lsm303d_Spi_TranRecv(hspi, LSM_OUT_X_L_M, 6, lsm_dat);

	MagRaw[0] = (lsm_dat[1]<<8) + lsm_dat[0];
	MagRaw[1] = (lsm_dat[3]<<8) + lsm_dat[2];
	MagRaw[2] = (lsm_dat[5]<<8) + lsm_dat[4];
//	TmpRaw[0] = (lsm_dat[7]<<8) + lsm_dat[6];
//	AccRaw[0] = (lsm_dat[9]<<8) + lsm_dat[8];
//	AccRaw[1] = (lsm_dat[11]<<8) + lsm_dat[10];
//	AccRaw[2] = (lsm_dat[13]<<8) + lsm_dat[12];
	return true;

}

void LSM303D::lsm_Update()
{
	if(lsm_Read() == false) return;



	MagRel[0] = (float)Dir[0]*(MagRaw[Dir[1]]-MagOff[Dir[1]])*LSM_MAG_FCT;
	MagRel[1] = (float)Dir[2]*(MagRaw[Dir[3]]-MagOff[Dir[3]])*LSM_MAG_FCT;
	MagRel[2] = (float)Dir[4]*(MagRaw[Dir[5]]-MagOff[Dir[5]])*LSM_MAG_FCT;

	SlideFilt(MagFil, MagRel, 3, &Filt, 2);

	getTimer_us(&MagUpdate_time_us);

	MagUpdate_time = (float)MagUpdate_time_us/1000000.0f;
	MagDeltaT_Sample = MagUpdate_time - MagLastUpdate_time;
	MagLastUpdate_time = MagUpdate_time;

	mag.timestamp = MagUpdate_time;
	mag.dt = MagDeltaT_Sample;
	for(uint8_t i=0;i<3;i++)
		mag.mag[i] = MagRel[i];
	xQueueOverwrite(queueMagDat,&mag);

	Update=true;
	return;

}

LSM303D lsm303d(SPI4,(char *)"lsm303d",(char *)"-Y-X-Z");
extern "C" void lsm303d_main(void *argument)
{
	lsm303d.lsm_Init();
	osDelay(100);
	for(;;)
	{
		osSemaphoreAcquire(semLsm303d,0xffffffff);
		lsm303d.lsm_Update();
	}
}


/************************ (C) COPYRIGHT Longmen Drone Team *****END OF FILE****/
