/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : ak8975.cpp
  * Description        : 
  ******************************************************************************
  * Function           :
  *
  ******************************************************************************
  * Author             : Zhiping Fu
  * Creation Date      : Oct 19, 2020
  ******************************************************************************
  */
/* USER CODE END Header */

#include "ak8975.hpp"

AK8975::AK8975()
{

}

AK8975::~AK8975()
{

}

bool AK8975::Ak8975_Init()
{
	Update = false;
	Err = ERR_NONE;
	Ready = false;
	if(Dir_Trans(Dir, DirChr) == false) Err = ERR_SOFT;
	MagOff[0] = 20;
	MagOff[1] = 63;
	MagOff[2] = 73;
	osDelay(50);
	Sta = STA_CHK;
	u8 ID;
	u8 info;
	Ak8975_Spi_TranRecv(hspi,AK8975_WIA_REG,1,&ID);		//读取传感器的ID号
	Ak8975_Spi_TranRecv(hspi,AK8975_WIA_REG,1,&ID);		//读取传感器的ID号
	Ak8975_Spi_TranRecv(hspi,AK8975_INFO_REG,1,&info);		//读取传感器的INFO
	if(ID != AK8975_ID)
	{
		Err=ERR_LOST;
		return false;
	}
	Ak8975_Spi_Tran(hspi,AK8975_CNTL_REG,0x01);

	Sta = STA_RUN;

	return true;
}


void AK8975::Ak8975_Spi_Tran(SPI_TypeDef *hspi,uint8_t add,uint8_t dat)
{
	AK8975_SPI4_CS_CLR;
	SPI_ReadWriteByte(hspi,add);
	SPI_ReadWriteByte(hspi,dat);
	AK8975_SPI4_CS_SET;
	return;
}

void AK8975::Ak8975_Spi_TranRecv(SPI_TypeDef *hspi,uint8_t add,uint8_t length,uint8_t *TxData)
{
	AK8975_SPI4_CS_CLR;
	SPI_ReadWriteByte(hspi,add|0x80);
	SPI_Recesive(hspi,TxData,length);
	AK8975_SPI4_CS_SET;
	return;
}

bool AK8975::Ak8975_Read_Magnetometer()
{
	u8 ak_dat[6];
	u8 temp;
	Ak8975_Spi_TranRecv(hspi,AK8975_ST1_REG,1,&temp);
	Ready = 0x01&temp;

	if(Ready)
	{
		Ak8975_Spi_TranRecv(hspi,AK8975_HXL_REG,6,ak_dat);
		//获取陀螺仪采集时间
		getTimer_us(&MagUpdate_time_us);
		MagRaw[0] = (ak_dat[1]<<8) + ak_dat[0];
		MagRaw[1] = (ak_dat[3]<<8) + ak_dat[2];
		MagRaw[2] = (ak_dat[5]<<8) + ak_dat[4];

		Ak8975_Spi_Tran(hspi,AK8975_CNTL_REG,0x01);
	}
	else
		return false;

	return true;
}

void AK8975::Ak8975_Update_Magnetometer()
{
	if(Ak8975_Read_Magnetometer() == false) return;

	MagRel[0] = (float)Dir[0]*(MagRaw[Dir[1]]-MagOff[Dir[1]])*AK_MAG_FCT;
	MagRel[1] = (float)Dir[2]*(MagRaw[Dir[3]]-MagOff[Dir[3]])*AK_MAG_FCT;
	MagRel[2] = (float)Dir[4]*(MagRaw[Dir[5]]-MagOff[Dir[5]])*AK_MAG_FCT;
//	MagRel[0] = (float)Dir[0]*(MagRaw[Dir[1]]);
//	MagRel[1] = (float)Dir[2]*(MagRaw[Dir[3]]);
//	MagRel[2] = (float)Dir[4]*(MagRaw[Dir[5]]);

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

AK8975 ak8975(SPI4,(char *)"AK8975",(char *) "+Y+X-Z");
extern "C" void Ak8975_main_mag(void *argument)
{
	ak8975.Ak8975_Init();
	osDelay(10);
	for(;;)
	{
		osSemaphoreAcquire(semAk8975Mag,0xffffffff);
		ak8975.Ak8975_Update_Magnetometer();
	}
}


/************************ (C) COPYRIGHT Longmen Drone Team *****END OF FILE****/
