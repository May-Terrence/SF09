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

#include "icm20602.hpp"

ICM20602::ICM20602()
{


}

ICM20602::~ICM20602()
{

}

bool ICM20602::Icm20602_Init()
{
	GyrUpdate = false;
	AccUpdate = false;
	Sta = STA_INI;
	Err = ERR_NONE;

	AccOff[0] = -57;
	AccOff[1] = -38;
	AccOff[2] = 27;

	GyrOff[0] = -2;
	GyrOff[1] = 9;
	GyrOff[2] = -13;

	if(Dir_Trans(Dir, DirChr) == false) Err = ERR_SOFT;
	OneG = 9.8621491f;

	//Reset the internal registers and restores the default settings
	Icm20602_Spi_Tran(hspi,MPU_RA_PWR_MGMT_1,0x80);
	osDelay(10);
	//It is required that CLKSEL[2:0] be set to 001 to achieve full gyroscope performance.
	Icm20602_Spi_Tran(hspi,MPU_RA_PWR_MGMT_1,0x01);
	osDelay(10);

	Sta = STA_CHK;
	u8 ID;
	//Read chip ID
	Icm20602_Spi_TranRecv(hspi,MPUREG_WHOAMI|0x80,1,&ID);		//读取传感器的ID号

	if(ID != MPU_WHOAMI_20602)
	{
		Err=ERR_LOST;
		return false;
	}

	/*复位加速度计和温度计reg*/
	Icm20602_Spi_Tran(hspi,MPU_RA_SIGNAL_PATH_RESET,0x03);
	osDelay(10);
  /*复位陀螺仪reg*/
	Icm20602_Spi_Tran(hspi,MPU_RA_USER_CTRL,0x01);
	osDelay(10);

	//Disable I2C Slave module and put the serial interface in SPI mode only.
	Icm20602_Spi_Tran(hspi,0x70,0x40);
	osDelay(10);
	//开启加速度计和陀螺仪
	Icm20602_Spi_Tran(hspi,MPU_RA_PWR_MGMT_2,0x00);
	osDelay(10);
	/*设置采样频率为8KHz，硬件低通滤波250Hz*/
	Icm20602_Spi_Tran(hspi,MPU_RA_CONFIG,ICM20602_LPF_250HZ);
	osDelay(10);
	Icm20602_Spi_Tran(hspi,MPU_RA_GYRO_CONFIG,(3 << 3));	//陀螺仪量程正负2000dps
	osDelay(10);
	Icm20602_Spi_Tran(hspi,MPU_RA_ACCEL_CONFIG,(2 << 3));	//加速度计量程正负8g
	osDelay(10);
	/*加速度计LPF 10HZ*/
	Icm20602_Spi_Tran(hspi,MPU_RA_ACCEL_CONFIG2,0x05);
	osDelay(10);
	/*关闭低功耗*/
	Icm20602_Spi_Tran(hspi,MPU_RA_LP_MODE_CFG,0x00);
	osDelay(10);
	/*关闭FIFO*/
	Icm20602_Spi_Tran(hspi,MPU_RA_FIFO_EN,0x00);
	osDelay(10);

	Sta = STA_RUN;

	return true;

}

void ICM20602::Icm20602_Spi_Tran(SPI_TypeDef *hspi,uint8_t add,uint8_t dat)
{
	ICM20602_SPI_CS_CLR;
	SPI_ReadWriteByte(hspi,add);
	SPI_ReadWriteByte(hspi,dat);
	ICM20602_SPI_CS_SET;
	return;
}

void ICM20602::Icm20602_Spi_TranRecv(SPI_TypeDef *hspi,uint8_t add,uint8_t length,uint8_t *TxData)
{
	ICM20602_SPI_CS_CLR;
	SPI_ReadWriteByte(hspi,add|0x80);
	SPI_Recesive(hspi,TxData,length);
	ICM20602_SPI_CS_SET;
	return ;
}

void ICM20602::Icm20602_Read_Acceleration()
{
	u8 icm_dat[6];
	Icm20602_Spi_TranRecv(hspi,MPU_RA_ACCEL_XOUT_H,6,icm_dat);
	//获取加速度计采集时间
	getTimer_us(&AccUpdate_time_us);
	AccRaw[0] = (icm_dat[0]<<8) + icm_dat[1];
	AccRaw[1] = (icm_dat[2]<<8) + icm_dat[3];
	AccRaw[2] = (icm_dat[4]<<8) + icm_dat[5];

	return;
}

void ICM20602::Icm20602_Read_Gyro()
{
	u8 icm_dat[8];
	Icm20602_Spi_TranRecv(hspi,MPU_RA_TEMP_OUT_H,8,icm_dat);
	//获取陀螺仪采集时间
	getTimer_us(&GyroUpdate_time_us);
	TmpRaw[0] = (icm_dat[0]<<8) + icm_dat[1];
	GyrRaw[0] = (icm_dat[2]<<8) + icm_dat[3];
	GyrRaw[1] = (icm_dat[4]<<8) + icm_dat[5];
	GyrRaw[2] = (icm_dat[6]<<8) + icm_dat[7];

	return;
}

void ICM20602::Icm20602_filter()
{
	for(int i=0;i<2;i++)
	{
		GyrFil_2nd[i] = a0_2nd * GyrRel[i] + a1_2nd * A_last[i] + a2_2nd * A_last_last[i] - b1_2nd * B_last[i] - b2_2nd * B_last_last[i];
	}
	GyrFil_2nd[2] = a0_2nd2 * GyrRel[2] + a1_2nd2 * A_last[2] + a2_2nd2 * A_last_last[2] - b1_2nd2 * B_last[2] - b2_2nd2 * B_last_last[2];

	for(int i=0;i<3;i++)
	{
		A_last_last[i] = A_last[i];
		A_last[i] = GyrRel[i];

		B_last_last[i] = B_last[i];
		B_last[i] = GyrFil_2nd[i];
	}
}

void ICM20602::Icm20602_Update_Acceleration()
{
	//获取寄存器数据
	Icm20602_Read_Acceleration();

	AccRel[0] = -Dir[0]*(AccRaw[Dir[1]]-AccOff[Dir[1]])*ICM_ACC_FCT;
	AccRel[1] = -Dir[2]*(AccRaw[Dir[3]]-AccOff[Dir[3]])*ICM_ACC_FCT;
	AccRel[2] = -Dir[4]*(AccRaw[Dir[5]]-AccOff[Dir[5]])*ICM_ACC_FCT;

	AccUpdate_time = (float)AccUpdate_time_us/1000000.0f;
	AccDeltaT_Sample = AccUpdate_time - AccLastUpdate_time;
	AccLastUpdate_time = AccUpdate_time;

	acc.timestamp = AccUpdate_time;
	acc.dt = AccDeltaT_Sample;
	for(uint8_t i=0;i<3;i++)
		acc.acc[i] = AccRel[i];
	xQueueOverwrite(queueAccDat,&acc);

	AccUpdate=true;
	return;
}

void ICM20602::Icm20602_Update_Gyro()
{
	getTimer_us(&startTimer);
	//获取寄存器数据
	Icm20602_Read_Gyro();
	//获取温度和温飘值
	TmpRel[0]=(float)TmpRaw[0]/326.8f+25.0f;
	GyrRel[0] = Dir[0]*(GyrRaw[Dir[1]]-GyrOff[Dir[1]])*ICM_GYR_FCT;
	GyrRel[1] = Dir[2]*(GyrRaw[Dir[3]]-GyrOff[Dir[3]])*ICM_GYR_FCT;
	GyrRel[2] = Dir[4]*(GyrRaw[Dir[5]]-GyrOff[Dir[5]])*ICM_GYR_FCT;

	Icm20602_filter();

	GyroUpdate_time = (float)GyroUpdate_time_us/1000000.0f;
	GyroDeltaT_Sample = GyroUpdate_time - GyroLastUpdate_time;
	GyroLastUpdate_time = GyroUpdate_time;

	gyro.timestamp = GyroUpdate_time;
	gyro.dt = GyroDeltaT_Sample;
	gyro.temperature = TmpRel[0];
	for(uint8_t i=0;i<3;i++)
		gyro.gyro[i] = GyrRel[i];
	xQueueOverwrite(queueGyrDat,&gyro);


	GyrUpdate=true;
	getTimer_us(&stopTimer);

	executionTime_us = stopTimer - startTimer;
	return;
}



ICM20602 icm20602(SPI4,(char *)"ICM20602",(char *) "-Y+X+Z");

extern "C" void Icm20602_main_acc(void *argument)
{
	osDelay(150);
	for(;;)
	{
		osSemaphoreAcquire(semIcm20602Acc,0xffffffff);
			icm20602.Icm20602_Update_Acceleration();
	}
}

extern "C" void Icm20602_main_gyro(void *argument)
{
	icm20602.Icm20602_Init();
	for(;;)
	{
		osSemaphoreAcquire(semIcm20602Gyr,0xffffffff);
		icm20602.Icm20602_Update_Gyro();

	}
}
/************************ (C) COPYRIGHT Longmen Drone Team *****END OF FILE****/
