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

bool ICM20602::Icm20602_Init()
{
	GyrUpdate = false;
	AccUpdate = false;
	Sta = STA_INI;
	Err = ERR_NONE;

	led.LED_Clr();


	AccOff[0] = -0.075616;	//-0.113787 0828_0 // -0.075616  0828_2
	AccOff[1] = 0.017159;//0.029446//0.017159
	AccOff[2] = 0.169008;//-0.170574//0.169008
	AccMagnitudeFac[0] = 0.999803;//0.999005//0.999803
	AccMagnitudeFac[1] = 0.999389;//0.997869//0.999389
	AccMagnitudeFac[2] = 0.996976;//0.997713//0.996976
	GyrOff[0] =    0.0027;//0.0109 //0.0027
	GyrOff[1] =    0.0103;//-0.0109//0.0103
	GyrOff[2] =    0.0034;//0.00547//0.0034

	Filt.CNT = 0;
	Filt.CCR = 500;
	Gyro_Filt.CNT=0;
	Gyro_Filt.CCR=10;

	if(Dir_Trans(Dir, DirChr) == false) Err = ERR_SOFT;

	for(u8 i=0;i<3;i++)
	{
		AccFil[i] = 0;
		GyrFil[i] = 0;
	}

	//Reset the internal registers and restores the default settings
	Icm20602_Spi_Tran(hspi,MPU_RA_PWR_MGMT_1,0x80);
	osDelay(10);
	//It is required that CLKSEL[2:0] be set to 001 to achieve full gyroscope performance.
	Icm20602_Spi_Tran(hspi,MPU_RA_PWR_MGMT_1,0x01);
	osDelay(10);

	Sta = STA_CHK;
//	u8 ID;
	//Read chip ID
	Icm20602_Spi_TranRecv(hspi,MPUREG_WHOAMI|0x80,1,&ID);		//读取传感器的ID号

	if((ID != MPU_WHOAMI_20602_1)&&(ID!= MPU_WHOAMI_20602_2))
	{
		Err=ERR_LOST;
		led.LED_Set();
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
	/*设置采样频率为1KHz，硬件低通滤波250Hz*/
	Icm20602_Spi_Tran(hspi,MPU_RA_SMPLRT_DIV,0);
	osDelay(10);
	Icm20602_Spi_Tran(hspi,MPU_RA_CONFIG,ICM20602_LPF_20HZ);
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

void ICM20602::Butterworth_2nd()
{
	//15HZ..19.10.28
	#define a0 0.2928932188134524f
	#define a1 0.5857864376269048f
	#define a2 0.2928932188134524f
	#define b1 -0.00000000000000013007f
	#define b2  0.1715728752538099588509f

	static float A_last[3] = {0,0,0};
	static float A_last_last[3] = {0,0,0};
	static float B_last[3] = {0,0,0};
	static float B_last_last[3] = {0,0,0};

	for(int i=0;i<3;i++)
	{
		GyrFil_2nd[i] = a0 * GyrRel[i] + a1 * A_last[i] + a2 * A_last_last[i] - b1 * B_last[i] - b2 * B_last_last[i];
	}

	for(int i=0;i<3;i++)
	{
		A_last_last[i] = A_last[i];
		A_last[i] = GyrRel[i];

		B_last_last[i] = B_last[i];
		B_last[i] = GyrFil_2nd[i];
	}
}

void ICM20602::Butterworth_3nd()
{
	//XY 10HZ   Z 10HZ   19.10.30
	#define a0_3nd 0.002898194633721429703393512866682613094f
	#define a1_3nd 0.008694583901164289543861407594249612885f
	#define a2_3nd 0.008694583901164289543861407594249612885f
	#define a3_3nd 0.002898194633721429703393512866682613094f
	#define b1_3nd -2.374094743709352250959909724770113825798f
	#define b2_3nd  1.929355669091215252919369049777742475271f
	#define b3_3nd -0.532075368312091900868665561574744060636f
	static float A_last_3nd[3] = {0,0,0};
	static float A_last_last_3nd[3] = {0,0,0};
	static float A_last_last_last_3nd[3] = {0,0,0};
	static float B_last_3nd[3] = {0,0,0};
	static float B_last_last_3nd[3] = {0,0,0};
	static float B_last_last_last_3nd[3] = {0,0,0};
	#define a0_3nd2 0.002898194633721429703393512866682613094f
	#define a1_3nd2 0.008694583901164289543861407594249612885f
	#define a2_3nd2 0.008694583901164289543861407594249612885f
	#define a3_3nd2 0.002898194633721429703393512866682613094f
	#define b1_3nd2 -2.374094743709352250959909724770113825798f
	#define b2_3nd2  1.929355669091215252919369049777742475271f
	#define b3_3nd2 -0.532075368312091900868665561574744060636f

	for(int i=0;i<2;i++)
	{
		AccFil_3nd[i] = a0_3nd * AccRel[i] + a1_3nd * A_last_3nd[i] + a2_3nd * A_last_last_3nd[i] + a3_3nd * A_last_last_last_3nd[i] - b1_3nd * B_last_3nd[i] - b2_3nd * B_last_last_3nd[i] - b3_3nd * B_last_last_last_3nd[i];
	}
		AccFil_3nd[2] = a0_3nd2 * AccRel[2] + a1_3nd2 * A_last_3nd[2] + a2_3nd2 * A_last_last_3nd[2] + a3_3nd2 * A_last_last_last_3nd[2] - b1_3nd2 * B_last_3nd[2] - b2_3nd2 * B_last_last_3nd[2] - b3_3nd2 * B_last_last_last_3nd[2];

	for(int i=0;i<3;i++)
	{
		A_last_last_last_3nd[i] = A_last_last_3nd[i];
		A_last_last_3nd[i] = A_last_3nd[i];
		A_last_3nd[i] = AccRel[i];

		B_last_last_last_3nd[i] = B_last_last_3nd[i];
		B_last_last_3nd[i] = B_last_3nd[i];
		B_last_3nd[i] = AccFil_3nd[i];
	}
}


void ICM20602::Butterworth_3nd2()
{
	#define a0_3nd3 0.000003756838019f
	#define a1_3nd3 0.0000112705140592f
	#define a2_3nd3 0.0000112705140592f
	#define a3_3nd3 0.0000037568380197f
	#define b1_3nd3 -2.937170728449890f
	#define b2_3nd3  2.876299723479331f
	#define b3_3nd3 -0.939098940325282f
	static float A_last_3nd2[3] = {0,0,0};
	static float A_last_last_3nd2[3] = {0,0,0};
	static float A_last_last_last_3nd2[3] = {0,0,0};
	static float B_last_3nd2[3] = {0,0,0};
	static float B_last_last_3nd2[3] = {0,0,0};
	static float B_last_last_last_3nd2[3] = {0,0,0};
	#define a0_3nd4 0.000029146494465697653702745614778812921f
	#define a1_3nd4 0.000087439483397092957720105055319237408f
	#define a2_3nd4 0.000087439483397092957720105055319237408f
	#define a3_3nd4 0.000029146494465697653702745614778812921f
	#define b1_3nd4 -2.874356892677484509590613015461713075638f
	#define b2_3nd4  2.756483195225695403962618001969531178474f
	#define b3_3nd4 -0.881893130592485419150250436359783634543f
	for(int i=0;i<2;i++)
	{
		AccFil_3nd2[i] = a0_3nd3 * AccRel[i] + a1_3nd3 * A_last_3nd2[i] + a2_3nd3 * A_last_last_3nd2[i] + a3_3nd3 * A_last_last_last_3nd2[i] - b1_3nd3 * B_last_3nd2[i] - b2_3nd3 * B_last_last_3nd2[i] - b3_3nd3 * B_last_last_last_3nd2[i];
	}
		AccFil_3nd2[2] = a0_3nd4 * AccRel[2] + a1_3nd4 * A_last_3nd2[2] + a2_3nd4 * A_last_last_3nd2[2] + a3_3nd4 * A_last_last_last_3nd2[2] - b1_3nd4 * B_last_3nd2[2] - b2_3nd4 * B_last_last_3nd2[2] - b3_3nd4 * B_last_last_last_3nd2[2];

	for(int i=0;i<3;i++)
	{
		A_last_last_last_3nd2[i] = A_last_last_3nd2[i];
		A_last_last_3nd2[i] = A_last_3nd2[i];
		A_last_3nd2[i] = AccRel[i];

		B_last_last_last_3nd2[i] = B_last_last_3nd2[i];
		B_last_last_3nd2[i] = B_last_3nd2[i];
		B_last_3nd2[i] = AccFil_3nd2[i];
	}
}


void ICM20602::Icm20602_Update_Acceleration()
{
	startTimerLast = startTimer;
	getTimer_us(&startTimer);
	cycleTime_us = startTimer - startTimerLast;
	//获取寄存器数据
	Icm20602_Read_Acceleration();

	AccRot[0] =  Dir[0]*(AccRaw[Dir[1]]);
	AccRot[1] =  Dir[2]*(AccRaw[Dir[3]]);
	AccRot[2] =  Dir[4]*(AccRaw[Dir[5]]);

	AccRel[0] = AccMagnitudeFac[0]*(AccRot[0]*ICM_ACC_FCT - AccOff[0]);
	AccRel[1] = AccMagnitudeFac[1]*(AccRot[1]*ICM_ACC_FCT - AccOff[1]);
	AccRel[2] = AccMagnitudeFac[2]*(AccRot[2]*ICM_ACC_FCT - AccOff[2]);

	SlideFilt(AccFil,AccRel,3,&Filt,1);
//	Butterworth_3nd2();
//	Butterworth_3nd();

	AccUpdate_time = (float)AccUpdate_time_us/1000000.0f;
	AccDeltaT_Sample = AccUpdate_time - AccLastUpdate_time;
	AccLastUpdate_time = AccUpdate_time;

	acc.timestamp = AccUpdate_time_us;
	acc.dt = AccDeltaT_Sample;
	for (int i=0;i<3;++i)
	{
		jerkRaw[i] = (AccFil[i] - AccFilLast[i])/dt;
		jerkFil[i]    = (jerkFil[i]*(500-1) + jerkRaw[i])/500;//滤波
	}
	for(uint8_t i=0;i<3;i++)
	{
		acc.acc[i] = AccRel[i];
		acc_filter.acc_filter[i] = AccFilLast[i] = AccFil[i];
		acc_filter.jerk_filter[i] = jerkFil[i];
		downsample_imu.acc[i] = (downsample_imu.acc[i]*sample_cnt+AccRel[i])/(sample_cnt+1);
	}
	xQueueOverwrite(queueAccDat,&acc);
	xQueueOverwrite(queueAccDatFil,&acc_filter);
	AccUpdate=true;
	return;
}

void ICM20602::Icm20602_Update_Gyro()
{
	//获取寄存器数据
	Icm20602_Read_Gyro();
	//获取温度和温飘值
	TmpRel[0]=(float)TmpRaw[0]/326.8f+25.0f;

	GyrRot[0] =  Dir[0]*(GyrRaw[Dir[1]]);
	GyrRot[1] =  Dir[2]*(GyrRaw[Dir[3]]);
	GyrRot[2] =  Dir[4]*(GyrRaw[Dir[5]]);

	GyrRel[0] = (GyrRot[0])*ICM_GYR_FCT - GyrOff[0];
	GyrRel[1] = (GyrRot[1])*ICM_GYR_FCT - GyrOff[1];
	GyrRel[2] = (GyrRot[2])*ICM_GYR_FCT - GyrOff[2];

	SlideFilt(GyrFil,GyrRel,3,&Gyro_Filt,1);
//	Butterworth_2nd();

	GyroUpdate_time = (float)GyroUpdate_time_us/1000000.0f;
	GyroDeltaT_Sample = GyroUpdate_time - GyroLastUpdate_time;
	GyroLastUpdate_time = GyroUpdate_time;

	gyro.timestamp = GyroUpdate_time_us;
	downsample_imu.timestamp = GyroUpdate_time_us;
	gyro.dt = GyroDeltaT_Sample;
	gyro.temperature = TmpRel[0];
	for(uint8_t i=0;i<3;i++)
	{
		gyro.gyro[i] = GyrRel[i];
//		gyro_filter.gyro_filter[i] = GyrFil_2nd[i];
		gyro_filter.gyro_filter[i] = GyrFil[i];
		downsample_imu.gyro[i] = (downsample_imu.gyro[i]*sample_cnt+GyrRel[i])/(sample_cnt+1);
	}
	sample_cnt += 1;
	downsample_imu.sample_cnt = &sample_cnt;

	xQueueOverwrite(queueGyrDat,&gyro);
	xQueueOverwrite(queueGyrDatFil,&gyro_filter);
	xQueueOverwrite(queueDownsampleIMU,&downsample_imu);


	GyrUpdate=true;
	getTimer_us(&stopTimer);

	executionTime_us = stopTimer - startTimer;
	return;
}



ICM20602 icm20602(SPI4,(char *)"ICM20602",(char *) "+X+Y+Z",GPIOE,LL_GPIO_PIN_2);

extern "C" void Icm20602_main(void *argument)
{
	icm20602.Icm20602_Init();
	for(;;)
	{
		osSemaphoreAcquire(semIcm20602,0xffffffff);
		icm20602.Icm20602_Update_Acceleration();
		icm20602.Icm20602_Update_Gyro();

	}
}
/************************ (C) COPYRIGHT Longmen Drone Team *****END OF FILE****/
