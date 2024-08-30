/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : rm3100.hpp
  * Description        :
  ******************************************************************************
  * Function           :
  *
  ******************************************************************************
  * Author             : Zengjie Lian
  * Creation Date      : 2021年xx月xx日
  ******************************************************************************
  */
/* USER CODE END Header */

#ifndef __RM3100_HPP
#define __RM3100_HPP
#include "system/system.hpp"
#include "userlib/userlib.hpp"
#include "hspi/hspi.hpp"
#include "led/led.hpp"


#ifdef __cplusplus
extern "C" {
#endif



#define MPU_WHOAMI_RM3100		0x22	//rm3100的ID 暂不知道

#define RM_Single_OUT                      		0x00	//单次测量
#define RM_Continuous_OUT                       0x01	//连续测量模式
#define RM_Cycle_Count_XM                       0x04	//X轴周期计数值- MSB 0x00
#define RM_Cycle_Count_XL                       0x05	//X轴周期计数值- LSB 0xC8;200
#define RM_Cycle_Count_YM                       0x06	//Y轴周期计数值- MSB
#define RM_Cycle_Count_YL                       0x07	//Y轴周期计数值- LSB
#define RM_Cycle_Count_ZM                       0x08	//Z轴周期计数值- MSB
#define RM_Cycle_Count_ZL                       0x09	//Z轴周期计数值- LSB

#define RM_TMRC                     			0x0B	//设置连续测量模式数据速率

#define RM_ALLX_MSB								0x0C	//X轴报警下限 MSB
#define RM_ALLX_MID								0x0D	//X轴报警下限 MID
#define RM_ALLX_LSB								0x0E	//X轴报警下限 LSB
#define RM_AULX_MSB								0x0F	//X轴报警上限 MSB
#define RM_AULX_MID								0x10	//X轴报警上限 MID
#define RM_AULX_LSB								0x11	//X轴报警上限 LSB
#define RM_ALLY_MSB								0x12	//Y轴报警下限 MSB
#define RM_ALLY_MID								0x13	//Y轴报警下限 MID
#define RM_ALLY_LSB								0x14	//Y轴报警下限 LSB
#define RM_AULY_MSB								0x15	//Y轴报警上限 MSB
#define RM_AULY_MID								0x16	//Y轴报警上限 MID
#define RM_AULY_LSB								0x17	//Y轴报警上限 LSB
#define RM_ALLZ_MSB								0x18	//Z轴报警下限 MSB
#define RM_ALLZ_MID								0x19	//Z轴报警下限 MID
#define RM_ALLZ_LSB								0x1A	//Z轴报警下限 LSB
#define RM_AULZ_MSB								0x1B	//Z轴报警上限 MSB
#define RM_AULZ_MID								0x1C	//Z轴报警上限 MID
#define RM_AULZ_LSB								0x1D	//Z轴报警上限 LSB

#define RM_XOUT2			                  	0xA4	//X轴测量(2)(最高位)
#define RM_XOUT1			                  	0xA5	//X轴测量(1)(中间位)
#define RM_XOUT0			                  	0xA6	//X轴测量(0)(最低位)
#define RM_YOUT2			                  	0xA7	//Y轴测量(2)
#define RM_YOUT1			                  	0xA8	//Y轴测量(1)
#define RM_YOUT0			                  	0xA9	//Y轴测量(0)
#define RM_ZOUT2			                  	0xAA	//Z轴测量(2)
#define RM_ZOUT1			                  	0xAB	//Z轴测量(1)
#define RM_ZOUT0			                  	0xAC	//Z轴测量(0)

#define RM_BIST			                  	    0x33	//内置自检
#define RM_STATUS			                    0x34	//DRDY的状态  如果数据可用，第7位是HIGH，如果数据不可用，第7位是LOW
#define RM_HSHAKE								0X35	//握手寄存器
#define RM_REVID								0X36	//MagI2C版本标识

#define RM_ACC_FCT			0.00119708f		//单位换算为：m/s^2，重力是9.8065
#define RM_MAG_FCT			0.160f			//单位换算为：mgauss，磁场是500~600mguass
#define RM_MAG_GAIN			75.0f
//驱动程序：spi4读写RM3100底层程序
#define RM3100_SPI_CS_CLR	LL_GPIO_ResetOutputPin(SPI4_NSS_M_GPIO_Port, SPI4_NSS_M_Pin)
#define RM3100_SPI_CS_SET	LL_GPIO_SetOutputPin(SPI4_NSS_M_GPIO_Port, SPI4_NSS_M_Pin)

#ifdef __cplusplus
}
#endif

#ifdef __cplusplus
using namespace std;

class RM3100 :public SPI,LED
{
public:
	RM3100(SPI_TypeDef *h,char *n,char *s,GPIO_TypeDef *gpiox,uint32_t Pin) : hspi(h),name(n),DirChr(s),led(gpiox,Pin){};
	~RM3100(){}

	bool RM3100_Init();
	void RM3100_Read_Mag();


//	void Butterworth_2nd();
//	void Butterworth_3nd();//
//	void Butterworth_3nd2();//	3nd2，3nd只有定义的值不同

	bool RM3100_Check_MagDataReady();
	bool RM3100_Read_MagData();
	void RM3100_Update_Mag();

	void RM3100_Spi_Tran(SPI_TypeDef *hspi,uint8_t add,uint8_t dat);
	void RM3100_Spi_TranBytes(SPI_TypeDef *hspi,uint8_t add,uint8_t length,uint8_t *dat);
	void RM3100_Spi_TranRecv(SPI_TypeDef *hspi,uint8_t add,uint8_t length,uint8_t *TxData);

private:
	SPI_TypeDef *hspi;  //端口  --
	char *name;
	char *DirChr;
	LED led;
	sTIM  Tim;        //计时器
	sCNT Filt;
	int8_t    Dir[6];     //方向  --

	s32   MagOff[3];
	float	MagMagnitudeFac[3];
	u8 BIST;
	u8 temp;
	u8 ID;
	u8 TMRC;
	u8 test[6];

	u8 rm_dat[9];	  //原始值
	u8 rm_datx[3];	  //原始值
	u8 rm_daty[3];	  //原始值
	u8 rm_datz[3];	  //原始值
	s32   MagRaw[3];  //原始值
	s32   MagRot[3];  //旋转值
	float MagRel[3];  //实际值
	float MagFil[3];  //滤波值

//
//	float GyrFil_2nd[3];  //2阶角速度低通滤波值
//	float AccFil_2nd[3];//2阶低通滤波器
//	float AccFil_3nd[3];//3阶低通滤波器
//	float AccFil_3nd2[3];

	//用户访问数据
	bool  MagUpdate;     //更新  --


//
//	uint32_t startTimer;
//	uint32_t stopTimer;
//	uint32_t  executionTime_us;
//

	uint32_t startTimer;
	uint32_t stopTimer;
	uint32_t  startTimerLast;
	uint32_t  executionTime_us;
	uint32_t cycleTime_us;

	eSTA  Sta;        //状态  --
	eERR  Err;        //错误信息  --
	u32   Time;       //计时


	mag_msg mag;
//	sensor_gyro_filter_msg gyro_filter;
//	sensor_acc_filter_msg acc_filter;

};



#endif


#endif

/************************ (C) COPYRIGHT Longmen Drone Team *****END OF FILE****/
