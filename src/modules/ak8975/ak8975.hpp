/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : ak8975.hpp
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

#ifndef __AK8975_HPP
#define	__AK8975_HPP

#include "spi/spi.hpp"
#include "FreeRTOS.h"
#include "semphr.h"                     // ARM.FreeRTOS::RTOS:Core
#include "system/system.hpp"

#ifdef __cplusplus
extern "C" {
#endif

#define AK8975_ID					0x48

#define AK8975_WIA_REG          0X00
#define AK8975_INFO_REG         0X01
#define AK8975_ST1_REG          0X02
#define AK8975_HXL_REG          0X03
#define AK8975_HXH_REG          0X04
#define AK8975_HYL_REG          0X05
#define AK8975_HYH_REG          0X06
#define AK8975_HZL_REG          0X07
#define AK8975_HZH_REG          0X08
#define AK8975_ST2_REG          0X09
#define AK8975_CNTL_REG         0X0A
#define AK8975_RSV_REG          0X0B
#define AK8975_ASTC_REG         0X0C
#define AK8975_TS1_REG          0X0D
#define AK8975_TS2_REG          0X0E
#define AK8975_I2CDIS_REG       0X0F
#define AK8975_ASAX_REG         0X10
#define AK8975_ASAY_REG         0X11
#define AK8975_ASAZ_REG         0X12

#define AK_MAG_FCT		0.3f		//单位uT

#define AK8975_SPI4_CS_CLR	LL_GPIO_ResetOutputPin(SPI4_NSS_M_GPIO_Port, SPI4_NSS_M_Pin)
#define AK8975_SPI4_CS_SET	LL_GPIO_SetOutputPin(SPI4_NSS_M_GPIO_Port, SPI4_NSS_M_Pin)


#ifdef __cplusplus
}
#endif

#ifdef __cplusplus
using namespace std;
class AK8975 : public SPI
{
public:
	AK8975();
	AK8975(SPI_TypeDef *h,char *n,char *s) : hspi(h),name(n),DirChr(s){};
	~AK8975();
	bool Ak8975_Init();
	bool Ak8975_Read_Magnetometer();

	void Ak8975_Update_Magnetometer();

	void Ak8975_Spi_Tran(SPI_TypeDef *hspi,uint8_t add,uint8_t dat);
	void Ak8975_Spi_TranRecv(SPI_TypeDef *hspi,uint8_t add,uint8_t length,uint8_t *TxData);

private:
	SPI_TypeDef *hspi;  //端口  --
	char *name;
	sTIM  Tim;        //计时器
	bool  MagCal;     //校准
	char *DirChr;
	s8    Dir[6];     //方向  --
	s16   MagOff[3];
	s16   MagRaw[3];  //原始值
	float MagRel[3];  //实际值
	//用户访问数据
	bool  Update;     //更新  --
	bool  Ready;
	eSTA  Sta;        //状态  --
	eERR  Err;        //错误信息  --

	u32   MagUpdate_time_us;       //计时
	float MagUpdate_time;		//加速度计更新时间
	float MagLastUpdate_time;	//上一次加速度计更新时间
	float MagDeltaT_Sample;	//加速度计采集时间

	sensor_mag_msg mag;
};


#endif

#endif



/************************ (C) COPYRIGHT Longmen Drone Team *****END OF FILE****/
