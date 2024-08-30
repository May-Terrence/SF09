/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : lsm303d.hpp
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
#ifndef __LSM303D_HPP
#define __LSM303D_HPP
#include "system/system.hpp"
#include "hspi/hspi.hpp"


#ifdef __cplusplus
extern "C" {
#endif


//Register Address Map
#define LSM_TEMP_OUT_L		0x05U
#define LSM_TEMP_OUT_H		0x06U
#define LSM_STATUS_M		0x07U
#define LSM_OUT_X_L_M		0x08U
#define LSM_OUT_X_H_M		0x09U
#define LSM_OUT_Y_L_M		0x0AU
#define LSM_OUT_Y_H_M		0x0BU
#define LSM_OUT_Z_L_M		0x0CU
#define LSM_OUT_Z_H_M		0x0DU
#define LSM_WHO_AM_I		0x0FU	//0x49
#define LSM_INT_CTRL_M		0x12U	//0xE8  磁力计中断配置，不启用0x00，感觉默认值应该是0x00
#define LSM_INT_SRC_M		0x13U	//阈值触发配置
#define LSM_INT_THS_L_M		0x14U	//阈值配置
#define LSM_INT_THS_H_M		0x15U
#define LSM_OFFSET_X_L_M	0x16U	//漂移量设置
#define LSM_OFFSET_X_H_M	0x17U
#define LSM_OFFSET_Y_L_M	0x18U
#define LSM_OFFSET_Y_H_M	0x19U
#define LSM_OFFSET_Z_L_M	0x1AU
#define LSM_OFFSET_Z_H_M	0x1BU
#define LSM_REFERENCE_X		0x1CU	//与加速度数据进行高通滤波的参考值
#define LSM_REFERENCE_Y		0x1DU
#define LSM_REFERENCE_Z		0x1EU
#define LSM_CTRL0			0x1FU	//0x00  高通滤波器启用与FIFO启用 0x00
#define LSM_CTRL1			0x20U	//0x07  加速度计频率 加速度各轴使能与更新方式 0x37启用所有，频率为12.5Hz
#define LSM_CTRL2			0x21U	//0x00  加速度计带宽与范围以及自检 0xC8不自检 0xCA自检   50Hz带宽，+-4g量程
#define LSM_CTRL3			0x22U	//0x00  中断配置 0x00
#define LSM_CTRL4			0x23U	//0x00  中断配置 0x00
#define LSM_CTRL5			0x24U	//0x18  温度检测使能 磁力计精度与频率，0xE8 启用温度检测，频率为12.5Hz
#define LSM_CTRL6			0x25U	//0x20  磁力计范围 0x20默认4gauss
#define LSM_CTRL7			0x26U	//0x01  特殊模式 磁力计连续转换 0x80
#define LSM_STATUS_A		0x27U
#define LSM_OUT_X_L_A		0x28U
#define LSM_OUT_X_H_A		0x29U
#define LSM_OUT_Y_L_A		0x2AU
#define LSM_OUT_Y_H_A		0x2BU
#define LSM_OUT_Z_L_A		0x2CU
#define LSM_OUT_Z_H_A		0x2DU
#define LSM_FIFO_CTRL		0x2EU	//FIFO配置
#define LSM_FIFO_SRC		0x2FU	//FIFO状态
#define LSM_IG_CFG1			0x30U	//IG均为中断相关
#define LSM_IG_SRC1			0x31U
#define LSM_IG_THS1			0x32U
#define LSM_IG_DUR1			0x33U
#define LSM_IG_CFG2			0x34U
#define LSM_IG_SRC2			0x35U
#define LSM_IG_THS2			0x36U
#define LSM_IG_DUR2			0x37U
#define LSM_CLICK_CFG		0x38U	//中断相关
#define LSM_CLICK_SRC		0x39U
#define LSM_CLICK_THS		0x3AU
#define LSM_TIME_LIMIT		0x3BU
#define LSM_TIME_LATENCY	0x3CU
#define LSM_TIME_WINDOW		0x3DU
#define LSM_Act_THS			0x3EU
#define LSM_Act_DUR			0x3FU

#define LSM303D_ADDR		0x3C

#define LSM_ACC_FCT			0.00119708f		//单位换算为：m/s^2，重力是9.8065
#define LSM_MAG_FCT			0.160f			//单位换算为：mgauss，磁场是500~600mguass


//驱动程序：spi1读写LSM303D底层程序
#define LSM303D_SPI_CS_CLR	LL_GPIO_ResetOutputPin(SPI4_NSS_AM_GPIO_Port, SPI4_NSS_AM_Pin)
#define LSM303D_SPI_CS_SET	LL_GPIO_SetOutputPin(SPI4_NSS_AM_GPIO_Port, SPI4_NSS_AM_Pin)


#ifdef __cplusplus
}
#endif

#ifdef __cplusplus

//#include <Eigen>

class LSM303D :public SPI
{
public:
	LSM303D(){}
	LSM303D(SPI_TypeDef *h,char *n,char *s):hspi(h),name(n),DirChr(s){}
	~LSM303D(){}
	void lsm_Init();
	void lsm303d_Spi_Tran(SPI_TypeDef *hspi,uint8_t add,uint8_t dat);
	void lsm303d_Spi_TranRecv(SPI_TypeDef *hspi,uint8_t add,uint8_t length,uint8_t *TxData);
	bool lsm_Read();
	void lsm_Update();


private:
	SPI_TypeDef *hspi; //端口  --
	char *name;
	sTIM  Tim;			//计时器
	sCNT  Filt;
	char  *DirChr;
	s8    Dir[6];		//方向  --
	s16   AccOff[3];	//漂移量  --
	s16   MagOff[3];	//偏移量  --
	s16   AccRaw[3];	//原始值
	s16   MagRaw[3];	//原始值
	s16   TmpRaw[1];	//原始值
	float AccRel[3];	//实际值
	float MagRel[3];	//实际值
	float MagFil[3];	//实际值
	float TmpRel[1];	//实际值
	//用户访问数据
	bool  Update;		//更新  --
	eSTA  Sta;			//状态  --
	eERR  Err;			//错误信息  --

	u32   MagUpdate_time_us;       //计时
	float MagUpdate_time;		//加速度计更新时间
	float MagLastUpdate_time;	//上一次加速度计更新时间
	float MagDeltaT_Sample;	//加速度计采集时间

	sensor_mag_msg mag;
};


#endif


#endif


/************************ (C) COPYRIGHT Longmen Drone Team *****END OF FILE****/
