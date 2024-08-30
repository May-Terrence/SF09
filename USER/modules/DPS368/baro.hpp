/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : DPS368.hpp
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
#ifndef __DPS368_HPP
#define __DPS368_HPP
#include "hspi/hspi.hpp"
#include "system/system.hpp"
#include "spi.h"


#ifdef __cplusplus
extern "C" {
#endif


#define DPS368_CMD_PSR_READ2   	0x80U // 读取ADC
#define DPS368_CMD_PSR_READ1  	0x81U // 读取ADC
#define DPS368_CMD_PSR_READ0   	0x82U // 读取ADC
#define DPS368_CMD_TMP_READ2   	0x83U // 读取ADC
#define DPS368_CMD_TMP_READ1  	0x84U // 读取ADC
#define DPS368_CMD_TMP_READ0   	0x85U // 读取ADC
#define DPS368_CMD_PRS_CFG		0x06U //
#define DPS368_CMD_TMP_CFG		0x07U //
#define DPS368_CMD_MEAS_CFG_R	0x88U //
#define DPS368_CMD_MEAS_CFG_W	0x08U //
#define DPS368_CMD_CFG_REG		0x09U //
#define DPS368_CMD_INT_STS		0x8AU //
#define DPS368_CMD_FIFO_STS		0x8BU //
#define DPS368_CMD_RESET		0x0CU // 复位
#define DPS368_CMD_ID			0x8DU // ID
#define DPS368_CMD_COEF			0x90U // K
#define DPS368_CMD_RESERVED		0xA2U //
#define DPS368_CMD_COEF_SRCE	0xA8U //

#define MSLP	 101325.0f			// 平均海平面气压 = 1013.25 hPa (1hPa = 100Pa = 1mbar)
#define MS_SLOPE_NUM 16


#define msT_P 200.0f
#define msT_I 20.0f
#define msT_D 40.0f
#define msT_T 42.0f
//驱动程序：spi1读写LSM303D底层程序
#define DPS368_SPI2_CS_CLR	LL_GPIO_ResetOutputPin(SPI2_NSS_GPIO_Port, SPI2_NSS_Pin)
#define DPS368_SPI2_CS_SET	LL_GPIO_SetOutputPin(SPI2_NSS_GPIO_Port, SPI2_NSS_Pin)


#ifdef __cplusplus
}
#endif

#ifdef __cplusplus

class DPS368 :public SPI
{
public:
	DPS368(){}
	DPS368(SPI_HandleTypeDef *h,char *n):hspi(h),name(n){}
	~DPS368(){}
	void DPS368_Init(void);
	void DPS368_Spi_TranRecv(SPI_HandleTypeDef *hspi,u8 Cmd,uint8_t* ReadData);
	void DPS368_Spi_Tran(SPI_HandleTypeDef *hspi,u8 Cmd,u8 data);
	bool DPS368_Cali(void);
	void DPS368_Update(void);
	void DPS368_Loop(void);
	void PID_Temp();
	void getTwosComplement(int32_t *raw, uint8_t length);

private:
	u16 pid_tempout=0;
	float  pid_tempe=0.0f, pid_tempi=0.0f,pid_tempd=0.0f,pid_tempdi=0.0f;

	SPI_HandleTypeDef *hspi;
	//SPI_TypeDef *hspi; //端口  --
	char *name;
	sTIM  Tim;			//计时器
	sCNT  Filt1;
	sCNT  Filt2;
	sCNT  Filt3;
	float AltOff;    //m
	u16   PROM[7];   //PROM数据，解算时用
	uint8_t ID;
	uint8_t TMP_RDY;
	uint8_t PRS_RDY;
	uint8_t rxbuf[8];
	uint8_t CMD_RW[2];
	float TmpRaw;
	float PrsRaw;
	float TmpRel;    //°C
	float PrsRel;    //Pa
	float AltRel;    //m
	//用户访问数据
	bool  Update;     //更新  --
	eSTA  Sta;        //状态  --
	eERR  Err;        //错误信息  --
	HAL_StatusTypeDef    rwsta[2];
	u32   Time;
	bool  Cal;        //校准  --
	u8    StaCnt;
	u8    Delay;
	float TmpFil;    //°C
	float PrsFil;    //Pa
	float AltFil;    //m
	float AltTmp[MS_SLOPE_NUM];  //最小二乘法使用
	float AltSlope;

	int32_t c0,c1,c00,c10,c01,c11,c20,c21,c30;
	int32_t PRS,TMP,PRS_LST;

	uint32_t  startTimer;			//计时器
	uint32_t  stopTimer;			//计时器
	uint32_t  executionTime_us;			//计时器

	uint32_t  startTimerLast;			//计时器
	uint32_t  cycleTime_us;

	sensor_baroAlt_msg baroAlt;
	battery_msg battery;
};


#endif


#endif


/************************ (C) COPYRIGHT Longmen Drone Team *****END OF FILE****/
