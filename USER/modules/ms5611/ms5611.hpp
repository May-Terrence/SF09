/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : ms5611.hpp
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
#ifndef __MS5611_HPP
#define __MS5611_HPP
#include "hspi/hspi.hpp"
#include "system/system.hpp"

#ifdef __cplusplus
extern "C" {
#endif


#define MS5611_CMD_RESET			 0x1EU // ADC复位
#define MS5611_CMD_D1_256      0x40U // D1转换，8bit（ADC）
#define MS5611_CMD_D1_512      0x42U // D1转换，9bit（ADC）
#define MS5611_CMD_D1_1024     0x44U // D1转换，10bit（ADC）
#define MS5611_CMD_D1_2048     0x46U // D1转换，11bit（ADC）
#define MS5611_CMD_D1_4096     0x48U // D1转换，12bit（ADC）
#define MS5611_CMD_D2_256      0x50U // D2转换，8bit（ADC）
#define MS5611_CMD_D2_512      0x52U // D2转换，9bit（ADC）
#define MS5611_CMD_D2_1024     0x54U // D2转换，10bit（ADC）
#define MS5611_CMD_D2_2048     0x56U // D2转换，11bit（ADC）
#define MS5611_CMD_D2_4096     0x58U // D2转换，12bit（ADC）
#define MS5611_CMD_ADC_READ	   0x00U // 读取ADC，D1气压，D2温度
#define MS5611_CMD_PROM_RD	   0xA0U // 读取PROM，预留
#define MS5611_CMD_PROM_C1     0xA2U // 读取PROM，C1，校准用
#define MS5611_CMD_PROM_C2     0xA4U // 读取PROM，C2，校准用
#define MS5611_CMD_PROM_C3     0xA6U // 读取PROM，C3，校准用
#define MS5611_CMD_PROM_C4     0xA8U // 读取PROM，C4，校准用
#define MS5611_CMD_PROM_C5     0xAAU // 读取PROM，C5，校准用
#define MS5611_CMD_PROM_C6     0xACU // 读取PROM，C6，校准用
#define MS5611_CMD_PROM_CRC    0xAEU // 读取PROM，CRC，检验

#define MSLP	 101325.0f			// 平均海平面气压 = 1013.25 hPa (1hPa = 100Pa = 1mbar)
#define MS_SLOPE_NUM 20

//驱动程序：spi1读写LSM303D底层程序
#define MS5611_SPI2_CS_CLR	LL_GPIO_ResetOutputPin(SPI2_NSS_GPIO_Port, SPI2_NSS_Pin)
#define MS5611_SPI2_CS_SET	LL_GPIO_SetOutputPin(SPI2_NSS_GPIO_Port, SPI2_NSS_Pin)


#ifdef __cplusplus
}
#endif

#ifdef __cplusplus

class MS5611 :public SPI
{
public:
	MS5611(){}
	MS5611(SPI_TypeDef *h,char *n):hspi(h),name(n){}
	~MS5611(){}
	void Ms5611_Init(void);
	void Ms5611_Spi_TranRecv(SPI_TypeDef *hspi,u8 Cmd,u8* ReadData,u8 num);
	void Ms5611_Spi_Tran(SPI_TypeDef *hspi,uint8_t Cmd);
	bool Ms5611_Cali(void);
	void Ms5611_Update(void);
	void Ms5611_Loop_10ms(void);

private:
	SPI_TypeDef *hspi; //端口  --
	char *name;
	sTIM  Tim;			//计时器
	sCNT  Filt;
	float AltOff;    //m
	u16   PROM[7];   //PROM数据，解算时用
	u8 TmpRaw[3];
	u8 PrsRaw[3];
	float TmpRel;    //°C
	float PrsRel;    //Pa
	float AltRel;    //m
	//用户访问数据
	bool  Update;     //更新  --
	eSTA  Sta;        //状态  --
	eERR  Err;        //错误信息  --
	u32   Time;
	bool  Cal;        //校准  --
	u8    StaCnt;
	u8    Delay;
	float TmpFil;    //°C
	float PrsFil;    //Pa
	float AltFil;    //m
	float AltTmp[MS_SLOPE_NUM];  //最小二乘法使用
	float AltSlope;

	uint32_t  startTimer;			//计时器
	uint32_t  stopTimer;			//计时器
	uint32_t  executionTime_us;			//计时器

	uint32_t  startTimerLast;			//计时器
	uint32_t  cycleTime_us;

	sensor_baroAlt_msg baroAlt;
};


#endif


#endif


/************************ (C) COPYRIGHT Longmen Drone Team *****END OF FILE****/
