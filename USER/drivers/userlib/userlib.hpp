/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : userlib.hpp
  * Description        :
  ******************************************************************************
  * Function           :
  *
  ******************************************************************************
  * Author             : Zhiping Fu
  * Creation Date      : 2020年10月11日
  ******************************************************************************
  */
/* USER CODE END Header */
#ifndef __USERLIB_HPP
#define __USERLIB_HPP


#include "main.h"
#include "string.h"

#ifdef __cplusplus
extern "C" {
#endif
//#include "tim.h"

#include <math.h>
#include <stdbool.h>


//datatype define
typedef int64_t s64;
typedef int32_t s32;
typedef int16_t s16;
typedef int8_t  s8;
typedef const int64_t sc64;  /*!< Read Only */
typedef const int32_t sc32;  /*!< Read Only */
typedef const int16_t sc16;  /*!< Read Only */
typedef const int8_t sc8;   /*!< Read Only */
typedef __IO int64_t  vs64;
typedef __IO int32_t  vs32;
typedef __IO int16_t  vs16;
typedef __IO int8_t   vs8;
typedef __I int64_t vsc64;  /*!< Read Only */
typedef __I int32_t vsc32;  /*!< Read Only */
typedef __I int16_t vsc16;  /*!< Read Only */
typedef __I int8_t vsc8;   /*!< Read Only */
typedef uint64_t u64;
typedef uint32_t u32;
typedef uint16_t u16;
typedef uint8_t  u8;
typedef const uint64_t uc64;  /*!< Read Only */
typedef const uint32_t uc32;  /*!< Read Only */
typedef const uint16_t uc16;  /*!< Read Only */
typedef const uint8_t uc8;   /*!< Read Only */
typedef __IO uint64_t vu64;
typedef __IO uint32_t vu32;
typedef __IO uint16_t vu16;
typedef __IO uint8_t  vu8;
typedef __I uint64_t vuc64;  /*!< Read Only */
typedef __I uint32_t vuc32;  /*!< Read Only */
typedef __I uint16_t vuc16;  /*!< Read Only */
typedef __I uint8_t vuc8;   /*!< Read Only */

//常用定义
#define PI 3.1415926535898
#define D2R  (PI / 180.0f)
#define D2R_D 0.017453292519943295
#define R2D  (180.0f / PI)
#define R2D_D 57.295779513082323
#define  OneG   9.788
const float Dubins_Vel = 2.0;
const float DubinsTiltAng = 3*D2R; //OneG*tan(3*D2R)=0.512

//数据拆分宏定义
#define BYTE0(dwTemp)       ( *( (char *)(&dwTemp)    ) )
#define BYTE1(dwTemp)       ( *( (char *)(&dwTemp) + 1) )
#define BYTE2(dwTemp)       ( *( (char *)(&dwTemp) + 2) )
#define BYTE3(dwTemp)       ( *( (char *)(&dwTemp) + 3) )

typedef union 		//16位联合体
{
	uint8_t   uc[2];
	int8_t   sc[2];
	uint16_t  us;
	int16_t  ss;
}BIT16;

typedef union 		//32位联合体
{
	uint8_t   uc[4];
	int8_t   sc[4];
	uint16_t  us[2];
	int16_t  ss[2];
	uint32_t  ui;
	int32_t  si;
	float fp;
}BIT32;

typedef struct      //计时器，计算运行时间
{
	uint32_t CNT;        //计数值
	uint32_t OUT;        //输出值
}sTIM;

typedef enum            //错误类型
{
	ERR_NONE=0x00U,       //正常
	ERR_SOFT=0x01U,       //软件错误
	ERR_HARD=0x02U,       //硬件错误
	ERR_LOST=0x03U,       //对象不存在
	ERR_UPDT=0x04U,       //周期内未更新或者数据无效
}eERR;

typedef enum            //传感器或控制器状态
{
	STA_INI=0x01U,        //初始化参数，配置等
	STA_CHK=0x02U,        //自检状态，检测自身存在
	STA_CAL=0x03U,        //校准状态，校准参数
	STA_RUN=0x04U,        //运行状态，读取数据或执行控制
	STA_TST=0x05U,        //测试状态，测试
}eSTA;

typedef struct      //计数器，滤波等使用
{
	s16 CNT;        //当前计数值
	s16 CCR;        //比较值
}sCNT;



#define TIM_COUNT TIM2



void Tim_Calc(sTIM *timx);
void User_Delay(u32 nus);
bool Dir_Trans(s8 Dir[6],const char DirChr[6]);
void getTimer_us(uint32_t *timer);
float fConstrain(float Input, float minValue, float maxValue);
s16 iConstrain(s16 Input, s16 minValue, s16 maxValue);
float LoopConstrain(float Input, float minValue, float maxValue);
void SlideFilt(float *Dat,float *DatRel,u8 num,sCNT *Filt,u8 Cmd);
void LineFit(float* Y,u16 N,float *ab); //y=ax+b
float Sign(float value);
bool TarHit(sCNT *Hit, float Tol, double err);
void mod2PI(float *theta);
void modPI(float *theta);


float SQR(float x);
double SQRD(double x);
float absf(float x);
float MAX(float x,float y);
float MIN(float x,float y);
u8 StrSeg(const char *str,char chr,char *para[],u8 num);
inline bool isZero(float val) {
    return fabs(val) < 1e-3;
}
#ifdef __cplusplus
}
#endif

#endif
/************************ (C) COPYRIGHT Longmen Drone Team *****END OF FILE****/
