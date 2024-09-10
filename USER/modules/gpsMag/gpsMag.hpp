/*
 * gpsMag.hpp
 *
 *  Created on: 2020年11月28日
 *      Author: 刘成吉
 */

#ifndef MODULES_GPSMAG_GPSMAG_HPP_
#define MODULES_GPSMAG_GPSMAG_HPP_

#include "system/system.hpp"
#include "userlib/userlib.hpp"

#ifdef __cplusplus
extern "C" {
#endif

#define GPS_FREQ 10
#define C_WGS84_a 6378137.0        //地球长半轴
#define C_WGS84_b 6356752.314245   //地球短半轴   f=(a-b)/a
#define C_WGS84_f 0.00335281066474748072//地球偏心率
#define C_WGS84_e 0.081819790992   //第一偏心率
#define GPS_RX_LEN 200
#define GPS_TX_LEN 1000
#define GPS_TYPE_NUM 5     //GPS 传输类型 $GPGGA $GPVTG $HMC $GPS $CPM

void UART4_IRQHandler(void);
void DMA1_Stream4_IRQHandler(void);
bool uart4_Send_DMA(uint8_t * pData,uint16_t Size);
void USART6_IRQHandler(void);
void DMA2_Stream6_IRQHandler(void);
bool usart6_Send_DMA(uint8_t * pData,uint16_t Size);
bool get_mag_field_ef(float latitude_deg, float longitude_deg, float *intensity_gauss, float *declination_deg, float *inclination_deg);
float get_declination(float latitude_deg, float longitude_deg);


typedef enum
{
	GPGGA = 0,
	GPVTG = 1,
	HMC   = 2,
	GPS   = 3,
	CPM   = 4,
}GPS_TYPE;

typedef enum
{
        NO_GPS,                     ///< No GPS connected/detected
        NO_FIX ,                     ///< Receiving valid GPS messages but no lock
        GPS_OK_FIX_2D ,              ///< Receiving valid messages and 2D lock
        GPS_OK_FIX_3D ,              ///< Receiving valid messages and 3D lock
        GPS_OK_FIX_3D_DGPS ,           ///< Receiving valid messages and 3D lock with differential improvements
        GPS_OK_FIX_3D_RTK_FLOAT , ///< Receiving valid messages and 3D RTK Float
        GPS_OK_FIX_3D_RTK_FIXED , ///< Receiving valid messages and 3D RTK Fixed
}GPS_Status;

#ifdef __cplusplus
}
#endif


#ifdef __cplusplus
using namespace std;

class GPSMAG
{
public:
	GPSMAG(USART_TypeDef *h,char *n,char *s) : huart(h),name(n),DirChr(s){};
	void GpsMag_Init();

	uint16_t RxDataSize;
	uint8_t    RxRawDat[GPS_RX_LEN];	//数据
	uint8_t    RxDat[GPS_RX_LEN];
	HAL_LockTypeDef LockRx;
	bool  RxFlag;		//接收标志位
	bool  TxFlag;		//发送标志位

	uint32_t tran_err_cnt {0};
	uint32_t fifo_err_cnt {0};

	USART_TypeDef * huart;

	bool Gps_Cali();
	bool Gps_Calc();

private:
	int8_t  Dir[6];     //方向  --
	char *name;
	char *DirChr;
	double Re2t[3][3];
	uint8_t  TxDat[GPS_TX_LEN];

	eSTA  Sta;        //状态  --
	eERR  Err;        //错误信息  --
	uint32_t startTimer;
	uint32_t startTimerLast;
	uint32_t stopTimer;
	uint16_t  executionTime_us; //计时
	uint16_t cycleTime_us;


	bool MagUpdate;
	bool GpsUpdate;
	bool GpsCal;
	eERR MagErr;		 //错误信息  --
	eSTA MagSta;		//状态  --
	float yaw_declination;
	double lat,lng,alti;
	float time;
	uint8_t star;

	float hdop;
	float wgs_alt;
	float vtg_dir;
	double vtg_spd;
	double Zero_ECFF[3];
	double ECFF[3];
	double ECFF_Init[3];
	double LLH[3];
	double LLH_Init[3];
	double Zero_LLH[3];
	double N;
	double N_Init;


	float   MagOff[3];  //偏置
	s16   MagRaw[3];  //原始值
	float   MagRot[3];  //旋转值
	float MagRel[3];  //实际值

	bool  Update;

	double NED[3];
	double NED_Init[3];
	double NED_spd[3];

	u8  GPSUpdate;
	bool ECEF_Init_Flag;//ECEF坐标系原点标志位
	float MagFil[3];

	gps_msg gps;
	mag_msg mag;

	float gpsSpdAccuracy;
	float gpsPosAccuracy;
	float gpsHeiAccuracy;
	float gps_heading;
	float gps_headAccuracy;

	u32   gps_update_time_us;
	u32   mag_update_time_us;

	int  status;

	float alti_off;

};

class OrdinaryGps
{
public:
	OrdinaryGps(USART_TypeDef *h,char *n,char *s) : huart(h),name(n),DirChr(s){};
	void GpsMag_Init();

	uint16_t RxDataSize;
	uint8_t    RxRawDat[GPS_RX_LEN];	//数据
	uint8_t    RxDat[GPS_RX_LEN];
	HAL_LockTypeDef LockRx;
	bool  RxFlag;		//接收标志位
	bool  TxFlag;		//发送标志位
	USART_TypeDef * huart;

	bool Gps_Cali();
	bool Gps_Calc();

private:
	int8_t  Dir[6];     //方向  --
	char *name;
	char *DirChr;
	double Re2t[3][3];
	uint8_t  TxDat[GPS_TX_LEN];

	eSTA  Sta;        //状态  --
	eERR  Err;        //错误信息  --
	uint32_t startTimer;
	uint32_t startTimerLast;
	uint32_t stopTimer;
	uint16_t  executionTime_us; //计时
	uint16_t cycleTime_us;

	bool MagUpdate;
	bool GpsUpdate;
	bool GpsCal;
	eERR MagErr;		 //错误信息  --
	eSTA MagSta;		//状态  --
	float yaw_declination;
	double lat,lng,alti;
	float time;
	uint8_t star;

	float hdop;
	float wgs_alt;
	float vtg_dir;
	double vtg_spd;
	double Zero_ECFF[3];
	double ECFF[3];
	double ECFF_Init[3];
	double LLH[3];
	double LLH_Init[3];
	double Zero_LLH[3];
	double N;
	double N_Init;

	float   MagOff[3];  //偏置
	s16   MagRaw[3];  //原始值
	float   MagRot[3];  //旋转值
	float MagRel[3];  //实际值

	bool  Update;

	double NED[3];
	double NED_Init[3];
	double NED_spd[3];

	u8  GPSUpdate;
	bool ECEF_Init_Flag;//ECEF坐标系原点标志位
	float MagFil[3];

	OrdinaryGps_msg gps;
	mag_msg mag;

	float gpsSpdAccuracy;
	float gpsPosAccuracy;
	float gpsHeiAccuracy;
	float gps_heading;
	float gps_headAccuracy;

	u32   gps_update_time_us;
	u32   mag_update_time_us;

	int  status;
	float alti_off;
};
#endif

#endif /* MODULES_GPSMAG_GPSMAG_HPP_ */
