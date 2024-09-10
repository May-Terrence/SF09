#include "system/system.hpp"
#include "userlib/userlib.hpp"
#include "openlog.h"
#ifdef __cplusplus
extern "C" {
#endif
void USART1_IRQHandler(void);
void DMA2_Stream7_IRQHandler(void);
bool uart_Send_DMA(uint8_t * pData,uint16_t Size);

#define SLAM

#ifdef SD
void TRAN_Init(void);
#ifdef __cplusplus
}
#endif
void EskfDataStorage(void);
void OutloopDataStorage(void);
eskf_msg EskfLog;
OrdinaryGps_msg GpsLog;
eskf_baro_msg Eskf_baroLog;
sensor_baroAlt_msg baroAlt;
mag_msg mag;
gps_msg gps;

downsample_imu_for_eskf_msg imu;
RC_command_msg rcCommand;
control_transfer_msg control_data;
trajectory_msg trajectoryData;
laserFlow_msg laserFlow;

uint32_t startTimer;
#endif

#ifdef SLAM
#define SLAM_RX_LEN 50
#define SLAM_TX_LEN 50

typedef enum
{
	Taking_off   		= 0x00U,	 //正在起飞
	Fail_to_take_off    = 0x01U,	 //坠毁
	Air	  				= 0x02U,	 //起飞成功
	Landing 	  		= 0x03U,	 //正在降落
	Fail_to_land   		= 0x04U,   	 //坠毁
	Ground        		= 0x05U,   	 //降落成功
}Status;

class SLAM_TRAN
{
public:
	SLAM_TRAN(USART_TypeDef *h,char *n,char *s) : huart(h),name(n),DirChr(s){};
	void SLAM_Init();

	uint16_t   RxDataSize;
	uint8_t    RxRawDat[SLAM_RX_LEN];	//数据
	uint8_t    RxDat[SLAM_RX_LEN];
	HAL_LockTypeDef LockRx;
	bool  RxFlag;		//接收标志位
	bool  TxFlag;		//发送标志位
	USART_TypeDef * huart;

	void Command_Receive(void);
	void Relative_Position_Transfer(void);
	void Status_Transfer(void);
	void Take_off_Request_Transfer(void);
	void Land_Request_Transfer(void);
	bool uart_Send_Check(void);

	bool isCommunicating{false};
	bool Ready_Take_off{false};
	bool Ready_Land{false};

	Status status;

private:
	int8_t  Dir[6];     //方向  --
	char *name;
	char *DirChr;
	double Re2t[3][3];
	uint8_t  TxDat[SLAM_TX_LEN];

	eSTA  Sta;        //状态  --
	eERR  Err;        //错误信息  --
	uint32_t startTimer;
	uint32_t startTimerLast;
	uint32_t stopTimer;
	uint16_t  executionTime_us; //计时
	uint16_t cycleTime_us;

	SLAM_msg slam_msg;
	eskf_msg eskf;
	control_transfer_msg control_data;
	gps_msg gps;
};

extern SLAM_TRAN slam;

}
#endif
