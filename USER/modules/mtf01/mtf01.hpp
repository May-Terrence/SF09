#pragma once

#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include "system/system.hpp"
#include "userlib/userlib.hpp"

#ifdef __cplusplus
extern "C" {
#endif

#define MICOLINK_MSG_HEAD            0xEF
#define MICOLINK_MAX_PAYLOAD_LEN     64
#define MICOLINK_MAX_LEN             MICOLINK_MAX_PAYLOAD_LEN + 7

void USART2_IRQHandler(void);

/*
    消息ID定义
*/
enum
{
    MICOLINK_MSG_ID_RANGE_SENSOR = 0x51,     // 测距传感器
};

/*
    消息结构体定义
*/
typedef struct
{
    uint8_t head;               //帧头
    uint8_t dev_id;             //设备ID
    uint8_t sys_id;				//系统ID
    uint8_t msg_id;             //消息ID
    uint8_t seq;                //包序列
    uint8_t len;                //负载长度
    uint8_t payload[MICOLINK_MAX_PAYLOAD_LEN]; //数据负载
    uint8_t checksum;            //帧校验

    uint8_t status;              //状态机
    uint8_t payload_cnt;         //数据负载计数
} MICOLINK_MSG_t;

/*
    数据负载定义
*/
#pragma pack (1)
// 测距传感器
typedef struct
{
    uint32_t  time_ms;			    // 系统时间 ms
    uint32_t  distance;			    // 距离(mm) 最小值为10，0表示数据不可用
    uint8_t   strength;	            // 信号强度
    uint8_t   precision;	        // 数据精度cm
    uint8_t   tof_status;	        // 状态 1表示测距数据可用
    uint8_t  reserved1;			    // 预留
    int16_t   flow_vel_x;	        // 光流速度x轴 	“计算公式：实际速度(cm/s)=光流速度*高度(m)”
    int16_t   flow_vel_y;	        // 光流速度y轴
    uint8_t   flow_quality;	        // 光流质量 数值越高表示数据可信度越高
    uint8_t   flow_status;	        // 光流状态 1表示光流数据可用
    uint16_t  reserved2;	        // 预留
} MICOLINK_PAYLOAD_RANGE_SENSOR_t;
#pragma pack ()

#ifdef __cplusplus
}
#endif

#ifdef __cplusplus
using namespace std;

class MTF01
{
public:
	MTF01(USART_TypeDef *h,char *n) : huart(h),name(n){};
	void MTF01_Init();
	bool micolink_parse_char(MICOLINK_MSG_t* msg, uint8_t data);
	void micolink_decode(uint8_t data);
	bool micolink_check_sum(MICOLINK_MSG_t* msg);
	void laserFlowRun();

	uint16_t RxDataSize;
	uint8_t    RxRawDat[MICOLINK_MAX_LEN];	//数据
	uint8_t    RxDat[MICOLINK_MAX_LEN];
	HAL_LockTypeDef LockRx;
	bool  RxFlag;		//接收标志位
	USART_TypeDef * huart;
	uint8_t laserFlowSum;

private:
	char *name;
	char *DirChr;
	MICOLINK_MSG_t msg;
	MICOLINK_PAYLOAD_RANGE_SENSOR_t payload;
	laserFlow_msg laserFlow;
	sCNT heightFil;
	sCNT VelxFil;
	sCNT VelyFil;
};
#endif
