#include "system/system.hpp"
#include "userlib/userlib.hpp"
#include "openlog.h"
#ifdef __cplusplus
extern "C" {
#endif
void USART1_IRQHandler(void);
void DMA2_Stream7_IRQHandler(void);
bool uart_Send_DMA(uint8_t * pData,uint16_t Size);
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
