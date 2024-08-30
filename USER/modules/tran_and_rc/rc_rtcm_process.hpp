/*
 * rc_rtcm_process.hpp
 *
 *  Created on: 2021年5月10日
 *      Author: lvguogang
 */

#ifndef MODULES_TRAN_AND_RC_RC_RTCM_PROCESS_HPP_
#define MODULES_TRAN_AND_RC_RC_RTCM_PROCESS_HPP_


#define RTCM3_PREAMBLE 0xD3

//RTCM缓存区定义
#define RTCM_PACKET_LEN 1000  //一帧rtcm最大长度
#define RTCM_TRANSBUFFER_SIZE 8  //RTCM缓存区大小

//数据链基站通信帧定义
#define FRAME_HEADER_0         0xFD   //帧头0
#define FRAME_HEADER_1         0xDF   //帧头1
#define RC_FRAME_LEN            27  //遥控数据帧长度
#define RKT_FRAME_DATA_LEN      60
#define RKT_FRAME_HEADER_LEN     2
#define RTK_FRAME_LEN     (RKT_FRAME_DATA_LEN +RKT_FRAME_HEADER_LEN)   //RTK数据帧长度

void Rc_And_Rtcm_Task(void);


#endif /* MODULES_TRAN_AND_RC_RC_RTCM_PROCESS_HPP_ */
