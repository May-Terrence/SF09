/*
 * rc_rtcm_process.cpp
 *
 *  Created on: 2021年5月10日
 *      Author: lvguogang
 */
#include <string.h>
#include <stdbool.h>
#include <tran_and_rc/rc_rtcm_process.hpp>
#include <tran_and_rc/ringbuffer.hpp>
#include <tran_and_rc/transfer_and_rc.hpp>


//RTCM解析数据存储
static uint8_t trans_buffer[RTCM_TRANSBUFFER_SIZE][RTCM_PACKET_LEN+2];
static uint8_t trans_write_index = 0;
static uint8_t trans_read_index = 0;


static void Rx_State_Machine(uint8_t msg);
static bool RTCM_Checksum(uint8_t *msg, uint16_t msg_size);
static void Transfer_Rtcm(void);
static void Rc_And_Rtcm_Receive_Data_Process(void);


//遥控和RTCM校正信号处理任务，最好1000Hz执行频率以上
void Rc_And_Rtcm_Task(void)
{
	Rc_And_Rtcm_Receive_Data_Process();
	Transfer_Rtcm();
}


//遥控和RTK基站接受数据处理
static void Rc_And_Rtcm_Receive_Data_Process(void)
{
	static uint8_t tmp_data[RINGBUFFER_LEN];

	uint16_t read_len = Read_all_from_ringbuffer(tmp_data); //获取接收到的数据和长度
	for(uint16_t i=0;i<read_len;++i)
	{
		//接受数据状态机
		Rx_State_Machine(tmp_data[i]);
	}
}

//处理好的RTCM校正信号发到RTK移动站
static void Transfer_Rtcm(void)
{
	uint8_t ready_to_read_space = (trans_write_index - trans_read_index + RTCM_TRANSBUFFER_SIZE ) % RTCM_TRANSBUFFER_SIZE;
	if(ready_to_read_space < 1)
	{
		return;
	}
	uint16_t len = trans_buffer[trans_read_index][1]<< 8 | trans_buffer[trans_read_index][0];

	if(uart4_Send_DMA(&trans_buffer[trans_read_index][2],len))
	{
		trans_read_index = (trans_read_index + 1)% RTCM_TRANSBUFFER_SIZE;
	}
//		if(usart6_Send_DMA(&trans_buffer[trans_read_index][2],len))
//		{
//			trans_read_index = (trans_read_index + 1)% RTCM_TRANSBUFFER_SIZE;
//		}
}

//一帧RTCM数据处理、末尾还有多余的0
//如果有效，则存入缓存区，等待发送出去
static bool RTCM_Calc(uint8_t *rtcm_raw_data,uint16_t rtcm_get_len)
{
	if(rtcm_raw_data[0] != RTCM3_PREAMBLE)
		return false;
	uint16_t rtcm_length = (((uint16_t)rtcm_raw_data[1] & 3) << 8) | (rtcm_raw_data[2]);
	if(rtcm_get_len < (rtcm_length + 6))
		return false;
	if(!RTCM_Checksum(rtcm_raw_data,rtcm_length+6))
		return false;
	uint8_t free_space = (trans_read_index - trans_write_index + RTCM_TRANSBUFFER_SIZE - 1) % RTCM_TRANSBUFFER_SIZE;
	if(free_space < 1)
		return false;
	trans_buffer[trans_write_index][0] = (rtcm_length+6);
	trans_buffer[trans_write_index][1] = (rtcm_length+6)>>8;
	memcpy(&trans_buffer[trans_write_index][2],rtcm_raw_data,rtcm_length+6);
	trans_write_index =(trans_write_index + 1) % RTCM_TRANSBUFFER_SIZE;
	return true;
}



static void Get_Rc_And_Rtcm(uint8_t *data,uint16_t len)
{
	static uint8_t last_rtcm_seg_num = 0;
	static uint8_t last_rtcm_seg_index = 0;
	static uint8_t rtcm_buf[1000];
	static uint8_t rtcm_id_cnt = 0;

	if(len == RC_FRAME_LEN)
	{
		if( (data[0] == '$') && (data[25] == '\r') && (data[26] == '\n') )
		{
			//遥控器数据处理
//			rc.RxRawHead = true;
			if(rc.LockRx == HAL_LOCKED)
				return;
			rc.LockRx = HAL_LOCKED;
			memcpy(rc.RxDat,data, RC_FRAME_LEN);
			rc.RxDat[RC_FRAME_LEN]=0;
			rc.LockRx = HAL_UNLOCKED;
			rc.RxFlag = true;   //收到完整一帧
//			rc[0].RxRawHead = false;
		}
	}
	else if(len == RTK_FRAME_LEN)
	{
		uint8_t rtcm_seg_num = data[0];
		uint8_t rtcm_seg_index = data[1];
		if(rtcm_seg_index == 0 || ((rtcm_seg_num == last_rtcm_seg_num) && (rtcm_seg_index == last_rtcm_seg_index + 1)))
		{
			memcpy(&rtcm_buf[(RTK_FRAME_LEN-2)*rtcm_seg_index],&data[2],RTK_FRAME_LEN-2);
			if(rtcm_seg_index == rtcm_seg_num - 1) //一帧数据
			{

				if(RTCM_Calc(rtcm_buf,(RTK_FRAME_LEN-2)*rtcm_seg_num))
				{
					uint16_t rtcm_id = (uint16_t)rtcm_buf[3]<<4|rtcm_buf[4]>>4;
					if(rtcm_id != 1005){
						rc.rc_status_msg.rctm_id_cnt = rtcm_id_cnt;
						rc.rc_status_msg.rtcm_id[rtcm_id_cnt++] = rtcm_id;
						rtcm_id_cnt %= 5;
						xQueueOverwrite(queueRC_Status, &rc.rc_status_msg);
					}
				}
			}
		}
		last_rtcm_seg_num = rtcm_seg_num;
		last_rtcm_seg_index = rtcm_seg_index;
	}
}

static void Add_checksum(uint8_t *check,uint8_t data)
{
	*check += data;
}

static void Rx_State_Machine(uint8_t msg)
{
	static uint8_t state = 0;
	static uint16_t count = 0;
	static uint8_t len;
	static uint8_t checksum = 0;
	static uint8_t rd_buf[RTK_FRAME_LEN];
	switch(state)
	{
		case 0:
		{
			if(msg == FRAME_HEADER_0)
			{
				state = 1;
				count = 0;
				Add_checksum(&checksum,msg);
			}
			else if(msg == 0xAA)//地面站数据
			{
				rd_buf[0] = msg;
				Add_checksum(&checksum,msg);
				state = 5;
			}
			else
			{
				checksum = 0;
			}
		}break;
		case 1:
		{
			if(msg == FRAME_HEADER_1)
			{
				state = 2;
				Add_checksum(&checksum,msg);
			}
			else
			{
				state = 0;
				checksum = 0;
			}
		}break;
		case 2:
		{
			len = msg;
			if(len == RC_FRAME_LEN || len == RTK_FRAME_LEN)
			{
				state = 3;
				Add_checksum(&checksum,msg);
			}
			else
			{
				state = 0;
				checksum = 0;
			}
		}break;
		case 3:
		{
			rd_buf[count++] = msg;
			Add_checksum(&checksum,msg);
			if(count == len)
			{
				state = 4;
			}
		}break;
		case 4:
		{
			if(checksum == msg)
			{
				//处理有效数据
				Get_Rc_And_Rtcm(rd_buf,len);
			}
			state = 0;
			checksum = 0;
		}break;
		case 5:				//地面站数据
		{
			if(msg == 0xAF)
			{
				rd_buf[1] = msg;
				Add_checksum(&checksum,msg);
				state = 6;
			}
			else
			{
				state = 0;
				checksum = 0;
			}
		}break;
		case 6:
		{
			rd_buf[2] = msg;
			Add_checksum(&checksum,msg);
			state = 7;
		}break;
		case 7:
		{
			len = msg;
			rd_buf[3] = msg;
			Add_checksum(&checksum,msg);
			count = 0;
			state = 8;
		}break;
		case 8:
		{
			rd_buf[4+count] = msg;
			Add_checksum(&checksum,msg);
			count ++;
			if(count == len)
			{
				state = 9;
				count = 0;
			}
		}break;
		case 9:
		{
			rd_buf[4+len] = msg;
			if(rd_buf[4+len] == checksum)
			{
				tran.LockRx = HAL_LOCKED;
				memcpy(tran.RxDat,&rd_buf[0], len+5);
				tran.LockRx = HAL_UNLOCKED;
				tran.RxFlag = true;   //收到完整一帧

				BaseType_t YieldRequired = xTaskResumeFromISR(tranReceiveTaskHandle);//地面站数据接收中断
				if(YieldRequired==pdTRUE)
				{
					/*如果函数xTaskResumeFromISR()返回值为pdTRUE，那么说明要恢复的这个
					任务的任务优先级等于或者高于正在运行的任务(被中断打断的任务),所以在
					退出中断的时候一定要进行上下文切换！*/
					portYIELD_FROM_ISR(YieldRequired);
				}
			}
			state = 0;
			checksum = 0;
		}break;
		default:
    break;
	}
}

const uint32_t table[]={ //CRC24校验码
0x000000, 0x864CFB, 0x8AD50D, 0x0C99F6, 0x93E6E1, 0x15AA1A, 0x1933EC, 0x9F7F17,
0xA18139, 0x27CDC2, 0x2B5434, 0xAD18CF, 0x3267D8, 0xB42B23, 0xB8B2D5, 0x3EFE2E, 0xC54E89, 0x430272,
0x4F9B84, 0xC9D77F, 0x56A868, 0xD0E493, 0xDC7D65, 0x5A319E, 0x64CFB0, 0xE2834B, 0xEE1ABD, 0x685646,
0xF72951, 0x7165AA, 0x7DFC5C, 0xFBB0A7, 0x0CD1E9, 0x8A9D12, 0x8604E4, 0x00481F, 0x9F3708, 0x197BF3,
0x15E205, 0x93AEFE, 0xAD50D0, 0x2B1C2B, 0x2785DD, 0xA1C926, 0x3EB631, 0xB8FACA, 0xB4633C, 0x322FC7,
0xC99F60, 0x4FD39B, 0x434A6D, 0xC50696, 0x5A7981, 0xDC357A, 0xD0AC8C, 0x56E077, 0x681E59, 0xEE52A2,
0xE2CB54, 0x6487AF, 0xFBF8B8, 0x7DB443, 0x712DB5, 0xF7614E, 0x19A3D2, 0x9FEF29, 0x9376DF, 0x153A24,
0x8A4533, 0x0C09C8, 0x00903E, 0x86DCC5, 0xB822EB, 0x3E6E10, 0x32F7E6, 0xB4BB1D, 0x2BC40A, 0xAD88F1,
0xA11107, 0x275DFC, 0xDCED5B, 0x5AA1A0, 0x563856, 0xD074AD, 0x4F0BBA, 0xC94741, 0xC5DEB7, 0x43924C,
0x7D6C62, 0xFB2099, 0xF7B96F, 0x71F594, 0xEE8A83, 0x68C678, 0x645F8E, 0xE21375, 0x15723B, 0x933EC0,
0x9FA736, 0x19EBCD, 0x8694DA, 0x00D821, 0x0C41D7, 0x8A0D2C, 0xB4F302, 0x32BFF9, 0x3E260F, 0xB86AF4,
0x2715E3, 0xA15918, 0xADC0EE, 0x2B8C15, 0xD03CB2, 0x567049, 0x5AE9BF, 0xDCA544, 0x43DA53, 0xC596A8,
0xC90F5E, 0x4F43A5, 0x71BD8B, 0xF7F170, 0xFB6886, 0x7D247D, 0xE25B6A, 0x641791, 0x688E67, 0xEEC29C,
0x3347A4, 0xB50B5F, 0xB992A9, 0x3FDE52, 0xA0A145, 0x26EDBE, 0x2A7448, 0xAC38B3, 0x92C69D, 0x148A66,
0x181390, 0x9E5F6B, 0x01207C, 0x876C87, 0x8BF571, 0x0DB98A, 0xF6092D, 0x7045D6, 0x7CDC20, 0xFA90DB,
0x65EFCC, 0xE3A337, 0xEF3AC1, 0x69763A, 0x578814, 0xD1C4EF, 0xDD5D19, 0x5B11E2, 0xC46EF5, 0x42220E,
0x4EBBF8, 0xC8F703, 0x3F964D, 0xB9DAB6, 0xB54340, 0x330FBB, 0xAC70AC, 0x2A3C57, 0x26A5A1, 0xA0E95A,
0x9E1774, 0x185B8F, 0x14C279, 0x928E82, 0x0DF195, 0x8BBD6E, 0x872498, 0x016863, 0xFAD8C4, 0x7C943F,
0x700DC9, 0xF64132, 0x693E25, 0xEF72DE, 0xE3EB28, 0x65A7D3, 0x5B59FD, 0xDD1506, 0xD18CF0, 0x57C00B,
0xC8BF1C, 0x4EF3E7, 0x426A11, 0xC426EA, 0x2AE476, 0xACA88D, 0xA0317B, 0x267D80, 0xB90297, 0x3F4E6C,
0x33D79A, 0xB59B61, 0x8B654F, 0x0D29B4, 0x01B042, 0x87FCB9, 0x1883AE, 0x9ECF55, 0x9256A3, 0x141A58,
0xEFAAFF, 0x69E604, 0x657FF2, 0xE33309, 0x7C4C1E, 0xFA00E5, 0xF69913, 0x70D5E8, 0x4E2BC6, 0xC8673D,
0xC4FECB, 0x42B230, 0xDDCD27, 0x5B81DC, 0x57182A, 0xD154D1, 0x26359F, 0xA07964, 0xACE092, 0x2AAC69,
0xB5D37E, 0x339F85, 0x3F0673, 0xB94A88, 0x87B4A6, 0x01F85D, 0x0D61AB, 0x8B2D50, 0x145247, 0x921EBC,
0x9E874A, 0x18CBB1, 0xE37B16, 0x6537ED, 0x69AE1B, 0xEFE2E0, 0x709DF7, 0xF6D10C, 0xFA48FA, 0x7C0401,
0x42FA2F, 0xC4B6D4, 0xC82F22, 0x4E63D9, 0xD11CCE, 0x575035, 0x5BC9C3, 0xDD8538};

static bool RTCM_Checksum(uint8_t *msg,uint16_t msg_size)
{
	uint32_t crc = 0;
	for (uint16_t i=0;i<msg_size-3;++i)
	{
		crc = ((crc << 8) & 0xFFFFFF) ^ table[(crc >> 16) ^ (msg[i])];
	}
	uint8_t ck[3];
	ck[0] = (crc >> 16) & 0xFF;
	ck[1] = (crc >> 8) & 0xFF;
	ck[2] = crc & 0xFF;
	if( (ck[0] == msg[msg_size-3]) && (ck[1] == msg[msg_size-2]) && (ck[2] == msg[msg_size-1]))
		return true;
	return false;
}


