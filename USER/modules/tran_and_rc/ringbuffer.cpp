/*
 * ringbuffer.cpp
 *
 *  Created on: 2021年5月10日
 *      Author: lvguogang
 */

#include <tran_and_rc/ringbuffer.hpp>
#include "string.h"

static uint8_t ringbuffer[RINGBUFFER_LEN];
static uint16_t write_index = 0;
static uint16_t read_index = 0;


static bool Can_write_to_ringbuffer(uint16_t len)
{
	uint16_t free_space = (read_index - write_index + RINGBUFFER_LEN - 1) % RINGBUFFER_LEN;
	return (len <= free_space ? true:false);
}

static bool Can_read_from_ringbuffer(uint16_t len)
{
	uint16_t read_space = (write_index - read_index + RINGBUFFER_LEN) % RINGBUFFER_LEN;
	return (len <= read_space ? true:false);
}

bool Write_to_ringbuffer(uint8_t *data,uint16_t len)
{
	if(!Can_write_to_ringbuffer(len)) //判断接收的数据长度是否小于2000
		return false;
	if( write_index + len > RINGBUFFER_LEN)
	{
		uint16_t first_num = RINGBUFFER_LEN - write_index;
		uint16_t second_num = len - first_num;
		memcpy(&ringbuffer[write_index],data,first_num);
		memcpy(&ringbuffer[0],&data[first_num],second_num);
	}
	else
	{
		memcpy(&ringbuffer[write_index],data,len); //把接收的数据写入环形缓冲区
	}
	write_index  = (write_index + len)%RINGBUFFER_LEN;
	return true;
}

bool Read_from_ringbuffer(uint8_t *data,uint16_t len)
{
	if(!Can_read_from_ringbuffer(len))
		return false;
	if( read_index + len > RINGBUFFER_LEN)
	{
		uint16_t first_num = RINGBUFFER_LEN - read_index;
		uint16_t second_num = len - first_num;
		memcpy(data,&ringbuffer[read_index],first_num);
		memcpy(&data[first_num],&ringbuffer[0],second_num);
	}
	else
	{
		memcpy(data,&ringbuffer[read_index],len);
	}
	read_index  = (read_index + len)%RINGBUFFER_LEN;
	return true;
}

uint16_t Read_all_from_ringbuffer(uint8_t *data)
{
	uint16_t read_space = (write_index - read_index + RINGBUFFER_LEN) % RINGBUFFER_LEN;
	if(read_space == 0)
		return 0;
	if(read_index + read_space > RINGBUFFER_LEN)
	{
		uint16_t first_num = RINGBUFFER_LEN - read_index;
		uint16_t second_num = read_space - first_num;
		memcpy(data,&ringbuffer[read_index],first_num);
		memcpy(&data[first_num],&ringbuffer[0],second_num);
	}
	else
	{
		memcpy(data,&ringbuffer[read_index],read_space);
	}
	read_index  = (read_index + read_space)%RINGBUFFER_LEN;
	return read_space;
}


