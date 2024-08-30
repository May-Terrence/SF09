/*
 * ringbuffer.hpp
 *
 *  Created on: 2021年5月10日
 *      Author: lvguogang
 */

#ifndef MODULES_TRAN_AND_RC_RINGBUFFER_HPP_
#define MODULES_TRAN_AND_RC_RINGBUFFER_HPP_

#include <stdbool.h>
#include <cstdint>

#define RINGBUFFER_LEN 2000

bool Write_to_ringbuffer(uint8_t *data,uint16_t len);
bool Read_from_ringbuffer(uint8_t *data,uint16_t len);
uint16_t Read_all_from_ringbuffer(uint8_t *data);



#endif /* MODULES_TRAN_AND_RC_RINGBUFFER_HPP_ */
