/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : spi.hpp
  * Description        :
  ******************************************************************************
  * Function           :
  *
  ******************************************************************************
  * Author             : Zhiping Fu
  * Creation Date      : 2020年10月10日
  ******************************************************************************
  */
/* USER CODE END Header */

#ifndef __HSPI_HPP
#define __HSPI_HPP

#include "userlib/userlib.hpp"

#ifdef __cplusplus
using namespace std;

class SPI
{
public:
	SPI(){}
	~SPI(){}

	uint8_t SPI_ReadWriteByte(SPI_TypeDef *hspi, uint8_t TxData);
	void SPI_Recesive(SPI_TypeDef *hspi,uint8_t *pData,uint16_t Size);

private:


};

#endif

#endif

/************************ (C) COPYRIGHT Longmen Drone Team *****END OF FILE****/
