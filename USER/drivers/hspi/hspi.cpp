/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : spi.cpp
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

#include "hspi.hpp"

uint8_t SPI::SPI_ReadWriteByte(SPI_TypeDef *hspi, uint8_t TxData)
{
	uint8_t retry = 0;

	/* Check if Tx buffer is empty */
	while (!LL_SPI_IsActiveFlag_TXE(hspi))
	{
		retry++;
		if(retry > 200) return 0;
	}

	/* Write character in Data register.
	TXE flag is cleared by reading data in DR register */
	LL_SPI_TransmitData8(hspi, TxData);
	retry = 0;

	/* Check if Rx buffer is not empty */
	while (!LL_SPI_IsActiveFlag_RXNE(hspi))
	{
		retry++;
		if(retry > 200) return 0;
	}

	/* received byte from SPI lines. */
	return LL_SPI_ReceiveData8(hspi);
}

void SPI::SPI_Recesive(SPI_TypeDef *hspi,uint8_t *pData,uint16_t Size)
{
	for(uint16_t i=0; i<Size; i++)
	{
		pData[i] = SPI_ReadWriteByte(hspi,0);
	}
}


/************************ (C) COPYRIGHT Longmen Drone Team *****END OF FILE****/


