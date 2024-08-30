/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : adc.cpp
  * Description        :
  ******************************************************************************
  * Function           :
  *
  ******************************************************************************
  * Author             : Zhiping Fu
  * Creation Date      : Oct 22, 2020
  ******************************************************************************
  */
/* USER CODE END Header */

#include "power.hpp"


void DMA2_Stream0_IRQHandler(void)
{
	if(LL_DMA_IsActiveFlag_TC0(DMA2) == 1)
	{
		LL_ADC_Disable(ADC1);

		LL_DMA_DisableStream(DMA2, LL_DMA_STREAM_0);
		LL_DMA_ClearFlag_TC0(DMA2);
		LL_DMA_EnableStream(DMA2, LL_DMA_STREAM_0);
	}
}

void POWER::power_Init(void)
{
	Update = false;
	AdcRaw = 0;
	AdcRel = 0.0f;

	osDelay(200);
	LL_DMA_DisableStream(DMA2, LL_DMA_STREAM_0);
	LL_DMA_SetDataLength(DMA2, LL_DMA_STREAM_0, 1);
	LL_DMA_SetPeriphAddress(DMA2, LL_DMA_CHANNEL_0, LL_ADC_DMA_GetRegAddr(hadc, LL_ADC_DMA_REG_REGULAR_DATA));
	LL_DMA_SetMemoryAddress(DMA2, LL_DMA_STREAM_0, (uint32_t)&AdcRaw);
	LL_DMA_ClearFlag_TC0(DMA2);
	LL_DMA_EnableIT_TC(DMA2, LL_DMA_STREAM_0);
	LL_DMA_EnableStream(DMA2, LL_DMA_STREAM_0);
	LL_ADC_Enable(hadc);
	LL_ADC_REG_SetDMATransfer(hadc, LL_ADC_REG_DMA_TRANSFER_UNLIMITED);
	LL_ADC_REG_StartConversionSWStart(hadc);
}

float POWER::get_Voltage(void)
{
	AdcRel = AdcRaw/4096.0f*3.3f*(VOL_RES1+VOL_RES2)/VOL_RES2;
	return this->AdcRel;
}

void POWER::battery_Update()
{
	battery.battery_Voltage = get_Voltage();
	xQueueOverwrite(queueBattery,&battery);

	LL_ADC_Enable(hadc);
	LL_ADC_REG_SetDMATransfer(hadc, LL_ADC_REG_DMA_TRANSFER_UNLIMITED);
	LL_ADC_REG_StartConversionSWStart(hadc);
}

POWER power(ADC1,(char *)"battery",LL_ADC_CHANNEL_4);
extern "C" void battery_main(void *argument)
{
	power.power_Init();
	osDelay(200);
	for(;;)
	{
		osSemaphoreAcquire(semBattery,0xffffffff);
		power.battery_Update();
	}

}

/************************ (C) COPYRIGHT Longmen Drone Team *****END OF FILE****/
