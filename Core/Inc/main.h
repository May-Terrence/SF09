/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f7xx_hal.h"
#include "stm32f7xx_ll_adc.h"
#include "stm32f7xx_ll_dma.h"
#include "stm32f7xx_ll_rcc.h"
#include "stm32f7xx_ll_bus.h"
#include "stm32f7xx_ll_system.h"
#include "stm32f7xx_ll_exti.h"
#include "stm32f7xx_ll_cortex.h"
#include "stm32f7xx_ll_utils.h"
#include "stm32f7xx_ll_pwr.h"
#include "stm32f7xx_ll_spi.h"
#include "stm32f7xx_ll_tim.h"
#include "stm32f7xx_ll_usart.h"
#include "stm32f7xx_ll_gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define LED1_Pin LL_GPIO_PIN_2
#define LED1_GPIO_Port GPIOE
#define LED2_Pin LL_GPIO_PIN_3
#define LED2_GPIO_Port GPIOE
#define LDE3_Pin LL_GPIO_PIN_4
#define LDE3_GPIO_Port GPIOE
#define LED4_Pin LL_GPIO_PIN_5
#define LED4_GPIO_Port GPIOE
#define SPI4_NSS_G_Pin LL_GPIO_PIN_9
#define SPI4_NSS_G_GPIO_Port GPIOE
#define SPI4_NSS_AM_Pin LL_GPIO_PIN_10
#define SPI4_NSS_AM_GPIO_Port GPIOE
#define SPI4_NSS_M_Pin LL_GPIO_PIN_11
#define SPI4_NSS_M_GPIO_Port GPIOE
#define SPI4_NSS_AG_Pin LL_GPIO_PIN_15
#define SPI4_NSS_AG_GPIO_Port GPIOE
#define SPI2_NSS_Pin LL_GPIO_PIN_12
#define SPI2_NSS_GPIO_Port GPIOB
#define SPI1_NSS_Pin LL_GPIO_PIN_7
#define SPI1_NSS_GPIO_Port GPIOD
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
