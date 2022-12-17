/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
	#pragma once 
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32l1xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */
void write(uint8_t addr, uint32_t regVal);
uint8_t calcCRC(uint8_t datagram[], uint8_t len);
/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define PC15_OSC32_OUT_Pin GPIO_PIN_15
#define PC15_OSC32_OUT_GPIO_Port GPIOC
#define knDown_Pin GPIO_PIN_3
#define knDown_GPIO_Port GPIOC
#define knUp_Pin GPIO_PIN_1
#define knUp_GPIO_Port GPIOC

#define datTop_Pin GPIO_PIN_2
#define datTop_GPIO_Port GPIOC
#define datBot_Pin GPIO_PIN_0
#define datBot_GPIO_Port GPIOC
#define proj_Pin GPIO_PIN_1
#define proj_GPIO_Port GPIOA
#define enable_Pin GPIO_PIN_9
#define enable_GPIO_Port GPIOA
#define dir_Pin GPIO_PIN_10
#define dir_GPIO_Port GPIOA
#define step_Pin GPIO_PIN_11
#define step_GPIO_Port GPIOA
#define LD4_Pin GPIO_PIN_6
#define LD4_GPIO_Port GPIOB
#define LD3_Pin GPIO_PIN_7
#define LD3_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
