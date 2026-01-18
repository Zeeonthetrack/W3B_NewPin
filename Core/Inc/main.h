/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
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
#include "stm32f1xx_hal.h"

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
#define Lazer_Pin GPIO_PIN_0
#define Lazer_GPIO_Port GPIOA
#define BT_TX_Pin GPIO_PIN_2
#define BT_TX_GPIO_Port GPIOA
#define BT_RX_Pin GPIO_PIN_3
#define BT_RX_GPIO_Port GPIOA
#define RBIN2_Pin GPIO_PIN_0
#define RBIN2_GPIO_Port GPIOB
#define LAIN1_Pin GPIO_PIN_13
#define LAIN1_GPIO_Port GPIOB
#define LAIN2_Pin GPIO_PIN_14
#define LAIN2_GPIO_Port GPIOB
#define LBIN1_Pin GPIO_PIN_15
#define LBIN1_GPIO_Port GPIOB
#define LBIN2_Pin GPIO_PIN_8
#define LBIN2_GPIO_Port GPIOA
#define RAIN1_Pin GPIO_PIN_3
#define RAIN1_GPIO_Port GPIOB
#define RAIN2_Pin GPIO_PIN_4
#define RAIN2_GPIO_Port GPIOB
#define RBIN1_Pin GPIO_PIN_5
#define RBIN1_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
