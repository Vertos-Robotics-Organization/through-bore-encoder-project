/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
#include "stm32g0xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */
typedef enum {
	ENCODER_STATUS_OK = 0,
	ENCODER_STATUS_CAN_TX_FIFO_FULL = 1,
	ENCODER_STATUS_NO_CANBUS = 2,
	ENCODER_STATUS_LOOP_OVERRUN = 3,
	ENCODER_STATUS_ERROR = 4

} ErrorCode;
/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define CAL_EN_Pin GPIO_PIN_2
#define CAL_EN_GPIO_Port GPIOB
#define CAN_NSTANDBY_Pin GPIO_PIN_3
#define CAN_NSTANDBY_GPIO_Port GPIOB
#define CAN_ENABLE_Pin GPIO_PIN_5
#define CAN_ENABLE_GPIO_Port GPIOB
#define CAN_NERROR_Pin GPIO_PIN_7
#define CAN_NERROR_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
