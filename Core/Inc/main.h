/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
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
#include "stm32f2xx_hal.h"

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
#define LED_KLINE_Pin GPIO_PIN_13
#define LED_KLINE_GPIO_Port GPIOC
#define USART1_LO_Pin GPIO_PIN_8
#define USART1_LO_GPIO_Port GPIOA
#define SDIO_DET_Pin GPIO_PIN_15
#define SDIO_DET_GPIO_Port GPIOA
#define LED_SDIO_Pin GPIO_PIN_4
#define LED_SDIO_GPIO_Port GPIOB
#define LED_USB_Pin GPIO_PIN_5
#define LED_USB_GPIO_Port GPIOB
#define LED_CAN_Pin GPIO_PIN_6
#define LED_CAN_GPIO_Port GPIOB
#define CAN1_LBK_Pin GPIO_PIN_7
#define CAN1_LBK_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
