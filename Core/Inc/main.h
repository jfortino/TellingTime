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
#include "stm32l0xx_hal.h"

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
#define VIB_CTRL_Pin GPIO_PIN_0
#define VIB_CTRL_GPIO_Port GPIOA
#define SPO2_INT_Pin GPIO_PIN_1
#define SPO2_INT_GPIO_Port GPIOA
#define SPO2_INT_EXTI_IRQn EXTI0_1_IRQn
#define FLASH_WP_Pin GPIO_PIN_2
#define FLASH_WP_GPIO_Port GPIOA
#define TIME_BTN_Pin GPIO_PIN_3
#define TIME_BTN_GPIO_Port GPIOA
#define TIME_BTN_EXTI_IRQn EXTI2_3_IRQn
#define HR_BTN_Pin GPIO_PIN_4
#define HR_BTN_GPIO_Port GPIOA
#define HR_BTN_EXTI_IRQn EXTI4_15_IRQn
#define FLASH_SCK_Pin GPIO_PIN_5
#define FLASH_SCK_GPIO_Port GPIOA
#define FLASH_MISO_Pin GPIO_PIN_6
#define FLASH_MISO_GPIO_Port GPIOA
#define FLASH_MOSI_Pin GPIO_PIN_7
#define FLASH_MOSI_GPIO_Port GPIOA
#define FLASH_CS_Pin GPIO_PIN_0
#define FLASH_CS_GPIO_Port GPIOB
#define FLASH_RST_Pin GPIO_PIN_1
#define FLASH_RST_GPIO_Port GPIOB
#define SPO2_BTN_Pin GPIO_PIN_2
#define SPO2_BTN_GPIO_Port GPIOB
#define SPO2_BTN_EXTI_IRQn EXTI2_3_IRQn
#define UP_BTN_Pin GPIO_PIN_10
#define UP_BTN_GPIO_Port GPIOB
#define UP_BTN_EXTI_IRQn EXTI4_15_IRQn
#define DOWN_BTN_Pin GPIO_PIN_11
#define DOWN_BTN_GPIO_Port GPIOB
#define DOWN_BTN_EXTI_IRQn EXTI4_15_IRQn
#define SD_MODE_Pin GPIO_PIN_14
#define SD_MODE_GPIO_Port GPIOB
#define SPO2_SCL_Pin GPIO_PIN_6
#define SPO2_SCL_GPIO_Port GPIOB
#define SPO2_SDA_Pin GPIO_PIN_7
#define SPO2_SDA_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
