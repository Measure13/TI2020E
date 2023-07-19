/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "gpio.h"
#include <stdio.h>
#include <stdbool.h>
#include <string.h>
#include <stdlib.h>
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */
#define MAX_DATA_NUM_FFT 256
#define MAX_DATA_NUM_SPC 256
#define SAMPLE_RATE_FFT 12800
#define SAMPLE_RATE_SPC 64000

#define ACCEPT_PROPORTION 0.001f
#define ACCEPT_CNT_RANGE 4
#define ACCEPT_DIFF 0.02f
#define NOISE_LIMIT 0.02f

#define REMEASURE_NUM 20
#define TOP_SMOOTH_NUM 8
#define CROSS_OVER_ZEROS 24
#define DISTORTION_LIMIT 0.3f

#define TOP_DISTORTION 0
#define BOTTOM_DISTORTION 1
#define BOTH_DISTORTION 2
#define CO_DISTORTION 3
#define NO_DISTORTION 5
#define OTHER_DISTORTION 6

#define RELAY_DELAY 100

#define NON_D HAL_GPIO_WritePin(GPIOA, TOP_Pin|BTM_Pin|BTH_Pin|CRS_Pin, GPIO_PIN_RESET)
#define TOP_D do \
              { \
                HAL_GPIO_WritePin(GPIOA, BTM_Pin|BTH_Pin|CRS_Pin, GPIO_PIN_RESET);\
                HAL_GPIO_WritePin(GPIOA, TOP_Pin, GPIO_PIN_SET);\
              } while (false)
#define BTM_D do \
              { \
                HAL_GPIO_WritePin(GPIOA, TOP_Pin|BTH_Pin|CRS_Pin, GPIO_PIN_RESET);\
                HAL_GPIO_WritePin(GPIOA, BTM_Pin, GPIO_PIN_SET);\
              } while (false)
#define BTH_D do \
              { \
                HAL_GPIO_WritePin(GPIOA, TOP_Pin|BTM_Pin|CRS_Pin, GPIO_PIN_RESET);\
                HAL_GPIO_WritePin(GPIOA, BTH_Pin, GPIO_PIN_SET);\
              } while (false)
#define CRS_D do \
              { \
                HAL_GPIO_WritePin(GPIOA, TOP_Pin|BTH_Pin|BTM_Pin, GPIO_PIN_RESET);\
                HAL_GPIO_WritePin(GPIOA, CRS_Pin, GPIO_PIN_SET);\
              } while (false)
/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define TOP_Pin GPIO_PIN_11
#define TOP_GPIO_Port GPIOA
#define BTM_Pin GPIO_PIN_12
#define BTM_GPIO_Port GPIOA
#define BTH_Pin GPIO_PIN_13
#define BTH_GPIO_Port GPIOA
#define CRS_Pin GPIO_PIN_14
#define CRS_GPIO_Port GPIOA
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
