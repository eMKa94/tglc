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
#include "stm32g0xx_hal.h"

#include "stm32g0xx_ll_adc.h"
#include "stm32g0xx_ll_rcc.h"
#include "stm32g0xx_ll_bus.h"
#include "stm32g0xx_ll_system.h"
#include "stm32g0xx_ll_exti.h"
#include "stm32g0xx_ll_cortex.h"
#include "stm32g0xx_ll_utils.h"
#include "stm32g0xx_ll_pwr.h"
#include "stm32g0xx_ll_dma.h"
#include "stm32g0xx_ll_spi.h"
#include "stm32g0xx_ll_tim.h"
#include "stm32g0xx_ll_gpio.h"

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
#define PWM_DIG_4_Pin LL_GPIO_PIN_6
#define PWM_DIG_4_GPIO_Port GPIOA
#define PWM_DIG_3_Pin LL_GPIO_PIN_7
#define PWM_DIG_3_GPIO_Port GPIOA
#define PWM_DIG_2_Pin LL_GPIO_PIN_0
#define PWM_DIG_2_GPIO_Port GPIOB
#define PWM_DIG_1_Pin LL_GPIO_PIN_1
#define PWM_DIG_1_GPIO_Port GPIOB
#define DISP_RESET_Pin LL_GPIO_PIN_15
#define DISP_RESET_GPIO_Port GPIOA
#define DISPLAY_SCK_Pin LL_GPIO_PIN_3
#define DISPLAY_SCK_GPIO_Port GPIOB
#define PUSH_OUT_Pin LL_GPIO_PIN_4
#define PUSH_OUT_GPIO_Port GPIOB
#define DISPLAY_MOSI_Pin LL_GPIO_PIN_5
#define DISPLAY_MOSI_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
