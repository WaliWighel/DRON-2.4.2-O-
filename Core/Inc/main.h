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
#include "stm32h7xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "BMP180.h"
#include "NRF24_Defs.h"
#include "ESC.h"
#include "NRF24.h"
#include "MPU6050.h"
#include <stdio.h>
#include <stdlib.h>
#include "string.h"
#include "dron.h"
#include "math.h"
#include "Kalibracjia silnikow.h"
#include "USART komendy.h"
#include "HMC5883L.h"
#include <stdbool.h>
#include "MadgwickAHRS.h"
#include "HC-SR04.h"
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
#define LED_6_Pin GPIO_PIN_3
#define LED_6_GPIO_Port GPIOE
#define HC_SR04_Echo_Pin GPIO_PIN_1
#define HC_SR04_Echo_GPIO_Port GPIOA
#define HC_SR04_Echo_EXTI_IRQn EXTI1_IRQn
#define HC_SR04_Triger_Pin GPIO_PIN_3
#define HC_SR04_Triger_GPIO_Port GPIOA
#define BUZZ_Pin GPIO_PIN_0
#define BUZZ_GPIO_Port GPIOG
#define LED_G_Pin GPIO_PIN_15
#define LED_G_GPIO_Port GPIOE
#define LED_Y_Pin GPIO_PIN_10
#define LED_Y_GPIO_Port GPIOB
#define LED_R_Pin GPIO_PIN_11
#define LED_R_GPIO_Port GPIOB
#define uSD_LED_Pin GPIO_PIN_11
#define uSD_LED_GPIO_Port GPIOD
#define uSD_SC_Pin GPIO_PIN_15
#define uSD_SC_GPIO_Port GPIOD
#define ESC_Power_Pin GPIO_PIN_8
#define ESC_Power_GPIO_Port GPIOG
#define LED_7_Pin GPIO_PIN_8
#define LED_7_GPIO_Port GPIOA
#define LED_5_Pin GPIO_PIN_15
#define LED_5_GPIO_Port GPIOA
#define NRF24_IRQ_Pin GPIO_PIN_15
#define NRF24_IRQ_GPIO_Port GPIOG
#define NRF24_IRQ_EXTI_IRQn EXTI15_10_IRQn
#define NRF24_CE_Pin GPIO_PIN_6
#define NRF24_CE_GPIO_Port GPIOB
#define NRF24_CSN_Pin GPIO_PIN_7
#define NRF24_CSN_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */
typedef struct Inactivity_Struct{
	uint32_t Counter;
	uint32_t Compare_Value;// = 12937130;
	uint8_t Inact_Percent;
}Inactivity_Struct;

void nRF24_WriteTXPayload_IT(uint8_t * data);
void FPU_IRQ_CallBack(void);
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
