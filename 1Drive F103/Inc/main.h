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
#include "stm32f1xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "TransmitComand.h"
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */
void RxFunction(void)	;
char MotorControl (void);
void RxAnswerFunction(void );
void CalobrationPawMaxize(void);
void CalibrationMinDelayAndRange(void);
uint8_t CanTxMsg(uint64_t  Data, uint16_t Addres, uint8_t size);
void ScanBox (void);
/* USER CODE END EM */

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */
	#define    DWT_CYCCNT    *(volatile uint32_t*)0xE0001004
  #define    DWT_CONTROL   *(volatile uint32_t*)0xE0001000
  #define    SCB_DEMCR     *(volatile uint32_t*)0xE000EDFC

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define LED_Pin GPIO_PIN_13
#define LED_GPIO_Port GPIOC
#define RF2_Pin GPIO_PIN_14
#define RF2_GPIO_Port GPIOC
#define RF1_Pin GPIO_PIN_15
#define RF1_GPIO_Port GPIOC
#define PAW4CHECK_Pin GPIO_PIN_0
#define PAW4CHECK_GPIO_Port GPIOA
#define PAW3CHECK_Pin GPIO_PIN_1
#define PAW3CHECK_GPIO_Port GPIOA
#define PAW2CHECK_Pin GPIO_PIN_2
#define PAW2CHECK_GPIO_Port GPIOA
#define PAW1CHECK_Pin GPIO_PIN_3
#define PAW1CHECK_GPIO_Port GPIOA
#define DETECT34_Pin GPIO_PIN_4
#define DETECT34_GPIO_Port GPIOA
#define DETECT12_Pin GPIO_PIN_5
#define DETECT12_GPIO_Port GPIOA
#define SWHUCK4_Pin GPIO_PIN_6
#define SWHUCK4_GPIO_Port GPIOA
#define SWHUCK3_Pin GPIO_PIN_7
#define SWHUCK3_GPIO_Port GPIOA
#define SWHUCK2_Pin GPIO_PIN_0
#define SWHUCK2_GPIO_Port GPIOB
#define SWHUCK1_Pin GPIO_PIN_1
#define SWHUCK1_GPIO_Port GPIOB
#define ServoPwr_Pin GPIO_PIN_12
#define ServoPwr_GPIO_Port GPIOB
#define DIR_Pin GPIO_PIN_13
#define DIR_GPIO_Port GPIOB
#define STEP_Pin GPIO_PIN_14
#define STEP_GPIO_Port GPIOB
#define ENABLE2_Pin GPIO_PIN_15
#define ENABLE2_GPIO_Port GPIOB
#define CONVENABLE_Pin GPIO_PIN_8
#define CONVENABLE_GPIO_Port GPIOA
#define CONVSTEP_Pin GPIO_PIN_9
#define CONVSTEP_GPIO_Port GPIOA
#define CONVDIR_Pin GPIO_PIN_10
#define CONVDIR_GPIO_Port GPIOA
#define PAW34MaxRange_Pin GPIO_PIN_15
#define PAW34MaxRange_GPIO_Port GPIOA
#define PAW12MaxRange_Pin GPIO_PIN_3
#define PAW12MaxRange_GPIO_Port GPIOB
#define ENABLE1_Pin GPIO_PIN_4
#define ENABLE1_GPIO_Port GPIOB
#define PowerMon_Pin GPIO_PIN_5
#define PowerMon_GPIO_Port GPIOB
#define PowerMon_EXTI_IRQn EXTI9_5_IRQn
#define RF4_Pin GPIO_PIN_8
#define RF4_GPIO_Port GPIOB
#define RF3_Pin GPIO_PIN_9
#define RF3_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
