/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
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
#include "stm32f4xx_hal.h"

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
typedef enum {false =0,true =!false} bool;
/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define IR_Tx_FR_Pin GPIO_PIN_13
#define IR_Tx_FR_GPIO_Port GPIOC
#define IR_Tx_SR_Pin GPIO_PIN_14
#define IR_Tx_SR_GPIO_Port GPIOC
#define IR_Tx_SL_Pin GPIO_PIN_15
#define IR_Tx_SL_GPIO_Port GPIOC
#define IR_Rx_FR_Pin GPIO_PIN_0
#define IR_Rx_FR_GPIO_Port GPIOA
#define IR_Rx_SR_Pin GPIO_PIN_1
#define IR_Rx_SR_GPIO_Port GPIOA
#define IR_Rx_SL_Pin GPIO_PIN_2
#define IR_Rx_SL_GPIO_Port GPIOA
#define MPU_INT_Pin GPIO_PIN_3
#define MPU_INT_GPIO_Port GPIOA
#define MPU_INT_EXTI_IRQn EXTI3_IRQn
#define MPU_CS_Pin GPIO_PIN_4
#define MPU_CS_GPIO_Port GPIOA
#define MPU_SCK_Pin GPIO_PIN_5
#define MPU_SCK_GPIO_Port GPIOA
#define MPU_MISO_Pin GPIO_PIN_6
#define MPU_MISO_GPIO_Port GPIOA
#define MPU_MOSI_Pin GPIO_PIN_7
#define MPU_MOSI_GPIO_Port GPIOA
#define IR_Rx_FL_Pin GPIO_PIN_0
#define IR_Rx_FL_GPIO_Port GPIOB
#define Battery_Pin GPIO_PIN_1
#define Battery_GPIO_Port GPIOB
#define IR_Tx_FL_Pin GPIO_PIN_10
#define IR_Tx_FL_GPIO_Port GPIOB
#define FM_CS_Pin GPIO_PIN_12
#define FM_CS_GPIO_Port GPIOB
#define FM_SCK_Pin GPIO_PIN_13
#define FM_SCK_GPIO_Port GPIOB
#define FM_MISO_Pin GPIO_PIN_14
#define FM_MISO_GPIO_Port GPIOB
#define FM_MOSI_Pin GPIO_PIN_15
#define FM_MOSI_GPIO_Port GPIOB
#define LED_Pin GPIO_PIN_11
#define LED_GPIO_Port GPIOA
#define START_Pin GPIO_PIN_12
#define START_GPIO_Port GPIOA
#define LENC_A_Pin GPIO_PIN_15
#define LENC_A_GPIO_Port GPIOA
#define LENC_B_Pin GPIO_PIN_3
#define LENC_B_GPIO_Port GPIOB
#define RENC_A_Pin GPIO_PIN_4
#define RENC_A_GPIO_Port GPIOB
#define RENC_B_Pin GPIO_PIN_5
#define RENC_B_GPIO_Port GPIOB
#define LMOT_B_Pin GPIO_PIN_6
#define LMOT_B_GPIO_Port GPIOB
#define LMOT_F_Pin GPIO_PIN_7
#define LMOT_F_GPIO_Port GPIOB
#define RMOT_B_Pin GPIO_PIN_8
#define RMOT_B_GPIO_Port GPIOB
#define RMOT_F_Pin GPIO_PIN_9
#define RMOT_F_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
