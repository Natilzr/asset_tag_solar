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
#include "stm32f0xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */
#define RTC_CLOCK_SOURCE_LSI
/*#define RTC_CLOCK_SOURCE_LSE*/

#ifdef RTC_CLOCK_SOURCE_LSI
#define RTC_ASYNCH_PREDIV    0x7F
#define RTC_SYNCH_PREDIV     0x00F9
#endif

#ifdef RTC_CLOCK_SOURCE_LSE
#define RTC_ASYNCH_PREDIV  0x7F
#define RTC_SYNCH_PREDIV   0x00FF
#endif
  
  
  #define I2C_DELAY       30
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
#define LED_Pin GPIO_PIN_5
#define LED_GPIO_Port GPIOA
/* USER CODE BEGIN Private defines */
#define RF_SCL GPIO_PIN_0
#define RF_SCL_GPIO_Port GPIOF
#define RF_SDA GPIO_PIN_1
#define RF_SDA_GPIO_Port GPIOF

#define INT_LIS GPIO_PIN_0
#define INT_LIS_GPIO_Port GPIOA
#define ADC_I GPIO_PIN_1
#define ADC_GPIO_Port GPIOA
#define RX_Pin GPIO_PIN_2
#define RX_Pin_GPIO_Port GPIOA
#define TX_Pin GPIO_PIN_3
#define TX_Pin_GPIO_Port GPIOA
#define FREE_Pin GPIO_PIN_4
#define FREE_GPIO_Port GPIOA
#define IN2 GPIO_PIN_6
#define IN2_GPIO_Port GPIOA
#define IN1 GPIO_PIN_7
#define IN1_GPIO_Port GPIOA
#define SCL GPIO_PIN_9
#define SCL_GPIO_Port GPIOA
#define SDA GPIO_PIN_10
#define SDA_GPIO_Port GPIOA
#define SWDIO GPIO_PIN_13
#define SWDIO_GPIO_Port GPIOA
#define SWCLK GPIO_PIN_14
#define SWCLK_GPIO_Port GPIOA

#define GPIO3 GPIO_PIN_1
#define GPIO3_GPIO_Port GPIOB






/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
