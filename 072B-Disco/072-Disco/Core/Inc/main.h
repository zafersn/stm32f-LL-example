/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
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
#include "stm32f0xx_ll_i2c.h"
#include "stm32f0xx_ll_crs.h"
#include "stm32f0xx_ll_rcc.h"
#include "stm32f0xx_ll_bus.h"
#include "stm32f0xx_ll_system.h"
#include "stm32f0xx_ll_exti.h"
#include "stm32f0xx_ll_cortex.h"
#include "stm32f0xx_ll_utils.h"
#include "stm32f0xx_ll_pwr.h"
#include "stm32f0xx_ll_dma.h"
#include "stm32f0xx_ll_spi.h"
#include "stm32f0xx_ll_tim.h"
#include "stm32f0xx_ll_gpio.h"

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
#define NCS_MEMS_SPI_Pin LL_GPIO_PIN_0
#define NCS_MEMS_SPI_GPIO_Port GPIOC
#define MEMS_INT1_Pin LL_GPIO_PIN_1
#define MEMS_INT1_GPIO_Port GPIOC
#define MEMS_INT2_Pin LL_GPIO_PIN_2
#define MEMS_INT2_GPIO_Port GPIOC
#define B1_Pin LL_GPIO_PIN_0
#define B1_GPIO_Port GPIOA
#define EXT_RESET_Pin LL_GPIO_PIN_5
#define EXT_RESET_GPIO_Port GPIOC
#define DHT11_IN_Pin LL_GPIO_PIN_2
#define DHT11_IN_GPIO_Port GPIOB
#define I2C2_SCL_Pin LL_GPIO_PIN_10
#define I2C2_SCL_GPIO_Port GPIOB
#define I2C2_SDA_Pin LL_GPIO_PIN_11
#define I2C2_SDA_GPIO_Port GPIOB
#define SPI2_SCK_Pin LL_GPIO_PIN_13
#define SPI2_SCK_GPIO_Port GPIOB
#define SPI2_MISO_Pin LL_GPIO_PIN_14
#define SPI2_MISO_GPIO_Port GPIOB
#define SPI2_MOSI_Pin LL_GPIO_PIN_15
#define SPI2_MOSI_GPIO_Port GPIOB
#define LD3_Pin LL_GPIO_PIN_6
#define LD3_GPIO_Port GPIOC
#define LD6_Pin LL_GPIO_PIN_7
#define LD6_GPIO_Port GPIOC
#define LD4_Pin LL_GPIO_PIN_8
#define LD4_GPIO_Port GPIOC
#define LD5_Pin LL_GPIO_PIN_9
#define LD5_GPIO_Port GPIOC
#define TEST_INPUT_Pin LL_GPIO_PIN_8
#define TEST_INPUT_GPIO_Port GPIOA
#define USBF4_DM_Pin LL_GPIO_PIN_11
#define USBF4_DM_GPIO_Port GPIOA
#define USBF4_DP_Pin LL_GPIO_PIN_12
#define USBF4_DP_GPIO_Port GPIOA
#define SWDIO_Pin LL_GPIO_PIN_13
#define SWDIO_GPIO_Port GPIOA
#define SWCLK_Pin LL_GPIO_PIN_14
#define SWCLK_GPIO_Port GPIOA
void   MX_I2C2_Init(void);
/* USER CODE BEGIN Private defines */

#define LE_DC6_Pin LL_GPIO_PIN_6
#define LE_DC6_GPIO_Port GPIOC
#define LE_DC7_Pin LL_GPIO_PIN_7
#define LE_DC7_GPIO_Port GPIOC
#define LE_DC8_Pin LL_GPIO_PIN_8
#define LE_DC8_GPIO_Port GPIOC


#define DHT11_GPIO_CLK_ENABLE()             LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOB)

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
