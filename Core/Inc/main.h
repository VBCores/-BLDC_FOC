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
#include "stm32g4xx_hal.h"
#include "stm32g4xx_ll_cordic.h"
#include "stm32g4xx_ll_spi.h"
#include "stm32g4xx_ll_system.h"
#include "stm32g4xx_ll_gpio.h"
#include "stm32g4xx_ll_exti.h"
#include "stm32g4xx_ll_bus.h"
#include "stm32g4xx_ll_cortex.h"
#include "stm32g4xx_ll_rcc.h"
#include "stm32g4xx_ll_utils.h"
#include "stm32g4xx_ll_pwr.h"
#include "stm32g4xx_ll_dma.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */
typedef struct 
{
  uint8_t State; // Device state such as FOC, Calibration etc*/
  
  float RotorAngle;
  float ShaftAngle; // drive output shaft mechanical angle, rad
  float ShaftVelocity; // drive output shaft angular velocity, rad/s
  int32_t RotorTurns; // motor rotor rotation nuumber, int
  float ElecTheta; // motor electrical angle, electrical rad
  float ShaftTorque;

  // dynamically set FOC setpoints and gains
  float mech_SetP;
  float mech_gain;
  float vel_SetP;
  float vel_gain;
  float torq_SetP;
} DriverState_instance;

typedef struct 
{
  int16_t lookup[128];
  uint32_t elec_offset; // offset applied to encoder readings to match zero-point of encoder and rotor angle at which it's electrical angle is PI
  float extern_lookup[128];
  uint8_t gear_ratio;
} config_struct;
/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

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
#define DRV_INLA_Pin LL_GPIO_PIN_13
#define DRV_INLA_GPIO_Port GPIOB
#define DRV_INLB_Pin LL_GPIO_PIN_14
#define DRV_INLB_GPIO_Port GPIOB
#define DRV_INLC_Pin LL_GPIO_PIN_15
#define DRV_INLC_GPIO_Port GPIOB
#define SPI3_CS0_Pin LL_GPIO_PIN_15
#define SPI3_CS0_GPIO_Port GPIOA
#define DRV_EN_Pin LL_GPIO_PIN_3
#define DRV_EN_GPIO_Port GPIOB
#define DRV_FLT_Pin LL_GPIO_PIN_5
#define DRV_FLT_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
