/* USER CODE BEGIN Header */
/**
  ******************************************************************************
 *  Created on: Apr 18, 2025
 *      Author: Phoenix Cardwell
 *      UCONN Formula SAE 2025
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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


/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "screen.h"
#include "screenHandle.h"


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
#define ECT_Warning_Output_Pin GPIO_PIN_0
#define ECT_Warning_Output_GPIO_Port GPIOB
#define OilP_Warning_Output_Pin GPIO_PIN_1
#define OilP_Warning_Output_GPIO_Port GPIOB
#define UserInput1_Pin GPIO_PIN_10
#define UserInput1_GPIO_Port GPIOB
#define UserInput2_Pin GPIO_PIN_11
#define UserInput2_GPIO_Port GPIOB
#define Debug_LED_Pin GPIO_PIN_15
#define Debug_LED_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */
extern void display_FSAE_bootscreen();


/* USER CODE END Private defines */


#endif /* __MAIN_H */
