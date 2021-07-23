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
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdlib.h>
#include <stdbool.h>
#include <string.h>
#include <stdarg.h>
#include <stdio.h>
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
#define LEDred_Pin GPIO_PIN_2
#define LEDred_GPIO_Port GPIOE
#define SWEL_Pin GPIO_PIN_3
#define SWEL_GPIO_Port GPIOE
#define LEDgreen_Pin GPIO_PIN_4
#define LEDgreen_GPIO_Port GPIOE
#define LEDblue_Pin GPIO_PIN_5
#define LEDblue_GPIO_Port GPIOE
#define HAPTIC_Pin GPIO_PIN_6
#define HAPTIC_GPIO_Port GPIOE
#define KEYenter_Pin GPIO_PIN_8
#define KEYenter_GPIO_Port GPIOI
#define KEYpagenext_Pin GPIO_PIN_13
#define KEYpagenext_GPIO_Port GPIOC
#define INTMODboot_Pin GPIO_PIN_9
#define INTMODboot_GPIO_Port GPIOI
#define LCDnRST_Pin GPIO_PIN_10
#define LCDnRST_GPIO_Port GPIOI
#define KEYpageprevious_Pin GPIO_PIN_11
#define KEYpageprevious_GPIO_Port GPIOI
#define SLIDER1_Pin GPIO_PIN_6
#define SLIDER1_GPIO_Port GPIOF
#define VBattery_Pin GPIO_PIN_7
#define VBattery_GPIO_Port GPIOF
#define EX1_Pin GPIO_PIN_8
#define EX1_GPIO_Port GPIOF
#define EX2_Pin GPIO_PIN_9
#define EX2_GPIO_Port GPIOF
#define TOUCH_RST_Pin GPIO_PIN_10
#define TOUCH_RST_GPIO_Port GPIOF
#define POT1_Pin GPIO_PIN_0
#define POT1_GPIO_Port GPIOC
#define POS6_Pin GPIO_PIN_1
#define POS6_GPIO_Port GPIOC
#define POT2_Pin GPIO_PIN_2
#define POT2_GPIO_Port GPIOC
#define SLIDER2_Pin GPIO_PIN_3
#define SLIDER2_GPIO_Port GPIOC
#define StickLH_Pin GPIO_PIN_0
#define StickLH_GPIO_Port GPIOA
#define StickLV_Pin GPIO_PIN_1
#define StickLV_GPIO_Port GPIOA
#define StickRH_Pin GPIO_PIN_2
#define StickRH_GPIO_Port GPIOA
#define TOUCH_INT_Pin GPIO_PIN_2
#define TOUCH_INT_GPIO_Port GPIOH
#define SWF_Pin GPIO_PIN_3
#define SWF_GPIO_Port GPIOH
#define SWEH_Pin GPIO_PIN_4
#define SWEH_GPIO_Port GPIOH
#define StickRV_Pin GPIO_PIN_3
#define StickRV_GPIO_Port GPIOA
#define Audio_Pin GPIO_PIN_4
#define Audio_GPIO_Port GPIOA
#define TESTPOINT_Pin GPIO_PIN_5
#define TESTPOINT_GPIO_Port GPIOA
#define TrimLHL_Pin GPIO_PIN_6
#define TrimLHL_GPIO_Port GPIOA
#define AudioMute_Pin GPIO_PIN_7
#define AudioMute_GPIO_Port GPIOA
#define TrimLHR_Pin GPIO_PIN_4
#define TrimLHR_GPIO_Port GPIOC
#define SDpresent_Pin GPIO_PIN_5
#define SDpresent_GPIO_Port GPIOC
#define UART6pwr_Pin GPIO_PIN_0
#define UART6pwr_GPIO_Port GPIOB
#define LCDbacklight_Pin GPIO_PIN_1
#define LCDbacklight_GPIO_Port GPIOB
#define SWAL_Pin GPIO_PIN_15
#define SWAL_GPIO_Port GPIOI
#define PWRswitch_Pin GPIO_PIN_0
#define PWRswitch_GPIO_Port GPIOJ
#define PWRon_Pin GPIO_PIN_1
#define PWRon_GPIO_Port GPIOJ
#define PCBREV1_Pin GPIO_PIN_7
#define PCBREV1_GPIO_Port GPIOH
#define PCBREV2_Pin GPIO_PIN_8
#define PCBREV2_GPIO_Port GPIOH
#define SWAH_Pin GPIO_PIN_9
#define SWAH_GPIO_Port GPIOH
#define ROTENCB_Pin GPIO_PIN_10
#define ROTENCB_GPIO_Port GPIOH
#define ROTENCA_Pin GPIO_PIN_11
#define ROTENCA_GPIO_Port GPIOH
#define SWBH_Pin GPIO_PIN_12
#define SWBH_GPIO_Port GPIOH
#define SWBL_Pin GPIO_PIN_12
#define SWBL_GPIO_Port GPIOB
#define TrimRSD_Pin GPIO_PIN_13
#define TrimRSD_GPIO_Port GPIOB
#define TrimRSU_Pin GPIO_PIN_14
#define TrimRSU_GPIO_Port GPIOB
#define SWCL_Pin GPIO_PIN_15
#define SWCL_GPIO_Port GPIOB
#define SWCH_Pin GPIO_PIN_11
#define SWCH_GPIO_Port GPIOD
#define HEARTBEAT_Pin GPIO_PIN_12
#define HEARTBEAT_GPIO_Port GPIOD
#define TrimLSU_Pin GPIO_PIN_13
#define TrimLSU_GPIO_Port GPIOD
#define SWDH_Pin GPIO_PIN_7
#define SWDH_GPIO_Port GPIOJ
#define TrimLSD_Pin GPIO_PIN_8
#define TrimLSD_GPIO_Port GPIOJ
#define SWDL_Pin GPIO_PIN_2
#define SWDL_GPIO_Port GPIOG
#define SWGL_Pin GPIO_PIN_3
#define SWGL_GPIO_Port GPIOG
#define SWGH_Pin GPIO_PIN_6
#define SWGH_GPIO_Port GPIOG
#define SWH_Pin GPIO_PIN_7
#define SWH_GPIO_Port GPIOG
#define TrainerIn_Pin GPIO_PIN_6
#define TrainerIn_GPIO_Port GPIOC
#define TrainerOut_Pin GPIO_PIN_7
#define TrainerOut_GPIO_Port GPIOC
#define IntModPwr_Pin GPIO_PIN_8
#define IntModPwr_GPIO_Port GPIOA
#define ExtModTX_Pin GPIO_PIN_10
#define ExtModTX_GPIO_Port GPIOA
#define SWI_Pin GPIO_PIN_14
#define SWI_GPIO_Port GPIOH
#define SWJ_Pin GPIO_PIN_15
#define SWJ_GPIO_Port GPIOH
#define UART3Pwr_Pin GPIO_PIN_15
#define UART3Pwr_GPIO_Port GPIOA
#define TrimR_Pin GPIO_PIN_3
#define TrimR_GPIO_Port GPIOD
#define TelemDir_Pin GPIO_PIN_4
#define TelemDir_GPIO_Port GPIOD
#define TrimRHR_Pin GPIO_PIN_7
#define TrimRHR_GPIO_Port GPIOD
#define TrimRV_Pin GPIO_PIN_12
#define TrimRV_GPIO_Port GPIOJ
#define TrimRVD_Pin GPIO_PIN_13
#define TrimRVD_GPIO_Port GPIOJ
#define TrimLVU_Pin GPIO_PIN_14
#define TrimLVU_GPIO_Port GPIOJ
#define BluetoothEn_Pin GPIO_PIN_10
#define BluetoothEn_GPIO_Port GPIOG
#define USBchaCtrl_Pin GPIO_PIN_11
#define USBchaCtrl_GPIO_Port GPIOG
#define TrimLVD_Pin GPIO_PIN_12
#define TrimLVD_GPIO_Port GPIOG
#define USBchgDetect_Pin GPIO_PIN_13
#define USBchgDetect_GPIO_Port GPIOG
#define ExtModPwr_Pin GPIO_PIN_3
#define ExtModPwr_GPIO_Port GPIOB
#define TrainerDetect_Pin GPIO_PIN_4
#define TrainerDetect_GPIO_Port GPIOB
#define KEYrtn_Pin GPIO_PIN_4
#define KEYrtn_GPIO_Port GPIOI
#define KEYtelem_Pin GPIO_PIN_5
#define KEYtelem_GPIO_Port GPIOI
#define KEYmdl_Pin GPIO_PIN_6
#define KEYmdl_GPIO_Port GPIOI
#define KEYsys_Pin GPIO_PIN_7
#define KEYsys_GPIO_Port GPIOI
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
