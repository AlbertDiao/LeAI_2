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
#include "stdbool.h"
#include "bc20.h"
#include "track.h"

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */
/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */
//extern IWDG_HandleTypeDef hiwdg;
//#define FEED_DOG  __HAL_IWDG_RELOAD_COUNTER(&hiwdg)
#define FEED_DOG 

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
/* USER CODE BEGIN Private defines */
#define MSG_LEN 100

char *get_word(char *word, char word_len_max, char *str, char token);
extern char msg[MSG_LEN];


typedef struct
{
  uint8_t step;
  bool wait_stop;//等待车辆停止
}stu_father;

typedef struct
{
  uint8_t step;
  uint32_t times;
  uint32_t uc_times; //没有捕获到电流的次数
  bool need;
}stu_need_path;

typedef struct
{
  uint8_t step;
  bool to_stop;
  bool stopped;
}stu_bms_read;

typedef struct
{
  uint8_t step;
  bool to_stop;
  bool stopped;
}stu_bms_upload;

extern stu_track_t track;
typedef struct
{
  uint32_t time;
  uint8_t l_bms_read;
  uint8_t l_bms_upload;
  uint8_t l_gnss_read;
  uint8_t l_gnss_upload;
  uint8_t l_track;
  bool bms_data_new;
}stu_daemon;

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
