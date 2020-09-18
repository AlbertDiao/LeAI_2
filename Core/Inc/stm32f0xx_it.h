/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32f0xx_it.h
  * @brief   This file contains the headers of the interrupt handlers.
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
#ifndef __STM32F0xx_IT_H
#define __STM32F0xx_IT_H

#ifdef __cplusplus
 extern "C" {
#endif 

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "main.h"
#include <stdbool.h>
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */
#define BUF_LEN 300
#define UART_FIFO_LEN 5

typedef struct 
{
  uint8_t dat[BUF_LEN];
  uint16_t len;
}stu_uartDat;

typedef struct 
{
  stu_uartDat buf[UART_FIFO_LEN];
  uint8_t header; //队列头部指针,指向第一个可读元素
  uint8_t tail; //队列尾部指针，指向可读元素之后的第一个空元素，当header和tail重合时，说明队列为空。当队列溢出时不存数据
}stu_uartFifo;

extern uint8_t uart1_buf[BUF_LEN];
//extern uint8_t uart2_buf[BUF_LEN];
extern uint8_t uart3_buf[BUF_LEN];
extern bool uart1_recv,uart2_recv,uart3_recv;
extern uint32_t uart1_recv_len,uart2_recv_len,uart3_recv_len;
extern uint32_t uart1_dma_len, bc_uart_dma_len, uart3_dma_len;
extern stu_uartFifo bc_uart_fifo;
/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void NMI_Handler(void);
void HardFault_Handler(void);
void SVC_Handler(void);
void PendSV_Handler(void);
void SysTick_Handler(void);
void USART1_IRQHandler(void);
void USART3_4_IRQHandler(void);
/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

#ifdef __cplusplus
}
#endif

#endif /* __STM32F0xx_IT_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
