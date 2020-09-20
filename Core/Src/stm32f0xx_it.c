/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32f0xx_it.c
  * @brief   Interrupt Service Routines.
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

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f0xx_it.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>

#include "task.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */

/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
 
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */
uint8_t uart1_buf[BUF_LEN];
//uint8_t uart2_buf[BUF_LEN];
//uint8_t uart3_buf[BUF_LEN];

stu_uartFifo bc_uart_fifo;
bool uart1_recv, bc_uart_recv, bms_uart_recv;
uint32_t uart1_recv_len, bc_uart_recv_len, bms_uart_recv_len;
uint32_t uart1_dma_len, bc_uart_dma_len, bms_uart_dma_len;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern DMA_HandleTypeDef hdma_usart1_rx;
extern DMA_HandleTypeDef hdma_usart1_tx;
extern DMA_HandleTypeDef hdma_usart3_rx;
extern DMA_HandleTypeDef hdma_usart3_tx;
extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart3;
/* USER CODE BEGIN EV */

/* USER CODE END EV */

/******************************************************************************/
/*           Cortex-M0 Processor Interruption and Exception Handlers          */ 
/******************************************************************************/
/**
  * @brief This function handles Non maskable interrupt.
  */
void NMI_Handler(void)
{
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */

  /* USER CODE END NonMaskableInt_IRQn 0 */
  /* USER CODE BEGIN NonMaskableInt_IRQn 1 */

  /* USER CODE END NonMaskableInt_IRQn 1 */
}

/**
  * @brief This function handles Hard fault interrupt.
  */
void HardFault_Handler(void)
{
  /* USER CODE BEGIN HardFault_IRQn 0 */

  /* USER CODE END HardFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_HardFault_IRQn 0 */
    /* USER CODE END W1_HardFault_IRQn 0 */
  }
}

/**
  * @brief This function handles System service call via SWI instruction.
  */
void SVC_Handler(void)
{
  /* USER CODE BEGIN SVC_IRQn 0 */

  /* USER CODE END SVC_IRQn 0 */
  /* USER CODE BEGIN SVC_IRQn 1 */

  /* USER CODE END SVC_IRQn 1 */
}

/**
  * @brief This function handles Pendable request for system service.
  */
void PendSV_Handler(void)
{
  /* USER CODE BEGIN PendSV_IRQn 0 */

  /* USER CODE END PendSV_IRQn 0 */
  /* USER CODE BEGIN PendSV_IRQn 1 */

  /* USER CODE END PendSV_IRQn 1 */
}

/**
  * @brief This function handles System tick timer.
  */
void SysTick_Handler(void)
{
  /* USER CODE BEGIN SysTick_IRQn 0 */
  systick_count++;

  /* USER CODE END SysTick_IRQn 0 */
  HAL_IncTick();
  /* USER CODE BEGIN SysTick_IRQn 1 */

  /* USER CODE END SysTick_IRQn 1 */
}

/******************************************************************************/
/* STM32F0xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32f0xx.s).                    */
/******************************************************************************/

/**
  * @brief This function handles DMA1 channel 2 and 3 interrupts.
  */
void DMA1_Channel2_3_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Channel2_3_IRQn 0 */

  /* USER CODE END DMA1_Channel2_3_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_usart3_tx);
  HAL_DMA_IRQHandler(&hdma_usart3_rx);
  /* USER CODE BEGIN DMA1_Channel2_3_IRQn 1 */

  /* USER CODE END DMA1_Channel2_3_IRQn 1 */
}

/**
  * @brief This function handles DMA1 channel 4, 5, 6 and 7 interrupts.
  */
void DMA1_Channel4_5_6_7_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Channel4_5_6_7_IRQn 0 */

  /* USER CODE END DMA1_Channel4_5_6_7_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_usart1_tx);
  HAL_DMA_IRQHandler(&hdma_usart1_rx);
  /* USER CODE BEGIN DMA1_Channel4_5_6_7_IRQn 1 */

  /* USER CODE END DMA1_Channel4_5_6_7_IRQn 1 */
}

/**
  * @brief This function handles USART1 global interrupt / USART1 wake-up interrupt through EXTI line 25.
  */
void USART1_IRQHandler(void)
{
  /* USER CODE BEGIN USART1_IRQn 0 */
  uint32_t tmp_flag = 0;

  /* USER CODE END USART1_IRQn 0 */
  HAL_UART_IRQHandler(&huart1);

  /* USER CODE BEGIN USART1_IRQn 1 */
    tmp_flag = __HAL_UART_GET_FLAG(&huart1, UART_FLAG_IDLE);
    if ((tmp_flag != RESET))
    {
        HAL_UART_DMAStop(&huart1);

        __HAL_UART_CLEAR_IDLEFLAG(&huart1);

        HAL_UART_Receive_DMA(&huart1, uart1_buf, BUF_LEN); //重新打开DMA接收
    }
  /* USER CODE END USART1_IRQn 1 */
}

/**
  * @brief This function handles USART3 and USART4 global interrupts.
  */
void USART3_4_IRQHandler(void)
{
  /* USER CODE BEGIN USART3_4_IRQn 0 */
  uint32_t tmp_flag = 0;
  uint32_t temp;
  char *str;
  /* USER CODE END USART3_4_IRQn 0 */
  HAL_UART_IRQHandler(&huart3);
  /* USER CODE BEGIN USART3_4_IRQn 1 */
//尾部维护和数据接�?
  if (USART3 == huart3.Instance)
  {
    tmp_flag = __HAL_UART_GET_FLAG(&huart3, UART_FLAG_IDLE);
    if ((tmp_flag != RESET))
    {
      HAL_UART_DMAStop(&huart3);
      __HAL_UART_CLEAR_IDLEFLAG(&huart3);
      //判断fifo是否已经满了
      //将数据保�?
      str = strstr((char*)bc_uart_fifo.buf[bc_uart_fifo.tail].dat, "+MIPLEXECUTE:");
      temp = BUF_LEN - __HAL_DMA_GET_COUNTER(&hdma_usart3_rx); //获取DMA中未传输的数据个�?
      if(temp == 0)
          goto end_recv;
        //printf(">>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>\r\n");
      //printf(bc_uart_fifo.buf[bc_uart_fifo.tail].dat);
      //首先判断是否是控制命令，如果是则存入控制命令处理的fifo
//      if (str != NULL)
//      {
//        if (nb_ctrl.has_dat == false)
//        {
//          memcpy(nb_ctrl.dat, str, temp);
//          nb_ctrl.dat[temp] = 0x00;
//          nb_ctrl.has_dat = true;
//        }
//      }
//      else
      {
        //如果队列已经满了，直接忽�?
        if ((bc_uart_fifo.tail + 1 == bc_uart_fifo.header) || (bc_uart_fifo.tail == UART_FIFO_LEN - 1) & (bc_uart_fifo.header == 0))
        {
        }
        else
        {
          bc_uart_fifo.buf[bc_uart_fifo.tail].len = temp;
          bc_uart_fifo.tail++;
          if (bc_uart_fifo.tail >= UART_FIFO_LEN)
            bc_uart_fifo.tail = 0;
        }
      }
    }
    end_recv:
    HAL_UART_Receive_DMA(&huart3, bc_uart_fifo.buf[bc_uart_fifo.tail].dat, BUF_LEN); //重新打开DMA接收
  }
  /* USER CODE END USART3_4_IRQn 1 */
}

/* USER CODE BEGIN 1 */
void bms_uart_clear()
{
  //uart2_buf[0] = '\0';
  HAL_UART_DMAStop(&huart3); //
  __HAL_UART_CLEAR_IDLEFLAG(&huart3);

  memset(uart1_buf, 0x00, BUF_LEN);
  uart1_recv_len = 0;
  uart1_recv = false;
  uart1_dma_len = BUF_LEN;
  HAL_UART_Receive_DMA(&huart3, uart1_buf, uart1_dma_len); //重新打开DMA接收

  /*
    HAL_UART_DMAStop(&huart3);//
    __HAL_UART_CLEAR_IDLEFLAG(&huart3);
    HAL_UART_Receive_DMA(&huart3,uart2_buf, BUF_LEN);//重新打开DMA接收
    */
}

void bc_uart_clear()
{
  //uart2_buf[0] = '\0';
  HAL_UART_DMAStop(&huart3); //
  __HAL_UART_CLEAR_IDLEFLAG(&huart3);

  /*
  memset(uart2_buf, 0x00, BUF_LEN);
  uart2_recv_len = 0;
  uart2_recv = false;
  bc_uart_dma_len = BUF_LEN;
  */
  bc_uart_fifo.header = bc_uart_fifo.tail = 0;
  memset(&bc_uart_fifo.buf[bc_uart_fifo.header], 0x00, sizeof(stu_uartDat));
  HAL_UART_Receive_DMA(&huart3, bc_uart_fifo.buf[bc_uart_fifo.tail].dat, BUF_LEN); //重新打开DMA接收

  /*
    HAL_UART_DMAStop(&huart3);//
    __HAL_UART_CLEAR_IDLEFLAG(&huart3);
    HAL_UART_Receive_DMA(&huart3,uart2_buf, BUF_LEN);//重新打开DMA接收
    */
}

//当uart3缓冲区有数据时，将数据取出，并清空缓冲区
uint16_t bc_uart_load_buf(char *rcv)
{
  uint16_t len;

#ifdef _COMM_DEEP
  printf("\r\nuart3_load_buf,header=%u,tail=%u", bc_uart_fifo.header, bc_uart_fifo.tail);
#endif

  if (bc_uart_fifo.header == bc_uart_fifo.tail)
    return 0;

#ifdef _COMM_DEEP
  printf("|1");
#endif
  HAL_UART_DMAStop(&huart3);
  __HAL_UART_CLEAR_IDLEFLAG(&huart3);

  memcpy(rcv, bc_uart_fifo.buf[bc_uart_fifo.header].dat, bc_uart_fifo.buf[bc_uart_fifo.header].len);
  len = bc_uart_fifo.buf[bc_uart_fifo.header].len;
  memset(&bc_uart_fifo.buf[bc_uart_fifo.header], 0x00, sizeof(stu_uartDat));
  bc_uart_fifo.header++;
  if (bc_uart_fifo.header >= UART_FIFO_LEN)
    bc_uart_fifo.header = 0;

  bc_uart_dma_len = BUF_LEN;
  HAL_UART_Receive_DMA(&huart3, bc_uart_fifo.buf[bc_uart_fifo.header].dat, BUF_LEN); //重新打开DMA接收
  return len;
}
/* USER CODE END 1 */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
