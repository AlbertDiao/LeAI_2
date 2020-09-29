/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>
#include <stdbool.h>
#include "stdio.h"
#include "tool.h"
#include "bc20.h"
#include "bms.h"
#include "task.h"
#include "led.h"
#include "timer.h"
#include "track.h"
#include "app_gnss.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;
DMA_HandleTypeDef hdma_usart1_rx;
DMA_HandleTypeDef hdma_usart1_tx;
DMA_HandleTypeDef hdma_usart3_rx;
DMA_HandleTypeDef hdma_usart3_tx;

/* USER CODE BEGIN PV */
/*
uart1:dbg, 115200
uart2:bms, 4800
uart3:bc, 115200
*/
#define UPLOAD_RETRY_MAX 10 //数据上传�?大重试次�?
bool bms_data_new;
char msg[MSG_LEN];
uint8_t upload_retry; //数据上传重试次数

//uint8_t bms_read.step;
//uint8_t bms_upload.step;
//uint8_t gnss_read.step;
//uint8_t gnss_upload.step;
stu_father father;
stu_bms_read bms_read;
stu_bms_upload bms_upload;
stu_gnss_read gnss_read;
stu_need_path need_path;
stu_daemon daemon;

uint8_t task_led = Task_00;
uint8_t task_bms_read = Task_01;
uint8_t task_bms_upload = Task_02;
uint8_t task_upload_daemon = Task_03;
uint8_t task_gnss_read = Task_04;
uint8_t task_gnss_upload = Task_05;
uint8_t task_nb_cmd = Task_06;
uint8_t task_father = Task_07;
uint8_t task_track = Task_08;
uint8_t task_need_path = Task_09;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */
void led_blink(void);
bool sys_init(void);
void dcdc_open(void);
bool sys_init(void);
void led_blink(void);
void data_read(void);
void data_upload(void);
void data_upload_daemon(void);
void gnss_upload_task(void);
void gnss_read_task(void);
void nb_cmd_exe(void);
void father_task(void);
void need_path_task(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_USART1_UART_Init();
  MX_USART3_UART_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
  /*
  串口使用说明
  USART1: Debug
  USART2: 485,BMS
  USART3: NB,BC20
  */
  dcdc_open();                //�?启DCDC电源
  __HAL_DBGMCU_FREEZE_IWDG(); //调试时关闭看门狗

  dbg_uart_recv = bms_uart_recv = bc_uart_recv = false;
  dbg_uart_recv_len = bms_uart_recv_len = bc_uart_recv_len = 0;
  dbg_uart_dma_len = bms_uart_dma_len = bc_uart_dma_len = BUF_LEN;

  memset(&bms_upload, 0x00, sizeof(stu_bms_upload));
  memset(&bms_read, 0x00, sizeof(stu_bms_read));
  memset(&gnss_read, 0x00, sizeof(stu_gnss_read));
  memset(&gnss_upload, 0x00, sizeof(stu_gnss_upload));
  memset(&father, 0x00, sizeof(stu_father));
  memset(&daemon, 0x00, sizeof(stu_daemon));
#if defined(DBGMCU_APB1_FZ_DBG_IWDG_STOP)
  printf("DBGMCU_APB1_FZ_DBG_IWDG_STOP defined.\r\n");
#endif
  //看门狗时间Tout = 4*4096/40000 = 0.4096S
  led_close(0);
  led_close(1);
  led_close(2);
  led_close(3);
  led_close(4);
  led_close(5);

  led_open(LED_SYS);
  
  //开启485电路
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET); //TTL -> 485
  //HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET); //485 -> TTL
//  if (!sys_init())
//  {
//    printf("系统初始化失败！");
//    while (1)
//      ;
//  }

  if ((TASK_INITIALIZED != configTask(task_led, led_blink, TASK_1S)) || (TASK_STARTED != startTask(task_led, 1)))
  {
    printf("CREATE TASK ERROR(%u)!\n", __LINE__);
    return 1;
  }
  if ((TASK_INITIALIZED != configTask(task_bms_read, data_read, TASK_100MS)) || (TASK_STARTED != startTask(task_bms_read, 1)))
  {
    printf("CREATE TASK ERROR(%u)!\n", __LINE__);
    return 1;
  }

//  if ((TASK_INITIALIZED != configTask(task_bms_upload, data_upload, TASK_100MS)) || (TASK_STARTED != startTask(task_bms_upload, 1)))
//  {
//    printf("CREATE TASK ERROR(%u)!\n", __LINE__);
//    return 1;
//  }

//  if ((TASK_INITIALIZED != configTask(task_upload_daemon, data_upload_daemon, TASK_1S)) || (TASK_STARTED != startTask(task_upload_daemon, 1)))
//  {
//    printf("CREATE TASK ERROR(%u)!\n", __LINE__);
//    return 1;
//  }

//  if ((TASK_INITIALIZED != configTask(task_gnss_read, gnss_read_task, TASK_100MS)) || (TASK_STARTED != startTask(task_gnss_read, 1)))
//  {
//    printf("CREATE TASK ERROR(%u)!\n", __LINE__);
//    return 1;
//  }

//  if ((TASK_INITIALIZED != configTask(task_gnss_upload, gnss_upload_task, TASK_100MS)) || (TASK_STARTED != startTask(task_gnss_upload, 1)))
//  {
//    printf("CREATE TASK ERROR(%u)!\n", __LINE__);
//    return 1;
//  }

//  if ((TASK_INITIALIZED != configTask(task_nb_cmd, nb_cmd_exe, TASK_100MS)) || (TASK_STARTED != startTask(task_nb_cmd, 1)))
//  {
//    printf("CREATE TASK ERROR(%u)!\n", __LINE__);
//    return 1;
//  }

//  if ((TASK_INITIALIZED != configTask(task_father, father_task, TASK_1S)) || (TASK_STARTED != startTask(task_father, 1)))
//  {
//    printf("CREATE TASK ERROR(%u)!\n", __LINE__);
//    return 1;
//  }
//  printf("begin run\n");

//  if ((TASK_INITIALIZED != configTask(task_need_path, need_path_task, TASK_100MS * 5)) || (TASK_STARTED != startTask(task_need_path, 1)))
//  {
//    printf("CREATE TASK ERROR(%u)!\n", __LINE__);
//    return 1;
//  }
  printf("begin run\n");

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    FEED_DOG;
    task_handler();

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL6;
  RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV1;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1|RCC_PERIPHCLK_USART2;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK1;
  PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */
  __HAL_UART_ENABLE_IT(&huart1, UART_IT_IDLE);
  HAL_UART_Receive_DMA(&huart1, dbg_uart_buf, BUF_LEN); //

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 4800;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */
  __HAL_UART_ENABLE_IT(&huart2, UART_IT_IDLE);

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  huart3.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_SWAP_INIT;
  huart3.AdvancedInit.Swap = UART_ADVFEATURE_SWAP_ENABLE;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */
  bc_uart_fifo.header = bc_uart_fifo.tail = 0;
  __HAL_UART_ENABLE_IT(&huart3, UART_IT_IDLE);
  HAL_UART_Receive_DMA(&huart3, bc_uart_fifo.buf[bc_uart_fifo.header].dat, BUF_LEN); //

  /* USER CODE END USART3_Init 2 */

}

/** 
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void) 
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel2_3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel2_3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel2_3_IRQn);
  /* DMA1_Channel4_5_6_7_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel4_5_6_7_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel4_5_6_7_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_8 
                          |GPIO_PIN_12, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2|GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14 
                          |GPIO_PIN_6|GPIO_PIN_7, GPIO_PIN_RESET);

  /*Configure GPIO pin : PA4 */
  GPIO_InitStruct.Pin = GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PA5 PA6 PA8 PA12 */
  GPIO_InitStruct.Pin = GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_8|GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB2 PB12 PB13 PB14 
                           PB6 PB7 */
  GPIO_InitStruct.Pin = GPIO_PIN_2|GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14 
                          |GPIO_PIN_6|GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
//void SysTick_Handler(void)
//{
//  systick_count++;
//  HAL_IncTick();
//  HAL_SYSTICK_IRQHandler();
//}

void led_blink()
{
  led_toggen(LED_SYS);
}

#ifdef __GNUC__
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif
PUTCHAR_PROTOTYPE
{
  HAL_UART_Transmit(&huart1, (uint8_t *)&ch, 1, 0xFFFF);
  return ch;
}

/*
�?启dcdc�?
先开启VDD EN，使得DCDC电路�?始工�?
等待500ms（如果示波器测试ok可以缩短这个时间�?
再开启VCC SET，设置为通过DCDC供电
*/
void dcdc_open(void)
{
  //先开启VDD EN，使得DCDC电路�?始工�?
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);

  //等待500ms（如果示波器测试ok可以缩短这个时间�?
  osDelay(500);

  //再开启VCC SET，设置为通过DCDC供电
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_SET);
}

bool sys_init()
{
  led_toggen(LED_SYS);
  led_open(LED_NB);
  bc_reset();

  led_toggen(LED_SYS);
  if (!bc_init())
  {
    sys_reset();
    return false;
  }

  led_toggen(LED_SYS);
  if (!bc_pdp_act())
  {
    sys_reset();
    return false;
  }

  led_toggen(LED_SYS);
  if (!bc_conn_LWM2M())
  {
    sys_reset();
    return false;
  }

  led_toggen(LED_SYS);
  if (!bc_init_gnss())
  {
    sys_reset();
    return false;
  }
  led_toggen(LED_SYS);

  //bms_init();
  //led_toggen(LED_SYS);

  printf(">> System started.\n");
  printf("\n--- --- --- --- --- --- --- --- --- --- --- --- --- --- --- --- ---\n\n");
  printf("Hi, I'm LeAI mini:)\n\n");
  printf("LeAI was born on March 17, 2020,\n");
  printf("LeAI_mini was born on May 27, 2020,\n");
  printf("I am created on Sep 23, 2020.\n");
  printf("Time flies, come on!\n");
  printf("\n--- --- --- --- --- --- --- --- --- --- --- --- --- --- --- --- ---\n");

  return true;
}

void data_read()
{
  switch (bms_read.step)
  {
  case 0x00:
    //防止出现bms_data_new还没有被false，但是data_upload任务已经被干掉的情况导致锁死
    if (bms_read.to_stop)
    {
      bms_read.step = 0xFF;
      break;
    }
    else
    {
      if (bms_data_new)
      {
        bms_read.step = 0xFF;
      }
      else
      {
        printf("\r\nread data...");
        led_toggen(LED_BMS);
        bms_read.step++;
      }
    }
    break;

  case 0x01:
    printf("|ntc");
    if (!bms_get_ntc())
      bms_read.step = 0xFF;
    led_toggen(LED_BMS);
    bms_read.step++;
    break;

  case 0x02:
    printf("|cell_v");
    if (!bms_get_v())
      bms_read.step = 0xFF;
    led_toggen(LED_BMS);
    bms_read.step++;
    break;

  case 0x03:
    printf("|pack_i");
    if (!bms_get_i())
      bms_read.step = 0xFF;
    led_toggen(LED_BMS);
    bms_read.step++;
    break;

  case 0x04:
    printf("|pack_v");
    if (!bms_get_pack_v())
      bms_read.step = 0xFF;
    led_toggen(LED_BMS);
    bms_read.step++;
    break;

  case 0x05:
    printf("|fcc");
    if (!bms_get_fcc())
      bms_read.step = 0xFF;
    led_toggen(LED_BMS);
    bms_read.step++;
    break;

  case 0x06:
    printf("|soc");
    if (!bms_get_soc())
      bms_read.step = 0xFF;
    led_toggen(LED_BMS);
    bms_read.step++;
    break;

  case 0x07:
    printf("|bms_get_status");
    if (!bms_get_status())
      bms_read.step = 0xFF;
    led_toggen(LED_BMS);
    bms_read.step++;
    break;

  case 0x08:
    bms_data_new = true;
    bms_read.step++;
    led_open(LED_BMS);
    break;

  default:
    if (bms_read.to_stop)
    {
      bms_read.stopped = true;
    }
    else
    {
      bms_read.step = 0;
    }
    break;
  }
}

void data_upload()
{
#define BMS_UPLOAD_OVER 0xF0
  static uint8_t index;
  //  static uint8_t first;
  switch (bms_upload.step)
  {
  case 0x00:
    if (bms_upload.to_stop)
    {
      bms_upload.step = 0xFF;
      break;
    }
    else
    {
      if (bms_data_new)
      {
        if (!bc_lock())
        {
          printf("\r\ndata upload\r\n");
          set_bc_lock(true); //BMS upload�?�?
          printf("set_bc_lock(true) in %u\r\n", __LINE__);
          index = 6;
          bms_upload.step++;
        }
      }
    }
    break;

  case 0x01:
    sprintf(msg, "ntc,%u,%u,%u,%u,%u",
            ntc[0], ntc[1], ntc[2], ntc[3], ntc[4]);
    sprintf(atstr, "AT+MIPLNOTIFY=0,%s,%u,0,%u,1,%d,\"%s\",%u,0\r\n", objtnum, BAT_OBJ, NTC_RES, strlen(msg), msg, index);
    printf("atstr(%u)=%s\r\n:", strlen(atstr), atstr);
    nb_cmd_async(atstr);
    led_toggen(LED_NB);
    index--;
    //   first = true;
    tm_set(TM_BMS, 3000);
    bms_upload.step++;
    break;
  case 0x02:
    if (!tm_out(TM_BMS))
    {
      if (bc_uart_load_buf(nb_recv))
      {
        printf("*******************\r\n");
        printf(nb_recv);
        printf("*******************\r\n");
        /*
        if (first)
        {
          printf("first msg, skip.\r\n");
          first = false;
        }
        else
        {
        */
        if (strstr(nb_recv, "OK"))
        {
          bms_upload.step++;
        }
        //}
      }
    }
    else
    {
      //等待消息失败
      bms_upload.step = BMS_UPLOAD_OVER;
      upload_retry++;
    }
    break;

  case 0x03:
    sprintf(msg, "cell_v1,%u,%u,%u,%u,%u",
            cell_v[0], cell_v[1], cell_v[2], cell_v[3], cell_v[4]);
    sprintf(atstr, "AT+MIPLNOTIFY=0,%s,%u,0,%u,1,%d,\"%s\",%u,0\r\n", objtnum, BAT_OBJ, CELL_RES_1_5, strlen(msg), msg, index);
    printf("atstr(%u)=%s\r\n:", strlen(atstr), atstr);
    nb_cmd_async(atstr);
    led_toggen(LED_NB);
    index--;
    //    first = true;
    tm_set(TM_BMS, 3000);
    bms_upload.step++;
    break;
  case 0x04:
    if (!tm_out(TM_BMS))
    {
      if (bc_uart_load_buf(nb_recv))
      {
        printf("*******************\r\n");
        printf(nb_recv);
        printf("*******************\r\n");
        /*
        if (first)
        {
          printf("first msg, skip.\r\n");
          first = false;
        }
        else
        {
          */
        if (strstr(nb_recv, "OK"))
        {
          bms_upload.step++;
        }
        //}
      }
    }
    else
    {
      bms_upload.step = BMS_UPLOAD_OVER;
      upload_retry++;
    }
    break;
  case 0x05:
    sprintf(msg, "cell_v2,%u,%u,%u,%u,%u",
            cell_v[5], cell_v[6], cell_v[7], cell_v[8], cell_v[9]);
    sprintf(atstr, "AT+MIPLNOTIFY=0,%s,%u,0,%u,1,%d,\"%s\",%u,0\r\n", objtnum, BAT_OBJ, CELL_RES_6_10, strlen(msg), msg, index);
    printf("atstr(%u)=%s\r\n:", strlen(atstr), atstr);
    nb_cmd_async(atstr);
    led_toggen(LED_NB);
    index--;
    //    first = true;
    tm_set(TM_BMS, 3000);
    bms_upload.step++;
    break;
  case 0x06:
    if (!tm_out(TM_BMS))
    {
      if (bc_uart_load_buf(nb_recv))
      {
        printf("*******************\r\n");
        printf(nb_recv);
        printf("*******************\r\n");
        /*
        if (first)
        {
          printf("first msg, skip.\r\n");
          first = false;
        }
        else
        {

*/
        if (strstr(nb_recv, "OK"))
        {
          bms_upload.step++;
        }
        //}
      }
    }
    else
    {
      bms_upload.step = BMS_UPLOAD_OVER;
      upload_retry++;
    }
    break;
  case 0x07:
    sprintf(msg, "cell_v3,%u,%u,%u,%u,%u",
            cell_v[10], cell_v[11], cell_v[12], cell_v[13], cell_v[14]);
    sprintf(atstr, "AT+MIPLNOTIFY=0,%s,%u,0,%u,1,%d,\"%s\",%u,0\r\n", objtnum, BAT_OBJ, CELL_RES_11_15, strlen(msg), msg, index);
    printf("atstr(%u)=%s\r\n:", strlen(atstr), atstr);
    nb_cmd_async(atstr);
    led_toggen(LED_NB);
    index--;
    //first = true;
    tm_set(TM_BMS, 3000);
    bms_upload.step++;
    break;
  case 0x08:
    if (!tm_out(TM_BMS))
    {
      if (bc_uart_load_buf(nb_recv))
      {
        printf("*******************\r\n");
        printf(nb_recv);
        printf("*******************\r\n");
        /*
        if (first)
        {
          printf("first msg, skip.\r\n");
          first = false;
        }
        else
        {
          */
        if (strstr(nb_recv, "OK"))
        {
          bms_upload.step++;
        }
        //}
      }
    }
    else
    {
      bms_upload.step = BMS_UPLOAD_OVER;
      upload_retry++;
    }
    break;

  case 0x09:
    sprintf(msg, "cell_v4,%u,%u,%u,%u,%u",
            cell_v[15], cell_v[16], cell_v[17], cell_v[18], cell_v[19]);
    sprintf(atstr, "AT+MIPLNOTIFY=0,%s,%u,0,%u,1,%d,\"%s\",%u,0\r\n", objtnum, BAT_OBJ, CELL_RES_15_20, strlen(msg), msg, index);
    printf("atstr(%u)=%s\r\n:", strlen(atstr), atstr);
    nb_cmd_async(atstr);
    led_toggen(LED_NB);
    index--;
    //first = true;
    tm_set(TM_BMS, 3000);
    bms_upload.step++;
    break;

  case 0x0A:
    if (!tm_out(TM_BMS))
    {
      if (bc_uart_load_buf(nb_recv))
      {
        printf("*******************\r\n");
        printf(nb_recv);
        printf("*******************\r\n");
        /*
        if (first)
        {
          printf("first msg, skip.\r\n");
          first = false;
        }
        else
        {
*/
        if (strstr(nb_recv, "OK"))
        {
          bms_upload.step++;
        }
        //       }
      }
    }
    else
    {
      bms_upload.step = BMS_UPLOAD_OVER;
      upload_retry++;
    }
    break;
  case 0x0B:
    sprintf(msg, "pack_res,%u,%d,%u,%u",
            pack_v, pack_i, soc, 45); //fcc先传固定�?,Albert , 200717
    sprintf(atstr, "AT+MIPLNOTIFY=0,%s,%u,0,%u,1,%d,\"%s\",%u,0\r\n", objtnum, BAT_OBJ, PACK_RES, strlen(msg), msg, index);
    printf("atstr(%u)=%s\r\n:", strlen(atstr), atstr);
    nb_cmd_async(atstr);
    led_toggen(LED_NB);
    index--;
    //first = true;
    tm_set(TM_BMS, 3000);
    bms_upload.step++;
    break;
  case 0x0C:
    if (!tm_out(TM_BMS))
    {
      if (bc_uart_load_buf(nb_recv))
      {
        printf("*******************\r\n");
        printf(nb_recv);
        printf("*******************\r\n");
        /*
        if (first)
        {
          printf("first msg, skip.\r\n");
          first = false;
        }
        else
        {
*/
        if (strstr(nb_recv, "OK"))
        {
          bms_upload.step++;
        }
        //        }
      }
    }
    else
    {
      bms_upload.step = BMS_UPLOAD_OVER;
      upload_retry++;
    }
    break;
  case 0x0D:
    sprintf(msg, "pack_status,%u,%u,%u,%u",
            bat_status.warn.byte, bat_status.protect.byte, bat_status.sys.byte, bat_status.sys2.byte); //系统状�?�上�?
    sprintf(atstr, "AT+MIPLNOTIFY=0,%s,%u,0,%u,1,%d,\"%s\",%u,0\r\n", objtnum, BAT_OBJ, PACK_STATUS, strlen(msg), msg, index);
    printf("atstr(%u)=%s\r\n:", strlen(atstr), atstr);
    nb_cmd_async(atstr);
    led_toggen(LED_NB);
    tm_set(TM_BMS, 3000);
    bms_upload.step++;
    break;
  case 0x0E:
    if (!tm_out(TM_BMS))
    {
      if (bc_uart_load_buf(nb_recv))
      {
        printf("*******************\r\n");
        printf(nb_recv);
        printf("*******************\r\n");
        if (strstr(nb_recv, "OK"))
        {
          bms_upload.step = BMS_UPLOAD_OVER;
          upload_retry = 0; //全部上传成功
        }
        //}
      }
    }
    else
    {
      bms_upload.step = BMS_UPLOAD_OVER;
      upload_retry++;
    }
    break;

  case BMS_UPLOAD_OVER:
    printf("set_bc_lock(false) in %u\r\n", __LINE__);
    set_bc_lock(false);   //data upload结束, 释放bc资源
    tm_set(TM_BMS, 1000); //暂停bms upload�?段时�?
    bms_data_new = false;
    led_close(LED_NB);
    bms_upload.step++;
    break;

  case BMS_UPLOAD_OVER + 1:
    if (tm_out(TM_BMS))
    {
      bms_upload.step++;
    }
    break;

  default:
    if (bms_upload.to_stop)
    {
      bms_upload.stopped = true;
    }
    else
    {
      bms_upload.step = 0;
    }
    break;
  }
}

//数据上传守护进程,1S运行1�?
//如果连续30分钟step们都没有变化，那么就重启
void data_upload_daemon()
{
  if ((daemon.l_bms_upload != bms_upload.step) ||
      (daemon.l_gnss_read != gnss_read.step) ||
      (daemon.l_gnss_upload != gnss_upload.step) ||
      (daemon.l_track != track.step))
  {
    //有变�?
    daemon.time = 0;
    daemon.l_bms_upload = bms_upload.step;
    daemon.l_gnss_read = gnss_read.step;
    daemon.l_gnss_upload = gnss_upload.step;
    daemon.l_track = track.step;
  }
  else
  {
    daemon.time++;
  }

  if (daemon.time > 60 * 30)
  {
    printf("\r\n\r\n.restart of step\r\n\r\n");
    NVIC_SystemReset(); //重启系统
  }

  if (upload_retry >= UPLOAD_RETRY_MAX)
  {
    printf("\r\n\r\n.restart of upload\r\n\r\n");
    NVIC_SystemReset(); //重启系统
  }
}

//命令执行进程
void nb_cmd_exe()
{
#define EX_MSG_LEN 10
  char ex_msg[MSG_LEN]; //存放NB命令下发的body.arg的内�?
  char pri_atstr[50];   //存放NB命令下发的body.arg的内�?
  char *sx;
  int offset;

  uint32_t msg_res;
  uint32_t msg_id;

  if (nb_ctrl.has_dat)
  {
    sx = strstr(nb_ctrl.dat, "+MIPLEXECUTE:");
    printf("\r\nstr:%s\r\n", sx);
    printf("\r\nnb_recv:%s\r\n", nb_ctrl.dat);

    //获得命令下发的内�?
    sscanf(sx, "+MIPLEXECUTE: %u,%u",
           &msg_res, &msg_id);
    sx = strstr(nb_ctrl.dat, "\"");
    if (sx == NULL)
    {
      nb_ctrl.has_dat = false;
      memset(nb_ctrl.dat, 0x00, NB_BUF_LEN);
      return;
    }
    offset = 0;
    while (*(++sx) != '\"')
    {
      ex_msg[offset++] = *sx;
    }
    ex_msg[offset] = 0x00;
    printf("msg_res = %u\r\n", msg_res);
    printf("msg_id = %u\r\n", msg_id);
    printf("EXE: msg={%s}\r\n", ex_msg);
    //解析msg的�??,a:�?启， b：关�?
    if (ex_msg[0] == 'a')
    {
      //HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, GPIO_PIN_RESET);
      //HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_RESET);
      led_open(LED_CTRL);
    }
    else if (ex_msg[0] == 'b')
    {
      //HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, GPIO_PIN_SET);
      //HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_SET);
      led_close(LED_CTRL);
    }
    nb_ctrl.has_dat = false;
    memset(nb_ctrl.dat, 0x00, NB_BUF_LEN);
    //返回命令�?, 调用成功，第三个参数返回2
    //+MIPLEXECUTERSP: <ref>,<msgId>,<result> (2)
    sprintf(pri_atstr, "AT+MIPLEXECUTERSP=%u,%u,2\r\n", msg_res, msg_id);
    nb_cmd_async(pri_atstr);
  }
}

/*�?么时候需要采集轨迹？
 每次有放电电流持�?20秒钟后采集一�?
 采集完以后，除非连续20秒没有放电电流，否则不会继续计算是否�?要采�?
*/
void need_path_task()
{
  switch (need_path.step)
  {
  case 0x00:
    need_path.step++;
    break;

  case 0x01:
    bms_get_i();
//累计电流持续时间
//放电电流大于1A,pack_i在bms任务中被持续更新
#define CUR_RUN_THE -500
    if (pack_i < CUR_RUN_THE)
    {
      need_path.times++;
    }
    else
    {
      need_path.uc_times++;
    }

    if (need_path.times + need_path.uc_times > 60)
    {
      //�?始采集轨迹的条件，无电流点数占比不超�?70%
      if ((need_path.uc_times < 42))
      {
        need_path.need = true;
        led_open(LED_PATH);
        need_path.step = 0x02;
      }
      else
      {
        //这段时间的无效点太多，重置计数器
        need_path.times = 0;
        need_path.uc_times = 0;
      }
    }

    break;

  case 0x02:
    if (!need_path.need)
    {
      led_close(LED_PATH); //数据上传完以后关掉指示灯
      need_path.step = 0x03;
      need_path.times = 0;
      need_path.uc_times = 0;
    }
    break;

  case 0x03:
    bms_get_i();
    //累计电流持续时间
    //放电电流小于1A, 或�?�正在充电，被认为是不处于运行状�?
    if (pack_i >= CUR_RUN_THE)
    {
      need_path.uc_times++;
    }
    else
    {
      led_open(LED_PATH); //中�?�如果重新计数，那么重新打开�?
      need_path.uc_times = 0;
    }
    //持续若干次没有放电电流，则认为停止时间足�?
    if (need_path.uc_times > 30)
    {
      led_close(LED_PATH); //当空闲时间达标以后，关掉指示灯，提示可以上电流了
      need_path.times = 0;
      need_path.uc_times = 0;
      need_path.step = 0x01;
    }
    break;
  default:
    break;
  }
  return;
}

/*
高级策略（father task�?
判断是否�?要采集轨迹，如果�?要采集轨迹，则：
通知BMS采集和上传，gnss采集和上传的任务停掉
等待上述任务都停�?
�?掉上述任�?
�?始轨迹采集任�?
轨迹采集任务完成后，重启上述任务
*/
void father_task()
{
  switch (father.step)
  {
  case 0:
    if (need_path.need)
    {
      father.step++;
    }
    break;

  //通知任务停止
  case 1:
    bms_read.to_stop = true;
    bms_upload.to_stop = true;
    gnss_read.to_stop = true;
    gnss_upload.to_stop = true;
    //tm_set(TM_FATHER,30000);//等待30秒钟
    father.step++;
    break;

  case 2:
    if (bms_read.stopped && bms_upload.stopped && gnss_read.stopped && gnss_upload.stopped)
      father.step++;
    break;

  case 3:
    killTask(task_bms_read);
    killTask(task_bms_upload);
    killTask(task_gnss_read);
    killTask(task_gnss_upload);
    bms_data_new = false;
    gnss_data_new = false;
    father.step++;
    break;

  case 4:
    memset(&track, 0x00, sizeof(stu_track_t));
    if ((TASK_INITIALIZED != configTask(task_track, track_task, TASK_1MS)) || (TASK_STARTED != startTask(task_track, 1)))
    {
      printf("CREATE TASK ERROR(%u)!\n", __LINE__);
      return;
    }
    father.step++;

    break;

  case 0x05:
    if (track.run_over)
    {
      //外部置位need_path，重新检测车辆持续运�?
      need_path.need = false;
      killTask(task_track);
      printf("tasks reset\r\n");
      memset(&bms_read, 0x00, sizeof(stu_bms_upload));
      memset(&bms_upload, 0x00, sizeof(stu_bms_upload));
      memset(&gnss_read, 0x00, sizeof(stu_gnss_read));
      memset(&gnss_upload, 0x00, sizeof(stu_gnss_upload));

      if ((TASK_INITIALIZED != configTask(task_bms_read, data_read, TASK_100MS)) || (TASK_STARTED != startTask(task_bms_read, 1)))
      {
        printf("CREATE TASK ERROR(%u)!\n", __LINE__);
        return;
      }

      if ((TASK_INITIALIZED != configTask(task_bms_upload, data_upload, TASK_100MS)) || (TASK_STARTED != startTask(task_bms_upload, 1)))
      {
        printf("CREATE TASK ERROR(%u)!\n", __LINE__);
        return;
      }

      if ((TASK_INITIALIZED != configTask(task_gnss_read, gnss_read_task, TASK_100MS)) || (TASK_STARTED != startTask(task_gnss_read, 1)))
      {
        printf("CREATE TASK ERROR(%u)!\n", __LINE__);
        return;
      }

      if ((TASK_INITIALIZED != configTask(task_gnss_upload, gnss_upload_task, TASK_100MS)) || (TASK_STARTED != startTask(task_gnss_upload, 1)))
      {
        printf("CREATE TASK ERROR(%u)!\n", __LINE__);
        return;
      }
      father.step++;
    }
    break;

  default:
    father.step = 0;
    break;
  }
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
