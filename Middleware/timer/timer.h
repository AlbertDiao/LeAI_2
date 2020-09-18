/************************************************************************************
 * timer.h
 *
 * 超时定时器
 *
 * 作者：刁东旭
 *
 * 日期：2020-7-19
*************************************************************************************/
#ifndef _TIMER
#define _TIMER

#include "main.h"
#include <stdbool.h>

#define TIMER_NUM 16 //最大定时器数量
#define TM_BMS 1
#define TM_GPS_READ 2
#define TM_GPS_UPLOAD 3
#define TM_FATHER 4
#define TM_PATH 5
#define TM_PATH_2 6

typedef struct
{
  uint32_t start_ms; //定时器设置时候的系统时间戳（单位ms）
  uint32_t timeout_ms; //设定的超时时间戳（单位ms）
}stu_timer;
void tm_set(uint8_t t, uint32_t timeout_ms);
bool tm_out(uint8_t t);
#endif
