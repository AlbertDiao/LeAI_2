/************************************************************************************
 * timer.c
 *
 * 超时定时器
 *
 * 作者：刁东旭
 *
 * 日期：2020-7-19
*************************************************************************************/
#include "timer.h"
#include "task.h"

stu_timer timer[TIMER_NUM];

uint32_t tm_getms()
{
  return systick_count;
}

//设置超时定时器
void tm_set(uint8_t t, uint32_t tm)
{
  if(t>=TIMER_NUM)
    return ;
  timer[t].timeout_ms = tm;
  timer[t].start_ms = tm_getms();//获得系统当前的ms时间戳
  //printf("tm_set,tm=%u\r\n",tm);
  //printf("timer[%u],start_ms=%u,timeout_ms=%u\r\n",t,timer[t].start_ms,timer[t].timeout_ms);
}

//查询超时定时器是否超时
bool tm_out(uint8_t t)
{
  uint32_t wait_time=0;
  bool res=false;
  if(t>=TIMER_NUM)
    return false;
  uint32_t now = tm_getms();
  //printf("\r\ntm_out,now=%u,start=%u",now,timer[t].start_ms);

  if(now >= timer[t].start_ms)
  {
    wait_time = now - timer[t].start_ms ;
  }
  else
  {
    wait_time = MAX_GLOBAL_TIMING_COUNT - timer[t].start_ms + now;
  }
  res = wait_time > timer[t].timeout_ms; 
  //printf("|wait=%u,timeout_ms=%u,res=%u",wait_time,timer[t].timeout_ms,res);
  return res;
}
