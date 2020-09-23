/******************************
* 
* 文件名：track.h
* 功能：轨迹采集程序
* 作者：刁东旭
* 日期：2020-8-03
* 邮箱：ddx@outlook.com
*
*******************************/

#ifndef _TRACK
#define _TRACK
#include "stm32f0xx_hal.h"
#include <stdbool.h>
#define PATH_LEN 30

typedef enum trk_state
{
  START = 0,
  //IDEL,
  RECORD,
  PAUSE,
  SUBMIT,
  DROP,
  ERR,
  END,
  OTHER
}trk_state_t;

typedef enum trk_res
{
  GOING = 0,
  FAILED = 1,
  FINISH ,
  RETRY
}trk_res_t;

//轨迹点
/*
typedef struct stu_point
{
  double x;
  double y;
  float sp;
  float ag;
  uint32_t tm;
}stu_point_t;
*/

typedef struct stu_track
{
  trk_state_t state; //状态
  uint8_t step;  //子状态
  uint8_t data_retry; //采集数据重试次数
  uint8_t upload_retry; //上传重试次数
  uint16_t offset;//正在保存的数据点序号
  uint16_t send_off;//正在发送的数据点序号
  bool run_over;//是否跑完
}stu_track_t;

void track_task(void);

#endif
