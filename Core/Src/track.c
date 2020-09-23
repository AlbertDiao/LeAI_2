/******************************
* 
* 文件名：track.c
* 功能：轨迹采集程序
* 作者：刁东旭
* 日期：2020-8-03
* 邮箱：ddx@outlook.com
*
*******************************/

#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include "timer.h"
#include "track.h"
#include "main.h"
#include "bc20.h"
#include "bms.h"
#include "app_gnss.h"

//#define _MOCK_GNSS

static trk_res_t doStart(void);
static trk_res_t doRecord(void);
static trk_res_t doPause(void);
static trk_res_t doSubmit(void);
static trk_res_t doDrop(void);
static trk_res_t doErr(void);
static trk_res_t doEnd(void);
stu_track_t track;
trk_res_t res;

double x[PATH_LEN];
double y[PATH_LEN];
float sp[PATH_LEN];
float ag[PATH_LEN];
uint32_t tm[PATH_LEN];
float tsoc[PATH_LEN];

double m_x[PATH_LEN] = {119.4396033, 119.4395917, 119.439575, 119.43956, 119.4395417, 119.4395117, 119.4394717, 119.4394317, 119.4393933, 119.4393367, 119.4392783, 119.439205, 119.4391417, 119.4390817, 119.43901, 119.4389383, 119.4388617, 119.4387717, 119.438685, 119.4386017, 119.438525, 119.43845, 119.43838, 119.4383117, 119.4382483, 119.438185, 119.4381183, 119.438055, 119.4379883, 119.4379283};
double m_y[PATH_LEN] = {26.00128, 26.001245, 26.00120167, 26.00115333, 26.001105, 26.00105167, 26.001005, 26.00095833, 26.00090833, 26.000855, 26.0008, 26.000755, 26.00071833, 26.00068833, 26.00065833, 26.00063167, 26.00059833, 26.000575, 26.00054667, 26.00051, 26.00046833, 26.00042167, 26.00036833, 26.00031833, 26.00026, 26.00019667, 26.00013333, 26.00006667, 25.99999333, 25.99993167};
float m_sp[PATH_LEN] = {3.274, 4.883, 5.222, 5.135, 6.094, 6.826, 6.215, 6.819, 7.118, 8.572, 9.223, 7.551, 5.801, 6.903, 7.833, 7.217, 8.201, 7.823, 8.002, 8.114, 8.148, 8.439, 8.549, 8.586, 8.83, 9.01, 8.864, 10.303, 9.529, 8.767};
float m_ag[PATH_LEN] = {179.68, 198.2, 198.85, 194.2, 197.91, 212.87, 223.08, 215.8, 213.3, 229.96, 227.96, 240.55, 244.22, 243.58, 251.69, 248.75, 253.36, 257.09, 249.03, 242.09, 237.49, 235.1, 227.02, 226.58, 223.53, 223.47, 223.11, 222.88, 220.2, 218.47};
uint32_t m_tm[PATH_LEN] = {1478031031, 2, 4, 6, 8, 10, 12, 14, 16, 18, 20, 22, 24, 26, 28, 30, 32, 34, 36, 38, 40, 42, 44, 46, 48, 50, 52, 54, 56, 58};
float m_soc[PATH_LEN] = {35.6, 35.5, 35.4, 35.3, 35.2, 35.1, 35, 34.9, 34.8, 34.7, 34.6, 34.5, 34.4, 34.3, 34.2, 34.1, 34, 33.9, 33.8, 33.7, 33.6, 33.5, 33.4, 33.3, 33.2, 33.1, 33, 32.9, 32.8, 32.7};

#define TOUT 10000 //10秒收不到数据结束

/**
 * Create random UUID
 *
 * @param buf - buffer to be filled with the uuid string
 */
//static char uuid[37];

/*
void create_uuid(char* buf)
{
  const char *c = "89ab";
  char *p = buf;
  int n;

  for (n = 0; n < 16; ++n)
  {
    int b = rand() % 255;

    switch (n)
    {
    case 6:
      sprintf(
          p,
          "4%x",
          b % 15);
      break;
    case 8:
      sprintf(
          p,
          "%c%x",
          c[rand() % strlen(c)],
          b % 15);
      break;
    default:
      sprintf(
          p,
          "%02x",
          b);
      break;
    }

    p += 2;

    switch (n)
    {
    case 3:
    case 5:
    case 7:
    case 9:
      *p++ = '-';
      break;
    }
  }

  *p = 0;

  //return buf;
}
*/

void track_init()
{
  
}

//请求位置点
bool req_point()
{
    #ifndef _MOCK_GNSS
  //AT+QGNSSRD="NMEA/RMC"+QGNSSRD: $GNRMC,125213.00,A,2600.0986,N,11926.3359,E,0.281,,060320,,,A,V*1C
  //strcpy(gnss_keyword, "$GNRMC");
  gnss_keyword_len = strlen(gnss_keyword);
  printf("QGNSSRD RMC\r\n");
  sprintf(atstr, "AT+QGNSSRD=\"NMEA/RMC\"\r\n");
  return nb_cmd_async(atstr);
    #else
    return true;
    #endif
}

//获取位置点，如果获取到了存入offset指向的位置，并且范围真，否则返回假
bool get_point(uint16_t offset)
{
  
  /*
  x[offset] = m_x[offset];
  y[offset] = m_y[offset];
  sp[offset] = m_sp[offset];
  ag[offset] = m_ag[offset];
  tm[offset] = m_tm[offset];
  tsoc[offset] = m_soc[offset];
  return true;
  
  */
    
#if 1
  char *cmd_header;
  char *pack_ptr; //报文处理游标
  uint16_t nb_len = bc_uart_load_buf(nb_recv);
  if (nb_len > 0)
  {
    printf("nb_recv=%s\r\n", nb_recv);
  }
  if ((nb_len > 0) & ((cmd_header = strstr((char *)nb_recv, gnss_keyword)) != NULL))
  {
    printf("recv:%s\n", nb_recv);
    sprintf(atstr, "%s", cmd_header + gnss_keyword_len + 1); //数据接收 去掉前面的回显，也要去掉keyword之后紧邻的逗号
    pack_ptr = atstr;
    printf(atstr);
    printf(pack_ptr);

    memset(utc_str, 0x00, GNSS_WORD_LEN);
    memset(pa_str, 0x00, GNSS_WORD_LEN);
    memset(la_str, 0x00, GNSS_WORD_LEN);
    memset(la_dir_str, 0x00, GNSS_WORD_LEN);
    memset(lo_str, 0x00, GNSS_WORD_LEN);
    memset(lo_dir_str, 0x00, GNSS_WORD_LEN);
    memset(sp_str, 0x00, GNSS_WORD_LEN);
    memset(ag_str, 0x00, GNSS_WORD_LEN);
    memset(utc_date_str, 0x00, GNSS_WORD_LEN);
    memset(dir_str, 0x00, GNSS_WORD_LEN);
    memset(dir_dir_str, 0x00, GNSS_WORD_LEN);
    //printf("开始提取GNSS数据\r\n");
    if ((pack_ptr = get_word(utc_str, GNSS_WORD_LEN - 1, pack_ptr, ',')) == NULL)
    {
      printf("gnss get word err,%u", __LINE__);
      return false;
    }
    else
    {
      printf("utc_str=%s\r\n", utc_str);
    }

    if ((pack_ptr = get_word(pa_str, GNSS_WORD_LEN - 1, pack_ptr, ',')) == NULL)
    {
      printf("gnss get word err,%u", __LINE__);
      return false;
    }
    if ((pack_ptr = get_word(la_str, GNSS_WORD_LEN - 1, pack_ptr, ',')) == NULL)
    {
      printf("gnss get word err,%u", __LINE__);
      return false;
    }
    if ((pack_ptr = get_word(la_dir_str, GNSS_WORD_LEN - 1, pack_ptr, ',')) == NULL)
    {
      printf("gnss get word err,%u", __LINE__);
      return false;
    }
    if ((pack_ptr = get_word(lo_str, GNSS_WORD_LEN - 1, pack_ptr, ',')) == NULL)
    {
      printf("gnss get word err,%u", __LINE__);
      return false;
    }
    if ((pack_ptr = get_word(lo_dir_str, GNSS_WORD_LEN - 1, pack_ptr, ',')) == NULL)
    {
      printf("gnss get word err,%u", __LINE__);
      return false;
    }
    if ((pack_ptr = get_word(sp_str, GNSS_WORD_LEN - 1, pack_ptr, ',')) == NULL)
    {
      printf("gnss get word err,%u", __LINE__);
      return false;
    }
    if ((pack_ptr = get_word(ag_str, GNSS_WORD_LEN - 1, pack_ptr, ',')) == NULL)
    {
      printf("gnss get word err,%u", __LINE__);
      return false;
    }
    if ((pack_ptr = get_word(utc_date_str, GNSS_WORD_LEN - 1, pack_ptr, ',')) == NULL)
    {
      printf("gnss get word err,%u", __LINE__);
      return false;
    }
    if ((pack_ptr = get_word(dir_str, GNSS_WORD_LEN - 1, pack_ptr, ',')) == NULL)
    {
      printf("gnss get word err,%u", __LINE__);
      return false;
    }
    if ((pack_ptr = get_word(dir_dir_str, GNSS_WORD_LEN - 1, pack_ptr, ',')) == NULL)
    {
      printf("gnss get word err,%u", __LINE__);
      return false;
    }
    //printf("提取GNSS数据完成\r\n");

    //判断GNSS数据是否都采集到
    //printf("判断静态位置信息 \r\n");
    if ((strlen(utc_str) <= 0) || (strlen(la_str) <= 0) || (strlen(lo_str) <= 0))
    {
      return false;
    }

    printf("utc_str:%s,la:%s,lo:%s\r\n", utc_str, la_str, lo_str);

    //判断GNSS动态数据
    //printf("判断动态位置信息 \r\n");

    //数据写入轨迹点
    

#ifndef _MOCK_GNSS
    tm[offset] = atoi(utc_str);
    x[offset] = atof(lo_str);
    y[offset] = atof(la_str);
    tsoc[offset] = soc;
    if ((strlen(sp_str) <= 0) || (strlen(ag_str) <= 0) || (strlen(dir_str) <= 0))
    {
      //return false;
      sp[offset] = 0;
      ag[offset] = 0;
    }
    else
    {
      sp[offset] = atof(sp_str);
      ag[offset] = atof(ag_str);
    }
    return true;
#else
   tm[offset] = atoi(utc_str);
    x[offset] = atof(lo_str);
    y[offset] = atof(la_str);
    tsoc[offset] = soc;

    if ((strlen(sp_str) <= 0) || (strlen(ag_str) <= 0) || (strlen(dir_str) <= 0))
    {
        sp[offset] = m_sp[offset];
        ag[offset] = m_ag[offset];    
    }
    else
    {
        sp[offset] = atof(sp_str);
        ag[offset] = atof(ag_str);
    }
    

    return true;
#endif

  }
  else
  {
      printf("FAILED>>%s\r\n",nb_recv);
    return false;
  }
  #endif
}

void track_task()
{
  switch (track.state)
  {
  case START:
    if (doStart() == FINISH)
    {
      track.state = RECORD;
      track.step = 0x00;
    }
    break;

/*
  case IDEL:
    res = doIdel();
    if (res == FINISH)
    {
      track.state = RECORD;
      track.step = 0x00;
    }
    else if (res == FAILED)
    {
      track.state = DROP;
      track.step = 0x00;
    }
    break;
    */

  case RECORD:
    res = doRecord();
    if (res == FINISH)
    {
      track.state = SUBMIT;
      track.step = 0x00;
    }
    else if (res == FAILED)
    {
      track.state = PAUSE;
      track.step = 0x00;
    }
    break;

  case PAUSE:
    res = doPause();
    if (res == FINISH)
    {
      track.state = RECORD;
      track.step = 0x00;
    }
    else if (res == FAILED)
    {
      track.state = DROP;
      track.step = 0x00;
    }
    break;

  case SUBMIT:
    res = doSubmit();
    if (res == FINISH)
    {
      track.state = END;
      track.step = 0x00;
    }
    else if (res == FAILED)
    {
      track.state = ERR;
      track.step = 0x00;
    }
    else if (res == RETRY)
    {
      track.state = SUBMIT;
      track.step = 0x00;
    }

    break;
  case DROP:
    res = doDrop();
    if (res == FINISH)
    {
      track.state = START;
      track.step = 0x00;
    }
    else if (res == FAILED)
    {
      track.state = ERR;
      track.step = 0x00;
    }
    break;

  case ERR:
    if (doErr() == FINISH)
    {
      track.state = END;
      track.step = 0x00;
    }
    break;

  case END:
    if (doEnd() == FINISH)
    {
      track.state = OTHER;
      track.step = 0x00;
    }
    break;

  default:
    break;
  }
}

static trk_res_t doStart()
{
  track.offset = 0;
  //track.data_retry = 0;
  //track.upload_retry = 0;
  strcpy(gnss_keyword, "$GNRMC");
  memset(x, 0x00, PATH_LEN * sizeof(double));
  memset(y, 0x00, PATH_LEN * sizeof(double));
  memset(sp, 0x00, PATH_LEN * sizeof(float));
  memset(ag, 0x00, PATH_LEN * sizeof(float));
  memset(tm, 0x00, PATH_LEN * sizeof(uint32_t));
  memset(tsoc, 0x00, PATH_LEN* sizeof(float)); 
  set_bc_lock(true);//轨迹处理程序开始
  return FINISH;
}

static trk_res_t doRecord()
{
  switch (track.step)
  {
  case 0x00:
    req_point();
    tm_set(TM_PATH, 2000);
    track.step++;
    break;

  case 0x01:
    if (!tm_out(TM_PATH))
    {
      if (get_point(track.offset))
      {
        track.offset++;
        track.step++;
      }
    }
    else
    {
        return FAILED;
    }
    break;


  case 0x02:
  tm_set(TM_PATH, 2000);
  track.step++;
  break;

  case 0x03:
  if(tm_out(TM_PATH))
  {
    track.step++;
  }
  break;

  case 0x04:
    if (track.offset >= PATH_LEN)
    {
      track.step = 0xAF;
    }
    else
    {
      track.step = 0x00;
    }
    break;

  default:
    return FINISH;
  }
  return GOING;
}

static trk_res_t doPause()
{
  switch (track.step)
  {
  case 0x00:
    tm_set(TM_PATH_2, TOUT);
    track.step++;
    break;

  case 0x01:
    req_point();
    tm_set(TM_PATH, 2000);
    track.step++;
    break;

  case 0x02:
    if (!tm_out(TM_PATH_2))
    {
      if (!tm_out(TM_PATH))
      {
        if (get_point(track.offset))
        {
          track.offset++;
          track.step++;
        }
      }
      else
      {
        track.step = 0x01;
      }
    }
    else
    {
      return FAILED;
    }
    break;

  default:
    return FINISH;
  }
  return GOING;
}

static trk_res_t doSubmit()
{
  switch (track.step)
  {
  case 0x00:
      track.send_off = 0;
      track.upload_retry = 0;
      track.step++;
    break;

  case 0x01:
    sprintf(msg, "path,%f,%f,%f,%f,%u,%f",
            x[track.send_off], y[track.send_off], sp[track.send_off], ag[track.send_off], tm[track.send_off], tsoc[track.send_off]);
    sprintf(atstr, "AT+MIPLNOTIFY=0,%s,%u,0,%u,1,%d,\"%s\",0,0\r\n", objtnum, BAT_OBJ, PATH_RES, strlen(msg), msg);
    printf("send_off:%u\r\n", track.send_off);
    printf("atstr(%u)=%s\r\n:", strlen(atstr), atstr);
    nb_cmd_async(atstr);
    //index--;
    //first = true;
    tm_set(TM_PATH, 3000);
    track.step++;
    break;

  case 0x02:
    if (!tm_out(TM_PATH))
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
          track.step++;
        }
        //}
      }
    }
    else
    {
      track.upload_retry++;
      if (track.upload_retry <= 3)
      {
        track.step = 0x01;
      }
      else
      {
        return FAILED;
      }
    }
    break;

  case 0x03:
  tm_set(TM_PATH, 100);
  track.step++;
  break;

  case 0x04:
  if(tm_out(TM_PATH))
    track.step++;
  break;

  case 0x05:
    track.send_off++;
    if (track.send_off < PATH_LEN)
    {
      track.step = 0x01;
    }
    else
    {
      return FINISH;
    }
    break;

  default:
    break;
  }
  return GOING;
}

static trk_res_t doDrop()
{
  if (track.data_retry < 3)
  {
    track.data_retry++;
    return FINISH;
  }
  else
  {
    return FAILED;
  }
}
static trk_res_t doErr()
{
  return FINISH;
}

static trk_res_t doEnd()
{
    track.run_over = true;
    set_bc_lock(false);//轨迹处理程序结束
  return FINISH;
}
