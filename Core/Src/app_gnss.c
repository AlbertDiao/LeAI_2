/************************************************************************************
 * app_gnss.c
 *
 * gnss_应用层
 *
 * 作者：刁东旭
 *
 * 日期：2020-8-30
*************************************************************************************/

#include <stdio.h>
#include <string.h>
#include "stm32f0xx_it.h"
#include "app_gnss.h"
#include "timer.h"
#include "led.h"
#include "tool.h"

#define GNSS_MSG_LEN 100

stu_gnss_upload gnss_upload;

char gnss_msg[GNSS_MSG_LEN];
char la_dir_str[GNSS_WORD_LEN];
char lo_dir_str[GNSS_WORD_LEN];
char state_str[GNSS_WORD_LEN];
char star_num_str[GNSS_WORD_LEN];
char hdop_str[GNSS_WORD_LEN];
char height_str[GNSS_WORD_LEN];
char pa_str[GNSS_WORD_LEN];
char utc_date_str[GNSS_WORD_LEN];
char dir_dir_str[GNSS_WORD_LEN];
char utc_str[GNSS_WORD_LEN];
char la_str[GNSS_WORD_LEN];
char lo_str[GNSS_WORD_LEN];
char sp_str[GNSS_WORD_LEN];
char ag_str[GNSS_WORD_LEN];
char dir_str[GNSS_WORD_LEN];
char gnss_atstr[BUF_LEN];

bool gnss_data_new;
char gnss_keyword[GNSS_WORD_LEN];
int gnss_keyword_len;
/*
 * 从str处获得token之前的字符
 * 返回值：如果获取成功，返回token之后的位置，
 *         如果找不到token，返回NULL
 *         如果word的长度（不包含结束符\0），超过了word_len_max，返回NULL
 */
char *get_word(char *word, char word_len_max, char *str, char token)
{
  int word_len = 0;
  ;
  *word = 0;
  while (*str != '\0')
  {
    //结束条件1：找到token
    if (*str == token)
    {
      *word = '\0';
      return str + 1;
    }
    *word = *str;
    str++;
    word++;
    word_len++;
    //结束条件2：word长度超限
    if (word_len > word_len_max)
      return 0x00;
  }
  //结束条件3：字符串扫描已结束，没有找到token
  return 0x00;
}

int32_t gdb_u_l, gdb_la_l, gdb_lo_l;
char *gdb_cmd_header;
char *gdb_pack_ptr; //报文处理游标
void gnss_read_task()
{
#define GNSS_READ_OVER 0xF0

  printf("gr=%02X,%d,%d|", gnss_read.step, gnss_data_new, bc_lock());
  switch (gnss_read.step)
  {
  case 0x00:
    if (gnss_read.to_stop)
    {
      gnss_read.step = 0xFF;
      break;
    }
    else
    {
      if (!gnss_data_new)
      {
        if (!bc_lock())
        {
          set_bc_lock(true); //GNSS read开始
          printf("set_bc_lock(true) in %u\r\n", __LINE__);
          printf("Get gnss data.\r\n");
          gnss_read.stopped = false;
          gnss_read.step++;
        }
      }
    }
    break;

  case 0x01:
    strcpy(gnss_keyword, "$GNRMC");
    gnss_keyword_len = strlen(gnss_keyword);
    printf("QGNSSRD RMC\r\n");
    sprintf(gnss_atstr, "AT+QGNSSRD=\"NMEA/RMC\"\r\n");
    if (!nb_cmd_async(gnss_atstr))
    {
      gnss_read.step = GNSS_READ_OVER; //GNSS read指令发送失败
    }
    else
    {
      tm_set(TM_GPS_READ, 5000);
      gnss_read.step++;
    }
    break;

  case 0x02:
    if (!tm_out(TM_GPS_READ))
    {
      if (bc_uart_load_buf(gnss_atstr) > 0)
      {
        gdb_cmd_header = strstr(gnss_atstr, gnss_keyword);
        if (gdb_cmd_header != NULL)
        {
          printf("recv:%s\n", gnss_atstr);

          //sprintf(gnss_atstr, "%s", gdb_cmd_header + gnss_keyword_len + 1); //数据接收 去掉前面的回显，也要去掉keyword之后紧邻的逗号
          //gdb_pack_ptr = gnss_atstr;
          gdb_pack_ptr = gdb_cmd_header + gnss_keyword_len + 1;
          printf(gnss_atstr);
          printf(gdb_pack_ptr);

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
          if ((gdb_pack_ptr = get_word(utc_str, GNSS_WORD_LEN - 1, gdb_pack_ptr, ',')) == NULL)
          {
            printf("gnss get word err,%u", __LINE__);
            gnss_read.step = GNSS_READ_OVER;
            break;
          }
          else
          {
            printf("utc_str=%s\r\n", utc_str);
          }

          if ((gdb_pack_ptr = get_word(pa_str, GNSS_WORD_LEN - 1, gdb_pack_ptr, ',')) == NULL)
          {
            printf("gnss get word err,%u", __LINE__);
            gnss_read.step = GNSS_READ_OVER;
            break;
          }
          if ((gdb_pack_ptr = get_word(la_str, GNSS_WORD_LEN - 1, gdb_pack_ptr, ',')) == NULL)
          {
            printf("gnss get word err,%u", __LINE__);
            gnss_read.step = GNSS_READ_OVER;
            break;
          }
          if ((gdb_pack_ptr = get_word(la_dir_str, GNSS_WORD_LEN - 1, gdb_pack_ptr, ',')) == NULL)
          {
            printf("gnss get word err,%u", __LINE__);
            gnss_read.step = GNSS_READ_OVER;
            break;
          }
          if ((gdb_pack_ptr = get_word(lo_str, GNSS_WORD_LEN - 1, gdb_pack_ptr, ',')) == NULL)
          {
            printf("gnss get word err,%u", __LINE__);
            gnss_read.step = GNSS_READ_OVER;
            break;
          }
          if ((gdb_pack_ptr = get_word(lo_dir_str, GNSS_WORD_LEN - 1, gdb_pack_ptr, ',')) == NULL)
          {
            printf("gnss get word err,%u", __LINE__);
            gnss_read.step = GNSS_READ_OVER;
            break;
          }
          if ((gdb_pack_ptr = get_word(sp_str, GNSS_WORD_LEN - 1, gdb_pack_ptr, ',')) == NULL)
          {
            printf("gnss get word err,%u", __LINE__);
            gnss_read.step = GNSS_READ_OVER;
            break;
          }
          if ((gdb_pack_ptr = get_word(ag_str, GNSS_WORD_LEN - 1, gdb_pack_ptr, ',')) == NULL)
          {
            printf("gnss get word err,%u", __LINE__);
            gnss_read.step = GNSS_READ_OVER;
            break;
          }
          if ((gdb_pack_ptr = get_word(utc_date_str, GNSS_WORD_LEN - 1, gdb_pack_ptr, ',')) == NULL)
          {
            printf("gnss get word err,%u", __LINE__);
            gnss_read.step = GNSS_READ_OVER;
            break;
          }
          if ((gdb_pack_ptr = get_word(dir_str, GNSS_WORD_LEN - 1, gdb_pack_ptr, ',')) == NULL)
          {
            printf("gnss get word err,%u", __LINE__);
            gnss_read.step = GNSS_READ_OVER;
            break;
          }
          if ((gdb_pack_ptr = get_word(dir_dir_str, GNSS_WORD_LEN - 1, gdb_pack_ptr, ',')) == NULL)
          {
            printf("gnss get word err,%u", __LINE__);
            gnss_read.step = GNSS_READ_OVER;
            break;
          }
          //printf("提取GNSS数据完成\r\n");

          //判断GNSS数据是否都采集到
          //printf("判断静态位置信息 \r\n");
          //        gdb_u_l=nn_strlen_s(utc_str, GNSS_WORD_LEN);
          //        gdb_la_l=nn_strlen_s(la_str, GNSS_WORD_LEN);
          //        gdb_lo_l=nn_strlen_s(lo_str, GNSS_WORD_LEN);
          gdb_u_l = strlen(utc_str);
          gdb_la_l = strlen(la_str);
          gdb_lo_l = strlen(lo_str);
          if ((gdb_u_l <= 0) || (gdb_la_l <= 0) || (gdb_lo_l <= 0))
          {
            //printf("GNSS位置数据缺失 \r\n");
            gnss_read.step = GNSS_READ_OVER;
            break;
          }
          else
          {
            printf("GNSS位置数据获取成功！\r\n");
            gnss_data_new = true;
          }

          printf("utc_str:%s,la:%s,lo:%s\r\n", utc_str, la_str, lo_str);

          //判断GNSS动态数据
          //printf("判断动态位置信息 \r\n");
          if ((strlen(sp_str) <= 0) || (strlen(ag_str) <= 0) || (strlen(dir_str) <= 0))
          {
            //printf("GNSS动态数据缺失 \r\n");
            //return true;
          }
          else
            printf("sp:%s,ag:%s,dir:%s\r\n", sp_str, ag_str, dir_str);
          //return true;false

          gnss_read.step = GNSS_READ_OVER;
        }
      }
    }
    else
    {
      printf("gnss read time out.\r\n");
      gnss_read.step = GNSS_READ_OVER;
    }

    break;

    //释放BC资源30秒，给其他模块工作
  case GNSS_READ_OVER:
    set_bc_lock(false); //GNSS数据读取结束，释放bc资源
    printf("set_bc_lock(false) in %u\r\n", __LINE__);
    tm_set(TM_GPS_READ, 1000);
    gnss_read.step++;
    break;

  case GNSS_READ_OVER + 1:
    if (tm_out(TM_GPS_READ) || gnss_read.to_stop)
      gnss_read.step++;
    break;

  default:
    if (gnss_read.to_stop)
    {
      gnss_read.stopped = true;
    }
    else
    {
      gnss_read.step = 0;
    }
    break;
  }
}

void gnss_upload_task()
{
#define GNSS_UPLOAD_OVER 0xF0
  if (gnss_upload.step != 4)
  {
    printf("gu=%02X,%u,%u|", gnss_upload.step, gnss_data_new, bc_lock());
  }
  switch (gnss_upload.step)
  {
  case 0x00:
    if (gnss_upload.to_stop)
    {
      gnss_upload.step = 0xFF;
      break;
    }
    else
    {
      if (gnss_data_new)
      {
        if (!bc_lock())
        {
          set_bc_lock(true); //Gnss upload开始
          printf("set_bc_lock(true) in %u\r\n", __LINE__);
          printf("Gnss data upload.\r\n");
          gnss_upload.step++;
        }
      }
      else
      {
        gnss_upload.step = 0xFF; //继续等待
      }
    }
    break;

  case 0x01:
    sprintf(gnss_msg, "loc,%s,%s,%s",
            utc_str, la_str, lo_str);
    sprintf(gnss_atstr, "AT+MIPLNOTIFY=0,%s,%u,0,%u,1,%d,\"%s\",0,0\r\n", objtnum, BAT_OBJ, LOC_RES, strlen(gnss_msg), gnss_msg);
    printf("gnss_atstr(%u)=%s\r\n:", strlen(gnss_atstr), gnss_atstr);
    nb_cmd_async(gnss_atstr);
    led_toggen(LED_NB);
    tm_set(TM_GPS_UPLOAD, 5000);
    gnss_upload.step++;
    break;

  case 0x02:
    if (!tm_out(TM_GPS_UPLOAD))
    {
      if ((bc_uart_load_buf(nb_recv) > 0) & ((strstr((char *)nb_recv, "OK")) != NULL))
      {
        printf("GNSS upload success.\r\n");
        gnss_data_new = false;
        gnss_upload.step = GNSS_UPLOAD_OVER;
      }
    }
    else
    {
      gnss_upload.step = GNSS_UPLOAD_OVER;
    }

    break;

  case GNSS_UPLOAD_OVER:
    //业务成功或失败都进入这个状态，释放gnss资源，并等待
    printf("set_bc_lock(false) in %u\r\n", __LINE__);
    set_bc_lock(false);           //GNSS upload结束，释放BC资源
    tm_set(TM_GPS_UPLOAD, 10000); //等待10秒
    gnss_upload.step++;
    break;

  case GNSS_UPLOAD_OVER + 1:
    if (tm_out(TM_GPS_UPLOAD))
    {
      printf("GPS TM got.\r\n");
      gnss_upload.step++;
    }
    break;

  default:
    //一旦通知停止任务，任务就锁死了
    if (gnss_upload.to_stop)
    {
      gnss_upload.stopped = true;
    }
    else
    {
      gnss_upload.step = 0;
    }
    break;
  }
}
