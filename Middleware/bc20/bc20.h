#ifndef _BC20_H
#define _BC20_H

/************************************************************************************
 * BC20.h
 *
 * 移远BC20模块驱动程序
 *
 * 作者：刁东旭
 *
 * 日期：2020-3-18
*************************************************************************************/
#include <stdbool.h>
#include "main.h"
#include "stm32f0xx_it.h"

#define BAT_OBJ 31234 //PACK的对象编号
#define NTC_RES 30000
#define CELL_RES_1_5 30001
#define CELL_RES_6_10 30002
#define CELL_RES_11_15 30003
#define CELL_RES_15_20 30004
#define PACK_RES 30005
#define LOC_RES 30006
#define PACK_STATUS 30007
#define PATH_RES 30008

#define NB_BUF_LEN BUF_LEN
#define DBG_BUF_LEN BUF_LEN
#define CMD_LEN BUF_LEN //nb的at指令最大字节数
#define NUM_LEN 50
#define RES_LEN BUF_LEN

typedef struct
{
  bool has_dat;
  char dat[NB_BUF_LEN];
} stu_nb_ctrl;

extern char nb_recv[NB_BUF_LEN];
extern char atstr[CMD_LEN];
extern char objtnum[NUM_LEN];
extern char distnum[NUM_LEN];
extern char nb_recv[NB_BUF_LEN];
extern uint32_t nb_recv_len;
extern stu_nb_ctrl nb_ctrl;

//bool nb_cmd_sync(char* cmd, char* rcv, uint32_t* len,uint32_t time_inv, uint32_t timeout);
bool nb_cmd_wait(char *cmd, char *resp, char *rcv, uint32_t time_inv, uint32_t timeout);
bool nb_cmd_wait_without(char *cmd, char *resp, char *rcv, uint32_t time_inv, uint32_t timeout);
bool bc_init(void);
bool bc_init_gnss(void);
bool bc_pdp_act(void);
bool bc_conn_LWM2M(void);
bool nb_cmd_async(char *cmd);
void load_buf(char *rcv);
void set_bc_lock(bool lock); //bc加锁
bool bc_lock(void);              //获取bc锁状态
extern void gnss_read_task(void);
extern void task_uart2_lookup(void);
extern void sys_reset(void);
extern void bc_reset(void);
#endif
