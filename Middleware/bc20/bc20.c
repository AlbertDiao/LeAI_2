/************************************************************************************
 * BC20.c
 *
 * 移远BC20模块驱动程序
 *
 * 作者：刁东旭
 *
 * 日期：2020-3-18
*************************************************************************************/

#include <string.h>
#include <stdio.h>
#include "bc20.h"
#include "tool.h"
#include "main.h"
#include "led.h"
#include "timer.h"
extern UART_HandleTypeDef huart_dbg;
extern UART_HandleTypeDef huart3;

char *strx;
uint32_t nb_recv_len;
char atstr[CMD_LEN];

char objtnum[NUM_LEN];
char distnum[NUM_LEN];
char resstr[RES_LEN]; //注册资源的时候组件
char imei[NUM_LEN];
char nb_recv[NB_BUF_LEN];
bool m_bc_lock;
static uint32_t once_time = 10000;
//static int retry_max = 6;
stu_nb_ctrl nb_ctrl;

//bc加锁
void set_bc_lock(bool lock)
{
    m_bc_lock = lock;
}

//获取bc锁状态
bool bc_lock()
{
    return m_bc_lock;
}

//bc模块重置
void bc_reset()
{
    //RESET
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET);
    osDelay(100);
    
    //POWKEY
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_PIN_SET);
    osDelay(800);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_PIN_RESET);
    osDelay(3000);
          
    //HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_PIN_SET);
}

void sys_reset()
{
    bc_reset();
    NVIC_SystemReset(); //没有创建TCP SOCKET就重启系统等到服务器就绪
}
bool bc_init(void)
{
//    int retry = 0;
//    bool err = false;

    set_bc_lock(true);
    FEED_DOG;
    bc_reset(); 
/*
    //RESET
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, GPIO_PIN_RESET);
    osDelay(100);
    //POWKEY
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PIN_SET);
    osDelay(800);
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PIN_RESET);
    */

    led_toggen(LED_SYS);
    printf("\r\n*************\n");
    printf("*bc模块初始化\n");
    printf("*************\n");
    bc_uart_clear();
    //第一步：查活
    printf(">> Detect Module...");
    if (!nb_cmd_wait("AT\r\n", "OK", nb_recv, 1000, once_time))
    {
        printf("Faild.\r\n");
        return false;
    }
    printf("Success.\r\n");

    printf(">> Get CIMI:");
    if (!nb_cmd_wait_without("AT+CIMI\r\n", "ERROR", nb_recv, 1000, once_time))
    {
        printf("Faild.\r\n");
        return false;
    }
    printf("Success.\r\n");

    // printf(">> AT+CGATT=1...");
    // if (!nb_cmd_wait("AT+CGATT=1\r\n", "OK", nb_recv, 1000, 75000))
    // {
    //     printf("Faild.\r\n");
    //     return false;
    // }
    // else
    //     printf("Success.\r\n");

    // printf(">> Confirm PDP...");
    // if (!nb_cmd_wait("AT+CGATT?\r\n", "+CGATT: 1", nb_recv, 1000, once_time))
    // {
    //     printf("Faild.\r\n");
    //     return false;
    // }
    // printf("Success.\r\n");

    printf(">> BAND:");
    if (!nb_cmd_wait("AT+QBAND?\r\n", "OK", nb_recv, 1000, once_time))
    {
        printf("Faild.\r\n");
    }
    printf("Success.\r\n");

    printf(">> CSQ:");
    if (!nb_cmd_wait("AT+CSQ\r\n", "+CSQ", nb_recv, 1000, once_time))
    {
        printf("Faild.\r\n");
    }
    printf("Success.\r\n");

    printf(">> Get CEREG...");
    /*
        if (strstr(nb_recv, "+CEREG: 0,1") != NULL)
            break;
        if (strstr(nb_recv, "+CEREG: 1,1") != NULL)
            break;
    */
    if (!nb_cmd_wait("AT+CEREG?\r\n", "+CEREG:", nb_recv, 1000, once_time))
    {
        printf("Faild.\r\n");
        return false;
    }
    printf("Success.\r\n");

    set_bc_lock(false);
    return true;
}

bool bc_init_gnss(void) //启动GPS
{
//    int retry = 0;
    uint32_t g_mode;
    char *str;

    while (bc_lock())
        FEED_DOG;

    set_bc_lock(true);
    printf("*************\r\n");
    printf("***GNSS SETUP\r\n");
    printf("*************\r\n");

    //retry = 0;
    bc_uart_clear();

    if (!nb_cmd_wait("AT+QGNSSC?\r\n", "+QGNSSC:", nb_recv, 1000, once_time))
    {
        printf("Get QGNSS mode failed.\r\n");
        return false;
    }
    else
    {
        str = strstr(nb_recv, "+QGNSSC:");
        sscanf(str, "+QGNSSC:%u", &g_mode);
        printf("QGNSSC mode=%u.\r\n", g_mode);
    }

    //如果gnss没有开启，则开启
    if (g_mode == 0)
    {
        printf("Send:AT+QGNSSC=1\r\n");
        if (!nb_cmd_wait("AT+QGNSSC=1\r\n", "AT+QGNSSC=1", nb_recv, 1000, once_time))
        {
            printf("Faild.\r\n");
            return false;
        }
        else
        {
            printf("GNSS Open Success.");
        }
    }

    // printf(">> set QGNSSAGPS..\r\n");
    // if (!nb_cmd_wait("AT+QGNSSAGPS=1\r\n", "OK", nb_recv, 1000, once_time))
    // {
    //     printf("Faild.\r\n");
    //     return false;
    // }
    // printf("Success.\r\n");

    // printf(">> set QGNSSDB..\r\n");
    // if (!nb_cmd_wait("AT+QGNSSDB=1\r\n", "OK", nb_recv, 1000, once_time))
    // {
    //     printf("Faild.\r\n");
    //     return false;
    // }
    // printf("Success.\r\n");

    printf(">> set QGNSSRD..\r\n");
    //暂时跳过失败，Albert
    //if (!nb_cmd_wait_without("AT+QGNSSRD=\"NMEA/RMC\"\r\n", "ERROR", nb_recv, 1000, once_time))
    if (!nb_cmd_wait("AT+QGNSSRD=\"NMEA/RMC\"\r\n", "$GNRMC", nb_recv, 1000, once_time))
    {
        printf("Faild.\r\n");
        return false;
    }
    printf("Success.\r\n");

    set_bc_lock(false);
    return true;
}

bool bc_pdp_act(void) //激活场景，为连接服务器做准备
{
    //int retry = 0;

    while (bc_lock())
        FEED_DOG;

    set_bc_lock(true);

    printf("*************\r\n");
    printf("****pdp初始化\r\n");
    printf("*************\r\n");

    printf(">> set CGPADDR..");
    bc_uart_clear();
    if (!nb_cmd_wait("AT+CGPADDR=1\r\n", "OK", nb_recv, 1000, once_time))
    {
        printf("Faild.\r\n");
        return false;
    }
    printf("Success.\r\n");

    printf(">> ACT..\r\n");
    if (!nb_cmd_wait("AT+CGSN=1\r\n", "+CGSN: ", nb_recv, 1000, once_time))
    {
        printf("Faild.\r\n");
        return false;
    }
    else
    {
        strx = strstr(nb_recv, "+CGSN: ");
        sscanf(strx, "+CGSN: %s\r\n", imei);
        printf("strx:%s\r\nimei:%s\r\n", strx, imei);
    }
    printf("Success.\r\n");

    printf(">> Check ACT...");
    if (!nb_cmd_wait("AT+CGATT?\r\n", "+CGATT: 1", nb_recv, 1000, once_time))
    {
        printf("Faild.\r\n");
        return false;
    }
    printf("Success.\r\n");

    set_bc_lock(false);
    return true;
}

bool bc_conn_LWM2M(void)
{
    char strtmp[100];
    int retry = 0;
    const int retry_max = 6; //超时时间为100 * 300 = 30秒
    while (bc_lock())
        FEED_DOG;

    set_bc_lock(true);

    printf("*************\r\n");
    printf("**LWM2M初始化\r\n");
    printf("*************\r\n");

    if (nb_cmd_wait("AT+MIPLDELETE=0\r\n", "OK", nb_recv, 1000, 2000))
    {
        printf("与平台连接已清除。\r\n");
    }
    else
    {
        printf("无连接或清除失败。\r\n");
    }

    if (!nb_cmd_wait("AT+MIPLCREATE\r\n", "OK", nb_recv, 1000, 5000))
    {
        //osDelay(1000);
        printf("Create connect failed, close at first.\n");
        //if (!nb_cmd_wait("AT+MIPLCLOSE=0\r\n", "+MIPLEVENT: 0,15", nb_recv, 1000, once_time))
        nb_cmd_wait("AT+MIPLCLOSE=0\r\n", "OK", nb_recv, 1000, once_time);
        printf("Start. LWM2M faild again, system restart.(ERROR: %s)\r\n", nb_recv);
        sys_reset();
    }
    else
    {
        printf("LWM2M Started.\n");
    }

    //注册PACK对象
    printf("Add Object.\n");
    sprintf(atstr, "AT+MIPLADDOBJ=0,%d,1,\"1\",10,10\r\n", BAT_OBJ);
    if (!nb_cmd_wait(atstr, "OK", nb_recv, 1000, once_time))
    //if (!nb_cmd_async(atstr))
    {
        printf("Faild.\r\n");
        return false;
    }
    printf("Success.\r\n");

    printf(">> Get object number..");
    retry = 0;
    while (retry <= retry_max)
    {
        if (nb_cmd_wait("AT+MIPLOPEN=0,86400\r\n", "OK", nb_recv, 1000, once_time))
            break;
        retry++;
        osDelay(300);
        printf("retry(%u)\r\n", retry);
    }

    if (retry > retry_max)
    {
        printf("Timeout error.\n");
        return false;
    }

    printf("\r\nwaiting for MIPLOBSERVE ");
    tm_set(1, 20000);
    do
    {
        FEED_DOG;
        if (bc_uart_load_buf(nb_recv) <= 0)
        {
            continue;
        }
        printf("\r\n>***************\r\n");
        printf(nb_recv);
        printf("****************\r\n");
        //开始阅读服务器发来的ogjtnum
        strx = strstr(nb_recv, "+MIPLOBSERVE");
        if (strx == NULL)
        {
            continue;
        }
        strx = strstr(strx + 1, ",");
        memset(objtnum, 0, NUM_LEN);
        for (int i = 0;; i++) //查询观察号
        {
            if (strx[i + 1] == ',')
                break;
            objtnum[i] = strx[i + 1];
        }
        //printf("Get AT cmd:atbuf = %s \r\n",atbuf);
        printf("Get object numb:objtnum = %s \r\n", objtnum);
        break;
    } while (!tm_out(1));

    if (tm_out(1))
    {
        printf("CAN NOT GET OBJTNUM.");
        return false;
    }

    //获取资源号
    printf(">> MIPLOBSERVERSP..\r\n");
    memset(atstr, 0, CMD_LEN);
    sprintf(atstr, "AT+MIPLOBSERVERSP=0,%s,1\r\n", objtnum);
    retry = 0;
    while (retry <= retry_max)
    {
        if (nb_cmd_wait(atstr, "+MIPLDISCOVER", nb_recv, 1000, once_time))
            break;
        retry++;
        osDelay(300);
        printf("retry(%u)\r\n", retry);
    }

    if (retry <= retry_max)
    {
        strx = strstr(nb_recv, "+MIPLDISCOVER");
        strx = strstr(strx + 1, ",");
        memset(distnum, 0x00, NUM_LEN);
        for (int i = 0;; i++)
        {
            if (strx[i + 1] == ',')
                break;
            distnum[i] = strx[i + 1];
        }
        printf("Get discover numb:distnum = %s \r\n", distnum);
    }
    else
    {
        printf("Timeout error.\n");
        return false;
    }

    printf(">> MIPLDISCOVERRSP..\n");
    memset(atstr, 0, CMD_LEN);
    sprintf(strtmp, "%u;%u;%u;%u;%u;%u;%u", NTC_RES, CELL_RES_1_5, CELL_RES_6_10, CELL_RES_11_15, CELL_RES_15_20, PACK_RES, PACK_STATUS);
    sprintf(atstr, "AT+MIPLDISCOVERRSP=0,%s,1,%d,\"%s\"\r\n", distnum, strlen(strtmp), strtmp);
    printf(atstr);

    nb_cmd_async(atstr);
    retry = 0;
    while (retry <= retry_max)
    {
        if (nb_cmd_wait(atstr, "OK", nb_recv, 1000, once_time))
            break;
        retry++;
        osDelay(300);
        printf("retry(%u)\r\n", retry);
    }
    if (retry <= retry_max)
    {
        printf("Success.\r\n");
    }
    else
    {
        printf("Faild.\r\n");
        return false;
    }

    set_bc_lock(false);
    return true;
}

//向nb模块发送指令
HAL_StatusTypeDef nb_send_cmd(char *str)
{
    //发送超时时间统一设定
    int len = strlen(str);
    //指令过长
    if (len > CMD_LEN)
        return HAL_ERROR;
    else
        return HAL_UART_Transmit(&huart3, (uint8_t *)str, len, 0xFFFF);

}

//安全向nb发送指令，包含重发
bool nb_send_cmd_s(char *str)
{
    const int TIMES = 10;
    HAL_StatusTypeDef res;
    int i = 0;
#ifdef _COMM_DEEP
    printf("nb_send_cmd_s\r\n");
#endif
    for (i = 0; i < TIMES; i++)
    {
#ifdef _COMM_DEEP
        printf("|s1");
#endif
        res = nb_send_cmd(str);
#ifdef _COMM_DEEP
        printf("|s2");
#endif
        if (res == HAL_OK)
        {
#ifdef _COMM_DEEP
            printf("|ok");
#endif
            break;
        }
#ifdef _COMM_DEEP
        printf("|s3");
#endif
        if (res == HAL_BUSY)
        {
            osDelay(300);
        }
#ifdef _COMM_DEEP
        printf("|s4");
#endif
    }

    if (i >= TIMES)
    {
#ifdef _COMM_DEEP
        printf("|false");
#endif
        return false;
    }
#ifdef _COMM_DEEP
    printf("|true");
#endif
    return true;
}

//同步收发消息，将cmd指令给NB模块，并将返回结果放入rev中，超时为timeout ms，备注：长度不超过BUF_LEN
/*
bool nb_cmd_sync(char *cmd, char *rcv, uint32_t *len, uint32_t time_inv, uint32_t timeout)
{
    //static uint32_t time_inv = 500;
    uint32_t times = timeout / time_inv;
    uint32_t t = 0;
    uint16_t len_tmp;

    if (times == 0)
        times = 1;

#ifdef _COMM_DEEP
    printf("\r\ntimes=%u, inv=%u, timeout=%u", times, time_inv, timeout);
#endif
    do
    {
        FEED_DOG;
        led_toggen(LED_SYS);
        //清除串口数据接收标志
        bc_uart_clear();

        //发送数据
        if (nb_send_cmd_s(cmd) != true)
        {
#ifdef _COMM_DEEP
            printf("|err");
#endif
            return false;
        }
        osDelay(time_inv);
#ifdef _COMM_DEEP
        printf("|w");
#endif

        len_tmp = bc_uart_load_buf(rcv);
        printf("|len=%u|",len_tmp);
        //超时处理
        if (t++ > times)
            break;
    }
    //查询串口数据接收标志
    while (len_tmp == 0);

    if (t <= times)
    {
        *len = len_tmp;
        //bc_uart_clear();
        return true;
    }
    else
        return false;
}
*/

//同步收发消息，将cmd指令给NB模块，并等待结果包含resp的报文，超时为timeout ms，备注：长度不超过BUF_LEN
bool nb_cmd_wait(char *cmd, char *resp, char *rcv, uint32_t time_inv, uint32_t timeout)
{
    //static uint32_t time_inv = 500;
    uint32_t times = timeout / time_inv;
    uint32_t t = 0;
    //uint8_t rcv[BUF_LEN];
    bool get_resp = false;

    if (times == 0)
        times = 1;

    //printf("\r\ntimes=%u, inv=%u, timeout=%u", times, time_inv, timeout);
    do
    {
        FEED_DOG;
        led_toggen(LED_SYS);
        //清除串口数据接收标志
        //bc_uart_clear();

        //发送数据

        printf("\r\nsend:>>>>>>>\r\n");
        printf(cmd);
        printf("<<<<<<<<<<<<<<<<\r\n");
        if (nb_send_cmd_s(cmd) != true)
        {
#ifdef _COMM_DEEP
            printf("|err");
#endif
            return false;
        }
        //osDelay(time_inv);
#ifdef _COMM_DEEP
        printf("|w");
#endif
        tm_set(1, time_inv); //设置超时时间值，如果在超时时间内获得到预期反馈，则结束流程，否则一直等待，直到超时
        while (!tm_out(1))
        {
            FEED_DOG;
            if (bc_uart_load_buf(rcv) > 0)
            {
                printf("\r\nrecv:>>>>>>>\r\n");
                printf(rcv);
                printf("<<<<<<<<<<<<<<<<\r\n");

                if (strstr(rcv, resp) != NULL)
                {
                    get_resp = true;
                    break;
                }
            }
        }

        if (get_resp)
        {
            break;
        }
        else
        {
            printf("tm_out");
        }

    } while (++t < times);

    if (get_resp)
    {
        return true;
    }
    else
        return false;
}

bool nb_cmd_wait_without(char *cmd, char *resp, char *rcv, uint32_t time_inv, uint32_t timeout)
{
    //static uint32_t time_inv = 500;
    uint32_t times = timeout / time_inv;
    uint32_t t = 0;
    //uint8_t rcv[BUF_LEN];
    bool get_resp = false;
    bool get_word = false;

    if (times == 0)
        times = 1;

#ifdef _COMM_DEEP
    printf("\r\ntimes=%u, inv=%u, timeout=%u", times, time_inv, timeout);
#endif
    do
    {
        FEED_DOG;
        led_toggen(LED_SYS);
        //清除串口数据接收标志
        bc_uart_clear();

        //发送数据
        if (nb_send_cmd_s(cmd) != true)
        {
#ifdef _COMM_DEEP
            printf("|err");
#endif
            return false;
        }
        //osDelay(time_inv);
#ifdef _COMM_DEEP
        printf("|w");
#endif

        tm_set(1, time_inv); //设置超时时间值，如果在超时时间内获得到预期反馈，则结束流程，否则一直等待，直到超时
        while (!tm_out(1))
        {
            FEED_DOG;
            if (bc_uart_load_buf(rcv) > 0)
            {
                get_resp = true;
                printf("\r\n****************\r\n");
                printf(rcv);
                printf("****************\r\n");
                if (strstr(nb_recv, resp))
                {
                    get_word = true;
                    break;
                }
            }
        }

        //超时处理
        if (get_resp)
            break;
    }
    //查询串口数据接收标志
    while (++t < times);

    if (get_resp)
    {
        if (get_word)
            return false;
        else
            return true;
    }
    else
    {
        return false;
    }
}

//异步收发消息，将cmd指令给NB模块, 直接返回
bool nb_cmd_async(char *cmd)
{
    //清除串口数据接收标志
    //bc_uart_clear();
    return nb_send_cmd_s(cmd);
}

void task_uart2_lookup()
{
#define LU_MSG_LEN 10
    char lu_msg[LU_MSG_LEN];  //存放NB命令下发的body.arg的内容
    char pri_atstr[50]; //存放NB命令下发的body.arg的内容
    char *sx;
    int offset;

    uint32_t msg_res;
    uint32_t msg_id;
    //uint16_t msg_obj_id;
    //uint16_t msg_ins_id;
    //uint16_t msg_res_id;
    //uint16_t msg_len;

    //for(;;)
    {
        osDelay(500);
        //taskENTER_CRITICAL();
        //printf("|t_u_l");
        //taskEXIT_CRITICAL();
        //提取usart数据
        if (!bc_uart_load_buf(nb_recv))
            return;
        //解析命令

        //处理命令下发
        //+MIPLEXECUTE: 0,11097,8863,0,5000,4,"ping"
        /*
		uint16_t msg_res;
		uint16_t msg_id;
		uint16_t msg_obj_id;
		uint16_t msg_ins_id;
		uint16_t msg_res_id;
		uint16_t msg_len;
		uint16_t msg_arg;
		*/
        sx = strstr(nb_recv, "+MIPLEXECUTE:");

        if (sx != NULL)
        {
            printf("\r\nstr:%s\r\n", sx);
            printf("\r\nnb_recv:%s\r\n", nb_recv);

            //获得命令下发的内容
            //sscanf(strx, "+MIPLEXECUTE: %u,%u,%u,%u,%u,%u,\"%s\"\r\n",
            //&msg_res, &msg_id, &msg_obj_id, &msg_ins_id, &msg_res_id, &msg_len, msg);
            sscanf(sx, "+MIPLEXECUTE: %u,%u",
                   &msg_res, &msg_id);
            sx = strstr(nb_recv, "\"");
            if (sx == NULL)
                return;
            offset = 0;
            while (*(++sx) != '\"')
            {
                lu_msg[offset++] = *sx;
            }
            lu_msg[offset] = 0x00;
            printf("msg_res = %u\r\n", msg_res);
            printf("msg_id = %u\r\n", msg_id);
            printf("EXE: lu_msg={%s}\r\n", lu_msg);
            //解析msg的值,a:开启， b：关闭
            if (lu_msg[0] == 'a')
            {
                HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, GPIO_PIN_RESET);
                HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_RESET);
            }
            else if (lu_msg[0] == 'b')
            {
                HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, GPIO_PIN_SET);
                HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_SET);
            }
            //返回命令包, 调用成功，第三个参数返回2
            //+MIPLEXECUTERSP: <ref>,<msgId>,<result> (2)
            sprintf(pri_atstr, "AT+MIPLEXECUTERSP=%u,%u,2\r\n", msg_res, msg_id);
            nb_cmd_async(pri_atstr);
        }
    }
}
