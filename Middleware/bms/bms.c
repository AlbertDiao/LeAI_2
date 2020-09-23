/******************************
* 
* 文件名：bms.c
* 功能：处理与bms的通信、控制
* 作者：刁东旭
* 日期：2020-4-27
* 邮箱：ddx@outlook.com
*
*******************************/
#include "sys.h"
#include "bms.h"
#include <string.h>
#include <stdio.h>
#include "tool.h"

#define BMS_SEND_SIZE 270
#define _MOCK_BAT

extern UART_HandleTypeDef huart2;

uint8_t bms_send_buf[BMS_SEND_SIZE];
uint8_t bms_send[100];
uint8_t bms_rcv[100];
uint32_t bms_rcv_len;
uint16_t cell_v[20];
uint8_t ntc[5];
int16_t pack_i;//pack电流
uint32_t pack_v;//pack电压
uint8_t soc;
uint16_t fcc;
stu_bat_status bat_status;

void bms_init()
{
}

uint8_t crc(uint8_t* dat, uint8_t len)
{
    return 0x00;
}

//向bms模块发送指令
HAL_StatusTypeDef bms_send_cmd(uint8_t* send)
{
    
    //发送超时时间统一设定
    uint32_t len = send[4] + 6;
    
    //指令过长
    if(len > BMS_SEND_SIZE)
        return HAL_ERROR;
    else
    {
        /*
        for(int i = 0;i<len;i++)
            printf("0x%02X,", send[i]);
        printf("\r\n");
        */
        HAL_StatusTypeDef res;
        HAL_GPIO_WritePin(GPIOC,GPIO_PIN_4, GPIO_PIN_SET);
        
        res = HAL_UART_Transmit(&huart2, send, len, 2000);
        HAL_GPIO_WritePin(GPIOC,GPIO_PIN_4, GPIO_PIN_RESET);
        return res;

    }
}

//安全向bms发送指令，包含重发
bool bms_send_cmd_s(uint8_t* send)
{
    const int TIMES = 10;
    HAL_StatusTypeDef res;
    int i = 0;

    //HAL_UART_StateTypeDef state;
    //等待串口空闲
	/*
    state = HAL_UART_GetState(&hbms_uart);
    while((state != HAL_UART_STATE_READY) && (state != HAL_UART_STATE_BUSY_RX))
    {
        printf("s=0x%X\n", state);
        state = HAL_UART_GetState(&hbms_uart);
        osDelay(1000);
    }
	*/

    for(i = 0; i< TIMES; i++)
    {
        //state = HAL_UART_GetState(&hbms_uart);
        //printf("s=0x%X\n", state);
        res = bms_send_cmd(send);
        //printf("r=0x%X\n", res);
        
        if(res == HAL_OK)
        {
            break;
        }

        if(res == HAL_BUSY)
        {
            osDelay(100);
        }

    }
    if(i == TIMES)
        return false;
    return true;
}

//同步收发消息，将cmd指令给bms模块，并将返回结果放入rev中，超时为timeout ms，备注：长度不超过BUF_LEN
bool bms_cmd_sync(uint8_t* send, uint8_t* rcv, uint32_t * rcv_len, uint32_t timeout)
{
    static uint32_t time_inv = 100;
    uint32_t times = timeout / time_inv;
    uint32_t t = 0;

    if(times == 0)
        times = 1;

    //printf("times=%u, inv=%u, timeout=%u",times, time_inv, timeout);
    do
    {
        //清除串口数据接收标志
        bms_uart_clear();

        //发送数据
        if(bms_send_cmd_s(send) != true)
            return false;

        osDelay(time_inv);
        //printf("|w|");

        //超时处理
        if(t++ > times)
            break;

    }
    //查询串口数据接收标志
    while(bms_uart_recv != true);

    if(t <= times)
    {
        if(bms_uart_recv_len > BUF_LEN)
            return false;

        memcpy(rcv, bms_uart_buf, bms_uart_recv_len);
        *rcv_len = bms_uart_recv_len;
        bms_uart_clear();
        return true;
    }
    else
        return false;
}

bool bms_get_v()
{
#ifndef _MOCK_BAT
    printf("\r\nbms_get_v:");
    //18 FF 80 10 00 A6
    bms_send[0] = 0x18;
    bms_send[1] = 0xFF;
    bms_send[2] = 0x80;
    bms_send[3] = 0x10;
    bms_send[4] = 0x00;
    bms_send[5] = 0xA6;
    bms_cmd_sync(bms_send, bms_rcv, &bms_rcv_len, 2000);
    /*
    for(int i = 0;i<bms_rcv_len;i++)
        printf("0x%02X,", bms_rcv[i]);
    printf("\r\n");
    */
    if(bms_rcv_len < 46)
    {
        printf("len error(%d)\r\n",bms_rcv_len);
        return false;
    }
    for(int i = 0; i< 20;i++)
    {
        cell_v[i] = bms_rcv[5+i*2] + bms_rcv[5+i*2 +1] * 0xFF;
        printf("%u,", cell_v[i]);
    }
    printf("\r\n");
	return true;
#else
    for(int i = 0; i< 20;i++)
    {
        cell_v[i] = 3250;
    }
	return true;
#endif
}

bool bms_get_ntc()
{
#ifndef _MOCK_BAT
    //printf("bms_get_ntc:");
    //18 FF 80 11 00 B3
    bms_send[0] = 0x18;
    bms_send[1] = 0xFF;
    bms_send[2] = 0x80;
    bms_send[3] = 0x11;
    bms_send[4] = 0x00;
    bms_send[5] = 0xB3;
    bms_cmd_sync(bms_send, bms_rcv, &bms_rcv_len, 2000);

    if(bms_rcv_len != 11)
        return false;
    for(int i = 0; i< 5;i++)
    {
        ntc[i] = bms_rcv[5+i];
        printf("|ntc[%d]=%d,",i, ntc[i]);
    }
    //printf("\r\n");
    return true;
#else
    for(int i = 0; i< 5;i++)
    {
        ntc[i] = 30;
    //    printf("%d,",ntc[i]);
	}
	return true;
#endif
}

bool bms_get_i()
{
#ifndef _MOCK_BAT
    int16_t tmp;
    //printf("bms_get_i:");
    //18 FF 80 13 00 99
    bms_send[0] = 0x18;
    bms_send[1] = 0xFF;
    bms_send[2] = 0x80;
    bms_send[3] = 0x13;
    bms_send[4] = 0x00;
    bms_send[5] = 0x99;
    bms_cmd_sync(bms_send, bms_rcv, &bms_rcv_len, 2000);
    if(bms_rcv_len != 8)
        return false;
    //memcpy(&pack_i, &bms_rcv[5], 1);
    //memcpy((&pack_i)+1, &bms_rcv[6], 1);
    //pack_i /= 2;
    //pack_i = (bms_rcv[5] + bms_rcv[6] * 0xFF) *2;
    tmp = bms_rcv[6];
    pack_i = bms_rcv[5] | (tmp << 8);
    pack_i *= 2;
    printf("pack_i = %d", pack_i);
    return true;
#else
	pack_i = 0;
	return true;
#endif
}

bool bms_get_pack_v()
{
#ifndef _MOCK_BAT
    //18 FF 80 15 00 E7
    bms_send[0] = 0x18;
    bms_send[1] = 0xFF;
    bms_send[2] = 0x80;
    bms_send[3] = 0x15;
    bms_send[4] = 0x00;
    bms_send[5] = 0xE7;
    bms_cmd_sync(bms_send, bms_rcv, &bms_rcv_len, 2000);
    if(bms_rcv_len != 8)
        return false;
    //printf("rcv 6 = 0x%02X, rcv7 = 0x%02X\r\n", bms_rcv[5], bms_rcv[6]);
    //memcpy(&pack_v, &bms_rcv[6], 1);
    //memcpy((&pack_v)+1, &bms_rcv[7], 1);
    pack_v = (bms_rcv[6] * 0xFF + bms_rcv[5])*2;
    //pack_v /= 2;
    printf("|pack_v=%u", pack_v);
	return true;
#else
	pack_v = 64000;
	return true;
#endif
}

bool bms_get_fcc()
{
#ifndef _MOCK_BAT
    //18 FF 80 17 00 CD
    bms_send[0] = 0x18;
    bms_send[1] = 0xFF;
    bms_send[2] = 0x80;
    bms_send[3] = 0x17;
    bms_send[4] = 0x00;
    bms_send[5] = 0xCD;
    bms_cmd_sync(bms_send, bms_rcv, &bms_rcv_len, 2000);
    if(bms_rcv_len != 8)
        return false;
    fcc = (bms_rcv[6] * 0xFF + bms_rcv[5]);
    printf("|fcc=%u", fcc);
	return true;
#else
	fcc = 30;
	return true;
#endif
}

bool bms_get_soc()
{
#ifndef _MOCK_BAT
    //18 FF 80 18 00 0E
    bms_send[0] = 0x18;
    bms_send[1] = 0xFF;
    bms_send[2] = 0x80;
    bms_send[3] = 0x18;
    bms_send[4] = 0x00;
    bms_send[5] = 0x0E;
    bms_cmd_sync(bms_send, bms_rcv, &bms_rcv_len, 2000);
    if(bms_rcv_len != 7)
        return false;
    soc = bms_rcv[5];
    printf("|soc=%u", soc);
    return true;
#else
	soc = 50;
	return true;
#endif
}

//获取电池系统的状态
bool bms_get_status(void)
{
    #ifndef _MOCK_BAT
    //18 FF 80 16 00 D8 
    //18 FF 80 16 04 00 00 0C 01 09 00 
    bms_send[0] = 0x18;
    bms_send[1] = 0xFF;
    bms_send[2] = 0x80;
    bms_send[3] = 0x16;
    bms_send[4] = 0x00;
    bms_send[5] = 0xD8;
    bms_cmd_sync(bms_send, bms_rcv, &bms_rcv_len, 2000);
    if(bms_rcv_len < 10)
    {
        printf("len err(%u)", bms_rcv_len);
        return false;

    }

    bat_status.warn.byte = bms_rcv[5];
    bat_status.protect.byte = bms_rcv[6];
    bat_status.sys.byte = bms_rcv[7];
    bat_status.sys2.byte = bms_rcv[8];
    printf("|bat_status, warn=0x%02X, protect=0x%02X,sys=0x%02X, sys2=0x%02X\r\n", bat_status.warn.byte, bat_status.protect.byte, bat_status.sys.byte, bat_status.sys2.byte);
    return true;
    #else
    bat_status.warn.byte = 0x00;
    bat_status.protect.byte = 0x00;
    bat_status.sys.byte = 0x00;
    bat_status.sys2.byte = 0x01;
    //printf("|bat_status, warn=0x%02X, protect=0x%02X,sys=0x%02X, sys2=0x%02X\r\n", bat_status.warn.byte, bat_status.protect.byte, bat_status.sys.byte, bat_status.sys2.byte);
    return true;
    #endif
}
