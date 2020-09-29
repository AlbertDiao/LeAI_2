#ifndef _BMS_H
#define _BMS_H

#include <stdbool.h>
#include "stm32f0xx_hal.h"
#include "stm32f0xx_it.h"
#include <stdbool.h>

struct stu_bms_cmd
{
    uint8_t cmd;
    uint8_t len;
    uint8_t data[255];
};

typedef struct 
{
    union {
        struct
        {
            uint8_t w_d_uv : 1;    //放电欠压警告
            uint8_t w_c_ov : 1;    //充电过压警告
            uint8_t w_d_oc : 1;    //放电过流警告
            uint8_t w_c_oc : 1;    //充电过流警告
            uint8_t w_d_ot : 1;    //放电温度警告
            uint8_t w_c_ot : 1;    //充电温度警告
            uint8_t w_soc_low : 1; //低soc警告
            uint8_t w_uv_lock : 1; //欠压锁
        } bits;
        uint8_t byte;
    } warn;

    union {
        struct
        {
            uint8_t p_d_uv : 1; //放电欠压保护
            uint8_t p_c_ov : 1; //充电过压保护
            uint8_t p_d_oc : 1; //放电过流保护
            uint8_t p_c_oc : 1; //充电过流保护
            uint8_t p_d_ot : 1; //放电温度保护
            uint8_t p_c_ot : 1; //充电温度保护
            uint8_t p_sc : 1;   //短路保护
            uint8_t pcb_ot : 1; //pcb过温保护
        } bits;
        uint8_t byte;
    } protect;

    union {
        struct
        {
            uint8_t ba : 1;       //自主均衡
            uint8_t force_ba : 1; //强制均衡
            uint8_t d_mos : 1;    //放电mos状态
            uint8_t c_mos : 1;    //充电mos状态
            uint8_t pack_ov : 1;  //电池组过压
            uint8_t resv : 1;     //保留位
            uint8_t cell_excp;    //电芯异常
            uint8_t ntc_excp;     //ntc异常
        } bits;
        uint8_t byte;
    } sys;

    union {
        struct
        {
            uint8_t fuse:1;//保险丝状态
        }bits;
        uint8_t byte;
    } sys2;
} stu_bat_status;

extern void bms_init();
extern bool bms_get_v(void);
extern bool bms_get_ntc(void);
extern bool bms_get_i(void);
extern bool bms_get_pack_v(void);
extern bool bms_get_soc(void);
extern bool bms_get_fcc(void);
extern bool bms_get_status(void);
extern void tx_to_bms(void);
extern void rx_from_bms(void);

extern uint16_t cell_v[20];
extern uint8_t ntc[5];
extern int16_t pack_i;  //pack电流
extern uint32_t pack_v; //pack电压
extern uint8_t soc;
extern uint16_t fcc;
extern stu_bat_status bat_status;

#endif
