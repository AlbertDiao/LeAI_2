#ifndef __LED
#define __LED
/************************************************************************************
 * led.h
 *
 * led控制程序
 *
 * 作者:刁东旭
 *
 * 日期:2020-7-14
*************************************************************************************/

#include "main.h"
#define LED_SYS 0
#define LED_BMS 1
#define LED_NB 2
#define LED_CTRL 3

void led_open(uint8_t led);
void led_close(uint8_t led);
void led_toggen(uint8_t led);
#endif
