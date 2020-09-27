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
#define LED_CTRL 0
#define LED_SYS 2
#define LED_BMS 3
#define LED_NB 4
#define LED_PATH 5

void led_open(uint8_t led);
void led_close(uint8_t led);
void led_toggen(uint8_t led);
#endif
