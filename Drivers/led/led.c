/************************************************************************************
 * led.c
 *
 * led控制程序
 *
 * 作者:刁东旭
 *
 * 日期:2020-7-144
*************************************************************************************/
#include "led.h"
/*
	HAL_GPIO_WritePin(GPIOA,GPIO_PIN_8, GPIO_PIN_SET);//LED1
	HAL_GPIO_WritePin(GPIOC,GPIO_PIN_9, GPIO_PIN_SET);//LED2
	HAL_GPIO_WritePin(GPIOC,GPIO_PIN_8, GPIO_PIN_SET);//LED3
*/
#define LED_MIN 0
#define LED_MAX 5

GPIO_TypeDef * LED_GROUP[] = {GPIOB, GPIOB, GPIOA};
uint16_t LED_PIN[] = {GPIO_PIN_7, GPIO_PIN_6, GPIO_PIN_13};

void led_init()
{
    for(int i = LED_MIN; i<=LED_MAX;i++)
        led_close(i);
}

void led_open(uint8_t led)
{
    if(led > LED_MAX)
        return;
    
    HAL_GPIO_WritePin(LED_GROUP[led],LED_PIN[led],GPIO_PIN_RESET);
}

void led_close(uint8_t led)
{
    if(led > LED_MAX)
        return;
    HAL_GPIO_WritePin(LED_GROUP[led],LED_PIN[led],GPIO_PIN_SET);
}

void led_toggen(uint8_t led)
{
    if(led > LED_MAX)
        return;
    
    HAL_GPIO_TogglePin(LED_GROUP[led],LED_PIN[led]);

}
