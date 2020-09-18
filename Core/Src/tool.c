/************************************************************************************
 * tool.c
 *
 * 工具函数
 *
 * 作者：刁东旭
 *
 * 日期：2020-5-27
*************************************************************************************/
#include "tool.h"
#include "main.h"

void osDelay(uint32_t ms)
{
    while(ms--)
    {
        for(int i =0; i<3333;i++)
        {
            //__NOP;
            FEED_DOG;
        }
    }
}

//返回字符串长度，不包含0，如果数组长度超过size，返回-1
//max是字符串数组元素个数，如果字符串数组被填满，并且末尾没有0，则字符串返回长度为0
int32_t nn_strlen_s(char* str, int32_t size)
{
    int32_t sub_len = 0;
    while(*str != 0x00)
    {
        sub_len ++;
        if(sub_len >= size)
            return -1;
    }
    return sub_len;
}
