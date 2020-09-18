/*******************************************************************************
 *
 *  task.h - header file for system settings of the task scheduler
 *                           definitions used in SCUD H32 Platform
 *
 *  Copyright(C) 2018 SCUD Group Ltd. All rights reserved. - http://www.scudcn.com/
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *    Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 *    Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the
 *    distribution.
 *
 *    Neither the name of SCUD nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 *  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 *  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 *  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 *  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 *  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 *  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 *  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 *
 ******************************************************************************/

#ifndef SYSTEM_TASK_H_
#define SYSTEM_TASK_H_
#include "main.h"

#define MAX_GLOBAL_TIMING_COUNT   ((uint32_t)(0xFFFFFFFF))
extern uint32_t systick_count;//systickֵ, 1ms�仯һ��

#define TASK_1MS 1
#define TASK_10MS 10
#define TASK_50MS 50
#define TASK_100MS 100
#define TASK_1S 1000

typedef enum task_numbers{
    Task_00 = 0U,
    Task_01,
    Task_02,
    Task_03,
    Task_04,
    Task_05,
    Task_06,
    Task_07,
    Task_08,
    Task_09,
    Task_10,
    Task_11,
    Task_12,
    Task_13,
    Task_14,
    Task_15
}task_number_t;

#define TASK_STARTED            ((uint8_t)(0x00))
#define TASK_NOT_READY          ((uint8_t)(0x01))
#define TASK_ALREADY_RUNNING    ((uint8_t)(0x02))
#define TASK_ALREADY_STOPPED    ((uint8_t)(0x03))
#define TASK_INITIALIZED        ((uint8_t)(0x04))

extern void task_sys_config(void);//??????????????????????????????
extern void task_handler(void); //????????????????main??????????????????
extern uint8_t configTask(uint8_t task_number, void (*function)(void), uint32_t cycle_time); //???????????????????????????????
extern uint8_t startTask(uint8_t task_number, uint8_t);
extern uint8_t stopTask(uint8_t task_number);
extern void killTask(uint8_t task_number);//????????
extern uint8_t get_task_status(uint8_t taskNumber);//??????????
extern void peek_task_status(uint8_t taskNumber);
/*
 * ????delay??????delay?????????????????
 * ???1.delay????????????????????????????????????
 *       2.?????????????????��??????????????????????????
 */
extern void task_delay(uint16_t);
extern uint32_t systick_count;
extern void* get_fun_ptr(uint8_t task_number);
#endif /* SYSTEM_TASK_H_ */
