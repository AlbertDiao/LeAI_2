/***********************************************************
 *
 * 飞毛腿动力电池BMS系统HT32平台软件
 *       SCUD HT32 Platform V1.0.0
 *
 * Copyright(C) 2018 SCUD Group Ltd. 
 *       All rights reserved.
 * 
 ***********************************************************/
 
 /***********************************************************
 *
 * 文件：task.c
 *
 * 作者：刁东旭
 * 日期：2018/12/15
 * 邮箱: ddx@outlook.com
 *
 * 合作式任务调度系统，时钟基于systick_count变量，每10us自增一次
 *
 *************************************************************/
#include "task.h"
//#include "debug.h"

#define MAX_TASK_NUMBERS 16
uint32_t systick_count;//systick值, 1ms变化一次
uint32_t td_deep; //任务执行深度

typedef struct sys_task
{
    uint8_t      status;
    uint32_t     cycle;
    void (*taskPointer)(void);
    uint32_t    lastCounterHeld;
    uint8_t     startUpOffset;
}sys_task_t;

typedef enum task_status{
    Uninitialized = 0U,        // not ready for kick-start, default status
    Active,                    // already running
    Deactive                   // initialized but not running, task "ready"
}task_status_t;

extern uint32_t systick_count;

uint8_t task_number_max = MAX_TASK_NUMBERS;//最大任务数
static sys_task_t Tasks[MAX_TASK_NUMBERS];

//任务号合法性检查
uint8_t task_num_avb(uint8_t taskNumber)
{
	if(taskNumber >= MAX_TASK_NUMBERS)
	{
		return 0;
	}	
	return 1;
}

void task_sys_config()
{
	uint8_t i;
	for(i=0;i<MAX_TASK_NUMBERS;i++)
	{
	  stopTask(i);
		killTask(i);
	}
}

uint8_t startTask(uint8_t taskNumber, uint8_t offset)
{		
	#ifdef _DBG
	printf("startTask:%u, %u|",taskNumber, offset);
	#endif
	
	if(task_num_avb(taskNumber) != 1)
	{
		#ifdef _DBG
			printf("TASK_NOT_READY\n");
		#endif
		return TASK_NOT_READY;
	}
		
    if(Tasks[taskNumber].status == Uninitialized)
    {
		#ifdef _DBG
		printf("TASK_NOT_READY\n");
		#endif
        //task not initialized, cannot be started
        return    TASK_NOT_READY;
    }
    else if(Tasks[taskNumber].status == Active)
    {
        //task already running
		#ifdef _DBG
		printf("TASK_ALREADY_RUNNING\n");
		#endif

        return    TASK_ALREADY_RUNNING;
    }
    else if(Tasks[taskNumber].status == Deactive)
    {
        Tasks[taskNumber].lastCounterHeld = systick_count;    //set current counter value + offset to the task counter holder
        Tasks[taskNumber].status = Active;                            //activate task
        Tasks[taskNumber].startUpOffset = offset;
		#ifdef _DBG
		printf("TASK_STARTED\n");
		#endif
        return TASK_STARTED;
    }
    else
    {
        //error status
        //consider re-initialize task
		#ifdef _DBG
		printf("TASK_NOT_READY\n");
		#endif
        return    TASK_NOT_READY;
    }
	
}

uint8_t stopTask(uint8_t taskNumber)
{
	if(task_num_avb(taskNumber) != 1)
	{
		#ifdef _DBG
			printf("TASK_NOT_READY\n");
		#endif
		return TASK_NOT_READY;
	}
		
    if(Tasks[taskNumber].status == Active)
    {
        Tasks[taskNumber].status = Deactive;

        //log("TASK_INITIALIZED\n");
        return TASK_INITIALIZED;
    }
    else if(Tasks[taskNumber].status == Deactive)    //task is deactive
    {
        //log("TASK_ALREADY_STOPPED\n");
        return TASK_ALREADY_STOPPED;
    }
    else    //task is un-initialized or error status
    {
        return TASK_NOT_READY;
    }
}

uint8_t configTask(uint8_t taskNumber, void (*function)(void), uint32_t cycle)
{
	////log("configTask:%u, 0x%08X, %u\n",taskNumber, (unsigned int)function, cycle);
	
	if(task_num_avb(taskNumber) != 1)
	{
		#ifdef _DBG
			printf("TASK_NOT_READY\n");
		#endif
		return TASK_NOT_READY;
	}
		
	if(function == 0x00)
	{
		return TASK_NOT_READY;
	}		
    if(Tasks[taskNumber].status == Active)
    {
        //task is still running.
        return TASK_ALREADY_RUNNING;

    }
    else    //task is deactive, un-initialized or error status
    {
        Tasks[taskNumber].taskPointer = function;
        Tasks[taskNumber].cycle = cycle;
        Tasks[taskNumber].lastCounterHeld = 0U;
        Tasks[taskNumber].startUpOffset = 0U;

        Tasks[taskNumber].status = Deactive;

        return TASK_INITIALIZED;
    }
}

void killTask(uint8_t taskNumber)
{
	if(task_num_avb(taskNumber) != 1)
	{
		#ifdef _DBG
			printf("TASK NUMBER ERROR\n");
		#endif
		return;
	}

    Tasks[taskNumber].taskPointer = 0U;
    Tasks[taskNumber].cycle = 0U;
    Tasks[taskNumber].lastCounterHeld = 0U;
    Tasks[taskNumber].startUpOffset = 0U;
    Tasks[taskNumber].status = Uninitialized;
}

uint8_t get_task_status(uint8_t taskNumber)
{
	if(task_num_avb(taskNumber) != 1)
	{
		#ifdef _DBG
			printf("TASK NUMBER ERROR\n");
		#endif
		return Uninitialized;
	}
	return Tasks[taskNumber].status;
}

void task_handler(void)
{
  uint8_t index;
  uint32_t wait_time = 0;

  // a basic task looper here
  for (index = 0; index < MAX_TASK_NUMBERS; index++)
  {
    if (Tasks[index].status == Active)
    {
      if (Tasks[index].lastCounterHeld > systick_count)
      {
        //systick_count rounded up
        wait_time = MAX_GLOBAL_TIMING_COUNT - Tasks[index].lastCounterHeld;
        wait_time += systick_count;
      }
      else
      {
        wait_time = systick_count - Tasks[index].lastCounterHeld;
      }

      //printf("%u|%u\n", wait_time, Tasks[index].cycle);
      if (wait_time >= Tasks[index].cycle)
      {
        //if(Tasks[index].startUpOffset != 0)
        //{
        //    Tasks[index].startUpOffset--;
        //}
        //else
        {
          //task cycle is valid
          //任务配置的时间是指任务两次被唤醒的时间的间隔
          Tasks[index].lastCounterHeld = systick_count; //save current counter value to counter holder
          //Tasks[index].status = TASK_ALREADY_RUNNING;
          ////log("t%u|", index);
          (*Tasks[index].taskPointer)();        //run task
          ////log("o|");
          //Tasks[index].status = Active;
        }
      }
    }
    else
    {
      //current task is not active
    }
  }
}

/*
 * 任务delay函数，delay时可以运行其他任务
 * 注意：1.delay的时间不精确，视任务队列中任务的实际情况而定
 *       2.本函数不一定能成功执行，如果任务深度过大，将不会执行延时
 */
void task_delay(uint16_t dt)
{	
	uint16_t i;
	extern uint32_t td_deep;
	
	//任务循环嵌套深度最大为5
	if(td_deep >= 5)
	{
		return;
	}
	
	td_deep++;
	for(i=0;i<dt;i++)
	{
		task_handler();
	}
	td_deep--;
}

void peek_task_status(uint8_t taskNumber)
{
  if(!task_num_avb(taskNumber))
    return;
  //log("\nTasks[%u].%u\n", taskNumber, Tasks[taskNumber].status);
}

void* get_fun_ptr(uint8_t task_number)
{
  if(!task_num_avb(task_number))
    return NULL;

  return Tasks[task_number].taskPointer;
}
