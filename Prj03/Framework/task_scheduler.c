/*******************************************************************************
 * 文件名：task_scheduler.c
 * 描述：  任务调度器实现 - 基于时间片的任务调度
 * 作者：  Auto-generated
 * 日期：  2025-10-20
 ******************************************************************************/

#include "task_scheduler.h"
#include "system_config.h"
#include "../Drivers/drv_timer.h"
#include "../Drivers/drv_watchdog.h"

/******************************************************************************
 * 私有变量
 ******************************************************************************/
static TaskFunc_t data_task = 0;     // 数据处理任务
static TaskFunc_t led1_task = 0;     // LED1任务
static TaskFunc_t led2_task = 0;     // LED2任务

static Uint32 data_task_counter = 0; // 数据处理任务计数器
static Uint32 led1_task_counter = 0; // LED1任务计数器
static Uint32 led2_task_counter = 0; // LED2任务计数器

/******************************************************************************
 * 私有函数声明
 ******************************************************************************/
static void scheduler_timer_callback(void);

/******************************************************************************
 * 函数实现
 ******************************************************************************/

/**
 * @brief  任务调度器初始化
 */
void Scheduler_Init(void)
{
    // 注册Timer0回调函数
    Drv_Timer_RegisterCallback(scheduler_timer_callback);
    
    // 清零计数器
    data_task_counter = 0;
    led1_task_counter = 0;
    led2_task_counter = 0;
}

/**
 * @brief  注册20ms数据处理任务
 */
void Scheduler_RegisterDataTask(TaskFunc_t task)
{
    data_task = task;
}

/**
 * @brief  注册LED1任务（1s周期）
 */
void Scheduler_RegisterLED1Task(TaskFunc_t task)
{
    led1_task = task;
}

/**
 * @brief  注册LED2任务（3s周期）
 */
void Scheduler_RegisterLED2Task(TaskFunc_t task)
{
    led2_task = task;
}

/**
 * @brief  任务调度器运行
 * @note   在主循环中调用，检查任务标志并执行
 */
void Scheduler_Run(void)
{
    // 标记：数据任务是否执行完成
    static bool data_task_completed = false;
    
    // 检查数据处理任务（20ms）
    if(data_task_counter >= DATA_PROCESS_COUNT && data_task != 0)
    {
        data_task_counter = 0;
        data_task_completed = false;  // 任务开始前标记为未完成
        data_task();                  // 执行任务（如果卡死，不会返回）
        data_task_completed = true;   // 任务完成后标记为已完成
    }
    
    // 检查LED1任务（1s）
    if(led1_task_counter >= LED1_BLINK_COUNT && led1_task != 0)
    {
        led1_task_counter = 0;
        led1_task();
    }
    
    // 检查LED2任务（3s）
    if(led2_task_counter >= LED2_BLINK_COUNT && led2_task != 0)
    {
        led2_task_counter = 0;
        led2_task();
    }
    
    //========================================================================
    // 看门狗喂狗服务（每20ms喂一次，只有任务完成后才喂）
    //========================================================================
    if(data_task_completed)
    {
        Drv_Watchdog_Service();
        data_task_completed = false;  // 清除标志
    }
}

/******************************************************************************
 * 私有函数实现
 ******************************************************************************/

/**
 * @brief  定时器回调函数（100us调用一次）
 * @note   在Timer0 ISR中被调用
 */
static void scheduler_timer_callback(void)
{
    // 递增各任务计数器
    data_task_counter++;
    led1_task_counter++;
    led2_task_counter++;
}

