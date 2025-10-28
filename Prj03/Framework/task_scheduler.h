/*******************************************************************************
 * 文件名：task_scheduler.h
 * 描述：  任务调度器头文件 - 基于时间片的任务调度
 * 作者：  Auto-generated
 * 日期：  2025-10-20
 ******************************************************************************/

#ifndef TASK_SCHEDULER_H_
#define TASK_SCHEDULER_H_

#include "F28x_Project.h"

/******************************************************************************
 * 类型定义
 ******************************************************************************/

/**
 * @brief  任务函数类型
 */
typedef void (*TaskFunc_t)(void);

/******************************************************************************
 * 函数声明
 ******************************************************************************/

/**
 * @brief  任务调度器初始化
 * @param  无
 * @return 无
 */
void Scheduler_Init(void);

/**
 * @brief  注册20ms数据处理任务
 * @param  task: 任务函数指针
 * @return 无
 */
void Scheduler_RegisterDataTask(TaskFunc_t task);

/**
 * @brief  注册LED1任务（1s周期）
 * @param  task: 任务函数指针
 * @return 无
 */
void Scheduler_RegisterLED1Task(TaskFunc_t task);

/**
 * @brief  注册LED2任务（3s周期）
 * @param  task: 任务函数指针
 * @return 无
 */
void Scheduler_RegisterLED2Task(TaskFunc_t task);

/**
 * @brief  任务调度器运行
 * @param  无
 * @return 无
 * @note   在主循环中调用
 */
void Scheduler_Run(void);

#endif /* TASK_SCHEDULER_H_ */

