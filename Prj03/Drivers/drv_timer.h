/*******************************************************************************
 * 文件名：drv_timer.h
 * 描述：  定时器驱动头文件 - Timer0配置和软计时器管理
 * 作者：  Auto-generated
 * 日期：  2025-10-20
 ******************************************************************************/

#ifndef DRV_TIMER_H_
#define DRV_TIMER_H_

#include "F28x_Project.h"

/******************************************************************************
 * 类型定义
 ******************************************************************************/

/**
 * @brief  定时器回调函数类型
 */
typedef void (*TimerCallback_t)(void);

/******************************************************************************
 * 函数声明
 ******************************************************************************/

/**
 * @brief  Timer0初始化（100us周期）
 * @param  无
 * @return 无
 */
void Drv_Timer_Init(void);

/**
 * @brief  注册Timer0中断回调函数
 * @param  callback: 回调函数指针
 * @return 无
 */
void Drv_Timer_RegisterCallback(TimerCallback_t callback);

/**
 * @brief  使能Timer0中断
 * @param  无
 * @return 无
 */
void Drv_Timer_EnableInterrupt(void);

/**
 * @brief  获取1ms tick计数
 * @return 1ms tick计数值
 */
Uint32 Drv_Timer_GetTick_1ms(void);

/**
 * @brief  获取10ms tick计数
 * @return 10ms tick计数值
 */
Uint32 Drv_Timer_GetTick_10ms(void);

/**
 * @brief  获取100us tick计数（原始tick）
 * @return 100us tick计数值
 */
Uint32 Drv_Timer_GetTick_100us(void);

#endif /* DRV_TIMER_H_ */

