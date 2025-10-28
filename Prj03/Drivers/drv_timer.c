/*******************************************************************************
 * 文件名：drv_timer.c
 * 描述：  定时器驱动实现 - Timer0配置和软计时器管理
 * 作者：  Auto-generated
 * 日期：  2025-10-20
 ******************************************************************************/

#include "drv_timer.h"

/******************************************************************************
 * 私有变量
 ******************************************************************************/
static TimerCallback_t timer0_callback = 0;  // Timer0回调函数指针
static Uint32 tick_100us = 0;                // 100us tick计数
static Uint32 tick_1ms = 0;                  // 1ms tick计数
static Uint32 tick_10ms = 0;                 // 10ms tick计数
static Uint16 tick_1ms_counter = 0;          // 1ms计数器(100us*10)
static Uint16 tick_10ms_counter = 0;         // 10ms计数器(1ms*10)

/******************************************************************************
 * 私有函数声明
 ******************************************************************************/
interrupt void timer0_isr(void);

/******************************************************************************
 * 函数实现
 ******************************************************************************/

/**
 * @brief  Timer0初始化（100us周期）
 */
void Drv_Timer_Init(void)
{
    // 初始化CPU定时器
    InitCpuTimers();
    
    // 配置Timer0: 200MHz CPU, 100us周期
    ConfigCpuTimer(&CpuTimer0, 200, 100);
    
    // 配置PIE中断向量
    EALLOW;
    PieVectTable.TIMER0_INT = &timer0_isr;
    EDIS;
    
    // 启动Timer0（但不使能中断，由外部调用EnableInterrupt）
    CpuTimer0Regs.TCR.all = 0x4000;
}

/**
 * @brief  注册Timer0中断回调函数
 */
void Drv_Timer_RegisterCallback(TimerCallback_t callback)
{
    timer0_callback = callback;
}

/**
 * @brief  使能Timer0中断
 */
void Drv_Timer_EnableInterrupt(void)
{
    // 使能PIE组1中断1 (TIMER0)
    IER |= M_INT1;
    PieCtrlRegs.PIEIER1.bit.INTx7 = 1;
    
    // 使能全局中断
    EINT;
    ERTM;
}

/**
 * @brief  获取1ms tick计数
 */
Uint32 Drv_Timer_GetTick_1ms(void)
{
    return tick_1ms;
}

/**
 * @brief  获取10ms tick计数
 */
Uint32 Drv_Timer_GetTick_10ms(void)
{
    return tick_10ms;
}

/**
 * @brief  获取100us tick计数（原始tick）
 */
Uint32 Drv_Timer_GetTick_100us(void)
{
    return tick_100us;
}

/******************************************************************************
 * 中断服务程序
 ******************************************************************************/

/**
 * @brief  Timer0中断服务程序（100us周期）
 */
interrupt void timer0_isr(void)
{
    // 100us tick递增
    tick_100us++;
    
    // 1ms tick计数
    tick_1ms_counter++;
    if(tick_1ms_counter >= 10)  // 100us * 10 = 1ms
    {
        tick_1ms_counter = 0;
        tick_1ms++;
        
        // 10ms tick计数
        tick_10ms_counter++;
        if(tick_10ms_counter >= 10)  // 1ms * 10 = 10ms
        {
            tick_10ms_counter = 0;
            tick_10ms++;
        }
    }
    
    // 调用注册的回调函数
    if(timer0_callback != 0)
    {
        timer0_callback();
    }
    
    // 系统中断计数（用于调试）
    CpuTimer0.InterruptCount++;
    
    // 响应PIE组1中断
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;
}

