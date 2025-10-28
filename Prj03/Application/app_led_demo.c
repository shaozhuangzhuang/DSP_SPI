/*******************************************************************************
 * 文件名：app_led_demo.c
 * 描述：  LED演示示例 - 展示如何控制6个LED
 * 作者：  Auto-generated
 * 日期：  2025-10-20
 * 
 * 使用说明：
 *   1. 在app_task.c中调用这些演示函数
 *   2. 可以在不同的任务中调用不同的LED模式
 ******************************************************************************/

#include "../Drivers/drv_led.h"

/******************************************************************************
 * LED演示函数
 ******************************************************************************/

/**
 * @brief  LED流水灯效果
 * @note   顺序点亮LED0->LED1->LED2->LED3->LED4->LED5
 */
void LED_Demo_RunningLight(void)
{
    static Uint16 current_led = 0;
    
    // 熄灭所有LED
    Drv_LED_Off(LED0);
    Drv_LED_Off(LED1);
    Drv_LED_Off(LED2);
    Drv_LED_Off(LED3);
    Drv_LED_Off(LED4);
    Drv_LED_Off(LED5);
    
    // 点亮当前LED
    Drv_LED_On((LED_ID_t)current_led);
    
    // 移动到下一个LED
    current_led++;
    if(current_led > 5)
    {
        current_led = 0;
    }
}

/**
 * @brief  LED全部闪烁
 * @note   所有LED同时翻转
 */
void LED_Demo_AllBlink(void)
{
    Drv_LED_Toggle(LED0);
    Drv_LED_Toggle(LED1);
    Drv_LED_Toggle(LED2);
    Drv_LED_Toggle(LED3);
    Drv_LED_Toggle(LED4);
    Drv_LED_Toggle(LED5);
}

/**
 * @brief  LED奇偶交替闪烁
 * @note   奇数LED和偶数LED交替
 */
void LED_Demo_AlternateBlink(void)
{
    static Uint16 state = 0;
    
    if(state == 0)
    {
        // 偶数LED亮
        Drv_LED_On(LED0);
        Drv_LED_Off(LED1);
        Drv_LED_On(LED2);
        Drv_LED_Off(LED3);
        Drv_LED_On(LED4);
        Drv_LED_Off(LED5);
        state = 1;
    }
    else
    {
        // 奇数LED亮
        Drv_LED_Off(LED0);
        Drv_LED_On(LED1);
        Drv_LED_Off(LED2);
        Drv_LED_On(LED3);
        Drv_LED_Off(LED4);
        Drv_LED_On(LED5);
        state = 0;
    }
}

/**
 * @brief  LED呼吸灯效果（简化版）
 * @note   逐个点亮再逐个熄灭
 */
void LED_Demo_BreathingLight(void)
{
    static Uint16 step = 0;
    
    switch(step)
    {
        case 0: Drv_LED_On(LED0); break;
        case 1: Drv_LED_On(LED1); break;
        case 2: Drv_LED_On(LED2); break;
        case 3: Drv_LED_On(LED3); break;
        case 4: Drv_LED_On(LED4); break;
        case 5: Drv_LED_On(LED5); break;
        case 6: Drv_LED_Off(LED0); break;
        case 7: Drv_LED_Off(LED1); break;
        case 8: Drv_LED_Off(LED2); break;
        case 9: Drv_LED_Off(LED3); break;
        case 10: Drv_LED_Off(LED4); break;
        case 11: Drv_LED_Off(LED5); break;
        default: break;
    }
    
    step++;
    if(step > 11)
    {
        step = 0;
    }
}

/**
 * @brief  LED二进制计数显示
 * @note   将计数值以二进制形式显示在LED上
 * @param  count: 计数值（0-63）
 */
void LED_Demo_BinaryCounter(Uint16 count)
{
    // 显示低6位二进制
    (count & 0x01) ? Drv_LED_On(LED0) : Drv_LED_Off(LED0);
    (count & 0x02) ? Drv_LED_On(LED1) : Drv_LED_Off(LED1);
    (count & 0x04) ? Drv_LED_On(LED2) : Drv_LED_Off(LED2);
    (count & 0x08) ? Drv_LED_On(LED3) : Drv_LED_Off(LED3);
    (count & 0x10) ? Drv_LED_On(LED4) : Drv_LED_Off(LED4);
    (count & 0x20) ? Drv_LED_On(LED5) : Drv_LED_Off(LED5);
}

