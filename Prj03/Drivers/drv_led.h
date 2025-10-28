/*******************************************************************************
 * 文件名：drv_led.h
 * 描述：  LED驱动头文件 - 控制6个LED（GPIO0-5）
 * 作者：  Auto-generated
 * 日期：  2025-10-20
 ******************************************************************************/

#ifndef DRV_LED_H_
#define DRV_LED_H_

#include "F28x_Project.h"

/******************************************************************************
 * LED定义
 ******************************************************************************/
typedef enum {
    LED0 = 0,   // GPIO0
    LED1 = 1,   // GPIO1
    LED2 = 2,   // GPIO2
    LED3 = 3,   // GPIO3
    LED4 = 4,   // GPIO4
    LED5 = 5    // GPIO5
} LED_ID_t;

/******************************************************************************
 * 函数声明
 ******************************************************************************/

/**
 * @brief  LED初始化
 * @param  无
 * @return 无
 */
void Drv_LED_Init(void);

/**
 * @brief  LED点亮
 * @param  led: LED编号(LED1或LED2)
 * @return 无
 */
void Drv_LED_On(LED_ID_t led);

/**
 * @brief  LED熄灭
 * @param  led: LED编号(LED1或LED2)
 * @return 无
 */
void Drv_LED_Off(LED_ID_t led);

/**
 * @brief  LED翻转
 * @param  led: LED编号(LED1或LED2)
 * @return 无
 */
void Drv_LED_Toggle(LED_ID_t led);

#endif /* DRV_LED_H_ */

