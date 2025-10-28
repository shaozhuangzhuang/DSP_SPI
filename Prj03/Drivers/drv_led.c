/*******************************************************************************
 * 文件名：drv_led.c
 * 描述：  LED驱动实现 - 控制6个LED（GPIO0-5）
 * 作者：  Auto-generated
 * 日期：  2025-10-20
 ******************************************************************************/

#include "drv_led.h"

/******************************************************************************
 * LED GPIO定义（GPIO0-5）
 ******************************************************************************/
#define LED0_GPIO   0
#define LED1_GPIO   1
#define LED2_GPIO   2
#define LED3_GPIO   3
#define LED4_GPIO   4
#define LED5_GPIO   5

/******************************************************************************
 * 函数实现
 ******************************************************************************/

/**
 * @brief  LED初始化
 * @note   GPIO0-5已在Drv_Board_GpioConfig中配置
 */
void Drv_LED_Init(void)
{
    // LED初始状态：全部熄灭
    GpioDataRegs.GPADAT.bit.GPIO0 = 0;
    GpioDataRegs.GPADAT.bit.GPIO1 = 0;
    GpioDataRegs.GPADAT.bit.GPIO2 = 0;
    GpioDataRegs.GPADAT.bit.GPIO3 = 0;
    GpioDataRegs.GPADAT.bit.GPIO4 = 0;
    GpioDataRegs.GPADAT.bit.GPIO5 = 0;
}

/**
 * @brief  LED点亮
 */
void Drv_LED_On(LED_ID_t led)
{
    switch(led)
    {
        case LED0: GpioDataRegs.GPADAT.bit.GPIO0 = 1; break;
        case LED1: GpioDataRegs.GPADAT.bit.GPIO1 = 1; break;
        case LED2: GpioDataRegs.GPADAT.bit.GPIO2 = 1; break;
        case LED3: GpioDataRegs.GPADAT.bit.GPIO3 = 1; break;
        case LED4: GpioDataRegs.GPADAT.bit.GPIO4 = 1; break;
        case LED5: GpioDataRegs.GPADAT.bit.GPIO5 = 1; break;
        default: break;
    }
}

/**
 * @brief  LED熄灭
 */
void Drv_LED_Off(LED_ID_t led)
{
    switch(led)
    {
        case LED0: GpioDataRegs.GPADAT.bit.GPIO0 = 0; break;
        case LED1: GpioDataRegs.GPADAT.bit.GPIO1 = 0; break;
        case LED2: GpioDataRegs.GPADAT.bit.GPIO2 = 0; break;
        case LED3: GpioDataRegs.GPADAT.bit.GPIO3 = 0; break;
        case LED4: GpioDataRegs.GPADAT.bit.GPIO4 = 0; break;
        case LED5: GpioDataRegs.GPADAT.bit.GPIO5 = 0; break;
        default: break;
    }
}

/**
 * @brief  LED翻转
 */
void Drv_LED_Toggle(LED_ID_t led)
{
    switch(led)
    {
        case LED0: GpioDataRegs.GPATOGGLE.bit.GPIO0 = 1; break;
        case LED1: GpioDataRegs.GPATOGGLE.bit.GPIO1 = 1; break;
        case LED2: GpioDataRegs.GPATOGGLE.bit.GPIO2 = 1; break;
        case LED3: GpioDataRegs.GPATOGGLE.bit.GPIO3 = 1; break;
        case LED4: GpioDataRegs.GPATOGGLE.bit.GPIO4 = 1; break;
        case LED5: GpioDataRegs.GPATOGGLE.bit.GPIO5 = 1; break;
        default: break;
    }
}

