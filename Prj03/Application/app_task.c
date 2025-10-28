/*******************************************************************************
 * 文件名：app_task.c
 * 描述：  应用任务实现 - 迁移自main.c的User函数
 * 作者：  Auto-generated
 * 日期：  2025-10-20
 ******************************************************************************/

#include "app_task.h"
#include "app_spi_demo.h"
#include "../Drivers/drv_led.h"

/******************************************************************************
 * 函数实现
 ******************************************************************************/

/**
 * @brief  用户初始化（应用层参数配置）
 * @note   v2.1更新：仅配置应用参数，硬件初始化已在App_Init()中完成
 *         硬件初始化顺序：System → Timer → LED → EMIF → SPI → DMA → ADS1278 → AD5754
 */
void App_Task_UserInit(void)
{
    //========================================================================
    // 应用层参数配置（非硬件初始化）
    //========================================================================
    
    // SPI应用演示：配置波形参数、滤波器参数、回调函数等
    // 注意：硬件初始化已在App_Init()的Drv_ADS1278_InitDefault()中完成
    App_Task_SPI_ConfigParams();
    
    // 其他应用层配置
    // ConfigDataProcessParams();   // 配置数据处理参数
    // ConfigCommunicationParams(); // 配置通信参数
    // ...
}

/**
 * @brief  用户主循环任务（迁移自User_MainLoop）
 */
void App_Task_UserMainLoop(void)
{
    // 用户主循环任务
    // 此函数会被持续调用
    
    // 示例：
    // UartRxProcess();     // 串口数据接收处理
    // SensorDataRead();    // 传感器数据读取
    
    // 空操作占位符
    asm(" NOP");
}

/**
 * @brief  数据处理任务（迁移自User_DataProcess）
 * @note   每20ms调用一次
 */
void App_Task_DataProcess(void)
{
    // SPI任务：ADC采样 + DAC输出
    App_Task_ADC_Sample();
    App_Task_DAC_Output();
    
    // 其他用户数据处理函数
    // DataCalculation();   // 数据计算
    // UartTxProcess();     // 数据发送
}

/**
 * @brief  LED1闪烁任务
 * @note   每1秒调用一次，控制LED0
 */
void App_Task_LED1Blink(void)
{
    Drv_LED_Toggle(LED0);
}

/**
 * @brief  LED2闪烁任务
 * @note   每3秒调用一次，控制LED1
 */
void App_Task_LED2Blink(void)
{
    Drv_LED_Toggle(LED1);
}

/**
 * @brief  SPI演示参数配置（ADS1278 + AD5754）
 * @note   v2.1更新：仅配置应用参数，硬件初始化已在App_Init()中完成
 */
void App_Task_SPI_ConfigParams(void)
{
    // 调用app_spi_demo模块的参数配置函数
    // 注意：硬件初始化已在App_Init()中完成，这里只配置业务参数
    App_SPI_Demo_ConfigParams();
}

// 注意：App_Task_ADC_Sample 和 App_Task_DAC_Output 的实现在 app_spi_demo.c 中
// 这里不再重复定义，避免链接错误

/**
 * @brief  SPI诊断任务
 * @note   可选调用，用于输出DMA/SPI统计信息
 */
void App_Task_SPI_Diagnostics(void)
{
    // 调用诊断输出函数
    App_Task_Diagnostics();
}

