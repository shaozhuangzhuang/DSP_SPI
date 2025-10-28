/*******************************************************************************
 * 文件名：drv_system.c
 * 描述：  系统驱动实现 - 封装系统初始化和板级配置
 * 作者：  Auto-generated
 * 日期：  2025-10-20
 * 版本：  v2.4 - 移除SPI/ADS1278/AD5754的GPIO配置（分层架构优化）
 ******************************************************************************/

#include "drv_system.h"

/******************************************************************************
 * 函数实现
 ******************************************************************************/

/**
 * @brief  系统初始化（时钟、GPIO、PIE控制器）
 */
void Drv_System_Init(void)
{
    // 系统控制初始化（时钟配置到200MHz）
    InitSysCtrl();
    
    // GPIO初始化
    InitGpio();
    
    // PIE中断控制器初始化
    DINT;                   // 禁用全局中断
    IER = 0x0000;           // 清除所有CPU中断使能
    IFR = 0x0000;           // 清除所有中断标志
    InitPieCtrl();          // 初始化PIE控制寄存器
    InitPieVectTable();     // 初始化PIE向量表
    
    // 板级GPIO配置
    Drv_Board_GpioConfig();
}

/**
 * @brief  板级GPIO配置（基础GPIO，不包括SPI/ADC/DAC）
 * @note   v2.4更新：仅配置基础GPIO（LED等）
 *         SPI/ADS1278/AD5754的GPIO配置已移至各自的驱动初始化函数中
 *         - SPI GPIO：在Drv_SPI_GPIO_Config()中配置（阶段2）
 *         - ADS1278 GPIO：在Drv_ADS1278_InitDefault()中配置（阶段3）
 *         - AD5754 GPIO：在Drv_AD5754_InitDefault()中配置（阶段3）
 */
void Drv_Board_GpioConfig(void)
{
    EALLOW;

    // LED配置 - GPIO0~5（6个LED）
    GPIO_SetupPinMux(0, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(0, GPIO_OUTPUT, GPIO_SYNC);
    
    GPIO_SetupPinMux(1, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(1, GPIO_OUTPUT, GPIO_SYNC);
    
    GPIO_SetupPinMux(2, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(2, GPIO_OUTPUT, GPIO_SYNC);
    
    GPIO_SetupPinMux(3, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(3, GPIO_OUTPUT, GPIO_SYNC);
    
    GPIO_SetupPinMux(4, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(4, GPIO_OUTPUT, GPIO_SYNC);
    
    GPIO_SetupPinMux(5, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(5, GPIO_OUTPUT, GPIO_SYNC);

    // 注意：SPI/ADS1278/AD5754的GPIO配置已移除，
    // 由各自的驱动模块在初始化时配置（符合分层架构原则）

    EDIS;
}

