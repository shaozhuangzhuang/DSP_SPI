/*******************************************************************************
 * 文件名：drv_spi.h
 * 描述：  SPI驱动头文件 - 支持SPIA/SPIB主模式DMA传输
 * 作者：  Auto-generated
 * 日期：  2025-10-20
 ******************************************************************************/

#ifndef DRV_SPI_H_
#define DRV_SPI_H_

#include "F28x_Project.h"
#include "system_config.h"

/******************************************************************************
 * 枚举定义
 ******************************************************************************/

/**
 * @brief  SPI模块选择
 */
typedef enum {
    SPI_MODULE_A = 0,    // SPIA - 连接ADS1278 ADC
    SPI_MODULE_B = 1,    // SPIB - 连接AD5754 DAC
    SPI_MODULE_C = 2     // SPIC - 保留
} SPI_Module_t;

/**
 * @brief  SPI时钟模式
 * Mode  CPOL  CPHA  采样沿      时钟空闲      应用
 *  0     0     0    上升沿      低电平        ADS1278 (要求)
 *  1     0     1    下降沿      低电平        AD5754 (推荐)
 *  2     1     0    下降沿      高电平        通用
 *  3     1     1    上升沿      高电平        AD5754 (可选)
 */
typedef enum {
    SPI_MODE_0 = 0,      // CPOL=0, CPHA=0 (ADS1278要求，上升沿采样)
    SPI_MODE_1 = 1,      // CPOL=0, CPHA=1 (AD5754推荐，下降沿采样)
    SPI_MODE_2 = 2,      // CPOL=1, CPHA=0
    SPI_MODE_3 = 3       // CPOL=1, CPHA=1 (AD5754可选)
} SPI_Mode_t;

/**
 * @brief  SPI配置结构体
 */
typedef struct {
    SPI_Module_t module;     // SPI模块选择
    Uint32 clock_hz;         // SPI时钟频率（Hz）
    SPI_Mode_t mode;         // SPI时钟模式
    Uint16 data_width;       // 数据位宽 (4-16位)
    bool fifo_enable;        // 是否使能FIFO
    Uint16 tx_fifo_level;    // TX FIFO中断触发电平 (0-16)
    Uint16 rx_fifo_level;    // RX FIFO中断触发电平 (0-16)
} SPI_Config_t;

/******************************************************************************
 * 全局变量声明
 ******************************************************************************/

// SPI寄存器基地址指针数组（用于统一访问）
extern volatile struct SPI_REGS *g_spi_regs[3];

/******************************************************************************
 * 函数声明
 ******************************************************************************/

/**
 * @brief  SPI GPIO引脚配置（所有SPI模块）
 * @param  无
 * @return 无
 * @note   配置SPIA(GPIO54-57)和SPIB(GPIO24-27)
 */
void Drv_SPI_GPIO_Config(void);

/**
 * @brief  SPI模块初始化
 * @param  config - SPI配置结构体指针
 * @return 无
 */
void Drv_SPI_Init(SPI_Config_t *config);

/**
 * @brief  SPIA初始化（ADS1278专用配置）
 * @param  无
 * @return 无
 * @note   主模式、16位、Mode1、10MHz、FIFO使能、RX触发=12
 */
void Drv_SPIA_Init(void);

/**
 * @brief  SPIB初始化（AD5754专用配置）
 * @param  无
 * @return 无
 * @note   主模式、16位、Mode1、10MHz、FIFO使能、TX触发=0
 */
void Drv_SPIB_Init(void);

/**
 * @brief  设置SPI片选信号
 * @param  module - SPI模块
 * @param  active - true=拉低CS（选中），false=拉高CS（未选中）
 * @return 无
 * @note   SPIA使用GPIO57，SPIB使用GPIO27
 */
void Drv_SPI_SetCS(SPI_Module_t module, bool active);

/**
 * @brief  清空SPI FIFO
 * @param  module - SPI模块
 * @return 无
 * @note   清除TX/RX FIFO及溢出标志
 */
void Drv_SPI_ClearFIFO(SPI_Module_t module);

/**
 * @brief  使能SPI DMA请求
 * @param  module - SPI模块
 * @return 无
 * @note   配置SPI以DMA模式工作，禁用SPI中断
 */
void Drv_SPI_EnableDMA(SPI_Module_t module);

/**
 * @brief  禁用SPI模块
 * @param  module - SPI模块
 * @return 无
 */
void Drv_SPI_Disable(SPI_Module_t module);

/**
 * @brief  启用SPI模块
 * @param  module - SPI模块
 * @return 无
 */
void Drv_SPI_Enable(SPI_Module_t module);

/**
 * @brief  获取SPI FIFO状态
 * @param  module - SPI模块
 * @param  tx_fifo_level - 输出TX FIFO当前字数
 * @param  rx_fifo_level - 输出RX FIFO当前字数
 * @return 无
 */
void Drv_SPI_GetFIFOStatus(SPI_Module_t module, Uint16 *tx_fifo_level, Uint16 *rx_fifo_level);

/******************************************************************************
 * 内联辅助函数
 ******************************************************************************/

/**
 * @brief  计算SPIBRR值
 * @param  spi_clk_hz - 目标SPI时钟频率
 * @return SPIBRR寄存器值
 * @note   SPICLK = LSPCLK / (SPIBRR + 1)
 */
static inline Uint16 Drv_SPI_CalcBRR(Uint32 spi_clk_hz)
{
    return (Uint16)((LSPCLK_HZ / spi_clk_hz) - 1);
}

/******************************************************************************
 * 配置宏定义
 ******************************************************************************/

// CS引脚定义
// 注意：SPIA的CS引脚在ADS1278硬件上已固定拉低，不需要GPIO控制
// GPIO57的实际用途：ADS1278的DRDY输入信号（由Drv_ADS1278_GPIO_Config配置）
// #define SPI_CS_SPIA_GPIO        57      // [已废弃] GPIO57实际用于DRDY，非CS
// #define SPI_CS_SPIA_GPIO        58      // [备用] 如需软件控制CS，可用GPIO58（当前硬件不需要）
#define SPI_CS_SPIB_GPIO        27      // SPIB片选GPIO（AD5754）

// FIFO配置
#define SPI_FIFO_DEPTH          16      // SPI FIFO深度
#define SPIA_RX_FIFO_LEVEL      12      // ADC接收：12字触发DMA
#define SPIA_TX_FIFO_LEVEL      0       // ADC发送：FIFO空时触发DMA
#define SPIB_RX_FIFO_LEVEL      6       // DAC接收：6字触发DMA
#define SPIB_TX_FIFO_LEVEL      0       // DAC发送：FIFO空时触发DMA

// 时序延迟
#define SPI_CS_SETUP_DELAY_NS   50      // CS建立时间延迟（ns）
#define SPI_CS_HOLD_DELAY_NS    50      // CS保持时间延迟（ns）

/******************************************************************************
 * 使用说明
 ******************************************************************************/

/*
 * 1. 初始化流程：
 *    a. 调用Drv_SPI_GPIO_Config()配置所有SPI引脚
 *    b. 调用Drv_SPIA_Init()或Drv_SPIB_Init()初始化对应模块
 *    c. 调用Drv_SPI_EnableDMA()使能DMA模式
 *
 * 2. DMA传输流程：
 *    a. Drv_SPI_ClearFIFO() - 清空FIFO
 *    b. 配置DMA通道地址和长度
 *    c. Drv_SPI_SetCS(module, true) - 拉低CS
 *    d. 启动DMA传输（RX先启动，TX后启动）
 *    e. 等待DMA完成中断
 *    f. Drv_SPI_SetCS(module, false) - 拉高CS
 *
 * 3. SPI模式说明：
 *    - ADS1278要求Mode1 (CPOL=0, CPHA=1)
 *    - AD5754支持Mode1或Mode3
 *    - 本驱动统一使用Mode1
 *
 * 4. 字长配置：
 *    - F28377D SPI支持4-16位字长
 *    - 24位数据需拆分为3次16位传输或2次传输(16+8)
 *    - 当前配置：16位字长，24位数据分3次传输
 *
 * 5. FIFO配置：
 *    - RX FIFO触发电平=12：当FIFO有12字时触发DMA
 *    - TX FIFO触发电平=0：当FIFO空时触发DMA
 *    - ONESHOT模式：每次传输完成后停止，需重新配置
 *
 * 6. 注意事项：
 *    - 使用DMA时必须禁用SPI中断
 *    - CS由GPIO手动控制，不使用SPI硬件CS
 *    - 每次传输前必须清空FIFO
 *    - DMA传输时CPU不应访问SPI寄存器
 */

#endif /* DRV_SPI_H_ */

