/*******************************************************************************
 * 文件名：drv_dma_buffers.h
 * 描述：  DMA缓冲区头文件
 * 作者：  Auto-generated
 * 日期：  2025-10-20
 ******************************************************************************/

#ifndef DRV_DMA_BUFFERS_H_
#define DRV_DMA_BUFFERS_H_

#include "F28x_Project.h"

/******************************************************************************
 * 外部变量声明（DMA缓冲区）
 * 注意：这些缓冲区由DMA硬件直接访问，不应声明为volatile
 ******************************************************************************/

// ADS1278 ADC双缓冲（Ping-Pong）
extern Uint16 g_adc_rx_ping[];      // ADC接收Ping缓冲：12个16位字 = 24字节
extern Uint16 g_adc_rx_pong[];      // ADC接收Pong缓冲：12个16位字 = 24字节
extern Uint16 g_adc_tx_dummy[];     // ADC虚拟TX缓冲：12个16位字（生成时钟用）

// AD5754 DAC缓冲
extern Uint16 g_dac_tx_buffer[];    // DAC发送缓冲：6个16位字 = 12字节
extern Uint16 g_dac_rx_buffer[];    // DAC接收缓冲：6个16位字（读回验证用）

/******************************************************************************
 * 函数声明
 ******************************************************************************/

/**
 * @brief  DMA缓冲区初始化
 * @param  无
 * @return 无
 */
void Drv_DMA_BuffersInit(void);

/**
 * @brief  获取ADC Ping缓冲区地址
 * @return Ping缓冲区首地址
 */
Uint16* Drv_DMA_GetAdcPingBuffer(void);

/**
 * @brief  获取ADC Pong缓冲区地址
 * @return Pong缓冲区首地址
 */
Uint16* Drv_DMA_GetAdcPongBuffer(void);

/**
 * @brief  获取ADC TX虚拟数据缓冲区地址
 * @return TX虚拟数据缓冲区首地址
 */
Uint16* Drv_DMA_GetAdcTxDummyBuffer(void);

/**
 * @brief  获取DAC TX缓冲区地址
 * @return DAC TX缓冲区首地址
 */
Uint16* Drv_DMA_GetDacTxBuffer(void);

/**
 * @brief  获取DAC RX缓冲区地址
 * @return DAC RX缓冲区首地址
 */
Uint16* Drv_DMA_GetDacRxBuffer(void);

/******************************************************************************
 * 重要说明
 ******************************************************************************/

/*
 * 1. 缓冲区内存段配置：
 *    - 所有DMA缓冲区放置在RAMGS0段（0x00C000 - 0x00FFFF）
 *    - RAMGS段是CPU1可访问的全局共享RAM
 *    - 2字节对齐确保DMA传输效率
 *
 * 2. volatile关键字使用：
 *    - DMA缓冲区**不应**使用volatile
 *    - 原因：DMA传输由硬件控制，编译器优化不会影响DMA行为
 *    - 使用volatile反而会降低CPU访问效率
 *    - 正确做法：在ISR中通过标志位通知CPU，CPU再访问缓冲区
 *
 * 3. Ping-Pong机制：
 *    - Ping缓冲：DMA正在写入时
 *    - Pong缓冲：CPU可以安全读取
 *    - 每次DMA完成中断后切换角色
 *    - 避免数据撕裂和竞态条件
 *
 * 4. 缓冲区大小计算：
 *    - ADS1278：8通道 × 24位/样本 = 24字节 = 12个16位字
 *    - AD5754：4通道 × 24位/命令 = 12字节 = 6个16位字
 *    - SPI字长=16位，24位数据需拆分为3次16位传输
 *
 * 5. Linker配置要求：
 *    在.cmd文件中确保：
 *    ramgs0: > RAMGS0, PAGE = 1  // 数据段映射
 *    确保RAMGS0已定义且足够大（至少96字节）
 */

#endif /* DRV_DMA_BUFFERS_H_ */

