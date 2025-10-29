/*******************************************************************************
 * 文件名：drv_dma_buffers.c
 * 描述：  DMA缓冲区定义 - 放置在RAMGS段，2字节对齐
 * 作者：  Auto-generated
 * 日期：  2025-10-20
 ******************************************************************************/

#include "F28x_Project.h"
#include "system_config.h"

/******************************************************************************
 * DMA缓冲区定义（RAMGS段，2字节对齐）
 ******************************************************************************/

// ADS1278 ADC接收缓冲区（Ping-Pong双缓冲）
// 8通道 × 3字节/样本 = 24字节，使用16位字长传输需要12个16位字
#pragma DATA_SECTION(g_adc_rx_ping, "ramgs0")
#pragma DATA_ALIGN(g_adc_rx_ping, 2)
Uint16 g_adc_rx_ping[DMA_ADC_BUFFER_SIZE / 2];  // 12个16位字

#pragma DATA_SECTION(g_adc_rx_pong, "ramgs0")
#pragma DATA_ALIGN(g_adc_rx_pong, 2)
Uint16 g_adc_rx_pong[DMA_ADC_BUFFER_SIZE / 2];  // 12个16位字

// SPIA TX虚拟数据缓冲区（用于生成时钟）
// ADC读取时，主机需要发送虚拟数据以生成时钟
#pragma DATA_SECTION(g_adc_tx_dummy, "ramgs0")
#pragma DATA_ALIGN(g_adc_tx_dummy, 2)
Uint16 g_adc_tx_dummy[DMA_ADC_BUFFER_SIZE / 2];  // 12个16位字

// AD5754 DAC发送缓冲区
// 4通道 × 3字节/命令 = 12字节，使用16位字长传输需要6个16位字
// 注：AD5754命令为24位(8控制+16数据)，需拆分为2或3个16位传输
#pragma DATA_SECTION(g_dac_tx_buffer, "ramgs0")
#pragma DATA_ALIGN(g_dac_tx_buffer, 2)
Uint16 g_dac_tx_buffer[DMA_DAC_BUFFER_SIZE / 2];  // 6个16位字

// AD5754 DAC接收缓冲区（读回验证）
#pragma DATA_SECTION(g_dac_rx_buffer, "ramgs0")
#pragma DATA_ALIGN(g_dac_rx_buffer, 2)
Uint16 g_dac_rx_buffer[DMA_DAC_BUFFER_SIZE / 2];  // 6个16位字

/******************************************************************************
 * DMA缓冲区初始化函数
 ******************************************************************************/

/**
 * @brief  DMA缓冲区初始化（清零并填充虚拟数据）
 */
void Drv_DMA_BuffersInit(void)
{
    Uint16 i;
    
    // 清零所有接收缓冲区
    for(i = 0; i < (DMA_ADC_BUFFER_SIZE / 2); i++) {
        g_adc_rx_ping[i] = 0;
        g_adc_rx_pong[i] = 0;
    }
    
    for(i = 0; i < (DMA_DAC_BUFFER_SIZE / 2); i++) {
        g_dac_rx_buffer[i] = 0;
        g_dac_tx_buffer[i] = 0;
    }
    
    // 填充ADC TX虚拟数据（用于生成SPI时钟）
    // 数据内容无关紧要，ADC只关心时钟
    for(i = 0; i < (DMA_ADC_BUFFER_SIZE / 2); i++) {
        g_adc_tx_dummy[i] = 0x0000;  // 发送全0即可
    }
}

/******************************************************************************
 * 缓冲区状态查询函数（调试用）
 ******************************************************************************/

/**
 * @brief  获取ADC Ping缓冲区地址
 * @return Ping缓冲区首地址
 */
Uint16* Drv_DMA_GetAdcPingBuffer(void)
{
    return g_adc_rx_ping;
}

/**
 * @brief  获取ADC Pong缓冲区地址
 * @return Pong缓冲区首地址
 */
Uint16* Drv_DMA_GetAdcPongBuffer(void)
{
    return g_adc_rx_pong;
}

/**
 * @brief  获取ADC TX虚拟数据缓冲区地址
 * @return TX虚拟数据缓冲区首地址
 */
Uint16* Drv_DMA_GetAdcTxDummyBuffer(void)
{
    return g_adc_tx_dummy;
}

/**
 * @brief  获取DAC TX缓冲区地址
 * @return DAC TX缓冲区首地址
 */
Uint16* Drv_DMA_GetDacTxBuffer(void)
{
    return g_dac_tx_buffer;
}

/**
 * @brief  获取DAC RX缓冲区地址
 * @return DAC RX缓冲区首地址
 */
Uint16* Drv_DMA_GetDacRxBuffer(void)
{
    return g_dac_rx_buffer;
}

