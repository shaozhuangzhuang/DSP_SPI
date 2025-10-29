/*******************************************************************************
 * 文件名：system_config.h
 * 描述：  系统配置头文件 - 系统级配置参数定义
 * 作者：  Auto-generated
 * 日期：  2025-10-20
 ******************************************************************************/

#ifndef SYSTEM_CONFIG_H_
#define SYSTEM_CONFIG_H_

/******************************************************************************
 * CPU配置
 ******************************************************************************/
#define CPU_FREQ_MHZ            200         // CPU频率：200MHz
#define SYSCLK_HZ               200000000UL // 系统时钟：200MHz (10MHz * IMULT_20 / PLLCLK_BY_2)
#define LSPCLK_HZ               50000000UL  // 低速外设时钟：50MHz (SYSCLK / 4, LSPCLKDIV=0)

/******************************************************************************
 * SPI配置
 ******************************************************************************/
#define SPIA_CLOCK_HZ           10000000UL  // SPIA时钟目标：10MHz
#define SPIB_CLOCK_HZ           10000000UL  // SPIB时钟目标：10MHz
#define SPIA_BRR                ((LSPCLK_HZ / SPIA_CLOCK_HZ) - 1)  // SPIBRR = 4
#define SPIB_BRR                ((LSPCLK_HZ / SPIB_CLOCK_HZ) - 1)  // SPIBRR = 4

// SPI FIFO触发电平配置
#define SPIA_TX_FIFO_LEVEL      0           // SPIA TX FIFO触发电平：0=FIFO空时触发DMA
#define SPIA_RX_FIFO_LEVEL      12          // SPIA RX FIFO触发电平：12=FIFO有12字时触发DMA
#define SPIB_TX_FIFO_LEVEL      0           // SPIB TX FIFO触发电平：0=FIFO空时触发DMA
#define SPIB_RX_FIFO_LEVEL      6           // SPIB RX FIFO触发电平：6=FIFO有6字时触发DMA

/******************************************************************************
 * DMA缓冲区配置
 ******************************************************************************/
#define DMA_ADC_BUFFER_SIZE     24          // ADS1278缓冲：24字节 (8通道 × 3字节/样本)
#define DMA_DAC_BUFFER_SIZE     12          // AD5754缓冲：12字节 (4通道 × 3字节/命令)
#define ADC_SAMPLE_RATE_HZ      52700       // ADC采样率：52.7kHz (High-Resolution模式)

/******************************************************************************
 * 硬件配置
 ******************************************************************************/
#define LED_COUNT               6           // LED数量：6个（GPIO0-5）

/******************************************************************************
 * 定时器配置
 ******************************************************************************/
#define TIMER0_PERIOD_US        100         // Timer0周期：100us
#define TIMER_COUNT_PER_MS      10          // 1ms = 10 * 100us

/******************************************************************************
 * 任务周期配置
 ******************************************************************************/
#define DATA_PROCESS_PERIOD_MS  20          // 数据处理周期：20ms
#define LED1_BLINK_PERIOD_S     1           // LED1闪烁周期：1秒（控制LED0）
#define LED2_BLINK_PERIOD_S     3           // LED2闪烁周期：3秒（控制LED1）

/******************************************************************************
 * 计数器阈值（基于Timer0的100us周期）
 ******************************************************************************/
#define DATA_PROCESS_COUNT      (DATA_PROCESS_PERIOD_MS * TIMER_COUNT_PER_MS)
#define LED1_BLINK_COUNT        (LED1_BLINK_PERIOD_S * 1000 * TIMER_COUNT_PER_MS)
#define LED2_BLINK_COUNT        (LED2_BLINK_PERIOD_S * 1000 * TIMER_COUNT_PER_MS)

/******************************************************************************
 * 系统状态定义
 ******************************************************************************/
#define SYSTEM_OK               0
#define SYSTEM_ERROR            1

#endif /* SYSTEM_CONFIG_H_ */

