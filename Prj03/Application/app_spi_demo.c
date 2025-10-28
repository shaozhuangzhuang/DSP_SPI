/**
 * @file    app_spi_demo.c
 * @brief   SPI+DMA应用演示：ADS1278 ADC采样 + AD5754 DAC输出
 * @details 包含ADC数据采集、滤波、DAC波形生成等应用层功能
 * @author  F28377D_Prj Team
 * @date    2025-01-20
 */

#include "app_spi_demo.h"
#include "system_config.h"
#include <string.h>

//=============================================================================
// 常量定义
//=============================================================================

// PI已在GlobalData.h中定义，这里使用条件编译避免重复定义警告
#ifndef PI
#define PI                          3.14159265358979f
#endif

#define TWO_PI                      (2.0f * PI)
#define ADC_FULLSCALE_24BIT         8388608.0f  // 2^23 (24位有符号)
#define DAC_FULLSCALE_16BIT         65535.0f    // 2^16 - 1

//=============================================================================
// 全局变量定义
//=============================================================================

ADC_ProcessedData_t g_adc_processed_data;
DAC_WaveformConfig_t g_dac_waveform_config;
App_SPI_Stats_t g_app_spi_stats;

//=============================================================================
// 静态变量 (内部使用)
//=============================================================================

// 移动平均滤波缓冲区 (每个通道独立)
static float32 g_adc_filter_buffers[ADC_CHANNELS][ADC_FILTER_WINDOW_SIZE];
static Uint16 g_adc_filter_index[ADC_CHANNELS] = {0};

// DAC波形生成的相位累加器
static float32 g_dac_phase_accumulator[DAC_CHANNELS] = {0.0f};

// ADC数据就绪标志 (由DRDY中断回调设置)
static volatile bool g_adc_data_ready = false;

//=============================================================================
// 私有函数声明
//=============================================================================

static void ADC_DataReady_Callback(void);
static float32 GenerateWaveform(Uint16 waveform_type, float32 phase);
static Uint16 FloatToDAC(float32 value, AD5754_Range_t range);
static void InitFilterBuffers(void);

//=============================================================================
// API函数实现
//=============================================================================

/**
 * @brief  初始化SPI演示应用（完整初始化，包含硬件+参数）
 * @note   仅在不使用App_Init()时调用（例如单独测试模块）
 * @deprecated v2.1：建议使用App_Init()中的分层初始化 + App_SPI_Demo_ConfigParams()
 */
void App_SPI_Demo_Init(void)
{
    Uint16 i;

    // 清零统计信息
    memset(&g_app_spi_stats, 0, sizeof(App_SPI_Stats_t));

    // 清零处理数据
    memset(&g_adc_processed_data, 0, sizeof(ADC_ProcessedData_t));

    // 初始化滤波缓冲区
    InitFilterBuffers();

    // 初始化ADS1278 (高分辨率模式, 24位SPI格式)
    Drv_ADS1278_InitDefault();

    // 注册ADC数据就绪回调
    Drv_ADS1278_RegisterCallback(ADC_DataReady_Callback);

    /* 
     * 注意：ADS1278的SYNC引脚已在硬件上固定连接（上拉至IOVDD），
     * 芯片在上电后自动进入连续采样模式，无需软件调用启动函数。
     * Drv_ADS1278_Start() 函数已删除。
     */

    // 初始化AD5754 (±10V双极性输出范围)
    // 使用默认配置函数（内部已正确配置为±10V范围）
    Drv_AD5754_InitDefault();

    // 配置默认DAC波形: 通道0-3为不同波形
    for (i = 0; i < DAC_CHANNELS; i++) {
        g_dac_waveform_config.waveform_type[i] = i; // 0=正弦, 1=方波, 2=三角波, 3=锯齿波
        g_dac_waveform_config.frequency[i] = DAC_OUTPUT_FREQ_HZ;
        g_dac_waveform_config.amplitude[i] = 0.8f; // 80%幅度（±8V摆幅）
        g_dac_waveform_config.offset[i] = 0.0f;    // 零点偏移（对应0V中心）
        g_dac_waveform_config.phase[i] = 0.0f;
        g_dac_phase_accumulator[i] = 0.0f;
    }

    // 初始化DAC输出到中点
    for (i = 0; i < DAC_CHANNELS; i++) {
        Drv_AD5754_WriteChannel(i, 0x8000); // 中点值
    }
}

/**
 * @brief  配置SPI演示应用参数（仅配置，不初始化硬件）
 * @note   v2.1新增：用于分层架构，硬件初始化已在App_Init()中完成
 *         调用时机：在App_Task_UserInit()中调用
 *         前提条件：Drv_ADS1278_InitDefault() 和 Drv_AD5754_InitDefault() 已调用
 */
void App_SPI_Demo_ConfigParams(void)
{
    Uint16 i;

    // 清零统计信息
    memset(&g_app_spi_stats, 0, sizeof(App_SPI_Stats_t));

    // 清零处理数据
    memset(&g_adc_processed_data, 0, sizeof(ADC_ProcessedData_t));

    // 初始化滤波缓冲区
    InitFilterBuffers();

    // 注册ADC数据就绪回调（应用层配置）
    Drv_ADS1278_RegisterCallback(ADC_DataReady_Callback);

    // 配置默认DAC波形参数: 通道0-3为不同波形
    for (i = 0; i < DAC_CHANNELS; i++) {
        g_dac_waveform_config.waveform_type[i] = i; // 0=正弦, 1=方波, 2=三角波, 3=锯齿波
        g_dac_waveform_config.frequency[i] = DAC_OUTPUT_FREQ_HZ;
        g_dac_waveform_config.amplitude[i] = 0.8f; // 80%幅度（±8V摆幅）
        g_dac_waveform_config.offset[i] = 0.0f;    // 零点偏移（对应0V中心）
        g_dac_waveform_config.phase[i] = 0.0f;
        g_dac_phase_accumulator[i] = 0.0f;
    }

    // 设置DAC输出到中点（0V）
    for (i = 0; i < DAC_CHANNELS; i++) {
        Drv_AD5754_WriteChannel(i, 0x8000); // 中点值 = 0V（±10V范围）
    }
}

/**
 * @brief  ADC数据采样任务
 */
void App_Task_ADC_Sample(void)
{
    ADS1278_Data_t raw_data;
    Uint16 i;
    Uint32 start_time, end_time;

    // 记录开始时间 (使用CPU Timer0计数)
    start_time = CpuTimer0Regs.TIM.all;

    // 检查是否有新数据
    if (Drv_ADS1278_GetData(&raw_data)) {
        // 处理8个通道的数据
        for (i = 0; i < ADC_CHANNELS; i++) {
            // 定标到 -1.0 ~ +1.0 浮点数
            float32 raw_float = ScaleInt(raw_data.ch[i], 24, true);
            g_adc_processed_data.ch_raw[i] = raw_float;

            // 移动平均滤波
            g_adc_processed_data.ch[i] = MovingAverage(
                g_adc_filter_buffers[i],
                ADC_FILTER_WINDOW_SIZE,
                &g_adc_filter_index[i],
                raw_float
            );

            // 限幅 (防止溢出)
            g_adc_processed_data.ch[i] = Clamp(g_adc_processed_data.ch[i], -1.0f, 1.0f);
        }

        // 更新时间戳和有效标志
        g_adc_processed_data.timestamp = raw_data.timestamp;
        g_adc_processed_data.valid = true;

        // 统计计数
        g_app_spi_stats.adc_sample_count++;

        // 清除数据就绪标志
        g_adc_data_ready = false;
    } else {
        // 无新数据，可能是溢出
        if (g_adc_data_ready) {
            g_app_spi_stats.adc_overrun_count++;
        }
    }

    // 记录结束时间
    end_time = CpuTimer0Regs.TIM.all;
    g_app_spi_stats.task_exec_time_us = (start_time - end_time) / (CPU_FREQ_MHZ); // TIM向下计数
}

/**
 * @brief  DAC波形输出任务
 */
void App_Task_DAC_Output(void)
{
    Uint16 i;
    float32 waveform_value;
    Uint16 dac_value;
    bool write_success = true;

    // 生成4个通道的波形数据
    for (i = 0; i < DAC_CHANNELS; i++) {
        // 生成波形 (返回-1.0~+1.0)
        waveform_value = GenerateWaveform(
            g_dac_waveform_config.waveform_type[i],
            g_dac_phase_accumulator[i]
        );

        // 应用幅度和偏移
        waveform_value = waveform_value * g_dac_waveform_config.amplitude[i] + 
                         g_dac_waveform_config.offset[i];

        // 限幅到-1.0~+1.0 (对应±10V输出范围)
        waveform_value = Clamp(waveform_value, -1.0f, 1.0f);

        // 转换为DAC码值 (0x0000~0xFFFF)，使用±10V双极性范围
        dac_value = FloatToDAC(waveform_value, AD5754_RANGE_NEG_10_10V);

        // 写入DAC通道
        if (!Drv_AD5754_WriteChannel(i, dac_value)) {
            write_success = false;
            g_app_spi_stats.dac_error_count++;
        }

        // 更新相位累加器
        float32 phase_increment = TWO_PI * g_dac_waveform_config.frequency[i] * 
                                  (DAC_UPDATE_PERIOD_MS / 1000.0f);
        g_dac_phase_accumulator[i] += phase_increment;

        // 相位归一化到 0~2π
        if (g_dac_phase_accumulator[i] >= TWO_PI) {
            g_dac_phase_accumulator[i] -= TWO_PI;
        }
    }

    // 统计计数
    if (write_success) {
        g_app_spi_stats.dac_output_count++;
    }
}

/**
 * @brief  诊断信息输出任务
 */
void App_Task_Diagnostics(void)
{
    DMA_Stats_t dma_stats_ch1, dma_stats_ch2, dma_stats_ch5, dma_stats_ch6;

    // 获取DMA统计信息
    Drv_DMA_GetStats(DMA_CH1, &dma_stats_ch1);
    Drv_DMA_GetStats(DMA_CH2, &dma_stats_ch2);
    Drv_DMA_GetStats(DMA_CH5, &dma_stats_ch5);
    Drv_DMA_GetStats(DMA_CH6, &dma_stats_ch6);

    // 这里可以添加UART/SCI输出日志的代码
    // 例如: printf("ADC Samples: %lu, DAC Outputs: %lu\n", 
    //               g_app_spi_stats.adc_sample_count, g_app_spi_stats.dac_output_count);
    // 
    // printf("DMA CH1 (SPIA RX): Complete=%lu, Overflow=%lu, Timeout=%lu\n",
    //        dma_stats_ch1.complete_count, dma_stats_ch1.overflow_count, 
    //        dma_stats_ch1.timeout_count);

    // 目前只做空实现，用户可根据需要添加日志输出
    (void)dma_stats_ch1; // 避免未使用警告
    (void)dma_stats_ch2;
    (void)dma_stats_ch5;
    (void)dma_stats_ch6;
}

/**
 * @brief  配置DAC通道波形
 */
void App_DAC_SetWaveform(Uint16 channel, Uint16 waveform, float32 frequency, 
                         float32 amplitude, float32 offset)
{
    if (channel < DAC_CHANNELS) {
        g_dac_waveform_config.waveform_type[channel] = waveform;
        g_dac_waveform_config.frequency[channel] = frequency;
        g_dac_waveform_config.amplitude[channel] = Clamp(amplitude, 0.0f, 1.0f);
        g_dac_waveform_config.offset[channel] = Clamp(offset, -1.0f, 1.0f);
        g_dac_phase_accumulator[channel] = 0.0f; // 重置相位
    }
}

/**
 * @brief  获取最新的ADC处理数据
 */
bool App_ADC_GetProcessedData(ADC_ProcessedData_t *out_data)
{
    if (out_data == NULL) {
        return false;
    }

    if (g_adc_processed_data.valid) {
        *out_data = g_adc_processed_data;
        g_adc_processed_data.valid = false; // 读取后清除标志
        return true;
    }

    return false;
}

/**
 * @brief  清零应用层统计信息
 */
void App_SPI_ClearStats(void)
{
    g_app_spi_stats.adc_sample_count = 0;
    g_app_spi_stats.dac_output_count = 0;
    g_app_spi_stats.adc_overrun_count = 0;
    g_app_spi_stats.dac_error_count = 0;
    g_app_spi_stats.task_exec_time_us = 0;
}

/**
 * @brief  获取应用层统计信息
 */
void App_SPI_GetStats(App_SPI_Stats_t *out_stats)
{
    if (out_stats != NULL) {
        *out_stats = g_app_spi_stats;
    }
}

//=============================================================================
// 公共工具函数实现
//=============================================================================

/**
 * @brief  移动平均滤波
 */
float32 MovingAverage(float32 *buffer, Uint16 buffer_size, Uint16 *index, float32 new_value)
{
    Uint16 i;
    float32 sum = 0.0f;

    // 将新值存入循环缓冲区
    buffer[*index] = new_value;
    *index = (*index + 1) % buffer_size;

    // 计算平均值
    for (i = 0; i < buffer_size; i++) {
        sum += buffer[i];
    }

    return sum / (float32)buffer_size;
}

/**
 * @brief  数据限幅
 */
float32 Clamp(float32 value, float32 min, float32 max)
{
    if (value < min) return min;
    if (value > max) return max;
    return value;
}

/**
 * @brief  数据定标 (线性映射)
 */
float32 Scale(float32 value, float32 in_min, float32 in_max, 
              float32 out_min, float32 out_max)
{
    return (value - in_min) / (in_max - in_min) * (out_max - out_min) + out_min;
}

/**
 * @brief  整数定标 (用于ADC/DAC数据转换)
 */
float32 ScaleInt(int32_t value, Uint16 bits, bool is_signed)
{
    if (is_signed) {
        // 有符号数: -2^(bits-1) ~ +2^(bits-1)-1 -> -1.0 ~ +1.0
        int32_t max_val = (1L << (bits - 1)) - 1; // 2^(bits-1) - 1
        return (float32)value / (float32)max_val;
    } else {
        // 无符号数: 0 ~ 2^bits-1 -> 0.0 ~ +1.0
        Uint32 max_val = (1UL << bits) - 1; // 2^bits - 1
        return (float32)value / (float32)max_val;
    }
}

//=============================================================================
// 私有函数实现
//=============================================================================

/**
 * @brief  ADC数据就绪回调 (由DMA CH1 ISR调用)
 */
static void ADC_DataReady_Callback(void)
{
    g_adc_data_ready = true;
}

/**
 * @brief  生成波形 (返回-1.0~+1.0)
 * @param  waveform_type 波形类型
 * @param  phase 相位 (0~2π)
 * @return 波形值 (-1.0~+1.0)
 */
static float32 GenerateWaveform(Uint16 waveform_type, float32 phase)
{
    float32 normalized_phase; // 归一化相位 (0~1)

    // 确保相位在 0~2π 范围内
    while (phase < 0.0f) phase += TWO_PI;
    while (phase >= TWO_PI) phase -= TWO_PI;

    normalized_phase = phase / TWO_PI;

    switch (waveform_type) {
        case DAC_WAVEFORM_SINE:
            // 正弦波: sin(phase)
            return sinf(phase);

        case DAC_WAVEFORM_SQUARE:
            // 方波: phase < π ? +1 : -1
            return (phase < PI) ? 1.0f : -1.0f;

        case DAC_WAVEFORM_TRIANGLE:
            // 三角波: 0~π递增, π~2π递减
            if (normalized_phase < 0.5f) {
                return (normalized_phase * 4.0f) - 1.0f; // 0~0.5 -> -1~+1
            } else {
                return 3.0f - (normalized_phase * 4.0f); // 0.5~1 -> +1~-1
            }

        case DAC_WAVEFORM_SAWTOOTH:
            // 锯齿波: 线性递增 0~2π -> -1~+1
            return (normalized_phase * 2.0f) - 1.0f;

        default:
            return 0.0f;
    }
}

/**
 * @brief  浮点数转DAC码值
 * @param  value 浮点值 (0.0~1.0)
 * @param  range 输出范围
 * @return DAC码值 (0x0000~0xFFFF)
 */
static Uint16 FloatToDAC(float32 value, AD5754_Range_t range)
{
    float32 clamped_value;
    Uint16 dac_code;

    // 根据输出范围调整映射
    switch (range) {
        case AD5754_RANGE_0_5V:
        case AD5754_RANGE_0_10V:
        case AD5754_RANGE_0_10V_8:
            // 单极性: 0.0~1.0 -> 0x0000~0xFFFF
            clamped_value = Clamp(value, 0.0f, 1.0f);
            dac_code = (Uint16)(clamped_value * DAC_FULLSCALE_16BIT);
            break;

        case AD5754_RANGE_NEG_5_5V:
        case AD5754_RANGE_NEG_10_10V:
        case AD5754_RANGE_NEG_10_8_10_8V:
            // 双极性: -1.0~+1.0 -> 0x0000~0xFFFF (0x8000为中点)
            clamped_value = Clamp(value, -1.0f, 1.0f);
            dac_code = (Uint16)((clamped_value + 1.0f) / 2.0f * DAC_FULLSCALE_16BIT);
            break;

        default:
            dac_code = 0x8000; // 默认中点
            break;
    }

    return dac_code;
}

/**
 * @brief  初始化滤波缓冲区
 */
static void InitFilterBuffers(void)
{
    Uint16 i, j;
    for (i = 0; i < ADC_CHANNELS; i++) {
        for (j = 0; j < ADC_FILTER_WINDOW_SIZE; j++) {
            g_adc_filter_buffers[i][j] = 0.0f;
        }
        g_adc_filter_index[i] = 0;
    }
}

