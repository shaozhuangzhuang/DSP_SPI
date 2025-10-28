/**
 * @file    app_spi_demo.h
 * @brief   SPI+DMA应用演示：ADS1278 ADC采样 + AD5754 DAC输出
 * @details 包含ADC数据采集、滤波、DAC波形生成等应用层功能
 * @author  F28377D_Prj Team
 * @date    2025-01-20
 */

#ifndef APP_SPI_DEMO_H_
#define APP_SPI_DEMO_H_

#include "F28x_Project.h"
#include <stdbool.h>
#include <math.h>
#include "drv_ads1278.h"
#include "drv_ad5754.h"
#include "drv_dma.h"

//=============================================================================
// 应用层配置参数
//=============================================================================

#define ADC_FILTER_WINDOW_SIZE      8       ///< 移动平均滤波窗口大小
#define ADC_CHANNELS                8       ///< ADC通道数
#define DAC_CHANNELS                4       ///< DAC通道数

#define DAC_WAVEFORM_SINE           0       ///< 正弦波
#define DAC_WAVEFORM_SQUARE         1       ///< 方波
#define DAC_WAVEFORM_TRIANGLE       2       ///< 三角波
#define DAC_WAVEFORM_SAWTOOTH       3       ///< 锯齿波

#define DAC_OUTPUT_FREQ_HZ          10.0f   ///< DAC输出波形频率 (Hz)
#define DAC_UPDATE_PERIOD_MS        20      ///< DAC更新周期 (ms)

//=============================================================================
// 数据结构定义
//=============================================================================

/**
 * @brief ADC处理后的数据结构
 */
typedef struct {
    float32 ch[ADC_CHANNELS];       ///< 8通道滤波后的浮点数据 (定标到-1.0~+1.0)
    float32 ch_raw[ADC_CHANNELS];   ///< 8通道原始浮点数据 (未滤波)
    Uint32 timestamp;               ///< 数据时间戳
    bool valid;                     ///< 数据有效标志
} ADC_ProcessedData_t;

/**
 * @brief DAC波形生成配置
 */
typedef struct {
    Uint16 waveform_type[DAC_CHANNELS]; ///< 波形类型 (0=正弦, 1=方波, 2=三角波, 3=锯齿波)
    float32 frequency[DAC_CHANNELS];    ///< 频率 (Hz)
    float32 amplitude[DAC_CHANNELS];    ///< 幅度 (0.0~1.0)
    float32 offset[DAC_CHANNELS];       ///< 偏移 (-1.0~+1.0)
    float32 phase[DAC_CHANNELS];        ///< 相位 (0~2π)
} DAC_WaveformConfig_t;

/**
 * @brief 应用层统计信息
 */
typedef struct {
    volatile Uint32 adc_sample_count;   ///< ADC采样计数
    volatile Uint32 dac_output_count;   ///< DAC输出计数
    volatile Uint32 adc_overrun_count;  ///< ADC数据溢出计数
    volatile Uint32 dac_error_count;    ///< DAC错误计数
    volatile Uint32 task_exec_time_us;  ///< 任务执行时间 (us)
} App_SPI_Stats_t;

//=============================================================================
// 全局变量声明
//=============================================================================

extern ADC_ProcessedData_t g_adc_processed_data;   ///< 全局ADC处理后数据
extern DAC_WaveformConfig_t g_dac_waveform_config; ///< DAC波形配置
extern App_SPI_Stats_t g_app_spi_stats;            ///< 应用层统计信息

//=============================================================================
// API函数声明
//=============================================================================

/**
 * @brief  初始化SPI演示应用（完整初始化，包含硬件+参数）
 * @details 初始化ADC/DAC驱动、配置默认参数、启动采集
 * @deprecated v2.1：建议使用App_Init()中的分层初始化 + App_SPI_Demo_ConfigParams()
 */
void App_SPI_Demo_Init(void);

/**
 * @brief  配置SPI演示应用参数（仅配置，不初始化硬件）
 * @details 配置滤波器、波形参数、注册回调、设置DAC中点
 * @note    v2.1新增：用于分层架构，硬件初始化已在App_Init()中完成
 */
void App_SPI_Demo_ConfigParams(void);

/**
 * @brief  ADC数据采样任务
 * @details 从ADS1278读取数据、移动平均滤波、限幅定标、更新全局变量
 * @note    由DRDY中断或20ms定时器调用
 */
void App_Task_ADC_Sample(void);

/**
 * @brief  DAC波形输出任务
 * @details 生成波形数据、批量写入AD5754、可选读回验证
 * @note    由20ms定时器调用
 */
void App_Task_DAC_Output(void);

/**
 * @brief  诊断信息输出任务
 * @details 输出DMA统计、ADC/DAC计数、CPU占用率等
 * @note    可选调用，用于调试
 */
void App_Task_Diagnostics(void);

/**
 * @brief  配置DAC通道波形
 * @param  channel 通道号 (0-3)
 * @param  waveform 波形类型 (DAC_WAVEFORM_SINE/SQUARE/TRIANGLE/SAWTOOTH)
 * @param  frequency 频率 (Hz)
 * @param  amplitude 幅度 (0.0~1.0)
 * @param  offset 偏移 (-1.0~+1.0)
 */
void App_DAC_SetWaveform(Uint16 channel, Uint16 waveform, float32 frequency, 
                         float32 amplitude, float32 offset);

/**
 * @brief  获取最新的ADC处理数据
 * @param  out_data 输出数据指针
 * @return true: 成功, false: 无新数据
 */
bool App_ADC_GetProcessedData(ADC_ProcessedData_t *out_data);

/**
 * @brief  清零应用层统计信息
 */
void App_SPI_ClearStats(void);

/**
 * @brief  获取应用层统计信息
 * @param  out_stats 输出统计信息指针
 */
void App_SPI_GetStats(App_SPI_Stats_t *out_stats);

//=============================================================================
// 公共工具函数
//=============================================================================

/**
 * @brief  移动平均滤波
 * @param  buffer 数据缓冲区 (循环缓冲)
 * @param  buffer_size 缓冲区大小
 * @param  index 当前索引指针 (会被更新)
 * @param  new_value 新数据
 * @return 滤波后的值
 */
float32 MovingAverage(float32 *buffer, Uint16 buffer_size, Uint16 *index, float32 new_value);

/**
 * @brief  数据限幅
 * @param  value 输入值
 * @param  min 最小值
 * @param  max 最大值
 * @return 限幅后的值
 */
float32 Clamp(float32 value, float32 min, float32 max);

/**
 * @brief  数据定标 (线性映射)
 * @param  value 输入值
 * @param  in_min 输入最小值
 * @param  in_max 输入最大值
 * @param  out_min 输出最小值
 * @param  out_max 输出最大值
 * @return 定标后的值
 */
float32 Scale(float32 value, float32 in_min, float32 in_max, 
              float32 out_min, float32 out_max);

/**
 * @brief  整数定标 (用于ADC/DAC数据转换)
 * @param  value 输入值 (ADC/DAC原始值)
 * @param  bits 分辨率位数 (如16位DAC传入16)
 * @param  is_signed 是否有符号 (true/false)
 * @return 定标到 -1.0~+1.0 或 0.0~+1.0
 */
float32 ScaleInt(int32_t value, Uint16 bits, bool is_signed);

#endif /* APP_SPI_DEMO_H_ */

