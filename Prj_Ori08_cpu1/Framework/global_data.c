/*******************************************************************************
 * 文件名：global_data.c
 * 描述：  全局数据结构实现 - ADS1278和AD5754芯片数据
 * 作者：  Auto-generated
 * 日期：  2025-10-30
 ******************************************************************************/

#include "global_data.h"
#include <string.h>

/******************************************************************************
 * 全局变量定义
 ******************************************************************************/

// 全局数据实例（使用volatile确保中断安全）
volatile GlobalData_t g_SystemData;

/******************************************************************************
 * ADC转换常量
 ******************************************************************************/

// ADS1278: 24位ADC，有符号数，满量程值
#define ADC_FULL_SCALE      8388608.0f      // 2^23 = 8388608
#define ADC_DEFAULT_VREF    2.5f            // 默认参考电压2.5V

/******************************************************************************
 * 函数实现
 ******************************************************************************/

/**
 * @brief  初始化全局数据结构
 */
void GlobalData_Init(void)
{
    Uint16 i;
    
    // 清零ADC数据
    for(i = 0; i < 7; i++) {
        g_SystemData.adc.voltage[i] = 0.0f;
    }
    g_SystemData.adc.valid = false;
    
    // 清零DAC数据（安全状态：0V）
    for(i = 0; i < 4; i++) {
        g_SystemData.dac.voltage[i] = 0.0f;
    }
}

/**
 * @brief  更新ADC数据到全局结构
 * @param  raw_data - 原始ADC数据数组（int32[8]）
 * @param  vref - ADC参考电压（V）
 */
void GlobalData_UpdateADC(int32 raw_data[8], float vref)
{
    Uint16 i;
    
    // 转换前7个通道的原始数据为电压值
    // 公式: voltage = (raw_data / 2^23) * VREF
    for(i = 0; i < 7; i++) {
        g_SystemData.adc.voltage[i] = ((float)raw_data[i] / ADC_FULL_SCALE) * vref;
    }
    
    // 设置数据有效标志
    g_SystemData.adc.valid = true;
}

/**
 * @brief  设置DAC通道电压
 * @param  channel - 通道号（0-3对应A-D）
 * @param  voltage - 目标电压值（V）
 * @return bool - true=设置成功，false=参数错误
 */
bool GlobalData_SetDACVoltage(Uint16 channel, float voltage)
{
    // 参数检查
    if(channel >= 4) {
        return false;
    }
    
    // 更新电压值
    g_SystemData.dac.voltage[channel] = voltage;
    
    return true;
}

/**
 * @brief  获取ADC通道电压
 * @param  channel - 通道号（0-6对应通道1-7）
 * @param  voltage - 输出电压值的指针
 * @return bool - true=数据有效，false=数据无效
 */
bool GlobalData_GetADCVoltage(Uint16 channel, float *voltage)
{
    // 参数检查
    if(channel >= 7 || voltage == NULL) {
        return false;
    }
    
    // 检查数据有效性
    if(!g_SystemData.adc.valid) {
        return false;
    }
    
    // 读取电压值
    *voltage = g_SystemData.adc.voltage[channel];
    
    return true;
}

/**
 * @brief  获取DAC通道电压
 * @param  channel - 通道号（0-3对应A-D）
 * @param  voltage - 输出电压值的指针
 * @return bool - true=获取成功，false=参数错误
 */
bool GlobalData_GetDACVoltage(Uint16 channel, float *voltage)
{
    // 参数检查
    if(channel >= 4 || voltage == NULL) {
        return false;
    }
    
    // 读取电压值
    *voltage = g_SystemData.dac.voltage[channel];
    
    return true;
}

/**
 * @brief  设置ADC数据有效标志
 * @param  valid - true=有效，false=无效
 */
void GlobalData_SetADCValid(bool valid)
{
    g_SystemData.adc.valid = valid;
}

/**
 * @brief  检查ADC数据是否有效
 * @return bool - true=有效，false=无效
 */
bool GlobalData_IsADCValid(void)
{
    return g_SystemData.adc.valid;
}
