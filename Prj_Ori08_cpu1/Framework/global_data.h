/*******************************************************************************
 * 文件名：global_data.h
 * 描述：  全局数据结构定义 - ADS1278和AD5754芯片数据
 * 作者：  Auto-generated
 * 日期：  2025-10-30
 ******************************************************************************/

#ifndef GLOBAL_DATA_H_
#define GLOBAL_DATA_H_

#include "F28x_Project.h"
#include <stdbool.h>

/******************************************************************************
 * ADS1278 ADC数据结构（通道1-7）
 ******************************************************************************/

/**
 * @brief  ADS1278采样数据结构（7通道）
 * @note   仅使用ADS1278的通道1-7，通道8保留未用
 */
typedef struct {
    float   voltage[7];     // 通道1-7电压值（V），范围：±2.5V（典型）
    bool    valid;          // 数据有效标志：true=数据有效，false=数据无效/未更新
} ADS1278_Channels_t;

/******************************************************************************
 * AD5754 DAC输出数据结构（通道A-D，4通道）
 ******************************************************************************/

/**
 * @brief  AD5754输出数据结构（4通道）
 * @note   存储各通道的目标输出电压值
 */
typedef struct {
    float   voltage[4];     // 通道A-D电压值（V），范围根据配置（默认±10V）
} AD5754_Channels_t;

/******************************************************************************
 * 全局数据结构（顶层）
 ******************************************************************************/

/**
 * @brief  系统全局数据结构
 * @note   包含ADS1278的ADC采样数据和AD5754的DAC输出数据
 */
typedef struct {
    ADS1278_Channels_t  adc;    // ADS1278 ADC数据（7通道输入）
    AD5754_Channels_t   dac;    // AD5754 DAC数据（4通道输出）
} GlobalData_t;

/******************************************************************************
 * 全局变量声明
 ******************************************************************************/

// 全局数据实例（定义在global_data.c中）
extern volatile GlobalData_t g_SystemData;

/******************************************************************************
 * 函数声明
 ******************************************************************************/

/**
 * @brief  初始化全局数据结构
 * @param  无
 * @return 无
 * @note   将所有字段清零，ADC.valid设为false
 */
void GlobalData_Init(void);

/**
 * @brief  更新ADC数据到全局结构
 * @param  raw_data - 原始ADC数据数组（int32[8]，取前7个通道）
 * @param  vref - ADC参考电压（V），典型值2.5V
 * @return 无
 * @note   将原始ADC码值转换为电压值并更新到全局结构
 */
void GlobalData_UpdateADC(int32 raw_data[8], float vref);

/**
 * @brief  设置DAC通道电压
 * @param  channel - 通道号（0-3对应A-D）
 * @param  voltage - 目标电压值（V）
 * @return bool - true=设置成功，false=参数错误
 * @note   仅更新全局结构，不直接操作硬件
 */
bool GlobalData_SetDACVoltage(Uint16 channel, float voltage);

/**
 * @brief  获取ADC通道电压
 * @param  channel - 通道号（0-6对应通道1-7）
 * @param  voltage - 输出电压值的指针
 * @return bool - true=数据有效，false=数据无效
 * @note   仅当ADC.valid=true时返回true
 */
bool GlobalData_GetADCVoltage(Uint16 channel, float *voltage);

/**
 * @brief  获取DAC通道电压
 * @param  channel - 通道号（0-3对应A-D）
 * @param  voltage - 输出电压值的指针
 * @return bool - true=获取成功，false=参数错误
 */
bool GlobalData_GetDACVoltage(Uint16 channel, float *voltage);

/**
 * @brief  设置ADC数据有效标志
 * @param  valid - true=有效，false=无效
 * @return 无
 */
void GlobalData_SetADCValid(bool valid);

/**
 * @brief  检查ADC数据是否有效
 * @return bool - true=有效，false=无效
 */
bool GlobalData_IsADCValid(void);

/******************************************************************************
 * 使用说明
 ******************************************************************************/

/*
 * 1. 初始化流程：
 *    在System_Init()中调用：
 *    GlobalData_Init();
 *
 * 2. ADC数据更新（在SPI_Task_50us中）：
 *    ADS1278_Data_t adc_data;
 *    if(Drv_ADS1278_GetData(&adc_data)) {
 *        GlobalData_UpdateADC(adc_data.ch, 2.5f);  // 使用2.5V参考电压
 *    }
 *
 * 3. 读取ADC电压（应用层）：
 *    float ch3_voltage;
 *    if(GlobalData_GetADCVoltage(2, &ch3_voltage)) {  // 通道3，索引2
 *        // 处理电压值
 *    }
 *
 * 4. 设置DAC电压（应用层）：
 *    GlobalData_SetDACVoltage(0, 5.0f);  // 通道A输出5.0V
 *
 * 5. 读取DAC电压（应用层）：
 *    float dac_voltage;
 *    if(GlobalData_GetDACVoltage(1, &dac_voltage)) {  // 通道B
 *        // 读取当前设定值
 *    }
 *
 * 6. 直接访问（高级用法）：
 *    // 读取ADC通道5的电压
 *    if(g_SystemData.adc.valid) {
 *        float voltage = g_SystemData.adc.voltage[4];  // 通道5，索引4
 *    }
 *    
 *    // 设置DAC通道C的电压
 *    g_SystemData.dac.voltage[2] = -3.5f;  // 通道C，索引2
 *
 * 7. 通道索引对照：
 *    ADC: voltage[0-6] 对应 ADS1278通道1-7
 *    DAC: voltage[0-3] 对应 AD5754通道A-D
 *
 * 8. 电压范围：
 *    ADC: ±2.5V（取决于ADS1278的VREF配置）
 *    DAC: 根据AD5754的range配置，默认±10V
 *
 * 9. 线程安全注意：
 *    g_SystemData使用volatile修饰，适用于中断与主循环间共享
 *    如需多核访问，应使用#pragma DATA_SECTION放入共享RAM
 */

#endif /* GLOBAL_DATA_H_ */
