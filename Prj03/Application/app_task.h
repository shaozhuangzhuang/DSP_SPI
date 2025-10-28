/*******************************************************************************
 * 文件名：app_task.h
 * 描述：  应用任务头文件 - 迁移自main.c的User函数
 * 作者：  Auto-generated
 * 日期：  2025-10-20
 ******************************************************************************/

#ifndef APP_TASK_H_
#define APP_TASK_H_

/******************************************************************************
 * 函数声明
 ******************************************************************************/

/**
 * @brief  用户初始化（迁移自User_Init）
 * @param  无
 * @return 无
 */
void App_Task_UserInit(void);

/**
 * @brief  用户主循环任务（迁移自User_MainLoop）
 * @param  无
 * @return 无
 * @note   在主循环中持续调用
 */
void App_Task_UserMainLoop(void);

/**
 * @brief  数据处理任务（迁移自User_DataProcess）
 * @param  无
 * @return 无
 * @note   每20ms调用一次
 */
void App_Task_DataProcess(void);

/**
 * @brief  LED1闪烁任务
 * @param  无
 * @return 无
 * @note   每1秒调用一次
 */
void App_Task_LED1Blink(void);

/**
 * @brief  LED2闪烁任务
 * @param  无
 * @return 无
 * @note   每3秒调用一次
 */
void App_Task_LED2Blink(void);

/**
 * @brief  SPI演示参数配置（ADS1278 + AD5754）
 * @param  无
 * @return 无
 * @note   在App_Task_UserInit中调用（仅配置应用参数，硬件初始化已在App_Init中完成）
 * @note   v2.1更新：函数改名 App_Task_SPI_Init → App_Task_SPI_ConfigParams
 */
void App_Task_SPI_ConfigParams(void);

/**
 * @brief  ADC采样任务（ADS1278）
 * @param  无
 * @return 无
 * @note   每20ms调用一次，或由DRDY中断驱动
 */
void App_Task_ADC_Sample(void);

/**
 * @brief  DAC输出任务（AD5754）
 * @param  无
 * @return 无
 * @note   每20ms调用一次
 */
void App_Task_DAC_Output(void);

/**
 * @brief  SPI诊断任务
 * @param  无
 * @return 无
 * @note   可选调用，用于输出DMA/SPI统计信息
 */
void App_Task_SPI_Diagnostics(void);

#endif /* APP_TASK_H_ */

