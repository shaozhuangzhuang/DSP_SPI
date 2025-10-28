/*******************************************************************************
 * 文件名：app_spi_test.h
 * 描述：  SPI+DMA功能测试头文件
 * 作者：  Auto-generated
 * 日期：  2025-10-20
 ******************************************************************************/

#ifndef APP_SPI_TEST_H_
#define APP_SPI_TEST_H_

#include "F28x_Project.h"
#include <stdbool.h>

/******************************************************************************
 * 测试结果结构体定义
 ******************************************************************************/
typedef struct {
    Uint32 total_tests;           // 总测试次数
    Uint32 pass_count;            // 通过次数
    Uint32 fail_count;            // 失败次数
    Uint32 timeout_count;         // 超时次数
    Uint32 mismatch_count;        // 数据不匹配次数
    Uint32 overflow_count;        // FIFO溢出次数
    bool   last_test_result;      // 最后一次测试结果
} SPI_Test_Result_t;

/******************************************************************************
 * 函数声明
 ******************************************************************************/

/**
 * @brief  SPI+DMA初始化并进入回环测试模式
 * @return bool - true=初始化成功，false=初始化失败
 */
bool App_SPI_LoopbackTest_Init(void);

/**
 * @brief  单次SPI内部回环测试
 * @return bool - true=测试通过，false=测试失败
 */
bool App_SPI_LoopbackTest_Single(void);

/**
 * @brief  多次SPI内部回环测试
 * @param  count - 测试次数
 * @return bool - true=全部通过，false=有失败
 */
bool App_SPI_LoopbackTest_Multiple(Uint32 count);

/**
 * @brief  退出回环测试模式
 */
void App_SPI_LoopbackTest_Exit(void);

/**
 * @brief  完整的SPI回环测试流程（1000次测试）
 * @return bool - true=100%通过，false=有失败
 */
bool App_SPI_LoopbackTest_Full(void);

/**
 * @brief  快速自检（10次测试）
 * @return bool - true=全部通过，false=有失败
 */
bool App_SPI_QuickSelfTest(void);

/**
 * @brief  获取测试结果
 * @param  result - 输出测试结果指针
 */
void App_SPI_GetTestResult(SPI_Test_Result_t *result);

/**
 * @brief  打印测试结果（调试用）
 * @note   需要UART/SCI支持
 */
void App_SPI_PrintTestResult(void);

/**
 * @brief  检查DMA统计信息
 * @return bool - true=无异常，false=有异常
 */
bool App_SPI_CheckDMAStats(void);

/**
 * @brief  清除DMA统计信息
 */
void App_SPI_ClearDMAStats(void);

#endif /* APP_SPI_TEST_H_ */

