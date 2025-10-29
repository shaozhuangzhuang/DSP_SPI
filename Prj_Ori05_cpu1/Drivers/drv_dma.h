/*******************************************************************************
 * 文件名：drv_dma.h
 * 描述：  DMA驱动头文件 - 支持SPI DMA传输
 * 作者：  Auto-generated
 * 日期：  2025-10-20
 ******************************************************************************/

#ifndef DRV_DMA_H_
#define DRV_DMA_H_

#include "F28x_Project.h"
#include <stdbool.h>

/******************************************************************************
 * 枚举定义
 ******************************************************************************/

/**
 * @brief  DMA通道选择
 * @note   F28377D共有6个DMA通道（CH1-CH6）
 */
typedef enum {
    DMA_CH1 = 0,    // SPIA RX - ADC接收（最高优先级）
    DMA_CH2 = 1,    // SPIA TX - ADC时钟生成
    DMA_CH3 = 2,    // 保留
    DMA_CH4 = 3,    // 保留
    DMA_CH5 = 4,    // SPIB TX - DAC发送
    DMA_CH6 = 5     // SPIB RX - DAC读回
} DMA_Channel_t;

/**
 * @brief  DMA回调函数类型
 */
typedef void (*DMA_Callback_t)(void);

/**
 * @brief  DMA错误类型
 */
typedef enum {
    DMA_ERROR_NONE = 0,
    DMA_ERROR_OVERFLOW,
    DMA_ERROR_TIMEOUT,
    DMA_ERROR_INVALID_CHANNEL
} DMA_Error_t;

/******************************************************************************
 * 结构体定义
 ******************************************************************************/

/**
 * @brief  DMA统计信息
 */
typedef struct {
    volatile Uint32 complete_count;    // 完成次数
    volatile Uint32 overflow_count;    // 溢出次数
    volatile Uint32 timeout_count;     // 超时次数
    volatile Uint32 error_count;       // 错误次数
} DMA_Stats_t;

/******************************************************************************
 * 外部变量声明
 ******************************************************************************/

// DMA完成标志（ISR设置，应用层检查）
extern volatile bool g_dma_ch1_done;  // SPIA RX
extern volatile bool g_dma_ch2_done;  // SPIA TX
extern volatile bool g_dma_ch5_done;  // SPIB TX
extern volatile bool g_dma_ch6_done;  // SPIB RX

// DMA统计信息
extern DMA_Stats_t g_dma_stats[6];

/******************************************************************************
 * 函数声明
 ******************************************************************************/

/**
 * @brief  DMA模块初始化
 * @param  无
 * @return 无
 * @note   初始化DMA控制器，配置优先级模式
 */
void Drv_DMA_Init(void);

/**
 * @brief  配置SPIA RX DMA通道（CH1）
 * @param  无
 * @return 无
 * @note   源：SPIRXBUF，目标：adc_rx_ping，BURST=11
 */
void Drv_DMA_ConfigChannel_SPIA_RX(void);

/**
 * @brief  配置SPIA TX DMA通道（CH2）
 * @param  无
 * @return 无
 * @note   源：dummy_tx，目标：SPITXBUF，BURST=11
 */
void Drv_DMA_ConfigChannel_SPIA_TX(void);

/**
 * @brief  配置SPIB TX DMA通道（CH5）
 * @param  无
 * @return 无
 * @note   源：dac_tx_buffer，目标：SPITXBUF，BURST=5
 */
void Drv_DMA_ConfigChannel_SPIB_TX(void);

/**
 * @brief  配置SPIB RX DMA通道（CH6）
 * @param  无
 * @return 无
 * @note   源：SPIRXBUF，目标：dac_rx_buffer，BURST=5
 */
void Drv_DMA_ConfigChannel_SPIB_RX(void);

/**
 * @brief  重新配置DMA通道地址
 * @param  channel - DMA通道
 * @param  src - 源地址
 * @param  dst - 目标地址
 * @param  burst_size - BURST大小（传输字数-1）
 * @return DMA_Error_t - 错误码
 * @note   ONESHOT模式下，每次传输前需重新配置地址
 * @deprecated v2.5：建议使用Drv_DMA_UpdateDestAddr()仅更新目标地址
 */
DMA_Error_t Drv_DMA_ReConfigAddr(DMA_Channel_t channel, void *src, void *dst, Uint16 burst_size);

/**
 * @brief  更新DMA通道目标地址（用于Ping-Pong切换）
 * @param  channel - DMA通道
 * @param  dst - 新的目标地址
 * @return DMA_Error_t - 错误码
 * @note   v2.5新增：仅更新目标地址，不重新配置整个通道（效率更高）
 */
DMA_Error_t Drv_DMA_UpdateDestAddr(DMA_Channel_t channel, void *dst);

/**
 * @brief  启动DMA通道对（RX+TX）
 * @param  rx_ch - RX通道
 * @param  tx_ch - TX通道
 * @return 无
 * @note   按RX→延迟→TX顺序启动，避免首字丢失
 */
void Drv_DMA_StartPair(DMA_Channel_t rx_ch, DMA_Channel_t tx_ch);

/**
 * @brief  启动单个DMA通道
 * @param  channel - DMA通道
 * @return 无
 */
void Drv_DMA_Start(DMA_Channel_t channel);

/**
 * @brief  停止DMA通道
 * @param  channel - DMA通道
 * @return 无
 */
void Drv_DMA_Stop(DMA_Channel_t channel);

/**
 * @brief  注册DMA完成回调函数
 * @param  channel - DMA通道
 * @param  callback - 回调函数指针
 * @return 无
 */
void Drv_DMA_RegisterCallback(DMA_Channel_t channel, DMA_Callback_t callback);

/**
 * @brief  清除DMA统计信息
 * @param  channel - DMA通道（255=清除所有）
 * @return 无
 */
void Drv_DMA_ClearStats(Uint16 channel);

/**
 * @brief  获取DMA统计信息
 * @param  channel - DMA通道
 * @param  stats - 输出统计信息指针
 * @return 无
 */
void Drv_DMA_GetStats(DMA_Channel_t channel, DMA_Stats_t *stats);

/**
 * @brief  检查DMA通道是否完成
 * @param  channel - DMA通道
 * @return bool - true=已完成，false=未完成
 */
bool Drv_DMA_IsComplete(DMA_Channel_t channel);

/**
 * @brief  等待DMA完成（带超时）
 * @param  channel - DMA通道
 * @param  timeout_us - 超时时间（微秒）
 * @return bool - true=成功完成，false=超时
 */
bool Drv_DMA_WaitComplete(DMA_Channel_t channel, Uint32 timeout_us);

/******************************************************************************
 * DMA中断服务函数声明（在interrupt.c中实现）
 ******************************************************************************/
__interrupt void DMA_CH1_ISR(void);  // SPIA RX完成中断
__interrupt void DMA_CH2_ISR(void);  // SPIA TX完成中断
__interrupt void DMA_CH5_ISR(void);  // SPIB TX完成中断
__interrupt void DMA_CH6_ISR(void);  // SPIB RX完成中断

/******************************************************************************
 * 配置宏定义
 ******************************************************************************/

// DMA通道BURST配置
#define DMA_BURST_SPIA_RX       11      // SPIA RX: 12字（0-11）
#define DMA_BURST_SPIA_TX       11      // SPIA TX: 12字（0-11）
#define DMA_BURST_SPIB_TX       5       // SPIB TX: 6字（0-5）
#define DMA_BURST_SPIB_RX       5       // SPIB RX: 6字（0-5）

// DMA触发源宏（使用TI库定义，参考F2837xD_Dma_defines.h）
// #define DMA_SPIATX      109  // 已在TI库中定义
// #define DMA_SPIARX      110  // 已在TI库中定义
// #define DMA_SPIBTX      111  // 已在TI库中定义
// #define DMA_SPIBRX      112  // 已在TI库中定义
// 注：直接使用TI库宏，无需重复定义

// DMA超时配置
#define DMA_TIMEOUT_MS          10      // 默认超时时间（毫秒）

/******************************************************************************
 * 使用说明
 ******************************************************************************/

/*
 * 1. 初始化流程：
 *    a. 调用Drv_DMA_Init()初始化DMA模块
 *    b. 调用Drv_DMA_ConfigChannel_XXX()配置各通道
 *    c. 调用Drv_DMA_RegisterCallback()注册回调（可选）
 *    d. 在interrupt.c中配置PIE中断向量
 *
 * 2. 传输流程（以SPIA为例）：
 *    a. Drv_DMA_ReConfigAddr(CH1, &SPIRXBUF, adc_rx_ping, 11)
 *    b. Drv_DMA_ReConfigAddr(CH2, dummy_tx, &SPITXBUF, 11)
 *    c. Drv_DMA_StartPair(CH1, CH2)  // 按RX→TX顺序启动
 *    d. 等待g_dma_ch1_done和g_dma_ch2_done标志
 *    e. 在ISR回调中处理数据
 *
 * 3. ONESHOT模式说明：
 *    - 每次传输完成后自动停止
 *    - 下次传输前必须重新配置地址
 *    - 适合Ping-Pong缓冲机制
 *
 * 4. 优先级配置：
 *    - CH1（SPIA RX）：最高优先级
 *    - CH2（SPIA TX）：高优先级
 *    - CH5（SPIB TX）：中等优先级
 *    - CH6（SPIB RX）：低优先级
 *
 * 5. 中断配置（PIE Group 7）：
 *    - CH1 → PIE7.1 (INT7.1)
 *    - CH2 → PIE7.2 (INT7.2)
 *    - CH5 → PIE7.5 (INT7.5)
 *    - CH6 → PIE7.6 (INT7.6)
 *
 * 6. 注意事项：
 *    - RX DMA必须先于TX DMA启动
 *    - 完成标志由ISR设置，应用层清除
 *    - 使用DMA时禁用SPI中断
 *    - 所有地址必须2字节对齐
 */

#endif /* DRV_DMA_H_ */

