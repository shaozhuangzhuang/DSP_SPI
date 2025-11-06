#ifndef DRV_DMA_H_
#define DRV_DMA_H_

#include "F28x_Project.h"
#include <stdbool.h>

//=============================================================================
// 数据结构定义
//=============================================================================

// DMA状态机枚举
typedef enum {
    DMA_STATE_IDLE,     // 空闲状态
    DMA_STATE_BUSY,     // 忙碌状态
    DMA_STATE_ERROR     // 错误状态
} DMA_State_t;

// DMA错误码枚举
typedef enum {
    DMA_ERR_NONE,       // 无错误
    DMA_ERR_BUSY,       // DMA通道忙碌
    DMA_ERR_TIMEOUT,    // 操作超时
    DMA_ERR_CONFIG,     // 配置错误
    DMA_ERR_HARDWARE    // DMA硬件错误
} DMA_ErrorCode_t;

//=============================================================================
// 接口函数声明
//=============================================================================

// DMA配置与初始化
void Drv_DMA_Config_SPIB(void);  // DMA控制器初始化 + SPIB通道完整配置（一站式接口）
void start_dma (void);
// 缓冲区访问接口
Uint16* Drv_DMA_Get_TX_Buffer(void);  // 获取发送缓冲区指针
Uint16* Drv_DMA_Get_RX_Buffer(void);  // 获取接收缓冲区指针

// 状态查询接口
DMA_State_t Drv_DMA_Get_State(void);  // 获取DMA状态

//=============================================================================
// 中断服务函数原型
//=============================================================================

__interrupt void DMA_CH6_ISR(void);  // RX完成中断

__interrupt void DMA_CH5_ISR(void);
#endif /* DRV_DMA_H_ */

