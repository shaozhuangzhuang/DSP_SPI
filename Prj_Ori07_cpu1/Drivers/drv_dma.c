/*******************************************************************************
 * 文件名：drv_dma.c
 * 描述：  DMA驱动实现 - 支持SPI DMA传输
 * 作者：  Auto-generated
 * 日期：  2025-10-20
 ******************************************************************************/

#include "drv_dma.h"
#include "drv_dma_buffers.h"
#include "F28x_Project.h"
#include "F2837xD_Dma_defines.h"

/******************************************************************************
 * 全局变量定义
 ******************************************************************************/

// DMA完成标志（ISR设置，应用层检查并清除）
volatile bool g_dma_ch1_done = false;  // SPIA RX完成
volatile bool g_dma_ch2_done = false;  // SPIA TX完成
volatile bool g_dma_ch5_done = false;  // SPIB TX完成
volatile bool g_dma_ch6_done = false;  // SPIB RX完成

// DMA统计信息
DMA_Stats_t g_dma_stats[6] = {0};

// DMA回调函数指针数组（静态）
static DMA_Callback_t g_dma_callbacks[6] = {NULL};

/******************************************************************************
 * 内部辅助函数
 ******************************************************************************/

/**
 * @brief  获取DMA通道寄存器指针
 * @param  channel - DMA通道
 * @return DMA通道寄存器指针
 */
static volatile struct CH_REGS* Drv_DMA_GetChannelRegs(DMA_Channel_t channel)
{
    switch(channel) {
        case DMA_CH1: return &DmaRegs.CH1;
        case DMA_CH2: return &DmaRegs.CH2;
        case DMA_CH5: return &DmaRegs.CH5;
        case DMA_CH6: return &DmaRegs.CH6;
        default: return NULL;
    }
}

/**
 * @brief  等待DMA通道停止
 * @param  channel - DMA通道
 * @param  timeout_us - 超时时间（微秒）
 * @return bool - true=成功停止，false=超时
 */
static bool Drv_DMA_WaitForStop(DMA_Channel_t channel, Uint32 timeout_us)
{
    volatile struct CH_REGS* ch_regs = Drv_DMA_GetChannelRegs(channel);
    Uint32 wait_count = 0;
    Uint32 max_count = timeout_us / 10;  // 假设每次循环约10us
    
    if(ch_regs == NULL) {
        return false;
    }
    
    // 等待RUNSTS=0
    while(ch_regs->CONTROL.bit.RUN && (wait_count < max_count)) {
        wait_count++;
        DELAY_US(1);
    }
    
    if(wait_count >= max_count) {
        g_dma_stats[channel].timeout_count++;
        return false;  // 超时
    }
    
    return true;
}

/******************************************************************************
 * DMA初始化函数
 ******************************************************************************/

/**
 * @brief  DMA模块初始化
 */
void Drv_DMA_Init(void)
{
    Uint16 i;
    
    EALLOW;
    
    // 调用TI官方库初始化DMA（包含硬复位）
    DMAInitialize();
    
    // 调试模式下DMA继续运行
    DmaRegs.DEBUGCTRL.bit.FREE = 1;
    
    // 配置PF2SEL（CPU1拥有Peripheral Frame 2）
    // 参考TRM Section: System Control
    // PF2SEL=1表示CPU1拥有PF2（包含DMA）
    CpuSysRegs.SECMSEL.bit.PF2SEL = 1;
    
    // 配置CH1高优先级模式
    // CH1PRIORITY=1: CH1具有高优先级，可以中断其他通道
    // CH1PRIORITY=0: CH1与其他通道具有相同优先级（轮询调度）
    DmaRegs.PRIORITYCTRL1.bit.CH1PRIORITY = 1;  // CH1: SPIA RX（高优先级模式）
    
    // 注意：CH2-CH6没有独立的优先级控制位，它们使用轮询调度
    // 在CH1高优先级模式下：
    // - CH1可以中断任何正在执行的CH2-CH6传输
    // - CH2-CH6之间按轮询顺序执行：CH2 -> CH3 -> CH4 -> CH5 -> CH6 -> CH2...
    
    // 复位优先级状态机到初始状态
    DmaRegs.DMACTRL.bit.PRIORITYRESET = 1;
    
    EDIS;
    
    // 清零统计信息
    for(i = 0; i < 6; i++) {
        g_dma_stats[i].complete_count = 0;
        g_dma_stats[i].overflow_count = 0;
        g_dma_stats[i].timeout_count = 0;
        g_dma_stats[i].error_count = 0;
        g_dma_callbacks[i] = NULL;
    }
    
    // 清零完成标志
    g_dma_ch1_done = false;
    g_dma_ch2_done = false;
    g_dma_ch5_done = false;
    g_dma_ch6_done = false;
    
    //========================================================================
    // v2.5新增：统一配置所有DMA通道
    //========================================================================
    Drv_DMA_ConfigChannel_SPIA_RX();  // 配置CH1（ADS1278 RX）
    Drv_DMA_ConfigChannel_SPIA_TX();  // 配置CH2（ADS1278 TX）
    Drv_DMA_ConfigChannel_SPIB_TX();  // 配置CH5（AD5754 TX）
    Drv_DMA_ConfigChannel_SPIB_RX();  // 配置CH6（AD5754 RX）
    
    // 注意：DMA回调注册将在芯片驱动初始化后通过Drv_DMA_RegisterCallback()完成
    // 这样设计的原因：
    // 1. 回调函数定义在芯片驱动中（访问芯片内部状态）
    // 2. 芯片驱动初始化时可以决定是否需要注册回调
    // 3. 保持接口灵活性
}

/******************************************************************************
 * DMA通道配置函数
 ******************************************************************************/

/**
 * @brief  配置SPIA RX DMA通道（CH1）
 * @note   源：SPIRXBUF，目标：adc_rx_ping，BURST=11（12字）
 */
void Drv_DMA_ConfigChannel_SPIA_RX(void)
{
    EALLOW;
    
    // 配置地址：源=SPIRXBUF，目标=adc_rx_ping
    DMACH1AddrConfig(&g_adc_rx_ping[0], (volatile Uint16 *)&SpiaRegs.SPIRXBUF);
    
    // 配置BURST：12字传输（BURST_SIZE=11）
    // SRCBURST=0（源地址不变，FIFO固定地址）
    // DSTBURST=1（目标地址递增1字）
    DMACH1BurstConfig(DMA_BURST_SPIA_RX, 0, 1);
    
    // 配置TRANSFER：1次BURST后完成（ONESHOT模式）
    // TRANSFER_SIZE=0表示1次BURST
    // SRCTRANSFER=0，DSTTRANSFER=1
    DMACH1TransferConfig(0, 0, 1);
    
    // 配置WRAP：不使用WRAP功能
    DMACH1WrapConfig(0, 0, 0, 0);
    
    // 配置MODE
    DMACH1ModeConfig(
        DMA_SPIARX,         // 触发源：SPIA RX FIFO (110)
        PERINT_ENABLE,      // 使能外设中断触发
        ONESHOT_ENABLE,     // ONESHOT模式（传输完成后自动停止）
        CONT_DISABLE,       // 禁用连续模式
        SYNC_DISABLE,       // 禁用同步
        SYNC_SRC,           // 同步源（SYNC_DISABLE时忽略）
        OVRFLOW_DISABLE,    // 禁用溢出中断
        SIXTEEN_BIT,        // 16位数据宽度
        CHINT_END,          // 传输结束时产生中断
        CHINT_ENABLE        // 使能DMA通道中断
    );
    
    EDIS;
}

/**
 * @brief  配置SPIA TX DMA通道（CH2）
 * @note   源：dummy_tx，目标：SPITXBUF，BURST=11（12字）
 */
void Drv_DMA_ConfigChannel_SPIA_TX(void)
{
    EALLOW;
    
    // 配置地址：源=dummy_tx，目标=SPITXBUF
    DMACH2AddrConfig((volatile Uint16 *)&SpiaRegs.SPITXBUF, &g_adc_tx_dummy[0]);
    
    // 配置BURST：12字传输（BURST_SIZE=11）
    // SRCBURST=1（源地址递增1字）
    // DSTBURST=0（目标地址不变，FIFO固定地址）
    DMACH2BurstConfig(DMA_BURST_SPIA_TX, 1, 0);
    
    // 配置TRANSFER：1次BURST后完成（ONESHOT模式）
    DMACH2TransferConfig(0, 1, 0);
    
    // 配置WRAP：不使用WRAP功能
    DMACH2WrapConfig(0, 0, 0, 0);
    
    // 配置MODE
    DMACH2ModeConfig(
        DMA_SPIATX,         // 触发源：SPIA TX FIFO (109)
        PERINT_ENABLE,
        ONESHOT_ENABLE,
        CONT_DISABLE,
        SYNC_DISABLE,
        SYNC_SRC,
        OVRFLOW_DISABLE,
        SIXTEEN_BIT,
        CHINT_END,
        CHINT_ENABLE
    );
    
    EDIS;
}

/**
 * @brief  配置SPIB TX DMA通道（CH5）
 * @note   源：dac_tx_buffer，目标：SPITXBUF，BURST=5（6字）
 */
void Drv_DMA_ConfigChannel_SPIB_TX(void)
{
    EALLOW;
    
    // 配置地址：源=dac_tx_buffer，目标=SPITXBUF
    DMACH5AddrConfig((volatile Uint16 *)&SpibRegs.SPITXBUF, &g_dac_tx_buffer[0]);
    
    // 配置BURST：6字传输（BURST_SIZE=5）
    // SRCBURST=1（源地址递增1字）
    // DSTBURST=0（目标地址不变，FIFO固定地址）
    DMACH5BurstConfig(DMA_BURST_SPIB_TX, 1, 0);
    
    // 配置TRANSFER：1次BURST后完成（ONESHOT模式）
    DMACH5TransferConfig(0, 1, 0);
    
    // 配置WRAP：不使用WRAP功能
    DMACH5WrapConfig(0, 0, 0, 0);
    
    // 配置MODE
    DMACH5ModeConfig(
        DMA_SPIBTX,         // 触发源：SPIB TX FIFO (111)
        PERINT_ENABLE,
        ONESHOT_ENABLE,
        CONT_DISABLE,
        SYNC_DISABLE,
        SYNC_SRC,
        OVRFLOW_DISABLE,
        SIXTEEN_BIT,
        CHINT_END,
        CHINT_ENABLE
    );
    
    EDIS;
}

/**
 * @brief  配置SPIB RX DMA通道（CH6）
 * @note   源：SPIRXBUF，目标：dac_rx_buffer，BURST=5（6字）
 */
void Drv_DMA_ConfigChannel_SPIB_RX(void)
{
    EALLOW;
    
    // 配置地址：源=SPIRXBUF，目标=dac_rx_buffer
    DMACH6AddrConfig(&g_dac_rx_buffer[0], (volatile Uint16 *)&SpibRegs.SPIRXBUF);
    
    // 配置BURST：6字传输（BURST_SIZE=5）
    // SRCBURST=0（源地址不变，FIFO固定地址）
    // DSTBURST=1（目标地址递增1字）
    DMACH6BurstConfig(DMA_BURST_SPIB_RX, 0, 1);
    
    // 配置TRANSFER：1次BURST后完成（ONESHOT模式）
    DMACH6TransferConfig(0, 0, 1);
    
    // 配置WRAP：不使用WRAP功能
    DMACH6WrapConfig(0, 0, 0, 0);
    
    // 配置MODE
    DMACH6ModeConfig(
        DMA_SPIBRX,         // 触发源：SPIB RX FIFO (112)
        PERINT_ENABLE,
        ONESHOT_ENABLE,
        CONT_DISABLE,
        SYNC_DISABLE,
        SYNC_SRC,
        OVRFLOW_DISABLE,
        SIXTEEN_BIT,
        CHINT_END,
        CHINT_ENABLE
    );
    
    EDIS;
}

/******************************************************************************
 * DMA控制函数
 ******************************************************************************/

/**
 * @brief  重新配置DMA通道地址（ONESHOT模式下每次传输前调用）
 * @param  channel - DMA通道
 * @param  src - 源地址
 * @param  dst - 目标地址
 * @param  burst_size - BURST大小（传输字数-1）
 * @return DMA_Error_t - 错误码
 */
DMA_Error_t Drv_DMA_ReConfigAddr(DMA_Channel_t channel, void *src, void *dst, Uint16 burst_size)
{
    Uint16 gie_state;
    
    // 参数检查
    if(src == NULL || dst == NULL) {
        return DMA_ERROR_INVALID_CHANNEL;
    }
    
    // 禁用全局中断（临界区保护）
    gie_state = __disable_interrupts();
    
    // 等待DMA通道停止（超时1ms）
    if(!Drv_DMA_WaitForStop(channel, 1000)) {
        __restore_interrupts(gie_state);
        return DMA_ERROR_TIMEOUT;
    }
    
    EALLOW;
    
    // 根据通道调用对应的配置函数
    switch(channel) {
        case DMA_CH1:
            DMACH1AddrConfig((volatile Uint16 *)dst, (volatile Uint16 *)src);
            if(burst_size != DMA_BURST_SPIA_RX) {
                DMACH1BurstConfig(burst_size, 0, 1);
            }
            break;
            
        case DMA_CH2:
            DMACH2AddrConfig((volatile Uint16 *)dst, (volatile Uint16 *)src);
            if(burst_size != DMA_BURST_SPIA_TX) {
                DMACH2BurstConfig(burst_size, 1, 0);
            }
            break;
            
        case DMA_CH5:
            DMACH5AddrConfig((volatile Uint16 *)dst, (volatile Uint16 *)src);
            if(burst_size != DMA_BURST_SPIB_TX) {
                DMACH5BurstConfig(burst_size, 1, 0);
            }
            break;
            
        case DMA_CH6:
            DMACH6AddrConfig((volatile Uint16 *)dst, (volatile Uint16 *)src);
            if(burst_size != DMA_BURST_SPIB_RX) {
                DMACH6BurstConfig(burst_size, 0, 1);
            }
            break;
            
        default:
            EDIS;
            __restore_interrupts(gie_state);
            return DMA_ERROR_INVALID_CHANNEL;
    }
    
    EDIS;
    
    // 恢复全局中断
    __restore_interrupts(gie_state);
    
    return DMA_ERROR_NONE;
}

/**
 * @brief  更新DMA通道目标地址（用于Ping-Pong切换）
 * @param  channel - DMA通道
 * @param  dst - 新的目标地址
 * @return DMA_Error_t - 错误码
 * @note   v2.5新增：仅更新目标地址，不重新配置整个通道（效率更高）
 *         用于Ping-Pong缓冲区切换，避免重复配置burst、mode等参数
 */
DMA_Error_t Drv_DMA_UpdateDestAddr(DMA_Channel_t channel, void *dst)
{
    volatile struct CH_REGS* ch_regs = Drv_DMA_GetChannelRegs(channel);
    Uint16 gie_state;
    
    // 参数检查
    if(dst == NULL || ch_regs == NULL) {
        return DMA_ERROR_INVALID_CHANNEL;
    }
    
    // 禁用全局中断（临界区保护）
    gie_state = __disable_interrupts();
    
    // 等待DMA通道停止（超时100us，Ping-Pong切换应该很快）
    if(!Drv_DMA_WaitForStop(channel, 100)) {
        __restore_interrupts(gie_state);
        return DMA_ERROR_TIMEOUT;
    }
    
    EALLOW;
    
    // 仅更新目标地址寄存器
    ch_regs->DST_ADDR_SHADOW = (Uint32)dst;
    
    EDIS;
    
    // 恢复全局中断
    __restore_interrupts(gie_state);
    
    return DMA_ERROR_NONE;
}

/**
 * @brief  启动单个DMA通道
 * @param  channel - DMA通道
 */
void Drv_DMA_Start(DMA_Channel_t channel)
{
    EALLOW;
    
    switch(channel) {
        case DMA_CH1:
            StartDMACH1();
            break;
        case DMA_CH2:
            StartDMACH2();
            break;
        case DMA_CH5:
            StartDMACH5();
            break;
        case DMA_CH6:
            StartDMACH6();
            break;
        default:
            break;
    }
    
    EDIS;
}

/**
 * @brief  启动DMA通道对（RX+TX），按正确顺序启动
 * @param  rx_ch - RX通道（先启动）
 * @param  tx_ch - TX通道（后启动）
 * @note   RX必须先于TX启动，避免首字丢失
 */
void Drv_DMA_StartPair(DMA_Channel_t rx_ch, DMA_Channel_t tx_ch)
{
    volatile struct CH_REGS* rx_regs = Drv_DMA_GetChannelRegs(rx_ch);
    Uint16 wait_count = 0;
    
    EALLOW;
    
    // 1. 启动RX通道
    Drv_DMA_Start(rx_ch);
    
    // 2. 等待RX通道RUN标志置位（或延时1us）
    if(rx_regs != NULL) {
        while(!rx_regs->CONTROL.bit.RUN && (wait_count < 100)) {
            wait_count++;
            __asm(" nop");
        }
    } else {
        DELAY_US(1);  // 保险起见，延时1us
    }
    
    // 3. 启动TX通道
    Drv_DMA_Start(tx_ch);
    
    EDIS;
}

/**
 * @brief  停止DMA通道
 * @param  channel - DMA通道
 */
void Drv_DMA_Stop(DMA_Channel_t channel)
{
    volatile struct CH_REGS* ch_regs = Drv_DMA_GetChannelRegs(channel);
    
    if(ch_regs == NULL) {
        return;
    }
    
    EALLOW;
    ch_regs->CONTROL.bit.HALT = 1;  // 设置HALT标志停止DMA
    EDIS;
}

/******************************************************************************
 * DMA回调与统计函数
 ******************************************************************************/

/**
 * @brief  注册DMA完成回调函数
 * @param  channel - DMA通道
 * @param  callback - 回调函数指针
 */
void Drv_DMA_RegisterCallback(DMA_Channel_t channel, DMA_Callback_t callback)
{
    if(channel < 6) {
        g_dma_callbacks[channel] = callback;
    }
}

/**
 * @brief  清除DMA统计信息
 * @param  channel - DMA通道（255=清除所有）
 */
void Drv_DMA_ClearStats(Uint16 channel)
{
    Uint16 i;
    
    if(channel == 255) {
        // 清除所有通道统计
        for(i = 0; i < 6; i++) {
            g_dma_stats[i].complete_count = 0;
            g_dma_stats[i].overflow_count = 0;
            g_dma_stats[i].timeout_count = 0;
            g_dma_stats[i].error_count = 0;
        }
    } else if(channel < 6) {
        // 清除指定通道统计
        g_dma_stats[channel].complete_count = 0;
        g_dma_stats[channel].overflow_count = 0;
        g_dma_stats[channel].timeout_count = 0;
        g_dma_stats[channel].error_count = 0;
    }
}

/**
 * @brief  获取DMA统计信息
 * @param  channel - DMA通道
 * @param  stats - 输出统计信息指针
 */
void Drv_DMA_GetStats(DMA_Channel_t channel, DMA_Stats_t *stats)
{
    if(stats != NULL && channel < 6) {
        stats->complete_count = g_dma_stats[channel].complete_count;
        stats->overflow_count = g_dma_stats[channel].overflow_count;
        stats->timeout_count = g_dma_stats[channel].timeout_count;
        stats->error_count = g_dma_stats[channel].error_count;
    }
}

/**
 * @brief  检查DMA通道是否完成
 * @param  channel - DMA通道
 * @return bool - true=已完成，false=未完成
 */
bool Drv_DMA_IsComplete(DMA_Channel_t channel)
{
    switch(channel) {
        case DMA_CH1: return g_dma_ch1_done;
        case DMA_CH2: return g_dma_ch2_done;
        case DMA_CH5: return g_dma_ch5_done;
        case DMA_CH6: return g_dma_ch6_done;
        default: return false;
    }
}

/**
 * @brief  等待DMA完成（带超时）
 * @param  channel - DMA通道
 * @param  timeout_us - 超时时间（微秒）
 * @return bool - true=成功完成，false=超时
 */
bool Drv_DMA_WaitComplete(DMA_Channel_t channel, Uint32 timeout_us)
{
    Uint32 wait_count = 0;
    Uint32 max_count = timeout_us / 10;
    
    while(!Drv_DMA_IsComplete(channel) && (wait_count < max_count)) {
        wait_count++;
        DELAY_US(10);
    }
    
    if(wait_count >= max_count) {
        g_dma_stats[channel].timeout_count++;
        return false;  // 超时
    }
    
    return true;
}

/******************************************************************************
 * DMA中断服务函数（在PIE中注册）
 ******************************************************************************/

/**
 * @brief  DMA CH1中断服务函数（SPIA RX完成）
 */
__interrupt void DMA_CH1_ISR(void)
{
    volatile struct CH_REGS* ch1 = &DmaRegs.CH1;
    
    EALLOW;
    
    // 停止DMA通道
    ch1->CONTROL.bit.HALT = 1;
    
    // 清除溢出标志（如果有）
    if(ch1->CONTROL.bit.OVRFLG) {
        ch1->CONTROL.bit.OVRFLG = 1;  // 写1清除
        g_dma_stats[DMA_CH1].overflow_count++;
    }
    
    // 清除同步标志（如果有）
    if(ch1->CONTROL.bit.SYNCFLG) {
        ch1->CONTROL.bit.SYNCFLG = 1;  // 写1清除
    }
    
    EDIS;
    
    // 清除PIE中断标志
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP7;
    
    // 设置完成标志
    g_dma_ch1_done = true;
    
    // 更新统计计数
    g_dma_stats[DMA_CH1].complete_count++;
    
    // 调用回调函数（如果已注册）
    if(g_dma_callbacks[DMA_CH1] != NULL) {
        g_dma_callbacks[DMA_CH1]();
    }
}

/**
 * @brief  DMA CH2中断服务函数（SPIA TX完成）
 */
__interrupt void DMA_CH2_ISR(void)
{
    volatile struct CH_REGS* ch2 = &DmaRegs.CH2;
    
    EALLOW;
    
    ch2->CONTROL.bit.HALT = 1;
    
    if(ch2->CONTROL.bit.OVRFLG) {
        ch2->CONTROL.bit.OVRFLG = 1;
        g_dma_stats[DMA_CH2].overflow_count++;
    }
    
    if(ch2->CONTROL.bit.SYNCFLG) {
        ch2->CONTROL.bit.SYNCFLG = 1;
    }
    
    EDIS;
    
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP7;
    
    g_dma_ch2_done = true;
    g_dma_stats[DMA_CH2].complete_count++;
    
    if(g_dma_callbacks[DMA_CH2] != NULL) {
        g_dma_callbacks[DMA_CH2]();
    }
}

/**
 * @brief  DMA CH5中断服务函数（SPIB TX完成）
 */
__interrupt void DMA_CH5_ISR(void)
{
    volatile struct CH_REGS* ch5 = &DmaRegs.CH5;
    
    EALLOW;
    
    ch5->CONTROL.bit.HALT = 1;
    
    if(ch5->CONTROL.bit.OVRFLG) {
        ch5->CONTROL.bit.OVRFLG = 1;
        g_dma_stats[DMA_CH5].overflow_count++;
    }
    
    if(ch5->CONTROL.bit.SYNCFLG) {
        ch5->CONTROL.bit.SYNCFLG = 1;
    }
    
    EDIS;
    
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP7;
    
    g_dma_ch5_done = true;
    g_dma_stats[DMA_CH5].complete_count++;
    
    if(g_dma_callbacks[DMA_CH5] != NULL) {
        g_dma_callbacks[DMA_CH5]();
    }
}

/**
 * @brief  DMA CH6中断服务函数（SPIB RX完成）
 */
__interrupt void DMA_CH6_ISR(void)
{
    volatile struct CH_REGS* ch6 = &DmaRegs.CH6;
    
    EALLOW;
    
    ch6->CONTROL.bit.HALT = 1;
    
    if(ch6->CONTROL.bit.OVRFLG) {
        ch6->CONTROL.bit.OVRFLG = 1;
        g_dma_stats[DMA_CH6].overflow_count++;
    }
    
    if(ch6->CONTROL.bit.SYNCFLG) {
        ch6->CONTROL.bit.SYNCFLG = 1;
    }
    
    EDIS;
    
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP7;
    
    g_dma_ch6_done = true;
    g_dma_stats[DMA_CH6].complete_count++;
    
    if(g_dma_callbacks[DMA_CH6] != NULL) {
        g_dma_callbacks[DMA_CH6]();
    }
}

/******************************************************************************
 * End of File
 ******************************************************************************/

