#include "drv_dma.h"

//=============================================================================
// 内部变量定义
//=============================================================================

// DMA缓冲区定义 (3个16位字，用于传输24位命令)
#pragma DATA_SECTION(s_tx_buffer, "ramgs1")
#pragma DATA_ALIGN(s_tx_buffer, 2)
static Uint16 s_tx_buffer[3];

#pragma DATA_SECTION(s_rx_buffer, "ramgs1")
#pragma DATA_ALIGN(s_rx_buffer, 2)
static Uint16 s_rx_buffer[3];


Uint16 DMA_Tx_Buffer[3] = {0};
Uint16 DMA_Rx_Buffer[3] = {0};
// DMA状态机变量
static volatile DMA_State_t s_dma_state = DMA_STATE_IDLE;

//=============================================================================
// 模块初始化
//=============================================================================

void Drv_DMA_Init(void)
{
    DMAInitialize(); // 硬复位并初始化DMA控制器

    // 调试模式下DMA继续运行
    DmaRegs.DEBUGCTRL.bit.FREE = 1;


}

void Drv_DMA_Config_SPIB(void)
{
    EALLOW;
    
    //=============================================================================
    // 步骤1：DMA控制器硬件复位
    //=============================================================================
    // 说明：写1到HARDRESET位会复位整个DMA模块并中止所有当前访问（类似芯片复位）
    // 注意：硬件设计要求在写入此位后至少需要1个NOP周期才能访问其他DMA寄存器
    DmaRegs.DMACTRL.bit.HARDRESET = 1;
    __asm(" NOP");  

    //=============================================================================
    // 步骤2：配置DMA CH5通道 - SPIB发送（TX）
    //=============================================================================
    // CH5通道负责：从内存缓冲区 DMA_Tx_Buffer[] 搬运数据到 SPIB发送寄存器 SPITXBUF
    // 数据流向：DMA_Tx_Buffer[0/1/2] → SPITXBUF → SPI总线
    
    // --- 2.1 暂时关闭中断（配置期间） ---
    DmaRegs.CH5.MODE.bit.CHINTE = 0;

    // --- 2.2 突发传输配置（Burst Loop - 内层循环） ---
    // 突发大小：BURST_SIZE = 0 表示每次突发传输 1 个16位字（0+1=1）
    // 突发步进：由于只传输1个字，步进值设为0（无效）
    DmaRegs.CH5.BURST_SIZE.all = 0;       // 每次突发传输1个字
    DmaRegs.CH5.SRC_BURST_STEP = 0;       // 源地址突发步进（单字时无效）
    DmaRegs.CH5.DST_BURST_STEP = 0;       // 目标地址突发步进（单字时无效）

    // --- 2.3 传输配置（Transfer Loop - 外层循环） ---
    // 传输大小：TRANSFER_SIZE = 2 表示每次传输包含 3 个突发（2+1=3）
    // 总传输字数 = (BURST_SIZE+1) × (TRANSFER_SIZE+1) = 1 × 3 = 3个字
    // 传输步进：控制每完成一个突发后地址如何变化
    DmaRegs.CH5.TRANSFER_SIZE = 2;        // 传输3个突发（即3个字）
    DmaRegs.CH5.SRC_TRANSFER_STEP = 1;    // 源地址每次突发后+1（读取下一个缓冲区元素）
    DmaRegs.CH5.DST_TRANSFER_STEP = 0;    // 目标地址不变（始终写入同一个SPITXBUF寄存器）
    
    // --- 2.4 源地址配置（Source Address） ---
    // ADDR_SHADOW：影子地址寄存器，传输开始时自动加载到ADDR_ACTIVE
    // BEG_ADDR_SHADOW：起始地址，用于环绕（Wrap）功能
    DmaRegs.CH5.SRC_ADDR_SHADOW = (Uint32) &DMA_Tx_Buffer[0];       // 源起始地址
    DmaRegs.CH5.SRC_BEG_ADDR_SHADOW = (Uint32) &DMA_Tx_Buffer[0];   // 环绕起始地址
    
    // --- 2.5 目标地址配置（Destination Address） ---
    DmaRegs.CH5.DST_ADDR_SHADOW = (Uint32) &SpibRegs.SPITXBUF;      // 目标地址：SPIB发送FIFO
    DmaRegs.CH5.DST_BEG_ADDR_SHADOW = (Uint32)&SpibRegs.SPITXBUF;   // 环绕起始地址
    
    // --- 2.6 清除标志位 ---
    DmaRegs.CH5.CONTROL.bit.PERINTCLR = 1;   // 清除外设中断事件标志（PERINTFLG）
    DmaRegs.CH5.CONTROL.bit.ERRCLR = 1;      // 清除溢出错误标志（OVRFLG）
    
    // --- 2.7 环绕配置（Wrap Function） ---
    // WRAP_SIZE = 0xFFFF 大于 TRANSFER_SIZE，环绕功能被禁用
    // 如果 WRAP_SIZE < TRANSFER_SIZE，则会启用循环缓冲区功能
    DmaRegs.CH5.DST_WRAP_SIZE = 0xFFFF;      // 禁用目标地址环绕
    DmaRegs.CH5.SRC_WRAP_SIZE = 0xFFFF;      // 禁用源地址环绕
    
    // --- 2.8 中断与模式配置 ---
    // CHINTE：使能通道中断到PIE模块
    // CHINTMODE=1：在传输结束时产生中断（非突发开始时）
    // PERINTE：使能外设事件触发
    // PERINTSEL：外设中断选择（遗留位，应设为通道号，实际触发源由DMACHSRCSEL配置）
    DmaRegs.CH5.MODE.bit.CHINTE = 1;         // 使能通道中断
    DmaRegs.CH5.MODE.bit.CHINTMODE = 1;      // 传输结束时产生中断
    DmaRegs.CH5.MODE.bit.PERINTE = 1;        // 使能外设中断事件
    DmaRegs.CH5.MODE.bit.PERINTSEL = 5;      // 外设中断选择（设为通道号5）
    
    // --- 2.9 触发源选择 ---
    // DMA_SPIBTX（值111）：当SPIB发送FIFO准备好接收数据时触发DMA
    DmaClaSrcSelRegs.DMACHSRCSEL2.bit.CH5 = DMA_SPIBTX;
    DmaRegs.CH5.CONTROL.bit.PERINTCLR = 1;   // 再次清除可能存在的虚假中断标志

    //=============================================================================
    // 步骤3：配置DMA CH6通道 - SPIB接收（RX）
    //=============================================================================
    // CH6通道负责：从 SPIB接收寄存器 SPIRXBUF 搬运数据到内存缓冲区 DMA_Rx_Buffer[]
    // 数据流向：SPI总线 → SPIRXBUF → DMA_Rx_Buffer[0/1/2]
    
    // --- 3.1 暂时关闭中断 ---
    DmaRegs.CH6.MODE.bit.CHINTE = 0;
    
    // --- 3.2 突发传输配置 ---
    DmaRegs.CH6.BURST_SIZE.all = 0;          // 每次突发传输1个字
    DmaRegs.CH6.SRC_BURST_STEP = 0;          // 源地址突发步进（单字时无效）
    DmaRegs.CH6.DST_BURST_STEP = 0;          // 目标地址突发步进（单字时无效）
    
    // --- 3.3 传输配置 ---
    DmaRegs.CH6.TRANSFER_SIZE = 2;           // 传输3个突发（3个字）
    DmaRegs.CH6.SRC_TRANSFER_STEP = 0;       // 源地址不变（始终从SPIRXBUF读取）
    DmaRegs.CH6.DST_TRANSFER_STEP = 1;       // 目标地址每次突发后+1（写入下一个缓冲区位置）
    
    // --- 3.4 源地址配置 ---
    DmaRegs.CH6.SRC_ADDR_SHADOW = (Uint32) &SpibRegs.SPIRXBUF;      // 源地址：SPIB接收FIFO
    DmaRegs.CH6.SRC_BEG_ADDR_SHADOW = (Uint32) &SpibRegs.SPIRXBUF;  // 环绕起始地址
    
    // --- 3.5 目标地址配置 ---
    DmaRegs.CH6.DST_ADDR_SHADOW = (Uint32) &DMA_Rx_Buffer[0];       // 目标起始地址
    DmaRegs.CH6.DST_BEG_ADDR_SHADOW = (Uint32) &DMA_Rx_Buffer[0];   // 环绕起始地址
    
    // --- 3.6 清除标志位 ---
    DmaRegs.CH6.CONTROL.bit.PERINTCLR = 1;   // 清除外设中断事件标志
    DmaRegs.CH6.CONTROL.bit.ERRCLR = 1;      // 清除溢出错误标志
    
    // --- 3.7 环绕配置 ---
    DmaRegs.CH6.DST_WRAP_SIZE = 0xFFFF;      // 禁用目标地址环绕
    DmaRegs.CH6.SRC_WRAP_SIZE = 0xFFFF;      // 禁用源地址环绕
    
    // --- 3.8 中断与模式配置 ---
    DmaRegs.CH6.MODE.bit.CHINTE = 1;         // 使能通道中断
    DmaRegs.CH6.MODE.bit.CHINTMODE = 1;      // 传输结束时产生中断
    DmaRegs.CH6.MODE.bit.PERINTE = 1;        // 使能外设中断事件
    DmaRegs.CH6.MODE.bit.PERINTSEL = 6;      // 外设中断选择（设为通道号6）
    
    // --- 3.9 触发源选择 ---
    // DMA_SPIBRX（值112）：当SPIB接收FIFO有数据可读时触发DMA
    DmaClaSrcSelRegs.DMACHSRCSEL2.bit.CH6 = DMA_SPIBRX;
    DmaRegs.CH6.CONTROL.bit.PERINTCLR = 1;   // 清除虚假中断标志
    
    EDIS;
    
    //=============================================================================
    // 配置完成说明
    //=============================================================================
    // 此时DMA通道已完全配置但尚未启动，需要通过以下方式启动传输：
    // 1. 设置 DmaRegs.CHx.CONTROL.bit.RUN = 1 启动通道
    // 2. 等待外设触发事件（SPIB TX/RX FIFO就绪）
    // 3. DMA自动完成数据传输
    // 4. 传输完成后产生中断（如果CHINTE=1）
    //=============================================================================
}
void start_dma (void)
{
  EALLOW;
  DmaRegs.CH5.CONTROL.bit.RUN = 1;
  DmaRegs.CH6.CONTROL.bit.RUN = 1;
  EDIS;
}
//=============================================================================
// DMA控制函数
//=============================================================================
// 注意：DMA通道在Drv_DMA_Config_SPIB()配置完成后，需要通过设置RUN位来启动
// 示例：
//   EALLOW;
//   DmaRegs.CH5.CONTROL.bit.RUN = 1;  // 启动TX通道
//   DmaRegs.CH6.CONTROL.bit.RUN = 1;  // 启动RX通道
//   EDIS;

//=============================================================================
// 状态与缓冲区访问
//=============================================================================

Uint16* Drv_DMA_Get_TX_Buffer(void)
{
    return s_tx_buffer;
}

Uint16* Drv_DMA_Get_RX_Buffer(void)
{
    return s_rx_buffer;
}

DMA_State_t Drv_DMA_Get_State(void)
{
    return s_dma_state;
}

//=============================================================================
// 中断服务函数
//=============================================================================

// ⭐ DMA中断执行次数计数器（外部可见，用于调试）
extern volatile uint32_t debug_dma_isr_tx_count;
extern volatile uint32_t debug_dma_isr_rx_count;
__interrupt void DMA_CH6_ISR(void) // RX 完成中断
{
    EALLOW;
    DmaRegs.CH6.CONTROL.bit.HALT = 1;
    debug_dma_isr_rx_count++;
    DmaRegs.CH6.CONTROL.bit.RUN = 1;
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP7;
    EDIS;

}
__interrupt void DMA_CH5_ISR(void) // RX 完成中断
{
    EALLOW;
    // 停止McBSP的DMA发送通道
    DmaRegs.CH5.CONTROL.bit.HALT = 1;
    debug_dma_isr_tx_count++;
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP7;
    EDIS;
}

