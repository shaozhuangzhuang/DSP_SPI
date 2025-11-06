//****************************************************************************
//
// 文件名: drv_spi.c
// 功能: SPIA与SPIB驱动实现
//
// 说明:
//   - SPIA: 主模式，GPIO54-57，16位，2MHz，Mode 0 (CPOL=0, CPHA=0)
//   - SPIB: 主模式，GPIO24-27，8位，500kHz，Mode 1 (CPOL=0, CPHA=1)
//
// 配置流程:
//   1. 使能外设时钟
//   2. 配置GPIO引脚
//   3. 配置SPI寄存器
//   4. 配置FIFO
//   5. 配置中断
//
//****************************************************************************

#include "F28x_Project.h"
#include "drv_spi.h"

//****************************************************************************
// 函数名: InitSpiGpios
// 功能: 初始化SPIA和SPIB的所有GPIO引脚
//****************************************************************************
void InitSpiGpios(void)
{
    Drv_InitSpiaGpio();
    Drv_InitSpibGpio();
}

//****************************************************************************
// 函数名: Drv_InitSpiaGpio
// 功能: 初始化SPIA的GPIO引脚（GPIO54-57）
// 说明: 使用GPIO库函数配置，分配给CPU1
//****************************************************************************
void Drv_InitSpiaGpio(void)
{
    EALLOW;

    //==========================================================
    // 配置引脚复用功能（设置为SPIA外设功能并分配给CPU1）
    //==========================================================
    // GPIO54 -> SPISIMOA (Master Out) - 由SPIA外设控制
    // GPIO55 -> SPISOMIA (Master In) - 由SPIA外设控制
    // GPIO56 -> SPICLKA (Clock) - 由SPIA外设控制
    // GPIO57 -> SPISTEA (Chip Select) - 由SPIA外设控制

    GPIO_SetupPinMux(54, GPIO_MUX_CPU1, 1); // SPISIMOA
    GPIO_SetupPinMux(55, GPIO_MUX_CPU1, 1); // SPISOMIA
    GPIO_SetupPinMux(56, GPIO_MUX_CPU1, 1); // SPICLKA
    GPIO_SetupPinMux(57, GPIO_MUX_CPU1, 1); // SPISTEA

    //==========================================================
    // 配置引脚选项
    // 注意：GPIO54-57已配置为SPI外设功能，由SPIA模块控制
    //       不需要再设置GPIO方向
    //==========================================================

    EDIS;
}

//****************************************************************************
// 函数名: Drv_InitSpibGpio
// 功能: 初始化SPIB的GPIO引脚（GPIO24-27）
// 说明: 使用寄存器直接配置方式（参考spi_loopback），分配给CPU1
//       GPIO27配置为SPISTEB硬件片选功能
//****************************************************************************
void Drv_InitSpibGpio(void)
{
    EALLOW;

    //==========================================================
    // 步骤1：配置内部上拉电阻
    //==========================================================
    // Pull-ups can be enabled or disabled by the user.
    // ⚠️ GPIO25(MISO)禁用上拉，避免浮空时读到高电平
    //
    GpioCtrlRegs.GPAPUD.bit.GPIO24 = 0;  // Enable pull-up on GPIO24 (SPISIMOB)
    // ⚠️ GPIO25(MISO)禁用上拉，避免浮空时读到高电平
    GpioCtrlRegs.GPAPUD.bit.GPIO25 = 1;  // Disable pull-up on GPIO25 (SPISOMIB) - 关键修改
    GpioCtrlRegs.GPAPUD.bit.GPIO26 = 0;  // Enable pull-up on GPIO26 (SPICLKB)
    GpioCtrlRegs.GPAPUD.bit.GPIO27 = 0;  // Enable pull-up on GPIO27 (GPIO SYNC)

    //==========================================================
    // 步骤2：设置异步限定模式
    //==========================================================
    // This will select asynch (no qualification) for the selected pins.
    //
    GpioCtrlRegs.GPAQSEL2.bit.GPIO24 = 3; // Asynch input GPIO24 (SPISIMOB)
    GpioCtrlRegs.GPAQSEL2.bit.GPIO25 = 3; // Asynch input GPIO25 (SPISOMIB)
    GpioCtrlRegs.GPAQSEL2.bit.GPIO26 = 3; // Asynch input GPIO26 (SPICLKB)
    GpioCtrlRegs.GPAQSEL2.bit.GPIO27 = 3; // Asynch input GPIO27 (SPISTEB)

    //==========================================================
    // 步骤3：配置GPIO方向为输出（外设模式）
    //==========================================================
    // 虽然配置为外设功能后由外设控制，但明确设置为输出
    //
    GpioCtrlRegs.GPADIR.bit.GPIO24 = 1;  // SPISIMOB输出（MOSI）
    GpioCtrlRegs.GPADIR.bit.GPIO25 = 0;  // SPISOMIB输入（MISO）
    GpioCtrlRegs.GPADIR.bit.GPIO26 = 1;  // SPICLKB输出（CLK）
    GpioCtrlRegs.GPADIR.bit.GPIO27 = 1;  // SPISTEB输出（CS）

    //==========================================================
    // 步骤4：配置GPIO为推挽输出模式（关键！）
    //==========================================================
    // 必须配置为推挽模式，否则GPIO无法输出高电平
    // ODR=0表示推挽模式，ODR=1表示开漏模式
    //
    GpioCtrlRegs.GPAODR.bit.GPIO24 = 0;  // SPISIMOB推挽模式（MOSI）
    GpioCtrlRegs.GPAODR.bit.GPIO25 = 0;  // SPISOMIB推挽模式（MISO）
    GpioCtrlRegs.GPAODR.bit.GPIO26 = 0;  // SPICLKB推挽模式（CLK）
    GpioCtrlRegs.GPAODR.bit.GPIO27 = 0;  // SPISTEB推挽模式（CS）

    //==========================================================
    // 步骤5：配置引脚复用功能为SPIB
    //==========================================================
    // This specifies which of the possible GPIO pins will be SPI functional
    // pins. GPAGMUX2=1, GPAMUX2=2 -> MUX=6 for SPIB on GPIO24-26.
    // GPIO27配置为普通GPIO用于手动SYNC控制（AD5754R要求）
    //
    GpioCtrlRegs.GPAGMUX2.bit.GPIO24 = 1; // Configure GPIO24 as SPISIMOB
    GpioCtrlRegs.GPAGMUX2.bit.GPIO25 = 1; // Configure GPIO25 as SPISOMIB
    GpioCtrlRegs.GPAGMUX2.bit.GPIO26 = 1; // Configure GPIO26 as SPICLKB
    // GPIO27不设置GPAGMUX2（保持默认0）

    GpioCtrlRegs.GPAMUX2.bit.GPIO24 = 2; // Configure GPIO24 as SPISIMOB
    GpioCtrlRegs.GPAMUX2.bit.GPIO25 = 2; // Configure GPIO25 as SPISOMIB
    GpioCtrlRegs.GPAMUX2.bit.GPIO26 = 2; // Configure GPIO26 as SPICLKB
    GpioCtrlRegs.GPAMUX2.bit.GPIO27 = 0; // Configure GPIO27 as GPIO (manual SYNC)

    // 初始化SYNC为高电平（未选中状态）
    GpioDataRegs.GPASET.bit.GPIO27 = 1;

    EDIS;
}

//****************************************************************************
// 函数名: InitSpiInterrupts
// 功能: 初始化SPI相关的PIE中断
//****************************************************************************
// ISR 函数外部声明
extern interrupt void cpu_timer0_isr(void);
extern interrupt void spiARxISR(void);

//****************************************************************************
// 函数名: InitSpiInterrupts
// 功能: 初始化SPI相关的PIE中断
//****************************************************************************
void InitSpiInterrupts(void)
{

}

//****************************************************************************
// 函数名: InitSpiModules
// 功能: 初始化SPIA和SPIB外设模块
//****************************************************************************
void InitSpiModules(void)
{
    //==========================================================
    // 初始化各个SPI模块
    //==========================================================
    Drv_InitSpiaModule();
    Drv_InitSpibModule();
}

//****************************************************************************
// 函数名: Drv_InitSpiaModule
// 功能: 初始化SPIA模块（主模式，2MHz，16位）
// 说明: 按照标准流程配置SPIA
//****************************************************************************
void Drv_InitSpiaModule(void)
{
    //==========================================================
    // 步骤1: 将SPI置于复位状态（必须先执行）
    //==========================================================
    SpiaRegs.SPICCR.bit.SPISWRESET = 0;

    //==========================================================
    // 步骤2: 配置SPICCR寄存器
    //==========================================================
    SpiaRegs.SPICCR.bit.CLKPOLARITY = SPIA_CLK_POLARITY;     // 时钟极性：Mode 0
    SpiaRegs.SPICCR.bit.SPICHAR = SPIA_CHAR_LENGTH;           // 字符长度：16位
    SpiaRegs.SPICCR.bit.SPILBK = 0;         // 禁用回环模式（连接外部设备）
    SpiaRegs.SPICCR.bit.HS_MODE = 0;        // 0=普通速度模式

    //==========================================================
    // 步骤3: 配置SPICTL寄存器
    //==========================================================
    SpiaRegs.SPICTL.bit.MASTER_SLAVE = SPI_MODE_MASTER;  // 1=主模式
    SpiaRegs.SPICTL.bit.CLK_PHASE = SPIA_CLK_PHASE;      // 时钟相位：Mode 0
    SpiaRegs.SPICTL.bit.TALK = 1;           // 使能发送
    SpiaRegs.SPICTL.bit.OVERRUNINTENA = 0;  // 禁用溢出中断（FIFO模式不使用）
    SpiaRegs.SPICTL.bit.SPIINTENA = 0;      // 禁用SPI中断（FIFO模式不使用）

    //==========================================================
    // 步骤4: 配置波特率
    // SPI波特率 = LSPCLK / (SPIBRR + 1)
    // LSPCLK=50MHz, SPIBRR=24, 则波特率=2MHz
    //==========================================================
    SpiaRegs.SPIBRR.bit.SPI_BIT_RATE = SPIA_BAUDRATE;  // 2MHz

    //==========================================================
    // 步骤5: 配置优先级和模式
    //==========================================================
    SpiaRegs.SPIPRI.bit.FREE = 1;           // 仿真时自由运行
    SpiaRegs.SPIPRI.bit.SOFT = 0;
    SpiaRegs.SPIPRI.bit.TRIWIRE = 0;        // 4线模式（0=4线，1=3线）
    SpiaRegs.SPIPRI.bit.STEINV = 0;         // SPISTE极性（0=低有效）

    //==========================================================
    // 步骤6: 配置FIFO（按照TI标准顺序）
    //==========================================================
    // 6.1 使能FIFO增强功能
    SpiaRegs.SPIFFTX.bit.SPIFFENA = 1;      // 使能FIFO功能

    // 6.2 复位TX FIFO（先复位）
    SpiaRegs.SPIFFTX.bit.TXFIFO = 0;        // 复位TX FIFO

    // 6.3 复位RX FIFO（先复位）
    SpiaRegs.SPIFFRX.bit.RXFIFORESET = 0;   // 复位RX FIFO

    // 6.4 配置发送FIFO
    SpiaRegs.SPIFFTX.bit.TXFFIL = 0;        // TX中断级别：0=FIFO空时中断
    SpiaRegs.SPIFFTX.bit.TXFFIENA = 0;      // 禁用TX FIFO中断（不使用发送中断）
    SpiaRegs.SPIFFTX.bit.TXFFINTCLR = 1;    // 清除TX中断标志

    // 6.5 配置接收FIFO
    SpiaRegs.SPIFFRX.bit.RXFFIL = 4;        // RX中断级别：4=收到4个字时中断
    SpiaRegs.SPIFFRX.bit.RXFFIENA = 1;      // 使能RX FIFO中断
    SpiaRegs.SPIFFRX.bit.RXFFINTCLR = 1;    // 清除RX中断标志
    SpiaRegs.SPIFFRX.bit.RXFFOVFCLR = 1;    // 清除溢出标志

    // 6.6 配置FIFO传输延迟
    SpiaRegs.SPIFFCT.bit.TXDLY = 0;         // 字间延迟：0=无延迟

    // 6.7 释放FIFO通道复位
    SpiaRegs.SPIFFTX.bit.SPIRST = 1;        // 释放SPI FIFO通道复位

    // 6.8 释放TX和RX FIFO复位（重新使能）
    SpiaRegs.SPIFFTX.bit.TXFIFO = 1;        // 释放TX FIFO复位
    SpiaRegs.SPIFFRX.bit.RXFIFORESET = 1;   // 释放RX FIFO复位

    //==========================================================
    // 步骤7: 启动SPI（最后执行）
    //==========================================================
    SpiaRegs.SPICCR.bit.SPISWRESET = 1;     // 释放SPI复位，开始工作
}

//****************************************************************************
// 函数名: Drv_InitSpibModule
// 功能: 初始化SPIB模块（主模式，1MHz，8位，Mode 0）
// 说明: 配置为SPI Mode 0 (CPOL=0, CPHA=0)用于AD5754R通信
//****************************************************************************
void Drv_InitSpibModule(void)
{
    //==========================================================
    // 步骤1: 将SPI置于复位状态（必须先执行）
    //==========================================================
    SpibRegs.SPICCR.bit.SPISWRESET = 0;

    //==========================================================
    // 步骤2: 配置SPICCR寄存器
    //==========================================================
    SpibRegs.SPICCR.bit.CLKPOLARITY = SPIB_CLK_POLARITY;     // ✅ Mode 0: CPOL=0 (SCLK空闲为低)
    SpibRegs.SPICCR.bit.SPICHAR = SPIB_CHAR_LENGTH;           // 字符长度：8位 (7表示8位)
    SpibRegs.SPICCR.bit.SPILBK = 0;         // 禁用回环模式（连接外部设备）
    SpibRegs.SPICCR.bit.HS_MODE = 0;        // 0=普通速度模式

    //==========================================================
    // 步骤3: 配置SPICTL寄存器
    //==========================================================
    SpibRegs.SPICTL.bit.MASTER_SLAVE = SPI_MODE_MASTER;  // 1=主模式
    SpibRegs.SPICTL.bit.CLK_PHASE = SPIB_CLK_PHASE;      // ✅ Mode 0: CPHA=0 (数据在上升沿采样)
    SpibRegs.SPICTL.bit.TALK = 1;           // 使能发送
    SpibRegs.SPICTL.bit.OVERRUNINTENA = 0;  // 禁用溢出中断（FIFO模式不使用）
    SpibRegs.SPICTL.bit.SPIINTENA = 0;      // 禁用SPI中断（FIFO模式不使用）

    //==========================================================
    // 步骤4: 配置波特率
    // SPI波特率 = LSPCLK / (SPIBRR + 1)
    // LSPCLK=50MHz, SPIBRR=99, 则波特率=500kHz
    //==========================================================
    SpibRegs.SPIBRR.bit.SPI_BIT_RATE = SPIB_BAUDRATE;  // 500kHz

    //==========================================================
    // 步骤5: 配置优先级和模式
    //==========================================================
    SpibRegs.SPIPRI.bit.FREE = 1;           // 仿真时自由运行
    SpibRegs.SPIPRI.bit.SOFT = 0;
    SpibRegs.SPIPRI.bit.TRIWIRE = 0;        // 4线模式（0=4线，1=3线）
    SpibRegs.SPIPRI.bit.STEINV = 0;         // SPISTE极性（0=低有效）

    //==========================================================
    // 步骤6: 配置FIFO（按照TI标准顺序）
    //==========================================================
    // 6.1 使能FIFO增强功能
    SpibRegs.SPIFFTX.bit.SPIFFENA = 1;      // 使能FIFO功能

    // 6.2 复位TX FIFO（先复位）
    SpibRegs.SPIFFTX.bit.TXFIFO = 0;        // 复位TX FIFO

    // 6.3 复位RX FIFO（先复位）
    SpibRegs.SPIFFRX.bit.RXFIFORESET = 0;   // 复位RX FIFO

    // 6.4 配置发送FIFO
    SpibRegs.SPIFFTX.bit.TXFFIL = 4;        // TX中断级别：0=FIFO空时中断
    SpibRegs.SPIFFTX.bit.TXFFIENA = 1;      // 使能TX FIFO中断/事件生成
    SpibRegs.SPIFFTX.bit.TXFFINTCLR = 1;    // 清除TX中断标志

    // 6.5 配置接收FIFO
    SpibRegs.SPIFFRX.bit.RXFFIL = 3;        // RX中断级别：3=收到3个字时中断 (匹配DMA传输长度)
    SpibRegs.SPIFFRX.bit.RXFFIENA = 1;      // 使能RX FIFO中断/事件生成
    SpibRegs.SPIFFRX.bit.RXFFINTCLR = 1;    // 清除RX中断标志
    SpibRegs.SPIFFRX.bit.RXFFOVFCLR = 1;    // 清除溢出标志

    // 6.6 配置FIFO传输延迟
    SpibRegs.SPIFFCT.bit.TXDLY = 0;         // 字间延迟：0=无延迟

    // 6.7 释放FIFO通道复位
    SpibRegs.SPIFFTX.bit.SPIRST = 1;        // 释放SPI FIFO通道复位

    // 6.8 释放TX和RX FIFO复位（重新使能）
    SpibRegs.SPIFFTX.bit.TXFIFO = 1;        // 释放TX FIFO复位
    SpibRegs.SPIFFRX.bit.RXFIFORESET = 1;   // 释放RX FIFO复位

    //==========================================================
    // 步骤7: 关键修正 - 在释放SPISWRESET前再次确认SPICTL配置
    //==========================================================
    // ❗某些操作可能清除SPICTL寄存器，在释放复位前再次设置
    SpibRegs.SPICTL.bit.MASTER_SLAVE = 1;   // 强制主模式
    SpibRegs.SPICTL.bit.CLK_PHASE = 0;      // ✅ Mode 0: CPHA=0 - 数据在上升沿采样
    SpibRegs.SPICTL.bit.TALK = 1;           // 强制使能发送

    //==========================================================
    // 步骤8: 启动SPI（最后执行）
    //==========================================================
    SpibRegs.SPICCR.bit.SPISWRESET = 1;     // 释放SPI复位，开始工作
}

//****************************************************************************
// 函数名: SpiaSendData
// 功能: 通过SPIA发送数据
// 参数: data - 数据数组指针, length - 数据长度（字数）
// 说明: 在定时器中断或主循环中调用
//****************************************************************************
void SpiaSendData(Uint16 *data, Uint16 length)
{
    Uint16 i;

    // 检查参数
    if(data == NULL || length == 0 || length > 16)
    {
        return;
    }

    // 写入TX FIFO
    for(i = 0; i < length; i++)
    {
        // 检查FIFO是否已满（可选）
        while(SpiaRegs.SPIFFTX.bit.TXFFST >= 16);

        SpiaRegs.SPITXBUF = data[i];
    }
}

//****************************************************************************
// 函数名: SpibSendData
// 功能: 通过SPIB发送数据
// 参数: data - 数据数组指针, length - 数据长度（字数）
// 说明: 在定时器中断或主循环中调用
//****************************************************************************
void SpibSendData(Uint16 *data, Uint16 length)
{
    Uint16 i;

    // 检查参数
    if(data == NULL || length == 0 || length > 16)
    {
        return;
    }

    // 写入TX FIFO
    for(i = 0; i < length; i++)
    {
        // 检查FIFO是否已满（可选）
        while(SpibRegs.SPIFFTX.bit.TXFFST >= 16);

        SpibRegs.SPITXBUF = data[i];
    }
}

//****************************************************************************
// 函数名: SpiaGetRxCount
// 功能: 获取SPIA接收FIFO中的数据数量
// 返回: 接收FIFO中的字数
//****************************************************************************
Uint16 SpiaGetRxCount(void)
{
    return SpiaRegs.SPIFFRX.bit.RXFFST;
}

//****************************************************************************
// 函数名: SpibGetRxCount
// 功能: 获取SPIB接收FIFO中的数据数量
// 返回: 接收FIFO中的字数
//****************************************************************************
Uint16 SpibGetRxCount(void)
{
    return SpibRegs.SPIFFRX.bit.RXFFST;
}

//****************************************************************************
// 函数名: AD5754_Send24BitCommand
// 功能: 发送24位命令到AD5754R（重构优化版）
// 说明: 使用SpibSendByte_Continuous函数简化实现，提高代码复用性
//****************************************************************************
void AD5754_Send24BitCommand(uint32_t command)
{
    // 开始SPI通信 - 拉低SYNC
    AD5754_SYNC_LOW();
    DELAY_US(1);

    // 连续发送3个字节
    SpibSendByte_Continuous((command >> 16) & 0xFF);
    SpibSendByte_Continuous((command >> 8) & 0xFF);
    SpibSendByte_Continuous(command & 0xFF);

    // 结束SPI通信 - 拉高SYNC
    AD5754_SYNC_HIGH();
    DELAY_US(1);
}

//****************************************************************************
// 函数名: SpibTransmitReceive_helper
// 功能: SPIB发送并接收一个字节（静态辅助函数）
// 说明: 包含超时处理，记录发送前的FIFO状态，确保读取正确的接收数据
//****************************************************************************
static uint16_t SpibTransmitReceive_helper(uint16_t data)
{
    uint16_t timeout = 0;
    uint16_t rx_count_before, rx_count_after;
    uint16_t rx_data;

    // 记录发送前RX FIFO的数据数量
    rx_count_before = SpibRegs.SPIFFRX.bit.RXFFST;

    // 发送数据（8位SPI模式：数据左对齐到高8位）
    SpibRegs.SPITXBUF = (data << 8);

    // 等待接收FIFO数量增加（表示新数据接收完成）
    do {
        rx_count_after = SpibRegs.SPIFFRX.bit.RXFFST;
        timeout++;
        if (timeout > 10000) return 0xFFFF; // 超时返回错误码（16位全1）
        DELAY_US(1);
    } while(rx_count_after <= rx_count_before);

    // 读取接收到的数据 - 保留完整16位，避免移位导致数据丢失
    // 在调试时可以观察数据实际在高8位还是低8位
    rx_data = SpibRegs.SPIRXBUF;
    return rx_data;  // 返回完整16位数据
}

//****************************************************************************
// 函数名: AD5754_Send24BitCommand_WithRead
// 功能: 发送24位命令并同时接收返回数据（重构优化版）
// 说明: 保留完整16位接收数据，提取低8位和高8位用于调试
//****************************************************************************
// ===== FIFO模式调试变量 =====
volatile uint16_t debug_rx_word1 = 0;
volatile uint16_t debug_rx_word2 = 0;
volatile uint16_t debug_rx_word3 = 0;
volatile uint32_t debug_result_low8 = 0;   // 从低8位提取的结果
volatile uint32_t debug_result_high8 = 0;  // 从高8位提取的结果

// ===== DMA模式调试变量 =====
volatile uint16_t debug_dma_rx_word1 = 0;  // DMA接收的第1个字
volatile uint16_t debug_dma_rx_word2 = 0;  // DMA接收的第2个字
volatile uint16_t debug_dma_rx_word3 = 0;  // DMA接收的第3个字
volatile uint32_t debug_dma_result = 0;    // DMA解析后的结果
volatile uint32_t debug_dma_isr_tx_count = 0; // DMA中断执行次数计数器
volatile uint32_t debug_dma_isr_rx_count = 0; // DMA中断执行次数计数器
uint32_t AD5754_Send24BitCommand_WithRead(uint32_t command)
{
    uint16_t rx_word1, rx_word2, rx_word3;

    // 开始SPI通信
    AD5754_SYNC_LOW();
    DELAY_US(1);

    // 连续发送和接收3个字节（保留完整16位）
    rx_word1 = SpibTransmitReceive_helper((command >> 16) & 0xFF);
    rx_word2 = SpibTransmitReceive_helper((command >> 8) & 0xFF);
    rx_word3 = SpibTransmitReceive_helper(command & 0xFF);

    // 结束SPI通信
    AD5754_SYNC_HIGH();
    DELAY_US(1);

    // 保存到调试变量，方便在调试器中观察原始数据
    debug_rx_word1 = rx_word1;
    debug_rx_word2 = rx_word2;
    debug_rx_word3 = rx_word3;

    // 方案1：从低8位提取数据（根据TI手册，8位SPI接收数据应该在低8位）
    debug_result_low8 = ((uint32_t)(rx_word1 & 0xFF) << 16) |
                        ((uint32_t)(rx_word2 & 0xFF) << 8) |
                        (rx_word3 & 0xFF);

    // 方案2：从高8位提取数据（备选方案）
    debug_result_high8 = ((uint32_t)(rx_word1 >> 8) << 16) |
                         ((uint32_t)(rx_word2 >> 8) << 8) |
                         (rx_word3 >> 8);

    // 先返回低8位方案（如果不对，可以在调试时查看debug_result_high8）
    return debug_result_low8;
}

//****************************************************************************
// 函数名: AD5754_Send24BitCommand_FIFO
// 功能: 使用FIFO批量模式发送24位命令到AD5754R
// 说明: 一次性写入3个字节到TX FIFO，硬件自动连续发送，无需逐字节等待
//       充分利用FIFO硬件特性，提高传输可靠性和效率
//****************************************************************************
void AD5754_Send24BitCommand_FIFO(uint32_t command)
{
    volatile Uint16 dummy;
    uint16_t timeout = 0;

    // 步骤1：清空RX FIFO（避免旧数据干扰）
    while(SpibRegs.SPIFFRX.bit.RXFFST > 0) {
        dummy = SpibRegs.SPIRXBUF;
    }

    // 步骤2：拉低SYNC，开始SPI通信
    AD5754_SYNC_LOW();
    DELAY_US(1);

    // 步骤3：批量写入3个字节到TX FIFO
    // 硬件会自动连续发送，产生连续的24个时钟脉冲
    SpibRegs.SPITXBUF = ((command >> 16) & 0xFF) << 8;  // 字节1（高字节）
    SpibRegs.SPITXBUF = ((command >> 8) & 0xFF) << 8;   // 字节2（中字节）
    SpibRegs.SPITXBUF = (command & 0xFF) << 8;          // 字节3（低字节）

    // 步骤4：等待传输完成（RX FIFO收到3个字节）
    while(SpibRegs.SPIFFRX.bit.RXFFST < 3) {
        timeout++;
        if(timeout > 10000) {
            AD5754_SYNC_HIGH();
            return;  // 超时退出
        }
        DELAY_US(1);
    }

    // 步骤5：清空RX FIFO（丢弃接收到的数据）
    while(SpibRegs.SPIFFRX.bit.RXFFST > 0) {
        dummy = SpibRegs.SPIRXBUF;
    }

    // 步骤6：拉高SYNC，结束SPI通信
    AD5754_SYNC_HIGH();
    DELAY_US(1);
}

//****************************************************************************
// 函数名: AD5754_Send24BitCommand_WithRead_FIFO
// 功能: 使用FIFO批量模式发送24位命令并读取返回数据
// 说明: 批量写入3字节，批量读取3字节，充分利用FIFO硬件
//       相比逐字节方式，减少软件干预，提高时序可靠性
//****************************************************************************
uint32_t AD5754_Send24BitCommand_WithRead_FIFO(uint32_t command)
{
    volatile Uint16 dummy;
    uint16_t rx_data1, rx_data2, rx_data3;
    uint16_t timeout = 0;

    // 步骤1：清空RX FIFO
    while(SpibRegs.SPIFFRX.bit.RXFFST > 0) {
        dummy = SpibRegs.SPIRXBUF;
    }

    // 步骤2：拉低SYNC
    AD5754_SYNC_LOW();
    DELAY_US(1);

    // 步骤3：批量写入3个字节到TX FIFO
    SpibRegs.SPITXBUF = ((command >> 16) & 0xFF) << 8;
    SpibRegs.SPITXBUF = ((command >> 8) & 0xFF) << 8;
    SpibRegs.SPITXBUF = (command & 0xFF) << 8;

    // 步骤4：等待RX FIFO收到3个字节
    while(SpibRegs.SPIFFRX.bit.RXFFST < 3) {
        timeout++;
        if(timeout > 10000) {
            AD5754_SYNC_HIGH();
            return 0xFFFFFF;  // 超时错误
        }
        DELAY_US(1);
    }

    // 步骤5：批量读取3个字节（FIFO先进先出）
    rx_data1 = SpibRegs.SPIRXBUF;  // 读取第1个字节
    rx_data2 = SpibRegs.SPIRXBUF;  // 读取第2个字节
    rx_data3 = SpibRegs.SPIRXBUF;  // 读取第3个字节

    // 步骤6：拉高SYNC
    AD5754_SYNC_HIGH();
    DELAY_US(1);

    // 步骤7：保存调试变量
    debug_rx_word1 = rx_data1;
    debug_rx_word2 = rx_data2;
    debug_rx_word3 = rx_data3;

    // 步骤8：提取低8位和高8位（用于调试判断数据位置）
    debug_result_low8 = ((uint32_t)(rx_data1 & 0xFF) << 16) |
                        ((uint32_t)(rx_data2 & 0xFF) << 8) |
                        (rx_data3 & 0xFF);

    debug_result_high8 = ((uint32_t)(rx_data1 >> 8) << 16) |
                         ((uint32_t)(rx_data2 >> 8) << 8) |
                         (rx_data3 >> 8);

    // 返回低8位结果（根据之前的分析，数据在低8位）
    return debug_result_low8;
}

//****************************************************************************
// 函数名: spib_xmit
// 功能: 通过SPIB发送单个数据字（参考spi_loopback的spi_xmit）
// 参数: a - 要发送的数据（8位）
// 说明: 简化的发送函数，硬件自动控制片选信号
//       ⚠️ 对于8位SPI，数据必须左对齐到16位字（左移8位）
//****************************************************************************
void spib_xmit(Uint16 a)
{
    // 8位SPI模式：数据必须左对齐（写入高8位）
    SpibRegs.SPITXBUF = (a << 8);
}

//****************************************************************************
// AD5754R通信测试变量
//****************************************************************************
volatile uint32_t ad5754_test_write_value = 0;
volatile uint32_t ad5754_test_read_value = 0;
volatile uint16_t ad5754_comm_test_pass = 0;
volatile uint16_t ad5754_initialized = 0;  // AD5754R初始化完成标志

// AD5754R初始化专用调试变量（不会被周期性通信覆盖）
volatile uint32_t ad5754_init_ctrl_cmd = 0;      // 初始化时写入控制寄存器的命令
volatile uint32_t ad5754_init_power_cmd = 0;     // 初始化时写入电源寄存器的命令
volatile uint32_t ad5754_init_power_readback = 0; // 初始化时读回的电源寄存器值
volatile uint16_t ad5754_init_success = 0;        // 初始化成功标志（1=成功，0=失败）

// GPIO25诊断测试变量
volatile uint16_t gpio25_with_pullup = 0;
volatile uint16_t gpio25_without_pullup = 0;
volatile uint16_t gpio25_test_done = 0;

// 电源寄存器测试变量
volatile uint16_t ad5754_power_test_done = 0;       // 电源寄存器测试完成标志
volatile uint32_t ad5754_power_readback = 0;        // 电源寄存器读回值
volatile uint16_t ad5754_power_test_pass = 0;       // 电源寄存器测试通过标志

// DAC寄存器测试变量
volatile uint32_t ad5754_dac_write_value = 0;       // DAC写入值
volatile uint32_t ad5754_dac_readback_value = 0;    // DAC读回值
volatile uint16_t ad5754_dac_test_pass = 0;         // DAC测试通过标志
volatile uint16_t ad5754_dac_test_count = 0;        // DAC测试计数器

//****************************************************************************
// 函数名: Test_GPIO25_InputCapability
// 功能: 测试GPIO25的输入功能（诊断MISO问题）
// 说明: 临时配置GPIO25为普通输入，读取引脚状态
//****************************************************************************
void Test_GPIO25_InputCapability(void)
{
    // 测试1：使能上拉时的状态
    EALLOW;
    GpioCtrlRegs.GPAPUD.bit.GPIO25 = 0;  // 使能上拉
    GpioCtrlRegs.GPAMUX2.bit.GPIO25 = 0;  // 普通GPIO
    GpioCtrlRegs.GPAGMUX2.bit.GPIO25 = 0;
    GpioCtrlRegs.GPADIR.bit.GPIO25 = 0;  // 输入
    EDIS;

    DELAY_US(100);
    gpio25_with_pullup = GpioDataRegs.GPADAT.bit.GPIO25;

    // 测试2：禁用上拉时的状态
    EALLOW;
    GpioCtrlRegs.GPAPUD.bit.GPIO25 = 1;  // 禁用上拉
    EDIS;

    DELAY_US(100);
    gpio25_without_pullup = GpioDataRegs.GPADAT.bit.GPIO25;

    // 恢复SPI功能
    EALLOW;
    GpioCtrlRegs.GPAPUD.bit.GPIO25 = 1;  // 禁用上拉
    GpioCtrlRegs.GPAGMUX2.bit.GPIO25 = 1;
    GpioCtrlRegs.GPAMUX2.bit.GPIO25 = 2;
    EDIS;

    gpio25_test_done = 1;  // 标记测试完成

    // 结果分析（在调试器中查看）：
    // gpio25_with_pullup=1, gpio25_without_pullup=1 → MISO被外部拉高或未连接
    // gpio25_with_pullup=1, gpio25_without_pullup=0 → 正常，上拉导致读到高
    // gpio25_with_pullup=0, gpio25_without_pullup=0 → MISO被接地
}

//****************************************************************************
// 函数名: AD5754R_Init
// 功能: AD5754R初始化（只执行一次）
// 说明: 上电初始化，配置控制寄存器并使能内部基准和DAC通道A
//****************************************************************************
void AD5754R_Init(void)
{
    uint32_t write_cmd, read_cmd, nop_cmd, read_back;
    uint16_t init_value = 0x0011; // PU_REF=1, PU_A=1
    volatile Uint16 dummy;

    // 步骤1：上电延时，让芯片内部电路稳定
    DELAY_US(50000);  // 50ms（增加延时）

    // 步骤2：配置控制寄存器（确保SDO输出使能）
    // REG=011(控制寄存器), A2:A0=001, Data=0x0000
    // DB3=0(TSD禁用), DB2=0(箝位使能), DB1=0(CLR选择=0V), DB0=0(SDO使能)
    write_cmd = (AD5754_WRITE_CMD) |
                ((uint32_t)AD5754_REG_CONTROL << 19) |  // REG位：DB21-19，左移19位
                (0x01UL << 16) |                        // A2:A0位：DB18-16，地址001
                0x0000;                                 // 数据位：DB15-0
    ad5754_init_ctrl_cmd = write_cmd;  // 保存控制寄存器命令（专用变量）
    AD5754_Send24BitCommand(write_cmd);
    DELAY_US(1000);  // 增加延时

    // 步骤3：写入电源控制寄存器（使能内部基准和DAC A）
    // REG=010(电源控制寄存器), A2:A0=000, Data=0x0011
    write_cmd = (AD5754_WRITE_CMD) |
                ((uint32_t)AD5754_REG_POWER << 19) |    // REG位：DB21-19，左移19位
                (0x00UL << 16) |                        // A2:A0位：DB18-16，地址000
                init_value;                             // 数据位：PU_REF=1, PU_A=1
    ad5754_init_power_cmd = write_cmd;  // 保存电源寄存器命令（专用变量）
    AD5754_Send24BitCommand(write_cmd);

    // 步骤4：等待内部基准稳定（关键！）
    DELAY_US(100000);  // 100ms等待基准稳定

    // === 步骤5：验证电源寄存器写入是否成功 ===
    // 清空RX FIFO
    while(SpibRegs.SPIFFRX.bit.RXFFST > 0)
    {
        dummy = SpibRegs.SPIRXBUF;
    }

    // 发送读电源寄存器命令
    read_cmd = (AD5754_READ_CMD) |
               ((uint32_t)AD5754_REG_POWER << 19) |
               (0x00UL << 16);
    AD5754_Send24BitCommand(read_cmd);
    DELAY_US(100);

    // 清空读命令产生的接收数据
    while(SpibRegs.SPIFFRX.bit.RXFFST > 0)
    {
        dummy = SpibRegs.SPIRXBUF;
    }

    // 发送NOP命令读取返回数据
    nop_cmd = (AD5754_WRITE_CMD) |
              ((uint32_t)AD5754_REG_CONTROL << 19) |
              (0x00UL << 16);
    read_back = AD5754_Send24BitCommand_WithRead(nop_cmd);

    // 保存读回的电源寄存器值（专用变量，不会被覆盖）
    ad5754_init_power_readback = read_back;

    // 验证写入是否成功
    // 预期：read_back的低5位应该是0x11（PU_REF=1, PU_A=1）
    if ((read_back & 0x001F) == 0x0011)
    {
        ad5754_init_success = 1;  // 初始化成功
    }
    else
    {
        ad5754_init_success = 0;  // 初始化失败
    }

    ad5754_initialized = 1; // 标记初始化完成
}

//****************************************************************************
// 函数名: Test_AD5754R_Communication
// 功能: 测试AD5754R通信（写入-读取验证）
// 说明: 周期性测试RANGE寄存器的读写功能
//       写入固定值0x0004（±10V范围），验证读回是否正确
//****************************************************************************
void Test_AD5754R_Communication(void)
{
    uint32_t write_cmd, read_cmd, nop_cmd;
    uint32_t read_back_value;
    uint16_t test_value = 0x0004; // 固定写入±10V范围
    volatile Uint16 dummy;

    // 如果未初始化，先执行初始化
    if (!ad5754_initialized)
    {
        AD5754R_Init();
        return;
    }

    // --- 步骤 0: 清空RX FIFO（重要！避免读取到旧数据） ---
    while(SpibRegs.SPIFFRX.bit.RXFFST > 0)
    {
        dummy = SpibRegs.SPIRXBUF; // 读取并丢弃
    }

    // --- 步骤 1: 写入测试值到通道A的输出范围寄存器 ---
    // 格式：R/W=0, REG=001(RANGE), Channel=000(A), Data=0x0004(±10V)
    write_cmd = (AD5754_WRITE_CMD) |
                ((uint32_t)AD5754_REG_RANGE << 19) |    // REG位：DB21-19，左移19位
                ((uint32_t)AD5754_CH_A << 16) |         // A2:A0位：DB18-16，左移16位
                test_value;                              // 数据位：DB15-0
    ad5754_test_write_value = write_cmd;
    AD5754_Send24BitCommand(write_cmd);
    DELAY_US(100); // 等待写入完成

    // --- 步骤 1.5: 清空写入操作产生的接收数据 ---
    while(SpibRegs.SPIFFRX.bit.RXFFST > 0)
    {
        dummy = SpibRegs.SPIRXBUF;
    }

    // --- 步骤 2: 发送读命令以选择要读取的寄存器 ---
    read_cmd = (AD5754_READ_CMD) |
               ((uint32_t)AD5754_REG_RANGE << 19) |     // REG位：DB21-19，左移19位
               ((uint32_t)AD5754_CH_A << 16);           // A2:A0位：DB18-16，左移16位
    AD5754_Send24BitCommand(read_cmd);
    DELAY_US(100);

    // --- 步骤 2.5: 清空读命令产生的接收数据 ---
    while(SpibRegs.SPIFFRX.bit.RXFFST > 0)
    {
        dummy = SpibRegs.SPIRXBUF;
    }

    // --- 步骤 3: 发送NOP命令，并在此时钟周期内读取返回的数据 ---
    // REG=011(控制寄存器), A2:A0=000(NOP), Data=无关
    nop_cmd = (AD5754_WRITE_CMD) |
              ((uint32_t)AD5754_REG_CONTROL << 19) |    // REG位：DB21-19，左移19位
              (0x00UL << 16) |                           // A2:A0=000：NOP命令
              0x0000;                                    // 数据位（无关）
    read_back_value = AD5754_Send24BitCommand_WithRead(nop_cmd);
    ad5754_test_read_value = read_back_value;

    // --- 步骤 4: 验证读取到的值是否与写入的值匹配 ---
    // 只比较低3位（输出范围代码）
    if ((read_back_value & 0x0007) == test_value)
    {
        ad5754_comm_test_pass = 1; // 测试通过
    }
    else
    {
        ad5754_comm_test_pass = 0; // 测试失败
    }
}

//****************************************************************************
// 函数名: Test_AD5754R_WriteTest
// 功能: 测试AD5754R写入功能（设置DAC输出0V）
// 说明: 按照指南要求，设置±10V范围并输出0V
//****************************************************************************
void Test_AD5754R_WriteTest(void)
{
    // 步骤1: 设置通道A输出范围为±10V
    // R/W=0, REG=001(RANGE), Channel=000(A), Range=100(±10V)
    // 命令格式：(0x01 << 19) | (0x00 << 16) | 0x0004 = 0x080004
    AD5754_Send24BitCommand(0x080004);
    DELAY_US(5000);

    // 步骤2: 使能内部基准和DAC A
    // R/W=0, REG=010(POWER), A2:A0=000, Data=0x0011 (PU_REF + PU_A)
    // 命令格式：(0x02 << 19) | (0x00 << 16) | 0x0011 = 0x100011
    AD5754_Send24BitCommand(0x100011);
    DELAY_US(10000);  // 等待上电稳定

    // 步骤3: 设置DAC A输出0V（中间值0x8000）
    // R/W=0, REG=000(DAC), Channel=000(A), Data=0x8000
    // 命令格式：(0x00 << 19) | (0x00 << 16) | 0x8000 = 0x008000
    AD5754_Send24BitCommand(0x008000);
    DELAY_US(5000);

    // 用万用表测量VoutA，应该接近0V
}

//****************************************************************************
// 函数名: Test_AD5754R_PowerSequence
// 功能: 测试AD5754R上电序列（简化版）
// 说明: 按照手册要求的上电初始化序列
//****************************************************************************
void Test_AD5754R_PowerSequence(void)
{
    // 步骤1: 设置通道A为+5V输出范围
    // REG=001(RANGE), Channel=000(A), Range=000(+5V)
    // 命令格式：(0x01 << 19) | (0x00 << 16) | 0x0000 = 0x080000
    AD5754_Send24BitCommand(0x080000);
    DELAY_US(5000);

    // 步骤2: 使能内部基准电压源
    // REG=010(POWER), A2:A0=000, Data=0x0010 (PU_REF=1)
    // 命令格式：(0x02 << 19) | (0x00 << 16) | 0x0010 = 0x100010
    AD5754_Send24BitCommand(0x100010);
    DELAY_US(10000);

    // 步骤3: 使能DAC通道A
    // REG=010(POWER), A2:A0=000, Data=0x0011 (PU_REF=1 + PU_A=1)
    // 命令格式：(0x02 << 19) | (0x00 << 16) | 0x0011 = 0x100011
    AD5754_Send24BitCommand(0x100011);
    DELAY_US(10000);

    // 步骤4: 设置DAC A输出中间值（2.5V在+5V范围内）
    // REG=000(DAC), Channel=000(A), Data=0x8000
    // 命令格式：(0x00 << 19) | (0x00 << 16) | 0x8000 = 0x008000
    AD5754_Send24BitCommand(0x008000);
    DELAY_US(5000);
}


//****************************************************************************
// 函数名: SpibSendByte_Fast
// 功能: 通过SPIB高效发送单个字节
// 参数: byte - 要发送的8位数据
// 说明: 不检查发送完成标志，适用于需要快速连续发送的场景
//****************************************************************************
void SpibSendByte_Fast(Uint16 byte)
{
    // 8位SPI模式：数据必须左对齐
    SpibRegs.SPITXBUF = (byte << 8);
}

//****************************************************************************
// 函数名: SpibSendByte_Continuous
// 功能: 在保持SYNC低电平的情况下连续发送字节
// 参数: byte - 要发送的8位数据
// 说明: 用于多字节命令的中间字节发送，不控制SYNC信号
//       使用FIFO状态位判断传输完成
//****************************************************************************
void SpibSendByte_Continuous(Uint16 byte)
{
    SpibRegs.SPITXBUF = (byte << 8);
    // 等待接收FIFO中有数据（表示传输完成）
    while(SpibRegs.SPIFFRX.bit.RXFFST == 0);
    // 读取接收数据清空FIFO
    volatile Uint16 dummy = SpibRegs.SPIRXBUF;
    (void)dummy; // 避免未使用变量警告
}

//****************************************************************************
// 函数名: AD5754_Send24BitCommand_Simple
// 功能: 发送24位命令到AD5754R（简化版）
// 参数: command - 24位命令数据
// 说明: 手动控制SYNC，连续发送3个8位字节，无超时保护
//****************************************************************************
void AD5754_Send24BitCommand_Simple(uint32_t command)
{
    // 开始SPI通信 - 拉低SYNC
    AD5754_SYNC_LOW();
    DELAY_US(1); // 确保SYNC稳定

    // 发送高字节
    SpibSendByte_Continuous((command >> 16) & 0xFF);
    // 发送中字节
    SpibSendByte_Continuous((command >> 8) & 0xFF);
    // 发送低字节
    SpibSendByte_Continuous(command & 0xFF);

    // 结束SPI通信 - 拉高SYNC
    AD5754_SYNC_HIGH();
    DELAY_US(1); // 确保SYNC稳定
}

//****************************************************************************
// 函数名: SpibSendDataBlock
// 功能: 通过SPIB发送数据块
// 参数: data - 数据数组指针, length - 数据长度（字节数）
// 说明: 适用于发送任意长度的数据块
//****************************************************************************
void SpibSendDataBlock(Uint16 *data, Uint16 length)
{
    Uint16 i;
    // 检查参数有效性
    if(data == NULL || length == 0 || length > 256)  // 添加最大长度检查
    {
        return;
    }

    AD5754_SYNC_LOW();
    DELAY_US(1);

    for(i = 0; i < length; i++)
    {
        SpibSendByte_Continuous(data[i]);
    }

    AD5754_SYNC_HIGH();
    DELAY_US(1);
}

//****************************************************************************
// 函数名: Test_AD5754R_PowerRegister
// 功能: 电源寄存器写入和读回验证（只执行一次）
// 说明:
//   1. 写入电源控制寄存器 = 0x1F (全部上电)
//      发送: 10 00 1F
//   2. 读回验证
//      发送: 90 00 00 (读电源寄存器命令)
//      发送: 18 00 00 (NOP命令获取读回数据)
//      预期: 00 00 1F
//****************************************************************************
void Test_AD5754R_PowerRegister(void)
{
    uint32_t write_cmd, read_cmd, nop_cmd;
    uint32_t read_back_value;
    volatile Uint16 dummy;

    // 如果已经测试过，直接返回
    if (ad5754_power_test_done)
    {
        return;
    }

    // === 步骤1: 清空RX FIFO ===
    while(SpibRegs.SPIFFRX.bit.RXFFST > 0)
    {
        dummy = SpibRegs.SPIRXBUF;
    }

    // === 步骤2: 写入电源控制寄存器 = 0x1F (全部上电) ===
    // 命令: 10 00 1F
    // R/W=0, REG=010(电源控制寄存器), A2:A0=000, Data=0x001F
    write_cmd = 0x10001F;  // 直接使用十六进制格式
    AD5754_Send24BitCommand_FIFO(write_cmd);
    DELAY_US(1000);  // 等待写入完成

    // === 步骤3: 清空写入操作产生的接收数据 ===
    while(SpibRegs.SPIFFRX.bit.RXFFST > 0)
    {
        dummy = SpibRegs.SPIRXBUF;
    }

    // === 步骤4: 发送读电源寄存器命令 ===
    // 命令: 90 00 00
    // R/W=1, REG=010(电源控制寄存器), A2:A0=000
    read_cmd = 0x900000;  // 直接使用十六进制格式
    AD5754_Send24BitCommand_FIFO(read_cmd);
    DELAY_US(100);

    // === 步骤5: 清空读命令产生的接收数据 ===
    while(SpibRegs.SPIFFRX.bit.RXFFST > 0)
    {
        dummy = SpibRegs.SPIRXBUF;
    }

    // === 步骤6: 发送NOP命令并读取返回数据 ===
    // 命令: 18 00 00
    // R/W=0, REG=011(控制寄存器), A2:A0=000 (NOP)
    // ⚠️ 关键：在此传输周期中，芯片会在SDO上输出之前读命令选择的寄存器数据
    nop_cmd = 0x180000;  // 直接使用十六进制格式

    // 使用FIFO批量传输方式（优化版本）
    read_back_value = AD5754_Send24BitCommand_WithRead_FIFO(nop_cmd);

    // 原方式（保留用于对比）
    // read_back_value = AD5754_Send24BitCommand_WithRead(nop_cmd);

    // 保存读回值
    ad5754_power_readback = read_back_value;

    // === 步骤7: 验证读取到的值 ===
    // 预期: 00 00 1F (低字节为0x1F)
    if ((read_back_value & 0x00001F) == 0x1F)
    {
        ad5754_power_test_pass = 1;  // 测试通过
    }
    else
    {
        ad5754_power_test_pass = 0;  // 测试失败
    }

    // 标记测试完成
    ad5754_power_test_done = 1;
}

//****************************************************************************
// 函数名: Test_AD5754R_DACRegister
// 功能: DAC寄存器测试
// 说明:
//   写入DAC A = 0xABCD，读回验证
//****************************************************************************
void Test_AD5754R_DACRegister(void)
{
    uint32_t write_cmd, read_cmd, nop_cmd;
    uint32_t read_back_value;
    uint16_t test_value;
    volatile Uint16 dummy;
    ad5754_dac_test_count ++;
    // 固定测试值
    test_value = 0xABCD;

    // === 步骤1: 清空RX FIFO ===
    while(SpibRegs.SPIFFRX.bit.RXFFST > 0)
    {
        dummy = SpibRegs.SPIRXBUF;
    }

    // === 步骤2: 写入DAC A寄存器 ===
    // 命令格式: 00 XX XX
    // R/W=0, REG=000(DAC寄存器), A2:A0=000(通道A), Data=test_value
    write_cmd = (AD5754_WRITE_CMD) |
                ((uint32_t)AD5754_REG_DAC << 19) |   // REG=000
                ((uint32_t)AD5754_CH_A << 16) |      // Channel A
                test_value;                           // 16位DAC值

    ad5754_dac_write_value = write_cmd;  // 保存写入命令
    AD5754_Send24BitCommand_FIFO(write_cmd);
    DELAY_US(100);

    // === 步骤3: 清空写入操作产生的接收数据 ===
    while(SpibRegs.SPIFFRX.bit.RXFFST > 0)
    {
        dummy = SpibRegs.SPIRXBUF;
    }

    // === 步骤4: 发送读DAC A寄存器命令 ===
    // 命令: 80 00 00
    // R/W=1, REG=000(DAC寄存器), A2:A0=000(通道A)
    read_cmd = 0x800000;  // 直接使用十六进制格式
    AD5754_Send24BitCommand_FIFO(read_cmd);
    DELAY_US(100);

    // === 步骤5: 清空读命令产生的接收数据 ===
    while(SpibRegs.SPIFFRX.bit.RXFFST > 0)
    {
        dummy = SpibRegs.SPIRXBUF;
    }

    // === 步骤6: 发送NOP命令并读取返回数据 ===
    // 命令: 18 00 00
    // R/W=0, REG=011(控制寄存器), A2:A0=000 (NOP)
    // ⚠️ 关键：在此传输周期中，芯片会在SDO上输出之前读命令选择的寄存器数据
    nop_cmd = 0x180000;  // 直接使用十六进制格式

    // 使用FIFO批量传输方式（优化版本）
    read_back_value = AD5754_Send24BitCommand_WithRead_FIFO(nop_cmd);

    // 原方式（保留用于对比）
    // read_back_value = AD5754_Send24BitCommand_WithRead(nop_cmd);

    // 保存读回值
    ad5754_dac_readback_value = read_back_value;

    // === 步骤7: 验证读取到的值 ===
    // 预期: 读回的低16位应该等于0xABCD
    if ((read_back_value & 0xFFFF) == 0xABCD)
    {
        ad5754_dac_test_pass = 1;  // 测试通过
    }
    else
    {
        ad5754_dac_test_pass = 0;  // 测试失败
    }
}

//
// End of file
//


/**
 * @brief  初始化SPIB的DMA相关配置
 */
void Drv_SPIB_DMA_Init(void)
{
    // 当前实现中无需额外初始化步骤，保留函数以兼容既有调用
}

/**
 * @brief  使用DMA进行SPIB数据收发（阻塞带超时）
 * @param  tx_data - 发送数据缓冲区指针
 * @param  rx_data - 接收数据缓冲区指针
 * @param  word_count - 传输的16位字数量
 * @param  timeout_ms - 超时时间（毫秒）
 * @return DMA_ErrorCode_t - 传输结果状态码
 */
DMA_ErrorCode_t Drv_SPIB_TransmitReceive_DMA(Uint16* tx_data, Uint16* rx_data, Uint16 word_count, uint32_t timeout_ms)
{
    Uint16* dma_tx_buf;
    Uint16* dma_rx_buf;
    uint32_t timeout_count = 0;
    uint32_t max_timeout_count = timeout_ms * 200; // 估算超时计数

    // 1. 检查参数
    if (tx_data == NULL || rx_data == NULL || word_count == 0 || word_count > 3) {
        return DMA_ERR_CONFIG;
    }

    // 2. 获取底层DMA缓冲区指针
    dma_tx_buf = Drv_DMA_Get_TX_Buffer();
    dma_rx_buf = Drv_DMA_Get_RX_Buffer();

    // 3. ⭐ 强制复位RX FIFO（确保彻底清空深层残留数据）
    SpibRegs.SPIFFRX.bit.RXFIFORESET = 0;  // 复位FIFO
    DELAY_US(1);                            // 短暂延时确保复位生效
    SpibRegs.SPIFFRX.bit.RXFIFORESET = 1;  // 释放复位

    // 4. 清空RX FIFO（双重保险：复位后再手动清空）
    while(SpibRegs.SPIFFRX.bit.RXFFST != 0) {
        volatile Uint16 dummy = SpibRegs.SPIRXBUF;
        (void)dummy; // 避免编译器警告
    }

    // 5. 准备发送数据
    memcpy(dma_tx_buf, tx_data, word_count * sizeof(Uint16));

    // 6. 拉低#SYNC，启动DMA传输
    AD5754_SYNC_LOW();
    
    // 启动DMA通道（按RX->TX顺序，确保接收通道先准备好）
    EALLOW;
    DmaRegs.CH6.CONTROL.bit.RUN = 1;  // 启动RX通道
    DmaRegs.CH5.CONTROL.bit.RUN = 1;  // 启动TX通道
    EDIS;

    // 7. 等待DMA完成（带超时）
    while (Drv_DMA_Get_State() == DMA_STATE_BUSY) {
        if (timeout_count++ > max_timeout_count) {
            // 超时后，尝试停止DMA并返回错误
            // 注意：此时#SYNC可能已在ISR中被拉高，也可能没有
            AD5754_SYNC_HIGH();
            return DMA_ERR_TIMEOUT;
        }
        DELAY_US(5);
    }

    // 8. DMA传输完成后恢复#SYNC
    AD5754_SYNC_HIGH();

    // 9. 检查最终状态
    if (Drv_DMA_Get_State() == DMA_STATE_ERROR) {
        return DMA_ERR_HARDWARE;
    }

    // 10. 传输成功，复制接收到的数据
    memcpy(rx_data, dma_rx_buf, word_count * sizeof(Uint16));

    return DMA_ERR_NONE;
}


//=============================================================================
// AD5754R DMA专用函数 (上层应用驱动)
//=============================================================================
extern Uint16 DMA_Tx_Buffer[3];
extern Uint16 DMA_Rx_Buffer[3];
/**
 * @brief  使用DMA发送一个24位命令 (阻塞)
 * @param  command - 24位命令字
 * @param  timeout_ms - 超时时间
 * @return bool - true:成功, false:失败
 */
bool AD5754_SendCommand_DMA(uint32_t command, uint32_t timeout_ms)
{
    Uint16 tx_buffer[3];
    Uint16 rx_buffer[3]; // 虚拟接收缓冲
    Uint16 status;
    // 关键：将24位命令打包到3个16位字中，并左移8位以适应8位SPI模式
    DMA_Tx_Buffer[0] = ((command >> 16) & 0xFF) << 8;
    DMA_Tx_Buffer[1] = ((command >> 8) & 0xFF) << 8;
    DMA_Tx_Buffer[2] = (command & 0xFF) << 8;
    if(DmaRegs.CH5.TRANSFER_COUNT == 0)
    {
        EALLOW;
        DmaRegs.CH5.CONTROL.bit.RUN = 1;      // Start DMA Transmit from McBSP-A
        EDIS;
    }
    DELAY_US(1000);
    return (status == DMA_ERR_NONE);
}

/**
 * @brief  使用DMA发送24位命令并读取返回的数据 (阻塞)
 * @param  command - 24位命令字
 * @param  response - 用于存储24位返回值的指针
 * @param  timeout_ms - 超时时间
 * @return bool - true:成功, false:失败
 */
bool AD5754_ReadCommand_DMA(uint32_t command, uint32_t* response, uint32_t timeout_ms)
{
    Uint16 tx_buffer[3];
    Uint16 rx_buffer[3];
    Uint16 status;
    // 准备发送命令
    DMA_Tx_Buffer[0] = ((command >> 16) & 0xFF) << 8;
    DMA_Tx_Buffer[1] = ((command >> 8) & 0xFF) << 8;
    DMA_Tx_Buffer[2] = (command & 0xFF) << 8;
    if(DmaRegs.CH5.TRANSFER_COUNT == 0)
    {
        EALLOW;
        DmaRegs.CH5.CONTROL.bit.RUN = 1;      // Start DMA Transmit from McBSP-A
        EDIS;
    }

    if (status == DMA_ERR_NONE)
    {
        // ⭐ 保存DMA接收到的原始数据（用于调试）
        debug_dma_rx_word1 = DMA_Rx_Buffer[0];
        debug_dma_rx_word2 = DMA_Rx_Buffer[1];
        debug_dma_rx_word3 = DMA_Rx_Buffer[2];
        
        // ✅ 修正：8位SPI的接收数据在16位字的低8位（与FIFO模式一致）
        *response = (((Uint32)(DMA_Rx_Buffer[0] & 0xFF)) << 16) |
                    (((Uint32)(DMA_Rx_Buffer[1] & 0xFF)) << 8) |
                    ((Uint32)(DMA_Rx_Buffer[2] & 0xFF));
        
        // ⭐ 保存解析后的结果（用于调试）
        debug_dma_result = *response;
        
        return true;
    }

    return false;
}

/**
 * @brief  使用DMA模式测试DAC寄存器的读写
 */
void Test_AD5754R_DACRegister_DMA(void)
{
    uint32_t write_cmd, read_cmd, nop_cmd;
    uint32_t read_back_value;
    uint16_t test_value = 0xABCD;

    ad5754_dac_test_count++;

    // === 步骤1: 写入DAC A寄存器 ===
    write_cmd = (AD5754_WRITE_CMD) |
                ((uint32_t)AD5754_REG_DAC << 19) |
                ((uint32_t)AD5754_CH_A << 16) |
                test_value;
    ad5754_dac_write_value = write_cmd; // 保存写入命令
    AD5754_SendCommand_DMA(write_cmd, 10); // 使用DMA发送

    DELAY_US(100);

    // === 步骤2: 发送读DAC A寄存器命令 ===
    read_cmd = 0x800000; // R/W=1, REG=000(DAC), A=000(A)
    AD5754_SendCommand_DMA(read_cmd, 10);

    DELAY_US(100);

    // === 步骤3: 发送NOP命令并读取返回数据 ===
    nop_cmd = 0x180000; // NOP命令
    if (AD5754_ReadCommand_DMA(nop_cmd, &read_back_value, 10))
    {
        ad5754_dac_readback_value = read_back_value;

        // === 步骤4: 验证读取到的值 ===
        if ((read_back_value & 0xFFFF) == test_value)
        {
            ad5754_dac_test_pass = 1; // 测试通过
        }
        else
        {
            ad5754_dac_test_pass = 0; // 测试失败
        }
    }
    else
    {
        ad5754_dac_test_pass = 0; // 读取失败
    }
}
