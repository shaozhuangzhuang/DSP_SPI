/*******************************************************************************
 * 文件名：drv_spi.c
 * 描述：  SPI驱动实现 - 支持SPIA/SPIB主模式DMA传输
 * 作者：  Auto-generated
 * 日期：  2025-10-20
 ******************************************************************************/

#include "drv_spi.h"
#include "F28x_Project.h"

/******************************************************************************
 * 全局变量定义
 ******************************************************************************/

// SPI寄存器基地址指针数组
volatile struct SPI_REGS *g_spi_regs[3] = {
    &SpiaRegs,   // SPI_MODULE_A
    &SpibRegs,   // SPI_MODULE_B
    &SpicRegs    // SPI_MODULE_C
};

/******************************************************************************
 * GPIO配置函数
 ******************************************************************************/

/**
 * @brief  SPI GPIO引脚配置（所有SPI模块）
 */
void Drv_SPI_GPIO_Config(void)
{
    EALLOW;
    
    //==========================================================================
    // SPIA配置 (GPIO54-56) - 连接ADS1278
    // GPIO54: SPISIMOA (MOSI - 主机输出)
    // GPIO55: 与ADS1278的DRDY连接,外部中断引脚
    // GPIO56: SPICLKA (时钟)
    //==========================================================================
    
    // GPIO54: SPISIMOA (MOSI - 主机输出)
    GPIO_SetupPinMux(54, GPIO_MUX_CPU1, 6);
    GPIO_SetupPinOptions(54, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioCtrlRegs.GPBPUD.bit.GPIO54 = 0;    // 使能上拉
    GpioCtrlRegs.GPBQSEL2.bit.GPIO54 = 3;  // 同步采样

    
    // GPIO56: SPICLKA (时钟)
    GPIO_SetupPinMux(56, GPIO_MUX_CPU1, 6);
    GPIO_SetupPinOptions(56, GPIO_OUTPUT, GPIO_PUSHPULL);

    
    //==========================================================================
    // SPIB配置 (GPIO24-27) - 连接AD5754
    //==========================================================================
    
    // GPIO24: SPISIMOB (MOSI - 主机输出)
    GPIO_SetupPinMux(24, GPIO_MUX_CPU1, 6);
    GPIO_SetupPinOptions(24, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioCtrlRegs.GPAPUD.bit.GPIO24 = 0;    // 使能上拉
    GpioCtrlRegs.GPAQSEL2.bit.GPIO24 = 3;  // 同步采样
    
    // GPIO25: SPISOMIB (MISO - 主机输入)
    GPIO_SetupPinMux(25, GPIO_MUX_CPU1, 6);
    GPIO_SetupPinOptions(25, GPIO_INPUT, GPIO_PUSHPULL);
    GpioCtrlRegs.GPAPUD.bit.GPIO25 = 0;    // 使能上拉
    GpioCtrlRegs.GPAQSEL2.bit.GPIO25 = 3;  // 同步采样
    
    // GPIO26: SPICLKB (时钟)
    GPIO_SetupPinMux(26, GPIO_MUX_CPU1, 6);
    GPIO_SetupPinOptions(26, GPIO_OUTPUT, GPIO_PUSHPULL);
    
    // GPIO27: CS片选 (GPIO模式，手动控制)
    GPIO_SetupPinMux(27, GPIO_MUX_CPU1, 0);  // GPIO模式
    GPIO_SetupPinOptions(27, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPASET.bit.GPIO27 = 1;      // 初始高电平（未选中）
    
    EDIS;
}

/******************************************************************************
 * 内部辅助函数
 ******************************************************************************/

// Drv_SPI_CalcBRR函数已在drv_spi.h中定义为static inline

/******************************************************************************
 * SPI初始化函数
 ******************************************************************************/

/**
 * @brief  SPI模块通用初始化
 */
void Drv_SPI_Init(SPI_Config_t *config)
{
    volatile struct SPI_REGS *spi = g_spi_regs[config->module];
    Uint16 brr;
    
    // 使能SPI时钟
    EALLOW;
    if(config->module == SPI_MODULE_A) {
        CpuSysRegs.PCLKCR8.bit.SPI_A = 1;
    }
    else if(config->module == SPI_MODULE_B) {
        CpuSysRegs.PCLKCR8.bit.SPI_B = 1;
    }
    else if(config->module == SPI_MODULE_C) {
        CpuSysRegs.PCLKCR8.bit.SPI_C = 1;
    }
    EDIS;
    
    // 复位SPI
    spi->SPICCR.bit.SPISWRESET = 0;  // 进入复位状态
    
    // 计算波特率
    brr = Drv_SPI_CalcBRR(config->clock_hz);
    if(brr > 127) brr = 127;  // 限制最大值
    if(brr < 3) brr = 3;      // 限制最小值（确保稳定）
    
    //==========================================================================
    // SPICCR: SPI配置控制寄存器
    //==========================================================================
    spi->SPICCR.all = 0x0000;
    spi->SPICCR.bit.SPICHAR = (config->data_width - 1);  // 数据位宽
    spi->SPICCR.bit.SPILBK = 0;                          // 禁用回环模式
    spi->SPICCR.bit.CLKPOLARITY = (config->mode >> 1);   // CPOL位
    
    //==========================================================================
    // SPICTL: SPI操作控制寄存器
    //==========================================================================
    spi->SPICTL.all = 0x0000;
    spi->SPICTL.bit.MASTER_SLAVE = 1;                    // 主模式
    spi->SPICTL.bit.CLK_PHASE = (config->mode & 0x01);   // CPHA位
    spi->SPICTL.bit.TALK = 1;                            // 使能发送
    spi->SPICTL.bit.SPIINTENA = 0;                       // 禁用SPI中断
    
    //==========================================================================
    // SPIBRR: 波特率寄存器
    //==========================================================================
    spi->SPIBRR.bit.SPI_BIT_RATE = brr;
    
    //==========================================================================
    // SPIFFTX: FIFO发送控制寄存器
    //==========================================================================
    if(config->fifo_enable) {
        spi->SPIFFTX.all = 0xC000;                       // 复位FIFO，增强模式
        spi->SPIFFTX.bit.SPIFFENA = 1;                   // 使能FIFO
        spi->SPIFFTX.bit.TXFIFO = 1;                     // 释放TX FIFO
        spi->SPIFFTX.bit.TXFFIENA = 0;                   // 禁用TX FIFO中断
        spi->SPIFFTX.bit.TXFFINTCLR = 1;                 // 清中断标志
        spi->SPIFFTX.bit.TXFFIL = config->tx_fifo_level; // TX FIFO触发电平
    }
    else {
        spi->SPIFFTX.bit.SPIFFENA = 0;                   // 禁用FIFO
    }
    
    //==========================================================================
    // SPIFFRX: FIFO接收控制寄存器
    //==========================================================================
    if(config->fifo_enable) {
        spi->SPIFFRX.all = 0x0000;
        spi->SPIFFRX.bit.RXFIFORESET = 1;                // 释放RX FIFO
        spi->SPIFFRX.bit.RXFFIENA = 0;                   // 禁用RX FIFO中断
        spi->SPIFFRX.bit.RXFFINTCLR = 1;                 // 清中断标志
        spi->SPIFFRX.bit.RXFFIL = config->rx_fifo_level; // RX FIFO触发电平
        spi->SPIFFRX.bit.RXFFOVFCLR = 1;                 // 清溢出标志
    }
    
    //==========================================================================
    // SPIFFCT: FIFO控制寄存器
    //==========================================================================
    spi->SPIFFCT.all = 0x0000;  // FIFO传输延迟=0
    
    //==========================================================================
    // SPIPRI: 优先级控制寄存器
    //==========================================================================
    spi->SPIPRI.all = 0x0000;
    spi->SPIPRI.bit.FREE = 1;   // 调试模式下继续运行
    
    // 释放SPI复位
    spi->SPICCR.bit.SPISWRESET = 1;
    
    // 短暂延迟，确保SPI稳定
    DELAY_US(10);
}

/**
 * @brief  SPIA初始化（ADS1278专用）
 */
void Drv_SPIA_Init(void)
{
    SPI_Config_t config;
    
    config.module = SPI_MODULE_A;
    config.clock_hz = SPIA_CLOCK_HZ;
    config.mode = SPI_MODE_0;           // CPOL=0, CPHA=0 (ADS1278要求，上升沿采样)
    config.data_width = 16;             // 16位字长
    config.fifo_enable = true;
    config.tx_fifo_level = SPIA_TX_FIFO_LEVEL;  // 0: FIFO空时触发
    config.rx_fifo_level = SPIA_RX_FIFO_LEVEL;  // 12: FIFO有12字时触发
    
    Drv_SPI_Init(&config);
}

/**
 * @brief  SPIB初始化（AD5754专用）
 */
void Drv_SPIB_Init(void)
{
    SPI_Config_t config;
    
    config.module = SPI_MODULE_B;
    config.clock_hz = SPIB_CLOCK_HZ;
    config.mode = SPI_MODE_1;           // CPOL=0, CPHA=1 (也可用MODE_3)
    config.data_width = 16;             // 16位字长
    config.fifo_enable = true;
    config.tx_fifo_level = SPIB_TX_FIFO_LEVEL;  // 0: FIFO空时触发
    config.rx_fifo_level = SPIB_RX_FIFO_LEVEL;  // 6: FIFO有6字时触发
    
    Drv_SPI_Init(&config);
}

/******************************************************************************
 * SPI控制函数
 ******************************************************************************/

/**
 * @brief  设置SPI片选信号
 */
void Drv_SPI_SetCS(SPI_Module_t module, bool active)
{
    if(module == SPI_MODULE_A) {
        // SPIA使用GPIO57
        if(active) {
            GpioDataRegs.GPBCLEAR.bit.GPIO57 = 1;  // 拉低CS（选中）
            DELAY_US(1);  // tCSS延迟（确保建立时间）
        }
        else {
            DELAY_US(1);  // tCSH延迟（确保保持时间）
            GpioDataRegs.GPBSET.bit.GPIO57 = 1;    // 拉高CS（未选中）
        }
    }
    else if(module == SPI_MODULE_B) {
        // SPIB使用GPIO27
        if(active) {
            GpioDataRegs.GPACLEAR.bit.GPIO27 = 1;  // 拉低CS（选中）
            DELAY_US(1);  // tCSS延迟
        }
        else {
            DELAY_US(1);  // tCSH延迟
            GpioDataRegs.GPASET.bit.GPIO27 = 1;    // 拉高CS（未选中）
        }
    }
}

/**
 * @brief  清空SPI FIFO
 */
void Drv_SPI_ClearFIFO(SPI_Module_t module)
{
    volatile struct SPI_REGS *spi = g_spi_regs[module];
    
    // 复位TX FIFO
    spi->SPIFFTX.bit.TXFIFO = 0;
    spi->SPIFFTX.bit.TXFIFO = 1;
    spi->SPIFFTX.bit.TXFFINTCLR = 1;  // 清中断标志
    
    // 复位RX FIFO
    spi->SPIFFRX.bit.RXFIFORESET = 0;
    spi->SPIFFRX.bit.RXFIFORESET = 1;
    spi->SPIFFRX.bit.RXFFINTCLR = 1;  // 清中断标志
    spi->SPIFFRX.bit.RXFFOVFCLR = 1;  // 清溢出标志
}

/**
 * @brief  使能SPI DMA模式
 */
void Drv_SPI_EnableDMA(SPI_Module_t module)
{
    volatile struct SPI_REGS *spi = g_spi_regs[module];
    
    // 禁用所有SPI中断
    spi->SPICTL.bit.SPIINTENA = 0;
    spi->SPIFFTX.bit.TXFFIENA = 0;
    spi->SPIFFRX.bit.RXFFIENA = 0;
    
    // DMA模式下，FIFO触发由DMA使用
    // RX FIFO触发电平已在初始化时设置
    // TX FIFO触发电平已在初始化时设置
}

/**
 * @brief  禁用SPI模块
 */
void Drv_SPI_Disable(SPI_Module_t module)
{
    volatile struct SPI_REGS *spi = g_spi_regs[module];
    spi->SPICCR.bit.SPISWRESET = 0;  // 进入复位状态
}

/**
 * @brief  启用SPI模块
 */
void Drv_SPI_Enable(SPI_Module_t module)
{
    volatile struct SPI_REGS *spi = g_spi_regs[module];
    spi->SPICCR.bit.SPISWRESET = 1;  // 释放复位
}

/**
 * @brief  获取SPI FIFO状态
 */
void Drv_SPI_GetFIFOStatus(SPI_Module_t module, Uint16 *tx_fifo_level, Uint16 *rx_fifo_level)
{
    volatile struct SPI_REGS *spi = g_spi_regs[module];
    
    if(tx_fifo_level != NULL) {
        *tx_fifo_level = spi->SPIFFTX.bit.TXFFST;  // TX FIFO当前字数
    }
    
    if(rx_fifo_level != NULL) {
        *rx_fifo_level = spi->SPIFFRX.bit.RXFFST;  // RX FIFO当前字数
    }
}

