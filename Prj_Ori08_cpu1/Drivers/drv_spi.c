//****************************************************************************
//
// 文件名: drv_spi.c
//
// 功能说明: SPIA与SPIB通信功能驱动
//
//****************************************************************************

#include "F28x_Project.h"
#include "drv_spi.h"

//****************************************************************************
// 函数名: InitSpiGpios
// 功能: 初始化SPIA和SPIB的GPIO引脚
//****************************************************************************
void InitSpiGpios(void)
{
    EALLOW;

    // SPIA pins
    GPIO_SetupPinMux(54, GPIO_MUX_CPU1, 1); // SPICLKA
    GPIO_SetupPinMux(55, GPIO_MUX_CPU1, 1); // SPISIMOA
    GPIO_SetupPinMux(56, GPIO_MUX_CPU1, 1); // SPISOMIA
    GPIO_SetupPinMux(57, GPIO_MUX_CPU1, 1); // SPISTEA

    // SPIB pins
    GPIO_SetupPinMux(24, GPIO_MUX_CPU1, 6); // SPISIMOB
    GPIO_SetupPinMux(25, GPIO_MUX_CPU1, 6); // SPISOMIB
    GPIO_SetupPinMux(26, GPIO_MUX_CPU1, 6); // SPICLKB
    GPIO_SetupPinMux(27, GPIO_MUX_CPU1, 6); // SPISTEB

    EDIS;
}

//****************************************************************************
// 函数名: InitSpiInterrupts
// 功能: 初始化SPI相关的PIE中断
//****************************************************************************
// ISR 函数外部声明
extern interrupt void cpu_timer0_isr(void);
extern interrupt void spiARxISR(void);
extern interrupt void spiBRxISR(void);

//****************************************************************************
// 函数名: InitSpiInterrupts
// 功能: 初始化SPI相关的PIE中断
//****************************************************************************
void InitSpiInterrupts(void)
{
    EALLOW;
    PieVectTable.TIMER0_INT = &cpu_timer0_isr;
    PieVectTable.SPIA_RX_INT = &spiARxISR;
    PieVectTable.SPIB_RX_INT = &spiBRxISR;
    EDIS;

    // 使能PIE中断
    PieCtrlRegs.PIEIER1.bit.INTx7 = 1; // CPU Timer 0
    PieCtrlRegs.PIEIER6.bit.INTx1 = 1; // SPIA RX
    PieCtrlRegs.PIEIER6.bit.INTx3 = 1; // SPIB RX
}

//****************************************************************************
// 函数名: InitSpiModules
// 功能: 初始化SPIA和SPIB外设模块
//****************************************************************************
void InitSpiModules(void)
{
    // Enable SPI clocks
    EALLOW;
    CpuSysRegs.PCLKCR8.bit.SPI_A = 1;
    CpuSysRegs.PCLKCR8.bit.SPI_B = 1;
    EDIS;

    // SPIB (Slave) configuration
    SpibRegs.SPICCR.bit.SPISWRESET = 0;
    SpibRegs.SPICCR.all = 0x000F;
    SpibRegs.SPICTL.all = 0x000A;
    SpibRegs.SPIFFTX.all = 0x0000;
    SpibRegs.SPIFFRX.all = 0x0000;
    SpibRegs.SPIFFCT.all = 0x0000;
    SpibRegs.SPIPRI.all = 0x0010;

    // Correct FIFO Initialization Sequence for SPIB
    SpibRegs.SPIFFTX.bit.SPIFFENA = 1;
    SpibRegs.SPIFFTX.bit.TXFIFO = 1;
    SpibRegs.SPIFFRX.bit.RXFIFORESET = 1;
    SpibRegs.SPIFFRX.bit.RXFFIENA = 1;
    SpibRegs.SPIFFRX.bit.RXFFIL = 4;
    SpibRegs.SPIFFTX.bit.TXFIFO = 0;
    SpibRegs.SPIFFRX.bit.RXFIFORESET = 0;
    SpibRegs.SPIFFTX.bit.SPIRST = 1;
    SpibRegs.SPICCR.bit.SPISWRESET = 1;

    // SPIA (Master) configuration
    SpiaRegs.SPICCR.bit.SPISWRESET = 0;
    SpiaRegs.SPICCR.all = 0x000F;
    SpiaRegs.SPICTL.all = 0x000E;
    SpiaRegs.SPIBRR.bit.SPI_BIT_RATE = 24;
    SpiaRegs.SPIFFTX.all = 0x0000;
    SpiaRegs.SPIFFRX.all = 0x0000;
    SpiaRegs.SPIFFCT.all = 0x0000;
    SpiaRegs.SPIPRI.all = 0x0010;

    // Correct FIFO Initialization Sequence for SPIA
    SpiaRegs.SPIFFTX.bit.SPIFFENA = 1;
    SpiaRegs.SPIFFTX.bit.TXFIFO = 1;
    SpiaRegs.SPIFFRX.bit.RXFIFORESET = 1;
    SpiaRegs.SPIFFRX.bit.RXFFIENA = 1;
    SpiaRegs.SPIFFRX.bit.RXFFIL = 4;
    SpiaRegs.SPIFFTX.bit.TXFIFO = 0;
    SpiaRegs.SPIFFRX.bit.RXFIFORESET = 0;
    SpiaRegs.SPIFFTX.bit.SPIRST = 1;
    SpiaRegs.SPICCR.bit.SPISWRESET = 1;
}



