/*
 * PieCtrl.c
 *
 *  Created on: 2022年8月12日
 *      Author: 110
 */

//###########################################################################
//
// FILE:    F2837xD_PieCtrl.c
//
// TITLE:   F2837xD Device PIE Control Register Initialization Functions.
//
//###########################################################################
// $TI Release: F2837xD Support Library v210 $
// $Release Date: Tue Nov  1 14:46:15 CDT 2016 $
// $Copyright: Copyright (C) 2013-2016 Texas Instruments Incorporated -
//             http://www.ti.com/ ALL RIGHTS RESERVED $
//###########################################################################

//
// Included Files
//
#include "F2837xD_device.h"     // F2837xD Headerfile Include File
#include "F2837xD_Examples.h"   // F2837xD Examples Include File

//
// InitPieCtrl - This function initializes the PIE control registers to a
//               known state.
//
void InitPieCtrl(void)
{
    //
    // Disable Interrupts at the CPU level:
    //
    DINT;

    //
    // Disable the PIE
    //
    PieCtrlRegs.PIECTRL.bit.ENPIE = 0;

    //
    // Clear all PIEIER registers:
    //
    PieCtrlRegs.PIEIER1.all = 0;
    PieCtrlRegs.PIEIER2.all = 0;
    PieCtrlRegs.PIEIER3.all = 0;
    PieCtrlRegs.PIEIER4.all = 0;
    PieCtrlRegs.PIEIER5.all = 0;
    PieCtrlRegs.PIEIER6.all = 0;
    PieCtrlRegs.PIEIER7.all = 0;
    PieCtrlRegs.PIEIER8.all = 0;
    PieCtrlRegs.PIEIER9.all = 0;
    PieCtrlRegs.PIEIER10.all = 0;
    PieCtrlRegs.PIEIER11.all = 0;
    PieCtrlRegs.PIEIER12.all = 0;

    //
    // Clear all PIEIFR registers:
    //
    PieCtrlRegs.PIEIFR1.all = 0;
    PieCtrlRegs.PIEIFR2.all = 0;
    PieCtrlRegs.PIEIFR3.all = 0;
    PieCtrlRegs.PIEIFR4.all = 0;
    PieCtrlRegs.PIEIFR5.all = 0;
    PieCtrlRegs.PIEIFR6.all = 0;
    PieCtrlRegs.PIEIFR7.all = 0;
    PieCtrlRegs.PIEIFR8.all = 0;
    PieCtrlRegs.PIEIFR9.all = 0;
    PieCtrlRegs.PIEIFR10.all = 0;
    PieCtrlRegs.PIEIFR11.all = 0;
    PieCtrlRegs.PIEIFR12.all = 0;
}

//
// EnableInterrupts - This function enables the PIE module and CPU interrupts
//
// 说明：这是一个通用函数，只负责全局使能中断。
//       具体的中断组（IER）和中断位（PIEIER）应由应用代码在调用此函数前配置。
//
void EnableInterrupts()
{
    //
    // Enable the PIE
    //
    PieCtrlRegs.PIECTRL.bit.ENPIE = 1;
    IER |= M_INT1 | M_INT6 | M_INT7; // 使能CPU中断组1(Timer0), 6(SPI), 7(DMA)



    // 使能PIE中断
    PieCtrlRegs.PIEIER1.bit.INTx7 = 1; // CPU Timer 0
    PieCtrlRegs.PIEIER6.bit.INTx1 = 1; // SPIA RX
    PieCtrlRegs.PIEIER7.bit.INTx6 = 1; // 使能DMA CH6中断
    PieCtrlRegs.PIEIER7.bit.INTx5 = 1; // 使能DMA CH5中断
    //
    // Clear all PIE acknowledge bits (optional, ensures clean state)
    //
    PieCtrlRegs.PIEACK.all = 0xFFFF;

    //
    // Enable Interrupts at the CPU level
    //
    EINT;   // Enable global interrupts
    ERTM;   // Enable global realtime interrupt DBGM
}

//
// End of file
//



