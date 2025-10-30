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
// EnableInterrupts - This function enables the PIE module and CPU __interrupts
//
void EnableInterrupts()
{
    //
    // Enable the PIE
    //
    PieCtrlRegs.PIECTRL.bit.ENPIE = 1;

    // 使能CPU中断组
    IER |= M_INT1;   // PIE组1（TIMER0中断）
    IER |= M_INT9;   //
    // 使能PIE中断
    PieCtrlRegs.PIEIER1.bit.INTx7 = 1;   // TIMER0 (PIE1.7)
    PieCtrlRegs.PIEIER9.bit.INTx1 = 1;  // Enable all SCIA RXINT interrupt
    PieCtrlRegs.PIEIER9.bit.INTx2 = 1;  // Enable all SCIA TXINT interrupt
    //
//  PieCtrlRegs.PIEACK.all = 0xFFFF;

    //
    // Enable Interrupts at the CPU level
    //
    EINT;
}

//
// End of file
//



