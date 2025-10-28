//###########################################################################
//
// FILE:	F2837xD_PieCtrl.c
//
// TITLE:	F2837xD Device PIE Control Register Initialization Functions.
//
//###########################################################################
// $TI Release: F2837xD Support Library v190 $
// $Release Date: Mon Feb  1 16:51:57 CST 2016 $
// $Copyright: Copyright (C) 2013-2016 Texas Instruments Incorporated -
//             http://www.ti.com/ ALL RIGHTS RESERVED $
//###########################################################################

#include "F2837xD_device.h"     // F2837xD Headerfile Include File
#include "F2837xD_Examples.h"   // F2837xD Examples Include File

//---------------------------------------------------------------------------
// InitPieCtrl: 
//---------------------------------------------------------------------------
// This function initializes the PIE control registers to a known state.
//
void InitPieCtrl(void)
{
    // Disable Interrupts at the CPU level:
    DINT;

    // Disable the PIE
    PieCtrlRegs.PIECTRL.bit.ENPIE = 0;

	// Clear all PIEIER registers:
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

	// Clear all PIEIFR registers:
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

//---------------------------------------------------------------------------
// EnableInterrupts: 
//---------------------------------------------------------------------------
// This function enables the PIE module and CPU __interrupts
//
void EnableInterrupts()
{

    // Enable the PIE
    PieCtrlRegs.PIECTRL.bit.ENPIE = 1;
    		
    // Enable CPU INT1 which is connected to CPU-Timer 0:
    IER |= M_INT1;
    IER |= M_INT8;
    IER |= M_INT9;
    IER |= M_INT13;
    IER |= M_INT14;
    // Enable TINT0 in the PIE: Group 1 interrupt 7
    PieCtrlRegs.PIEIER1.bit.INTx7 = 1;
    PieCtrlRegs.PIEIER1.bit.INTx13 = 1;   // CPU1 to CPU2 INT0
    PieCtrlRegs.PIEIER1.bit.INTx14 = 1;   // CPU1 to CPU2 INT0
    PieCtrlRegs.PIEIER1.bit.INTx4 = 1;          // Enable PIE Group 1 INT4
    PieCtrlRegs.PIEIER1.bit.INTx5 = 1;          // Enable PIE Group 1 INT5

    PieCtrlRegs.PIEIER9.bit.INTx1 = 1;	// Enable all SCIA RXINT interrupt
    PieCtrlRegs.PIEIER9.bit.INTx2 = 1;	// Enable all SCIA TXINT interrupt
    PieCtrlRegs.PIEIER9.bit.INTx3 = 1;	// Enable all SCIB RXINT interrupt
    PieCtrlRegs.PIEIER9.bit.INTx4 = 1;	// Enable all SCIB TXINT interrupt
    PieCtrlRegs.PIEIER8.bit.INTx5 = 1;  // Enable all SCIC RXINT interrupt
    PieCtrlRegs.PIEIER8.bit.INTx6 = 1;  // Enable all SCIC TXINT interrupt
    PieCtrlRegs.PIEIER8.bit.INTx7 = 1;  // Enable all SCID RXINT interrupt
    PieCtrlRegs.PIEIER8.bit.INTx8 = 1;  // Enable all SCID TXINT interrupt

    PieCtrlRegs.PIEIER9.bit.INTx5 = 0;	// Enable all CANA INT0 interrupt
    // Enable global Interrupts and higher priority real-time debug events:
    EINT;   // Enable Global interrupt INTM
    ERTM;   // Enable Global realtime interrupt DBGM


}
//===========================================================================
// End of file.
//===========================================================================
