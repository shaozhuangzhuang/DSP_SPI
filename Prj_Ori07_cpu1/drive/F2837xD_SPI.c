/*
 * F2837xD_SPI.c
 *
 *  Created on: 2022Äê8ÔÂ10ÈÕ
 *      Author: 110
 */


//###########################################################################
//
// FILE:   F2837xD_Spi.c
//
// TITLE:  F2837xD SPI Initialization & Support Functions.
//
//###########################################################################
// $TI Release: F2837xD Support Library v180 $
// $Release Date: Fri Nov  6 16:19:46 CST 2015 $
// $Copyright: Copyright (C) 2013-2015 Texas Instruments Incorporated -
//             http://www.ti.com/ ALL RIGHTS RESERVED $
//###########################################################################

#include "F2837xD_device.h"     // F2837xD Headerfile Include File
#include "F2837xD_Examples.h"   // F2837xD Examples Include File

//---------------------------------------------------------------------------
// Example: InitSpiGpio:
//---------------------------------------------------------------------------
// This function initializes GPIO pins to function as SPI pins

void InitSpiGpio(void)
{
      InitSpiaGpio();
}

void InitSpi()
{
    InitSpia();
}

void InitSpiaGpio()
{

   EALLOW;

   /* Enable internal pull-up for the selected pins */
   // Pull-ups can be enabled or disabled by the user.
   // This will enable the pullups for the specified pins.
   // Comment out other unwanted lines.

   GpioCtrlRegs.GPAPUD.bit.GPIO16 = 0;   // Enable pull-up on GPIO63 (SPISIMOA)
   GpioCtrlRegs.GPAPUD.bit.GPIO17 = 0;   // Enable pull-up on GPIO64 (SPISOMIA)
   GpioCtrlRegs.GPAPUD.bit.GPIO18 = 0;   // Enable pull-up on GPIO65 (SPICLKA)
//   GpioCtrlRegs.GPAPUD.bit.GPIO19 = 0;   // Enable pull-up on GPIO66 (SPISTEA)

   /* Set qualification for selected pins to asynch only */
   // This will select asynch (no qualification) for the selected pins.
   // Comment out other unwanted lines.

   GpioCtrlRegs.GPAQSEL2.bit.GPIO16 = 3; // Asynch input GPIO63 (SPISIMOA)
   GpioCtrlRegs.GPAQSEL2.bit.GPIO17 = 3; // Asynch input GPIO64 (SPISOMIA)
   GpioCtrlRegs.GPAQSEL2.bit.GPIO18 = 3; // Asynch input GPIO65 (SPICLKA)
//   GpioCtrlRegs.GPAQSEL2.bit.GPIO19 = 3; // Asynch input GPIO66 (SPISTEA)

   /* Configure SPI-A pins using GPIO regs*/
   // This specifies which of the possible GPIO pins will be SPI functional pins.
   // Comment out other unwanted lines.

//   GpioCtrlRegs.GPBGMUX2.bit.GPIO58 = 0;
//   GpioCtrlRegs.GPBMUX2.bit.GPIO58= 1; // Configure GPIO63 as SPISIMOA
//
//   GpioCtrlRegs.GPBGMUX2.bit.GPIO59 = 0;
//   GpioCtrlRegs.GPBMUX2.bit.GPIO59 = 1; // Configure GPIO64 as SPISOMIA
//
//   GpioCtrlRegs.GPBGMUX2.bit.GPIO60 = 0;
//   GpioCtrlRegs.GPBMUX2.bit.GPIO60 = 1; // Configure GPIO65 as SPICLKA

   GPIO_SetupPinMux(16, GPIO_MUX_CPU1, 1);
   GPIO_SetupPinMux(17, GPIO_MUX_CPU1, 1);
   GPIO_SetupPinMux(18, GPIO_MUX_CPU1, 1);

//   GpioCtrlRegs.GPAGMUX2.bit.GPIO27 = 1;
//   GpioCtrlRegs.GPAMUX2.bit.GPIO27 = 2; // Configure GPIO66 as SPISTEA

   EDIS;

}
//---------------------------------------------------------------------------
// InitSPI:
//---------------------------------------------------------------------------
// This function initializes the SPI(s) to a known state.
//
void InitSpia(void)
{
    // Initialize SPI-A
    SpiaRegs.SPICCR.bit.SPISWRESET=0; // Reset SPI

    SpiaRegs.SPICCR.all = 0x0007;       // 8-bit character,output on the rising edge,input data is latched on the falling edge
    SpiaRegs.SPICTL.all = 0x0006;       // Enable master mode, normal phase, enable talk, and disable SPI int.
    SpiaRegs.SPISTS.all = 0x0000;
    SpiaRegs.SPIBRR.all = 0x0015;       // LSPCLK/(SPIBRR+1) = 100M/(8) = 12.5M

    SpiaRegs.SPICCR.bit.SPISWRESET=1;   // Enable SPI

}
//===========================================================================
// End of file.
//===========================================================================






