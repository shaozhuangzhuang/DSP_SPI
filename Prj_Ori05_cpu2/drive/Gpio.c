/*
 * Gpio.c
 *
 *  Created on: 2022年8月12日
 *      Author: 110
 */

//###########################################################################
//
// FILE:    F2837xD_Gpio.c
//
// TITLE:    GPIO module support functions
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
#include "F2837xD_device.h"
#include "F2837xD_Examples.h"

//
//Low-level functions for GPIO configuration (CPU1 only)
//

#ifdef CPU1
    //
    // InitGpio - Sets all pins to be muxed to GPIO in input mode with pull-ups
    //            enabled. Also resets CPU control to CPU1 and disables open
    //            drain and polarity inversion and sets the qualification to
    //            synchronous. Also unlocks all GPIOs. Only one CPU should call
    //            this function.
    //
    void InitGpio()
    {
        volatile Uint32 *gpioBaseAddr;
        Uint16 regOffset;

        //
        //Disable pin locks
        //
        EALLOW;
        GpioCtrlRegs.GPALOCK.all = 0x00000000;
        GpioCtrlRegs.GPBLOCK.all = 0x00000000;
        GpioCtrlRegs.GPCLOCK.all = 0x00000000;
        GpioCtrlRegs.GPDLOCK.all = 0x00000000;
        GpioCtrlRegs.GPELOCK.all = 0x00000000;
        GpioCtrlRegs.GPFLOCK.all = 0x00000000;

        //
        // Fill all registers with zeros. Writing to each register separately
        // for six GPIO modules would make this function *very* long.
        // Fortunately, we'd be writing them all with zeros anyway, so this
        // saves a lot of space.
        //
        gpioBaseAddr = (Uint32 *)&GpioCtrlRegs;
        for (regOffset = 0; regOffset < sizeof(GpioCtrlRegs)/2; regOffset++)
        {
            //
            //Hack to avoid enabling pull-ups on all pins. GPyPUD is offset
            //0x0C in each register group of 0x40 words. Since this is a
            //32-bit pointer, the addresses must be divided by 2.
            //
            if (regOffset % (0x40/2) != (0x0C/2))
            {
                gpioBaseAddr[regOffset] = 0x00000000;
            }
        }

        gpioBaseAddr = (Uint32 *)&GpioDataRegs;
        for (regOffset = 0; regOffset < sizeof(GpioDataRegs)/2; regOffset++)
        {
            gpioBaseAddr[regOffset] = 0x00000000;
        }

        EDIS;
    }

    void Gpioselect()
    {
        EALLOW;

//        GpioCtrlRegs.GPBDIR.bit.GPIO41 = 1;         // A0
//        GpioCtrlRegs.GPBDIR.bit.GPIO48 = 1;         // A1
//        GpioCtrlRegs.GPBDIR.bit.GPIO50 = 1;         // SAMPLE
//        GpioCtrlRegs.GPBDIR.bit.GPIO61 = 1;         // RESET
//        GpioCtrlRegs.GPBDIR.bit.GPIO49 = 1;         // WR

        GPIO_SetupPinMux(26, GPIO_MUX_CPU1, 0);
        GPIO_SetupPinOptions(26, GPIO_OUTPUT, GPIO_PUSHPULL);

        GPIO_SetupPinMux(24, GPIO_MUX_CPU1, 0);
        GPIO_SetupPinOptions(24, GPIO_OUTPUT, GPIO_PUSHPULL);

        GPIO_SetupPinMux(19,GPIO_MUX_CPU1,0);
        GPIO_SetupPinOptions(19, GPIO_OUTPUT,GPIO_PUSHPULL);


        GPIO_SetupPinMux(1,GPIO_MUX_CPU1,0);
        GPIO_SetupPinOptions(1, GPIO_OUTPUT,GPIO_PUSHPULL);

        GPIO_SetupPinMux(27,GPIO_MUX_CPU1,0);
        GPIO_SetupPinOptions(27, GPIO_OUTPUT,GPIO_OUTPUT);

        GPIO_SetupPinMux(115, GPIO_MUX_CPU1, 0);
        GPIO_SetupPinOptions(115, GPIO_OUTPUT, 1);
        GpioDataRegs.GPDSET.bit.GPIO115 = 1;

        GPIO_SetupPinMux(116, GPIO_MUX_CPU1, 0);
        GPIO_SetupPinOptions(116, GPIO_OUTPUT, 1);
//        GpioDataRegs.GPDDAT.bit.GPIO116 = 1;

//        GPIO_SetupPinMux(50,GPIO_MUX_CPU1,0);
//        GPIO_SetupPinOptions(50, GPIO_OUTPUT,GPIO_PUSHPULL);
//
//
//        GPIO_SetupPinMux(61,GPIO_MUX_CPU1,0);
//        GPIO_SetupPinOptions(61, GPIO_OUTPUT,GPIO_PUSHPULL);
//
//        GPIO_SetupPinMux(49,GPIO_MUX_CPU1,0);
//        GPIO_SetupPinOptions(49, GPIO_OUTPUT,GPIO_OUTPUT);

//        GPIO_SetupPinMux(73,GPIO_MUX_CPU2,0);       // DE_B
//        GPIO_SetupPinOptions(73, GPIO_OUTPUT,1);
//        GPIO_SetupPinMux(74,GPIO_MUX_CPU2,0);       // /RE_B
//        GPIO_SetupPinOptions(74, GPIO_OUTPUT,0);
//
//        GPIO_SetupPinMux(91,GPIO_MUX_CPU2,0);       // DE_C
//        GPIO_SetupPinOptions(91, GPIO_OUTPUT,1);
//        GPIO_SetupPinMux(92,GPIO_MUX_CPU2,0);       // /RE_C
//        GPIO_SetupPinOptions(92, GPIO_OUTPUT,0);
//
        GPIO_SetupPinMux(11,GPIO_MUX_CPU1,0);     // DE_D
        GPIO_SetupPinOptions(11, GPIO_OUTPUT,1);
        GpioDataRegs.GPASET.bit.GPIO11 = 1;
        GPIO_SetupPinMux(12,GPIO_MUX_CPU1,0);     // /RE_D
        GPIO_SetupPinOptions(12, GPIO_OUTPUT,0);
        GpioDataRegs.GPACLEAR.bit.GPIO12 = 1;
        EDIS;
    }

    void InitSciGpio(void)
    {
        //串口A GPIO初始化
//        GPIO_SetupPinMux(65, GPIO_MUX_CPU1, 6); // GPIO65 - (SCITXDA)
//        GPIO_SetupPinMux(43, GPIO_MUX_CPU1, 15); // GPIO43 - (SCIRXDA)
//        GPIO_SetupPinOptions(43, GPIO_INPUT, GPIO_ASYNC);
//        GPIO_SetupPinOptions(65, GPIO_OUTPUT, GPIO_PUSHPULL);
//
//        //串口B GPIO初始化
//        GPIO_SetupPinMux(10, GPIO_MUX_CPU1, 6); // GPIO10 - (SCITXDB)
//        GPIO_SetupPinMux(87, GPIO_MUX_CPU1, 5); // GPIO87 - (SCIRXDB)
//        GPIO_SetupPinOptions(87, GPIO_INPUT, GPIO_ASYNC);
//        GPIO_SetupPinOptions(10, GPIO_OUTPUT, GPIO_PUSHPULL);
//
//        //串口C GPIO初始化
//        GPIO_SetupPinMux(89, GPIO_MUX_CPU2, 6); // GPIO63 - (SCITXDC)
//        GPIO_SetupPinMux(90, GPIO_MUX_CPU2, 6); // GPIO62 - (SCIRXDC)
//        GPIO_SetupPinOptions(90, GPIO_INPUT, GPIO_ASYNC);
//        GPIO_SetupPinOptions(89, GPIO_OUTPUT, GPIO_PUSHPULL);

        //串口D GPIO初始化
        GPIO_SetupPinMux(93, GPIO_MUX_CPU2, 6); // GPIO63 - (SCITXDC)
        GPIO_SetupPinMux(94, GPIO_MUX_CPU2, 6); // GPIO62 - (SCIRXDC)
        GPIO_SetupPinOptions(94, GPIO_INPUT, GPIO_ASYNC);
        GPIO_SetupPinOptions(93, GPIO_OUTPUT, GPIO_PUSHPULL);
    }

    //
    // GPIO_SetupPinMux - Set the peripheral muxing for the specified pin. The
    //                    appropriate parameters can be found in the GPIO Muxed
    //                    Pins table(4.4) in the  datasheet. Use the GPIO index
    //                    row (0 to 15) to select a muxing option for the GPIO.
    //
    void GPIO_SetupPinMux(Uint16 pin, Uint16 cpu, Uint16 peripheral)
    {
        volatile Uint32 *gpioBaseAddr;
        volatile Uint32 *mux, *gmux, *csel;
        Uint16 pin32, pin16, pin8;

        pin32 = pin % 32;
        pin16 = pin % 16;
        pin8 = pin % 8;
        gpioBaseAddr = (Uint32 *)&GpioCtrlRegs + (pin/32)*GPY_CTRL_OFFSET;

        //
        //Sanity check for valid cpu and peripheral values
        //
        if (cpu > GPIO_MUX_CPU2CLA || peripheral > 0xF)
            return;

        //
        //Create pointers to the appropriate registers. This is a workaround
        //for the way GPIO registers are defined. The standard definition
        //in the header file makes it very easy to do named accesses of one
        //register or bit, but hard to do arbitrary numerical accesses. It's
        //easier to have an array of GPIO modules with identical registers,
        //including arrays for multi-register groups like GPyCSEL1-4. But
        //the header file doesn't define anything we can turn into an array,
        //so manual pointer arithmetic is used instead.
        //
        mux = gpioBaseAddr + GPYMUX + pin32/16;
        gmux = gpioBaseAddr + GPYGMUX + pin32/16;
        csel = gpioBaseAddr + GPYCSEL + pin32/8;

        //
        //Now for the actual function
        //
        EALLOW;

        //
        //To change the muxing, set the peripheral mux to 0/GPIO first to avoid
        //glitches, then change the group mux, then set the peripheral mux to
        //its target value. Finally, set the CPU select. This procedure is
        //described in the TRM. Unfortunately, since we don't know the pin in
        //advance we can't hardcode a bitfield reference, so there's some
        //tricky bit twiddling here.
        //
        *mux &= ~(0x3UL << (2*pin16));
        *gmux &= ~(0x3UL << (2*pin16));
        *gmux |= (Uint32)((peripheral >> 2) & 0x3UL) << (2*pin16);
        *mux |= (Uint32)(peripheral & 0x3UL) << (2*pin16);

        *csel &= ~(0x3L << (4*pin8));
        *csel |= (Uint32)(cpu & 0x3L) << (4*pin8);

        //
        //WARNING: This code does not touch the analog mode select registers,
        //which are needed to give the USB module control of its IOs.
        //
        EDIS;
    }

    //
    // GPIO_SetupPinOptions - Setup up the GPIO input/output options for the
    //                        specified pin.
    //
    //The flags are a 16-bit mask produced by ORing together options.
    //For input pins, the valid flags are:
    //GPIO_PULLUP    Enable pull-up
    //GPIO_INVERT    Enable input polarity inversion
    //GPIO_SYNC        Synchronize the input latch to PLLSYSCLK
    //               (default -- you don't need to specify this)
    //GPIO_QUAL3    Use 3-sample qualification
    //GPIO_QUAL6    Use 6-sample qualification
    //GPIO_ASYNC    Do not use synchronization or qualification
    //(Note: only one of SYNC, QUAL3, QUAL6, or ASYNC is allowed)
    //
    //For output pins, the valid flags are:
    //GPIO_OPENDRAIN    Output in open drain mode
    //GPIO_PULLUP        If open drain enabled, also enable the pull-up
    //and the input qualification flags (SYNC/QUAL3/QUAL6/SYNC) listed above.
    //
    //With no flags, the default input state is synchronous with no
    //pull-up or polarity inversion. The default output state is
    //the standard digital output.
    //
    void GPIO_SetupPinOptions(Uint16 pin, Uint16 output, Uint16 flags)
    {
        volatile Uint32 *gpioBaseAddr;
        volatile Uint32 *dir, *pud, *inv, *odr, *qsel;
        Uint32 pin32, pin16, pinMask, qual;

        pin32 = pin % 32;
        pin16 = pin % 16;
        pinMask = 1UL << pin32;
        gpioBaseAddr = (Uint32 *)&GpioCtrlRegs + (pin/32)*GPY_CTRL_OFFSET;

        //
        //Create pointers to the appropriate registers. This is a workaround
        //for the way GPIO registers are defined. The standard definition
        //in the header file makes it very easy to do named accesses of one
        //register or bit, but hard to do arbitrary numerical accesses. It's
        //easier to have an array of GPIO modules with identical registers,
        //including arrays for multi-register groups like GPyQSEL1-2. But
        //the header file doesn't define anything we can turn into an array,
        //so manual pointer arithmetic is used instead.
        //
        dir = gpioBaseAddr + GPYDIR;
        pud = gpioBaseAddr + GPYPUD;
        inv = gpioBaseAddr + GPYINV;
        odr = gpioBaseAddr + GPYODR;
        qsel = gpioBaseAddr + GPYQSEL + pin32/16;

        EALLOW;

        //
        //Set the data direction
        //
        *dir &= ~pinMask;
        if (output == 1)
        {
            //
            //Output, with optional open drain mode and pull-up
            //
            *dir |= pinMask;

            //
            //Enable open drain if necessary
            //
            if (flags & GPIO_OPENDRAIN)
            {
                *odr |= pinMask;
            }
            else
            {
                *odr &= ~pinMask;
            }

            //
            //Enable pull-up if necessary. Open drain mode must be active.
            //
            if (flags & (GPIO_OPENDRAIN | GPIO_PULLUP))
            {
                *pud &= ~pinMask;
            }
            else
            {
                *pud |= pinMask;
            }
        }
        else
        {
            //
            //Input, with optional pull-up, qualification, and polarity
            //inversion
            //
            *dir &= ~pinMask;

            //
            //Enable pull-up if necessary
            //
            if (flags & GPIO_PULLUP)
            {
                *pud &= ~pinMask;
            }
            else
            {
                *pud |= pinMask;
            }

            //
            //Invert polarity if necessary
            //
            if (flags & GPIO_INVERT)
            {
                *inv |= pinMask;
            }
            else
            {
                *inv &= ~pinMask;
            }
        }

        //
        //Extract the qualification parameter and load it into the register.
        //This is also needed for open drain outputs, so we might as well do it
        //all the time.
        //
        qual = (flags & GPIO_ASYNC) / GPIO_QUAL3;
        *qsel &= ~(0x3L << (2 * pin16));
        if (qual != 0x0)
        {
            *qsel |= qual << (2 * pin16);
        }

        EDIS;
    }

    //
    // GPIO_SetupLock - Enable or disable the GPIO register bit lock for the
    //                  specified pin.
    //                  The valid flags are:
    //                  GPIO_UNLOCK - Unlock the pin setup register bits for
    //                                the specified pin
    //                  GPIO_LOCK - Lock the pin setup register bits for the
    //                              specified pin
    //
    void GPIO_SetupLock(Uint16 pin, Uint16 flags)
    {
        volatile Uint32 *gpioBaseAddr;
        volatile Uint32 *lock;
        Uint32 pin32, pinMask;

        pin32 = pin % 32;
        pinMask = 1UL << pin32;
        gpioBaseAddr = (Uint32 *)&GpioCtrlRegs + (pin/32)*GPY_CTRL_OFFSET;

        //
        //Create pointers to the appropriate registers. This is a workaround
        //for the way GPIO registers are defined. The standard definition
        //in the header file makes it very easy to do named accesses of one
        //register or bit, but hard to do arbitrary numerical accesses. It's
        //easier to have an array of GPIO modules with identical registers,
        //including arrays for multi-register groups like GPyQSEL1-2. But
        //the header file doesn't define anything we can turn into an array,
        //so manual pointer arithmetic is used instead.
        //
        lock = gpioBaseAddr + GPYLOCK;

        EALLOW;
        if(flags)
        {
            //Lock the pin
            *lock |= pinMask;
        }
        else
        {
            //Unlock the pin
            *lock &= ~pinMask;
        }
        EDIS;
    }

    //
    //External interrupt setup
    //
    void GPIO_SetupXINT1Gpio(Uint16 pin)
    {
        EALLOW;
        InputXbarRegs.INPUT4SELECT = pin;      //Set XINT1 source to GPIO-pin
        EDIS;
    }
    void GPIO_SetupXINT2Gpio(Uint16 pin)
    {
        EALLOW;
        InputXbarRegs.INPUT5SELECT = pin;      //Set XINT2 source to GPIO-pin
        EDIS;
    }
    void GPIO_SetupXINT3Gpio(Uint16 pin)
    {
        EALLOW;
        InputXbarRegs.INPUT6SELECT = pin;      //Set XINT3 source to GPIO-pin
        EDIS;
    }
    void GPIO_SetupXINT4Gpio(Uint16 pin)
    {
        EALLOW;
        InputXbarRegs.INPUT13SELECT = pin;     //Set XINT4 source to GPIO-pin
        EDIS;
    }
    void GPIO_SetupXINT5Gpio(Uint16 pin)
    {
        EALLOW;
        InputXbarRegs.INPUT14SELECT = pin;     //Set XINT5 source to GPIO-pin
        EDIS;
    }

    //
    //GPIO_EnableUnbondedIOPullupsFor176Pin - Enable pullups for the unbonded
    //                                        GPIOs on the 176PTP package:
    //                                        GPIOs     Grp Bits
    //                                        95-132    C   31
    //                                                  D   31:0
    //                                                  E   4:0
    //                                        134-168   E   31:6
    //                                                  F   8:0
    //
    void GPIO_EnableUnbondedIOPullupsFor176Pin()
    {
        EALLOW;
        GpioCtrlRegs.GPCPUD.all = ~0x80000000;  //GPIO 95
        GpioCtrlRegs.GPDPUD.all = ~0xFFFFFFF7;  //GPIOs 96-127
        GpioCtrlRegs.GPEPUD.all = ~0xFFFFFFDF;  //GPIOs 128-159 except for 133
        GpioCtrlRegs.GPFPUD.all = ~0x000001FF;  //GPIOs 160-168
        EDIS;
    }

    //
    // GPIO_EnableUnbondedIOPullupsFor100Pin - Enable pullups for the unbonded
    //                                         GPIOs on the 100PZ package:
    //                                         GPIOs     Grp Bits
    //                                         0-1       A   1:0
    //                                         5-9       A   9:5
    //                                         22-40     A   31:22
    //                                                   B   8:0
    //                                         44-57     B   25:12
    //                                         67-68     C   4:3
    //                                         74-77     C   13:10
    //                                         79-83     C   19:15
    //                                         93-168    C   31:29
    //                                                   D   31:0
    //                                                   E   31:0
    //                                                   F   8:0
    //
    void GPIO_EnableUnbondedIOPullupsFor100Pin()
    {
        EALLOW;
        GpioCtrlRegs.GPAPUD.all = ~0xFFC003E3;  //GPIOs 0-1, 5-9, 22-31
        GpioCtrlRegs.GPBPUD.all = ~0x03FFF1FF;  //GPIOs 32-40, 44-57
        GpioCtrlRegs.GPCPUD.all = ~0xE10FBC18;  //GPIOs 67-68, 74-77, 79-83, 93-95
        GpioCtrlRegs.GPDPUD.all = ~0xFFFFFFF7;  //GPIOs 96-127
        GpioCtrlRegs.GPEPUD.all = ~0xFFFFFFFF;  //GPIOs 128-159
        GpioCtrlRegs.GPFPUD.all = ~0x000001FF;  //GPIOs 160-168
        EDIS;
    }

    //
    // GPIO_EnableUnbondedIOPullups - InitSysCtrl would call this function
    //                                this takes care of enabling IO pullups.
    //
    void GPIO_EnableUnbondedIOPullups()
    {
        //
        //bits 8-10 have pin count
        //
        unsigned char pin_count = ((DevCfgRegs.PARTIDL.all & 0x00000700) >> 8) ;

        //
        //5 = 100 pin
        //6 = 176 pin
        //7 = 337 pin
        //
        if(pin_count == 5)
        {
            GPIO_EnableUnbondedIOPullupsFor100Pin();
        }
        else if (pin_count == 6)
        {
            GPIO_EnableUnbondedIOPullupsFor176Pin();
        }
        else
        {
            //do nothing - this is 337 pin package
        }
    }

#endif //CPU1

//
// GPIO_ReadPin - Read the GPyDAT register bit for the specified pin. Note that
//                this returns the actual state of the pin, not the state of
//                the output latch.
//
Uint16 GPIO_ReadPin(Uint16 pin)
{
    volatile Uint32 *gpioDataReg;
    Uint16 pinVal;

    gpioDataReg = (volatile Uint32 *)&GpioDataRegs + (pin/32)*GPY_DATA_OFFSET;
    pinVal = (gpioDataReg[GPYDAT] >> (pin % 32)) & 0x1;

    return pinVal;
}

//
// GPIO_WritePin - Set the GPyDAT register bit for the specified pin.
//
void GPIO_WritePin(Uint16 pin, Uint16 outVal)
{
    volatile Uint32 *gpioDataReg;
    Uint32 pinMask;

    gpioDataReg = (volatile Uint32 *)&GpioDataRegs + (pin/32)*GPY_DATA_OFFSET;
    pinMask = 1UL << (pin % 32);

    if (outVal == 0)
    {
        gpioDataReg[GPYCLEAR] = pinMask;
    }
    else
    {
        gpioDataReg[GPYSET] = pinMask;
    }
}

//
// End of file
//



