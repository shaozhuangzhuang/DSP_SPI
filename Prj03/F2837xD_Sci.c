//###########################################################################
//
// FILE:   F2837xD_Sci.c
//
// TITLE:  F2837xD SCI Initialization & Support Functions.
//
//###########################################################################
// $TI Release: F2837xD Support Library v190 $
// $Release Date: Mon Feb  1 16:51:57 CST 2016 $
// $Copyright: Copyright (C) 2013-2016 Texas Instruments Incorporated -
//             http://www.ti.com/ ALL RIGHTS RESERVED $
//###########################################################################

#include "F2837xD_device.h"     // F2837xD Headerfile Include File
#include "F2837xD_Examples.h"   // F2837xD Examples Include File
#include "Common_Definitions.h"     // Common Definitions Headerfile Include File


//unsigned int rdataA[256];    						// Receive Buffer for SCI-A
//unsigned int Rx_A.rx_head = 0, Rx_A.rx_tail = 0; 		// Used for checking the received data

//unsigned int tdataA[36];							// Transfer Buffer for SCI-A


//unsigned int rdataC[256];                           // Receive Buffer for SCI-C
//unsigned int rdataC_head = 0, rdataC_tail = 0;      // Used for checking the received data
//unsigned int tdataC[32];                            // Transfer Buffer for SCI-C
unsigned char tmp_cnt = 0, tmp_cnt1 = 0;
unsigned char data_tmp[64];
#define FRAME_LEN 48
////生成多项式：0x04C11DB7
//static const Uint32 crc32tab[] = {
//		0x00000000, 0x04c11db7, 0x09823b6e, 0x0d4326d9, 0x130476dc, 0x17c56b6b,
//		0x1a864db2, 0x1e475005, 0x2608edb8, 0x22c9f00f, 0x2f8ad6d6, 0x2b4bcb61,
//		0x350c9b64, 0x31cd86d3, 0x3c8ea00a, 0x384fbdbd, 0x4c11db70, 0x48d0c6c7,
//		0x4593e01e, 0x4152fda9, 0x5f15adac, 0x5bd4b01b, 0x569796c2, 0x52568b75,
//		0x6a1936c8, 0x6ed82b7f, 0x639b0da6, 0x675a1011, 0x791d4014, 0x7ddc5da3,
//		0x709f7b7a, 0x745e66cd, 0x9823b6e0, 0x9ce2ab57, 0x91a18d8e, 0x95609039,
//		0x8b27c03c, 0x8fe6dd8b, 0x82a5fb52, 0x8664e6e5, 0xbe2b5b58, 0xbaea46ef,
//		0xb7a96036, 0xb3687d81, 0xad2f2d84, 0xa9ee3033, 0xa4ad16ea, 0xa06c0b5d,
//		0xd4326d90, 0xd0f37027, 0xddb056fe, 0xd9714b49, 0xc7361b4c, 0xc3f706fb,
//		0xceb42022, 0xca753d95, 0xf23a8028, 0xf6fb9d9f, 0xfbb8bb46, 0xff79a6f1,
//		0xe13ef6f4, 0xe5ffeb43, 0xe8bccd9a, 0xec7dd02d, 0x34867077, 0x30476dc0,
//		0x3d044b19, 0x39c556ae, 0x278206ab, 0x23431b1c, 0x2e003dc5, 0x2ac12072,
//		0x128e9dcf, 0x164f8078, 0x1b0ca6a1, 0x1fcdbb16, 0x018aeb13, 0x054bf6a4,
//		0x0808d07d, 0x0cc9cdca, 0x7897ab07, 0x7c56b6b0, 0x71159069, 0x75d48dde,
//		0x6b93dddb, 0x6f52c06c, 0x6211e6b5, 0x66d0fb02, 0x5e9f46bf, 0x5a5e5b08,
//		0x571d7dd1, 0x53dc6066, 0x4d9b3063, 0x495a2dd4, 0x44190b0d, 0x40d816ba,
//		0xaca5c697, 0xa864db20, 0xa527fdf9, 0xa1e6e04e, 0xbfa1b04b, 0xbb60adfc,
//		0xb6238b25, 0xb2e29692, 0x8aad2b2f, 0x8e6c3698, 0x832f1041, 0x87ee0df6,
//		0x99a95df3, 0x9d684044, 0x902b669d, 0x94ea7b2a, 0xe0b41de7, 0xe4750050,
//		0xe9362689, 0xedf73b3e, 0xf3b06b3b, 0xf771768c, 0xfa325055, 0xfef34de2,
//		0xc6bcf05f, 0xc27dede8, 0xcf3ecb31, 0xcbffd686, 0xd5b88683, 0xd1799b34,
//		0xdc3abded, 0xd8fba05a, 0x690ce0ee, 0x6dcdfd59, 0x608edb80, 0x644fc637,
//		0x7a089632, 0x7ec98b85, 0x738aad5c, 0x774bb0eb, 0x4f040d56, 0x4bc510e1,
//		0x46863638, 0x42472b8f, 0x5c007b8a, 0x58c1663d, 0x558240e4, 0x51435d53,
//		0x251d3b9e, 0x21dc2629, 0x2c9f00f0, 0x285e1d47, 0x36194d42, 0x32d850f5,
//		0x3f9b762c, 0x3b5a6b9b, 0x0315d626, 0x07d4cb91, 0x0a97ed48, 0x0e56f0ff,
//		0x1011a0fa, 0x14d0bd4d, 0x19939b94, 0x1d528623, 0xf12f560e, 0xf5ee4bb9,
//		0xf8ad6d60, 0xfc6c70d7, 0xe22b20d2, 0xe6ea3d65, 0xeba91bbc, 0xef68060b,
//		0xd727bbb6, 0xd3e6a601, 0xdea580d8, 0xda649d6f, 0xc423cd6a, 0xc0e2d0dd,
//		0xcda1f604, 0xc960ebb3, 0xbd3e8d7e, 0xb9ff90c9, 0xb4bcb610, 0xb07daba7,
//		0xae3afba2, 0xaafbe615, 0xa7b8c0cc, 0xa379dd7b, 0x9b3660c6, 0x9ff77d71,
//		0x92b45ba8, 0x9675461f, 0x8832161a, 0x8cf30bad, 0x81b02d74, 0x857130c3,
//		0x5d8a9099, 0x594b8d2e, 0x5408abf7, 0x50c9b640, 0x4e8ee645, 0x4a4ffbf2,
//		0x470cdd2b, 0x43cdc09c, 0x7b827d21, 0x7f436096, 0x7200464f, 0x76c15bf8,
//		0x68860bfd, 0x6c47164a, 0x61043093, 0x65c52d24, 0x119b4be9, 0x155a565e,
//		0x18197087, 0x1cd86d30, 0x029f3d35, 0x065e2082, 0x0b1d065b, 0x0fdc1bec,
//		0x3793a651, 0x3352bbe6, 0x3e119d3f, 0x3ad08088, 0x2497d08d, 0x2056cd3a,
//		0x2d15ebe3, 0x29d4f654, 0xc5a92679, 0xc1683bce, 0xcc2b1d17, 0xc8ea00a0,
//		0xd6ad50a5, 0xd26c4d12, 0xdf2f6bcb, 0xdbee767c, 0xe3a1cbc1, 0xe760d676,
//		0xea23f0af, 0xeee2ed18, 0xf0a5bd1d, 0xf464a0aa, 0xf9278673, 0xfde69bc4,
//		0x89b8fd09, 0x8d79e0be, 0x803ac667, 0x84fbdbd0, 0x9abc8bd5, 0x9e7d9662,
//		0x933eb0bb, 0x97ffad0c, 0xafb010b1, 0xab710d06, 0xa6322bdf, 0xa2f33668,
//		0xbcb4666d, 0xb8757bda, 0xb5365d03, 0xb1f740b4
//};


void InitSciGpio(void)
{
   InitSciaGpio();
   InitScibGpio();
   InitScicGpio();
#ifdef NODE_MASTER
   InitScidGpio();
#endif
}

void InitSciaGpio()
{

  EALLOW;

/* Enable internal pull-up for the selected pins */
// Pull-ups can be enabled or disabled by the user.
// This will enable the pullups for the specified pins.
// Comment out other unwanted lines.
#ifdef _DSP_SOM
//	GpioCtrlRegs.GPAPUD.bit.GPIO9 = 0;    // Enable pull-up for GPIO9 (SCIRXDA)
//	GpioCtrlRegs.GPAPUD.bit.GPIO8 = 0;	  // Enable pull-up for GPIO8 (SCITXDA)
//	GpioCtrlRegs.GPAQSEL1.bit.GPIO9 = 3;  // Asynch input GPIO9 (SCIRXDA)
//	GpioCtrlRegs.GPAMUX1.bit.GPIO9 = 2;   // Configure GPIO9 for SCIRXDA operation
//	GpioCtrlRegs.GPAGMUX1.bit.GPIO9 = 1;   // Configure GPIO9 for SCIRXDA operation
//	GpioCtrlRegs.GPAMUX1.bit.GPIO8 = 2;   // Configure GPIO8 for SCITXDA operation
//	GpioCtrlRegs.GPAGMUX1.bit.GPIO8 = 1;   // Configure GPIO8 for SCITXDA operation

    GPIO_SetupPinMux(9, GPIO_MUX_CPU1, 6); // GPIO30 - (SCIRXDA)
    GPIO_SetupPinMux(8, GPIO_MUX_CPU1, 0); // GPIO31 - (SCITXDA)
    GPIO_SetupPinOptions(9, GPIO_INPUT, GPIO_ASYNC);
    GPIO_SetupPinOptions(8, GPIO_INPUT, GPIO_ASYNC);
//    GPIO_SetupPinOptions(8, GPIO_INPUT, GPIO_PUSHPULL);
#endif

#ifdef _DSP_C
//	GpioCtrlRegs.GPBPUD.bit.GPIO49 = 0;    // Enable pull-up for GPIO9 (SCIRXDA)
//	GpioCtrlRegs.GPBPUD.bit.GPIO48 = 0;	  // Enable pull-up for GPIO8 (SCITXDA)
//	GpioCtrlRegs.GPBQSEL2.bit.GPIO49 = 3;  // Asynch input GPIO9 (SCIRXDA)
//	GpioCtrlRegs.GPBMUX2.bit.GPIO49 = 2;   // Configure GPIO9 for SCIRXDA operation
//	GpioCtrlRegs.GPBGMUX2.bit.GPIO49 = 1;   // Configure GPIO9 for SCIRXDA operation
//	GpioCtrlRegs.GPBMUX2.bit.GPIO48 = 2;   // Configure GPIO8 for SCITXDA operation
//	GpioCtrlRegs.GPBGMUX2.bit.GPIO48 = 1;   // Configure GPIO8 for SCITXDA operation

    GPIO_SetupPinMux(49, GPIO_MUX_CPU1, 6); // GPIO30 - (SCIRXDB)
    GPIO_SetupPinMux(48, GPIO_MUX_CPU1, 6); // GPIO31 - (SCITXDB)
    GPIO_SetupPinOptions(49, GPIO_INPUT, GPIO_ASYNC);
    GPIO_SetupPinOptions(48, GPIO_OUTPUT, GPIO_PUSHPULL);
#endif
/* Set qualification for selected pins to asynch only */
// This will select asynch (no qualification) for the selected pins.
// Comment out other unwanted lines.



/* Configure SPI-A pins using GPIO regs*/
// This specifies which of the possible GPIO pins will be SPI functional pins.
// Comment out other unwanted lines.

//	GPIO_SetupPinMux(85, GPIO_MUX_CPU1, 5);
//	GPIO_SetupPinMux(84, GPIO_MUX_CPU1, 5);
/*
	GpioCtrlRegs.GPBMUX2.bit.GPIO49 = 2;   // Configure GPIO9 for SCIRXDA operation
	GpioCtrlRegs.GPBGMUX2.bit.GPIO49 = 1;   // Configure GPIO8 for SCITXDA operation
	GpioCtrlRegs.GPBMUX2.bit.GPIO48 = 2;   // Configure GPIO9 for SCIRXDA operation
	GpioCtrlRegs.GPBGMUX2.bit.GPIO48 = 1;   // Configure GPIO8 for SCITXDA operation
	*/


    EDIS;
}

void InitScibGpio()
{

   EALLOW;

/* Enable internal pull-up for the selected pins */
// Pull-ups can be enabled or disabled by the user.
// This will enable the pullups for the specified pins.
// Comment out other unwanted lines.
#ifdef _DSP_SOM
//	GpioCtrlRegs.GPBPUD.bit.GPIO55 = 0;    // Enable pull-up for GPIO23 (SCITXDB)
//	GpioCtrlRegs.GPBPUD.bit.GPIO54 = 0;	  // Enable pull-up for GPIO22 (SCIRXDB)
//	GpioCtrlRegs.GPBQSEL2.bit.GPIO55 = 3;  // Asynch input GPIO23 (SCIRXDB)
//    GpioCtrlRegs.GPBMUX2.bit.GPIO55 = 2;   // Configure GPIO55 for SCIRXDA operation
//    GpioCtrlRegs.GPBGMUX2.bit.GPIO55 = 1;   // Configure GPIO55 for SCIRXDA operation
//    GpioCtrlRegs.GPBMUX2.bit.GPIO54 = 2;   // Configure GPIO54 for SCITXDA operation
//    GpioCtrlRegs.GPBGMUX2.bit.GPIO54 = 1;   // Configure GPIO54 for SCITXDA operation

//	GpioCtrlRegs.GPDPUD.bit.GPIO119 = 1;    // GPIO119
//	GpioCtrlRegs.GPDQSEL2.bit.GPIO119 = 3;  // GPIO119
//    GpioCtrlRegs.GPDMUX2.bit.GPIO119 = 0;   // Configure GPIO55 for SCIRXDA operation
//    GpioCtrlRegs.GPDGMUX2.bit.GPIO119 = 0;   // Configure GPIO55 for SCIRXDA operation

    GPIO_SetupPinMux(55, GPIO_MUX_CPU1, 6); // GPIO30 - (SCIRXDB)
    GPIO_SetupPinMux(54, GPIO_MUX_CPU1, 0); // GPIO31 - (SCITXDB)
    GPIO_SetupPinOptions(55, GPIO_INPUT, GPIO_ASYNC);
    GPIO_SetupPinOptions(54, GPIO_INPUT, GPIO_ASYNC);
//    GPIO_SetupPinOptions(54, GPIO_INPUT, GPIO_PUSHPULL);
#endif
#ifdef _DSP_C
//	GpioCtrlRegs.GPBPUD.bit.GPIO55 = 0;    // Enable pull-up for GPIO23 (SCITXDB)
//	GpioCtrlRegs.GPBPUD.bit.GPIO54 = 0;	  // Enable pull-up for GPIO22 (SCIRXDB)
//	GpioCtrlRegs.GPBQSEL2.bit.GPIO55 = 3;  // Asynch input GPIO23 (SCIRXDB)
//    GpioCtrlRegs.GPBMUX2.bit.GPIO55 = 2;   // Configure GPIO55 for SCIRXDA operation
//    GpioCtrlRegs.GPBGMUX2.bit.GPIO55 = 1;   // Configure GPIO55 for SCIRXDA operation
//    GpioCtrlRegs.GPBMUX2.bit.GPIO54 = 2;   // Configure GPIO54 for SCITXDA operation
//    GpioCtrlRegs.GPBGMUX2.bit.GPIO54 = 1;   // Configure GPIO54 for SCITXDA operation

    GPIO_SetupPinMux(55, GPIO_MUX_CPU2, 6); // GPIO30 - (SCIRXDB)
    GPIO_SetupPinMux(54, GPIO_MUX_CPU2, 6); // GPIO31 - (SCITXDB)
    GPIO_SetupPinOptions(55, GPIO_INPUT, GPIO_ASYNC);
    GPIO_SetupPinOptions(54, GPIO_OUTPUT, GPIO_PUSHPULL);
#endif


    EDIS;
}

void InitScicGpio()
{

  EALLOW;

/* Enable internal pull-up for the selected pins */
// Pull-ups can be enabled or disabled by the user.
// This will enable the pullups for the specified pins.
// Comment out other unwanted lines.
#ifdef _DSP_SOM
//	  GpioCtrlRegs.GPEPUD.bit.GPIO140 = 0;    // Enable pull-up for GPIO72 (SCITXDC)
//	  GpioCtrlRegs.GPBPUD.bit.GPIO57 = 0;   // Enable pull-up for GPIO73 (SCIRXDC)
//	  GpioCtrlRegs.GPBQSEL2.bit.GPIO57 = 3;  // Asynch input GPIO9 (SCIRXDC)
//	  GpioCtrlRegs.GPEMUX1.bit.GPIO140 = 2;   // Configure GPIO72 for SCITXDC operation
//	  GpioCtrlRegs.GPEGMUX1.bit.GPIO140 = 1;   // Configure GPIO72 for SCITXDC operation
//	  GpioCtrlRegs.GPBMUX2.bit.GPIO57 = 2;   // Configure GPIO73 for SCIRXDC operation
//	  GpioCtrlRegs.GPBGMUX2.bit.GPIO57 = 1;   // Configure GPIO72 for SCIRXDC operation


        GPIO_SetupPinMux(57, GPIO_MUX_CPU1, 6); // GPIO30 - (SCIRXDB)
        GPIO_SetupPinMux(140, GPIO_MUX_CPU1, 0); // GPIO31 - (SCITXDB)
        GPIO_SetupPinOptions(57, GPIO_INPUT, GPIO_ASYNC);
        GPIO_SetupPinOptions(140, GPIO_INPUT, GPIO_ASYNC);
//        GPIO_SetupPinOptions(140, GPIO_INPUT, GPIO_PUSHPULL);
#endif
#ifdef _DSP_C
	  GpioCtrlRegs.GPBPUD.bit.GPIO63 = 0;    // Enable pull-up for GPIO72 (SCITXDC)
	  GpioCtrlRegs.GPBPUD.bit.GPIO62 = 0;   // Enable pull-up for GPIO73 (SCIRXDC)
	  GpioCtrlRegs.GPBQSEL2.bit.GPIO62 = 3;  // Asynch input GPIO9 (SCIRXDC)
	  GpioCtrlRegs.GPBMUX2.bit.GPIO63 = 1;   // Configure GPIO72 for SCITXDC operation
	  GpioCtrlRegs.GPBGMUX2.bit.GPIO63 = 0;   // Configure GPIO72 for SCITXDC operation
	  GpioCtrlRegs.GPBMUX2.bit.GPIO62 = 1;   // Configure GPIO73 for SCIRXDC operation
	  GpioCtrlRegs.GPBGMUX2.bit.GPIO62 = 0;   // Configure GPIO72 for SCIRXDC operation
#endif

/* Set qualification for selected pins to asynch only */
// This will select asynch (no qualification) for the selected pins.
// Comment out other unwanted lines.



/* Configure SPI-A pins using GPIO regs*/
// This specifies which of the possible GPIO pins will be SPI functional pins.
// Comment out other unwanted lines.

//  GPIO_SetupPinMux(85, GPIO_MUX_CPU1, 5);
//  GPIO_SetupPinMux(84, GPIO_MUX_CPU1, 5);


    EDIS;
}

void InitScidGpio()
{

   EALLOW;

/* Enable internal pull-up for the selected pins */
// Pull-ups can be enabled or disabled by the user.
// This will enable the pullups for the specified pins.
// Comment out other unwanted lines.
#ifdef _DSP_SOM
//    GpioCtrlRegs.GPCPUD.bit.GPIO76 = 0;    // Enable pull-up for GPIO76 (SCITXDD)
//    GpioCtrlRegs.GPCPUD.bit.GPIO77 = 0;   // Enable pull-up for GPIO77 (SCIRXDD)
//    GpioCtrlRegs.GPCQSEL1.bit.GPIO77 = 3;  // Asynch input GPIO77 (SCIRXDD)
//    GpioCtrlRegs.GPCMUX1.bit.GPIO76 = 2;   // Configure GPIO76 for SCITXDC operation
//    GpioCtrlRegs.GPCGMUX1.bit.GPIO76 = 1;   // Configure GPIO76 for SCITXDC operation
//    GpioCtrlRegs.GPCMUX1.bit.GPIO77 = 2;   // Configure GPIO77 for SCIRXDC operation
//    GpioCtrlRegs.GPCGMUX1.bit.GPIO77 = 1;   // Configure GPIO77 for SCIRXDC operation

    GPIO_SetupPinMux(105, GPIO_MUX_CPU1, 6); // GPIO105 - (SCIRXDB)
    GPIO_SetupPinMux(104, GPIO_MUX_CPU1, 0); // GPIO104 - (SCITXDB)
    GPIO_SetupPinOptions(105, GPIO_INPUT, GPIO_ASYNC);
    GPIO_SetupPinOptions(104, GPIO_INPUT, GPIO_ASYNC);
//    GPIO_SetupPinOptions(104, GPIO_INPUT, GPIO_PUSHPULL);
#endif

#ifdef _DSP_C
//    GpioCtrlRegs.GPBPUD.bit.GPIO47 = 0;    // Enable pull-up for GPIO76 (SCITXDD)
//    GpioCtrlRegs.GPBPUD.bit.GPIO46 = 0;   // Enable pull-up for GPIO77 (SCIRXDD)
//    GpioCtrlRegs.GPBQSEL1.bit.GPIO46 = 3;  // Asynch input GPIO77 (SCIRXDD)
//    GpioCtrlRegs.GPBMUX1.bit.GPIO47 = 2;   // Configure GPIO76 for SCITXDC operation
//    GpioCtrlRegs.GPBGMUX1.bit.GPIO47 = 1;   // Configure GPIO76 for SCITXDC operation
//    GpioCtrlRegs.GPBMUX1.bit.GPIO46 = 2;   // Configure GPIO77 for SCIRXDC operation
//    GpioCtrlRegs.GPBGMUX1.bit.GPIO46 = 1;   // Configure GPIO77 for SCIRXDC operation

    GPIO_SetupPinMux(46, GPIO_MUX_CPU2, 6); // GPIO30 - (SCIRXDB)
    GPIO_SetupPinMux(47, GPIO_MUX_CPU2, 6); // GPIO31 - (SCITXDB)
    GPIO_SetupPinOptions(46, GPIO_INPUT, GPIO_ASYNC);
    GPIO_SetupPinOptions(47, GPIO_OUTPUT, GPIO_PUSHPULL);
#endif
/* Set qualification for selected pins to asynch only */
// This will select asynch (no qualification) for the selected pins.
// Comment out other unwanted lines.



/* Configure SPI-B pins using GPIO regs*/
// This specifies which of the possible GPIO pins will be SPI functional pins.
// Comment out other unwanted lines.



    EDIS;
}



void InitSci()
{
   InitScia();
   InitScic();
   InitScib();
#ifdef NODE_MASTER
   InitScid();
#endif

}
void InitScib(void)
{
// Initialize SCI-B:

// Initialize SCI-B:
	ScibRegs.SCICCR.all =0x0007;   // 1 stop bit,  No loopback
	                               // No parity,8 char bits,
	                               // async mode, idle-line protocol
	ScibRegs.SCICTL1.all =0x0003;  // enable TX, RX, internal SCICLK,
	                                  // Disable RX ERR, SLEEP, TXWAKE
    ScibRegs.SCICTL2.bit.TXINTENA =1;
    ScibRegs.SCICTL2.bit.RXBKINTENA =1;
	ScibRegs.SCIHBAUD.all = 0x0000;
    ScibRegs.SCILBAUD.all = 1;   //SCIB_FREQ, 6.25MHz;
//  ScibRegs.SCICCR.bit.LOOPBKENA =1; // Enable loop back
	ScibRegs.SCIFFTX.all=0xC020;
	ScibRegs.SCIFFRX.all=0x0030;
	ScibRegs.SCIFFCT.all=0x00;

	ScibRegs.SCICTL1.all =0x0023;     // Relinquish SCI from Reset
	ScibRegs.SCIFFTX.bit.TXFIFORESET=1;
	ScibRegs.SCIFFRX.bit.RXFIFORESET=1;

// Initialize frame data


}
//---------------------------------------------------------------------------
// InitSci:
//---------------------------------------------------------------------------
// This function initializes the SCI(s) to a known state.
//
void InitScia(void)
{
    // Initialize SCI-A:
        SciaRegs.SCICCR.all =0x0007;                          // 1 stop bit,  No loopback, No parity,8 char bits,async mode, idle-line protocol
        SciaRegs.SCICTL1.all =0x0003;                         // enable TX, RX, internal SCICLK, Disable RX ERR, SLEEP, TXWAKE
        SciaRegs.SCICTL2.bit.TXINTENA = 1;
        SciaRegs.SCICTL2.bit.RXBKINTENA = 1;
        SciaRegs.SCIHBAUD.all = 0x00;//0x01;
        SciaRegs.SCILBAUD.all = 0x01;//0x44;//115200bps,0x006b;38400bps,x0144
        SciaRegs.SCIFFTX.all=0xC020;                          // 发送FIFO为空时产生中断
        SciaRegs.SCIFFRX.all=0x0030;                          // 接收FIFO有10个字节时产生中断
        SciaRegs.SCIFFCT.all=0x00;

        SciaRegs.SCICTL1.all =0x0023;                         // Relinquish SCI from Reset
        SciaRegs.SCIFFTX.bit.TXFIFORESET=1;
        SciaRegs.SCIFFRX.bit.RXFIFORESET=1;

}

void InitScic(void)
{
// Initialize SCI-C:

// Initialize SCI-C:
	ScicRegs.SCICCR.all =0x0007;   // 1 stop bit,  No loopback
	                               // No parity,8 char bits,
	                               // async mode, idle-line protocol
	ScicRegs.SCICTL1.all =0x0003;  // enable TX, RX, internal SCICLK,
	                                  // Disable RX ERR, SLEEP, TXWAKE
    ScicRegs.SCICTL2.bit.TXINTENA =1;
    ScicRegs.SCICTL2.bit.RXBKINTENA =1;
	ScicRegs.SCIHBAUD.all = 0x0000;
    ScicRegs.SCILBAUD.all = 0x01;   //Scic_FREQ, 6.25MHz;
//  SciaRegs.SCICCR.bit.LOOPBKENA =1; // Enable loop back
    ScicRegs.SCIFFTX.all=0xC020;
	ScicRegs.SCIFFRX.all=0x0030;
	ScicRegs.SCIFFCT.all=0x00;

	ScicRegs.SCICTL1.all =0x0023;     // Relinquish SCI from Reset
	ScicRegs.SCIFFTX.bit.TXFIFORESET=1;
	ScicRegs.SCIFFRX.bit.RXFIFORESET=1;


}
void InitScid(void)
{
// Initialize SCI-B:
    ScidRegs.SCICCR.all =0x0007;   // 1 stop bit,  No loopback
                                   // No parity,8 char bits,
                                   // async mode, idle-line protocol
    ScidRegs.SCICTL1.all =0x0003;  // enable TX, RX, internal SCICLK,
                                      // Disable RX ERR, SLEEP, TXWAKE
    ScidRegs.SCICTL2.bit.TXINTENA =1;
    ScidRegs.SCICTL2.bit.RXBKINTENA =1;
    ScidRegs.SCIHBAUD.all = 0x00;
    ScidRegs.SCILBAUD.all = 1;   //SCIB_FREQ, 6.25MHz;
//  SciaRegs.SCICCR.bit.LOOPBKENA =1; // Enable loop back
    ScidRegs.SCIFFTX.all=0xC020;
    ScidRegs.SCIFFRX.all=0x0030;
    ScidRegs.SCIFFCT.all=0x00;

    ScidRegs.SCICTL1.all =0x0023;     // Relinquish SCI from Reset
    ScidRegs.SCIFFTX.bit.TXFIFORESET=1;
    ScidRegs.SCIFFRX.bit.RXFIFORESET=1;

}
#ifdef COM_BY_DSP
void SCIA_ReadFIFO(void)
{
    if( SciaRegs.SCIRXST.bit.RXERROR )
    {
        SciaRegs.SCICTL1.bit.SWRESET = 0;
        SciaRegs.SCICTL1.bit.SWRESET = 1;
    }
     while(SciaRegs.SCIFFRX.bit.RXFFST != 0)
     {
         Rx_A.rx_buf[Rx_A.rx_tail] = SciaRegs.SCIRXBUF.all & 0x00FF;   // 读取数据

         Rx_A.rx_tail = Rx_A.rx_tail + 1;
         Rx_A.rx_tail = Rx_A.rx_tail & 0x00FF;                       // 每存一个字节，循环队列尾指针循环+1
     }
}

void SCIC_ReadFIFO(void)
{
    if( ScicRegs.SCIRXST.bit.RXERROR )
    {
        ScicRegs.SCICTL1.bit.SWRESET = 0;
        ScicRegs.SCICTL1.bit.SWRESET = 1;
    }
     while(ScicRegs.SCIFFRX.bit.RXFFST != 0)
     {
         Rx_C.rx_buf[Rx_C.rx_tail] = ScicRegs.SCIRXBUF.all & 0x00FF;   // 读取数据

         Rx_C.rx_tail = Rx_C.rx_tail + 1;
         Rx_C.rx_tail = Rx_C.rx_tail & 0x00FF;                       // 每存一个字节，循环队列尾指针循环+1
     }
}

void SCIB_ReadFIFO(void)
{
    if( ScibRegs.SCIRXST.bit.RXERROR )
    {
        ScibRegs.SCICTL1.bit.SWRESET = 0;
        ScibRegs.SCICTL1.bit.SWRESET = 1;
    }
     while(ScibRegs.SCIFFRX.bit.RXFFST != 0)
     {
         Rx_B.rx_buf[Rx_B.rx_tail] = ScibRegs.SCIRXBUF.all & RXBUF_MASK; // 读取数据

         Rx_B.rx_tail = Rx_B.rx_tail + 1;
         Rx_B.rx_tail = Rx_B.rx_tail & RXBUF_MASK;                       // 每存一个字节，循环队列尾指针循环+1
     }
}

void SCID_ReadFIFO(void)
{
    if( ScidRegs.SCIRXST.bit.RXERROR )
    {
        ScidRegs.SCICTL1.bit.SWRESET = 0;
        ScidRegs.SCICTL1.bit.SWRESET = 1;
    }
     while(ScidRegs.SCIFFRX.bit.RXFFST != 0)
     {
         Rx_D.rx_buf[Rx_D.rx_tail] = ScidRegs.SCIRXBUF.all & RXBUF_MASK; // 读取数据

         Rx_D.rx_tail = Rx_D.rx_tail + 1;
         Rx_D.rx_tail = Rx_D.rx_tail & RXBUF_MASK;                       // 每存一个字节，循环队列尾指针循环+1
     }
}

inline void U64_To_Byte(volatile unsigned char *buf, unsigned long long data)
{
    *(buf) = data & 0x00FF;
    *(buf + 1) = (data & 0x000000000000FF00) >> 8;
    *(buf + 2) = (data & 0x0000000000FF0000) >> 16;
    *(buf + 3) = (data & 0x00000000FF000000) >> 24;
    *(buf + 4) = (data & 0x000000FF00000000) >> 32;
    *(buf + 5) = (data & 0x0000FF0000000000) >> 40;
}

inline void I64_To_Byte(volatile unsigned char *buf, int64 data)
{
    *(buf) = data & 0x00FF;
    *(buf + 1) = (data & 0x000000000000FF00) >> 8;
    *(buf + 2) = (data & 0x0000000000FF0000) >> 16;
    *(buf + 3) = (data & 0x00000000FF000000) >> 24;
}

inline void U32_To_Byte(volatile unsigned char *buf, Uint32 data)
{
    *(buf) = data & 0x00FF;
    *(buf + 1) = (data & 0x0000FF00) >> 8;
    *(buf + 2) = (data & 0x00FF0000) >> 16;
    *(buf + 3) = (data & 0xFF000000) >> 24;
}

inline void I32_To_Byte(volatile unsigned char *buf, int32 data)
{
    *(buf) = data & 0x00FF;
    *(buf + 1) = (data & 0x0000FF00) >> 8;
    *(buf + 2) = (data & 0x00FF0000) >> 16;
    *(buf + 3) = (data & 0xFF000000) >> 24;
}

inline void U16_To_Byte(volatile unsigned char *buf, Uint16 data)
{
    *(buf) = data & 0x00FF;
    *(buf + 1) = (data & 0xFF00) >> 8;
}

inline void I16_To_Byte(volatile unsigned char *buf, int16 data)
{
    *(buf) = data & 0x00FF;
    *(buf + 1) = (data & 0xFF00) >> 8;
}

inline void U8_To_Byte(volatile unsigned char *buf, Uint16 data)
{
    *(buf) = data & 0x00FF;
}

inline void I8_To_Byte(volatile unsigned char *buf, int16 data)
{
    *(buf) = data & 0x00FF;
}

inline Uint32 Byte_To_U32( unsigned char *buf)
{
    Uint32 data;

    data = (unsigned long)( (((unsigned long)(*(buf + 3)) & 0x000000FF) << 24) | (((unsigned long)(*(buf + 2)) & 0x000000FF) << 16) | (((unsigned long)(*(buf + 1)) & 0x000000FF) << 8) | ((unsigned long)(*(buf)) & 0x000000FF) );

    return data;
}

inline int32 Byte_To_I32(volatile unsigned char *buf)  //改动，为了把警告去掉
{
    int32 data;

    data = (int32)( (((int32)(*(buf + 3)) & 0x000000FF) << 24) | (((int32)(*(buf + 2)) & 0x000000FF) << 16) | (((int32)(*(buf + 1)) & 0x000000FF) << 8) | ((int32)(*(buf)) & 0x000000FF) );

    return data;
}

inline int16 Byte_To_I16( unsigned char *buf)
{
    int16 data;

    data = (int16)( (((int16)(*(buf + 1)) & 0x00FF) << 8) | ((int16)(*(buf)) & 0x00FF) );

    return data;
}

inline Uint16 Byte_To_U16( unsigned char *buf)
{
    Uint16 data;

    data = (Uint16)( (((Uint16)(*(buf+1)) & 0x00FF)) << 8) | ((Uint16)(*(buf) & 0x00FF) );

    return data;
}
void SCIA_RX(void)
{
	static unsigned int rdataA_n = 0;
	static int rdataA_length = 0;
	static unsigned long CRC32A_R = 0;
//	static unsigned char buf32A[46];//46是啥特殊意义
	static unsigned char buf16A[(FRAME_LEN - 2)];
	Uint32 CRC_Rx=0;
	int i = 0;
	unsigned char Mode_tmp;
	double V=0;
	double omega = 0;
	const double GR = 9.8;
	const double R_wheel = 0.23/2.0;
	const double width_car = 0.5;
	double v_l = 0, v_r = 0;

//	static unsigned int err = 0;

	if( Rx_A.rx_tail >= Rx_A.rx_head )
		rdataA_length = Rx_A.rx_tail - Rx_A.rx_head;
	else
		rdataA_length = Rx_A.rx_tail + 256 - Rx_A.rx_head;

	// 如果接收缓存数据数有32个或更多，则可能有一帧完整数据
	if( rdataA_length >= FRAME_LEN )
	{
		// 如果接收缓存数据有超过63字节,正常情况下应有一帧完整数据，直接从倒数第63字节开始搜寻最新帧

		if( rdataA_length > (2*FRAME_LEN - 1) )
		{
			Rx_A.rx_head = Rx_A.rx_head + rdataA_length - (2*FRAME_LEN - 1);
			Rx_A.rx_head = Rx_A.rx_head & 0x00FF;
			rdataA_length = (2*FRAME_LEN - 1);
		}


		while( rdataA_length >= FRAME_LEN )
		{
			// 如果当前数据和同步帧数据吻合，则判断帧信息是否完整
			if( Rx_A.rx_buf[Rx_A.rx_head] == 0xEB && Rx_A.rx_buf[( Rx_A.rx_head + 1 ) & 0x00FF] == 0x90 )
			{
				// CRC校验
				for(rdataA_n = 0; rdataA_n < (FRAME_LEN - 2); rdataA_n++)
				    buf16A[rdataA_n] = Rx_A.rx_buf[(Rx_A.rx_head + rdataA_n + 2) & 0x00FF];

				CRC32A_R = crc32( buf16A, (FRAME_LEN - 6) );
	//			CRC_Rx = (unsigned long)( (((unsigned long)buf32A[45] & 0x000000FF) << 24) | (((unsigned long)buf32A[44] & 0x000000FF) << 16) | (((unsigned long)buf32A[43] & 0x000000FF) << 8) | ((unsigned long)buf32A[42] & 0x000000FF) );
				CRC_Rx = Byte_To_U32(buf16A + (FRAME_LEN - 6));

				// 接收处理
				if( CRC32A_R == CRC_Rx )
				{
                    for(rdataA_n = 0; rdataA_n < (FRAME_LEN - 6); rdataA_n++)
                        data_tmp[rdataA_n] = Rx_A.rx_buf[(Rx_A.rx_head + rdataA_n + 2) & 0x00FF];
                    Rx_Proc();

                    /*
					for(rdataA_n = 0; rdataA_n < 26; rdataA_n++)
					    RX_A_FRAME.DATA[rdataA_n] = Rx_A.rx_buf[(Rx_A.rx_head + rdataA_n + 2) & 0x00FF];

					// 获取指令数据
					i = 0;
					ComData_RxA.frame_func = RX_A_FRAME.DATA[i] & 0x00FF;         i = i + 1;
					ComData_RxA.rx_frame_cnt = Byte_To_U16(RX_A_FRAME.DATA + i); i = i + 2;

					ComData_RxA.refe_vel = Byte_To_I16(RX_A_FRAME.DATA + i);   i = i + 2;
					ComData_RxA.refe_omega = Byte_To_I16(RX_A_FRAME.DATA + i);   i = i + 2;
					ComData_RxA.stat_feedback = Byte_To_I16(RX_A_FRAME.DATA + i);   i = i + 2;


					if(ComData_RxA.frame_func == 0x63)
					    ComCAN_Frame.CAN_mode = 0x63;
					//ComCAN_Frame.CAN_mode =(unsigned char)(RX_A_FRAME.DATA[i] & 0x00FF);        i=i+1;
					i = 5;
					ComCAN_Frame.V = Byte_To_I16(RX_A_FRAME.DATA + i);   i = i + 2;
					ComCAN_Frame.omega = Byte_To_I16(RX_A_FRAME.DATA + i);   i = i + 2;
					V = ((double)ComCAN_Frame.V)/32768.0 * 1;
					omega = ((double)ComCAN_Frame.omega)/32768.0 * 45;
					v_l = V - width_car * omega/57.3;
					v_r = V + width_car * omega/57.3;

					ComCAN_Frame.Vel_L = (int)(v_l/(3.14 * R_wheel) * GR * 60);
					ComCAN_Frame.Vel_R = (int)(v_r/(3.14 * R_wheel) * GR * 60);
*/
					Com_A.Rx_Flag = 1;


					Rx_A.rx_head = Rx_A.rx_head + FRAME_LEN;
					Rx_A.rx_head = Rx_A.rx_head & 0x00FF;
					rdataA_length = rdataA_length - FRAME_LEN;
				}
				else
				{
					Rx_A.rx_head = Rx_A.rx_head + 1;
					Rx_A.rx_head = Rx_A.rx_head & 0x00FF;
					rdataA_length = rdataA_length - 1;
				}
			}
			else
			{
				Rx_A.rx_head = Rx_A.rx_head + 1;
				Rx_A.rx_head = Rx_A.rx_head & 0x00FF;
				rdataA_length = rdataA_length - 1;
			}
		}
	}
}


void SCIB_RX(void)
{
    static unsigned int rdataB_n = 0;
    static int rdataB_length = 0;
    static unsigned long CRC32B_R = 0;
    static unsigned char buf32B[(FRAME_LEN - 2)];
    Uint32 CRC_Rx = 0;

    if( Rx_B.rx_tail >= Rx_B.rx_head )
        rdataB_length = Rx_B.rx_tail - Rx_B.rx_head;
    else
        rdataB_length = Rx_B.rx_tail + 256 - Rx_B.rx_head;

    // 如果接收缓存数据数有32个或更多，则可能有一帧完整数据
    if( rdataB_length >= FRAME_LEN )
    {
        // 如果接收缓存数据有超过63字节,正常情况下应有一帧完整数据，直接从倒数第63字节开始搜寻最新帧
        if( rdataB_length > (2*FRAME_LEN - 1) )
        {
            Rx_B.rx_head = Rx_B.rx_head + rdataB_length - (2*FRAME_LEN - 1);
            Rx_B.rx_head = Rx_B.rx_head & RXBUF_MASK;
            rdataB_length = (2*FRAME_LEN - 1);
        }

        while( rdataB_length >= FRAME_LEN )
        {
            // 如果当前数据和同步帧数据吻合，则判断帧信息是否完整
            if( Rx_B.rx_buf[Rx_B.rx_head] == 0xEB && Rx_B.rx_buf[( Rx_B.rx_head + 1 ) & 0x00FF] == 0x90 )
            {
                // CRC校验
                for(rdataB_n = 0; rdataB_n < (FRAME_LEN - 2); rdataB_n++)
                    buf32B[rdataB_n] = Rx_B.rx_buf[(Rx_B.rx_head + rdataB_n + 2) & RXBUF_MASK];

                CRC32B_R =  crc32( buf32B, (FRAME_LEN - 6) );
                CRC_Rx = Byte_To_U32(buf32B + (FRAME_LEN - 6));

                // 接收处理
                if( CRC32B_R == CRC_Rx )
                {
                    for(rdataB_n = 0; rdataB_n < (FRAME_LEN - 6); rdataB_n++)
                        data_tmp[rdataB_n] = Rx_B.rx_buf[(Rx_B.rx_head + rdataB_n + 2) & 0x00FF];
                    Rx_Proc();
                    Com_B.Rx_Flag = 1;

                    Rx_B.rx_head = Rx_B.rx_head + FRAME_LEN;
                    Rx_B.rx_head = Rx_B.rx_head & RXBUF_MASK;
                    rdataB_length = rdataB_length - FRAME_LEN;
                }
                else
                {
                    Rx_B.rx_head = Rx_B.rx_head + 1;
                    Rx_B.rx_head = Rx_B.rx_head & RXBUF_MASK;
                    rdataB_length = rdataB_length - 1;
                }
            }
            else
            {
                Rx_B.rx_head = Rx_B.rx_head + 1;
                Rx_B.rx_head = Rx_B.rx_head & RXBUF_MASK;
                rdataB_length = rdataB_length - 1;
            }
        }
    }
}
void SCIC_RX(void)
{
    static unsigned int rdataC_n = 0;
    static int rdataC_length = 0;
    static unsigned long CRC32C_R = 0;
    static unsigned char buf32C[(FRAME_LEN - 2)];
    Uint32 CRC_Rx = 0;

    if( Rx_C.rx_tail >= Rx_C.rx_head )
        rdataC_length = Rx_C.rx_tail - Rx_C.rx_head;
    else
        rdataC_length = Rx_C.rx_tail + 256 - Rx_C.rx_head;

    // 如果接收缓存数据数有32个或更多，则可能有一帧完整数据
    if( rdataC_length >= FRAME_LEN )
    {
        // 如果接收缓存数据有超过63字节,正常情况下应有一帧完整数据，直接从倒数第63字节开始搜寻最新帧

        if( rdataC_length > (2*FRAME_LEN - 1) )
        {
            Rx_C.rx_head = Rx_C.rx_head + rdataC_length - (2*FRAME_LEN - 1);
            Rx_C.rx_head = Rx_C.rx_head & 0x00FF;
            rdataC_length = (2*FRAME_LEN - 1);
        }

        while( rdataC_length >= FRAME_LEN )
        {
            // 如果当前数据和同步帧数据吻合，则判断帧信息是否完整
            if( Rx_C.rx_buf[Rx_C.rx_head] == 0xEB && Rx_C.rx_buf[( Rx_C.rx_head + 1 ) & 0x00FF] == 0x90 )
            {
                // CRC校验
                for(rdataC_n = 0; rdataC_n < (FRAME_LEN - 2); rdataC_n++)
                    buf32C[rdataC_n] = Rx_C.rx_buf[(Rx_C.rx_head + rdataC_n + 2) & 0x00FF];

                CRC32C_R =  crc32( buf32C, (FRAME_LEN - 6) );
                CRC_Rx = Byte_To_U32(buf32C + (FRAME_LEN - 6));

                // 接收处理
                if( CRC32C_R == CRC_Rx )
                {
                    for(rdataC_n = 0; rdataC_n < (FRAME_LEN - 6); rdataC_n++)
                        data_tmp[rdataC_n] = Rx_C.rx_buf[(Rx_C.rx_head + rdataC_n + 2) & 0x00FF];
                    Rx_Proc();
                    Com_C.Rx_Flag = 1;

                    Rx_C.rx_head = Rx_C.rx_head + FRAME_LEN;
                    Rx_C.rx_head = Rx_C.rx_head & 0x00FF;
                    rdataC_length = rdataC_length - FRAME_LEN;
                }
                else
                {
                    Rx_C.rx_head = Rx_C.rx_head + 1;
                    Rx_C.rx_head = Rx_C.rx_head & 0x00FF;
                    rdataC_length = rdataC_length - 1;
                }
            }
            else
            {
                Rx_C.rx_head = Rx_C.rx_head + 1;
                Rx_C.rx_head = Rx_C.rx_head & 0x00FF;
                rdataC_length = rdataC_length - 1;
            }
        }
    }
}
#ifdef NODE_MASTER
void SCID_RX(void)
{
    static unsigned int rdataD_n = 0;
    static int rdataD_length = 0;
    static unsigned long CRC32D_R = 0;
    static unsigned char buf32D[30];
    Uint32 CRC_Rx = 0;

    if( Rx_D.rx_tail >= Rx_D.rx_head )
        rdataD_length = Rx_D.rx_tail - Rx_D.rx_head;
    else
        rdataD_length = Rx_D.rx_tail + 256 - Rx_D.rx_head;

    // 如果接收缓存数据数有32个或更多，则可能有一帧完整数据
    if( rdataD_length >= FRAME_LEN )
    {
        // 如果接收缓存数据有超过63字节,正常情况下应有一帧完整数据，直接从倒数第63字节开始搜寻最新帧
        if( rdataD_length > (2*FRAME_LEN - 1) )
        {
            Rx_D.rx_head = Rx_D.rx_head + rdataD_length - (2*FRAME_LEN - 1);
            Rx_D.rx_head = Rx_D.rx_head & RXBUF_MASK;
            rdataD_length = (2*FRAME_LEN - 1);
        }

        while( rdataD_length >= FRAME_LEN )
        {
            // 如果当前数据和同步帧数据吻合，则判断帧信息是否完整
            if( Rx_D.rx_buf[Rx_D.rx_head] == 0xEB && Rx_D.rx_buf[( Rx_D.rx_head + 1 ) & RXBUF_MASK] == 0x90 )
            {
                // CRC校验
                for(rdataD_n = 0; rdataD_n < (FRAME_LEN - 6); rdataD_n++)
                    buf32D[rdataD_n] = Rx_D.rx_buf[(Rx_D.rx_head + rdataD_n + 2) & RXBUF_MASK];

                CRC32D_R =  crc32( buf32D, (FRAME_LEN - 6) );
                CRC_Rx = Byte_To_U32(buf32D + (FRAME_LEN - 6));

                // 接收处理
                if( CRC32D_R == CRC_Rx )
                {
                    for(rdataD_n = 0; rdataD_n < (FRAME_LEN - 6); rdataD_n++)
                        data_tmp[rdataD_n] = Rx_A.rx_buf[(Rx_A.rx_head + rdataD_n + 2) & 0x00FF];
                    Rx_Proc();
                    Com_D.Rx_Flag = 1;


                    Rx_D.rx_head = Rx_D.rx_head + FRAME_LEN;
                    Rx_D.rx_head = Rx_D.rx_head & RXBUF_MASK;
                    rdataD_length = rdataD_length - FRAME_LEN;
                }
                else
                {
                    Rx_D.rx_head = Rx_D.rx_head + 1;
                    Rx_D.rx_head = Rx_D.rx_head & RXBUF_MASK;
                    rdataD_length = rdataD_length - 1;
                }
            }
            else
            {
                Rx_D.rx_head = Rx_D.rx_head + 1;
                Rx_D.rx_head = Rx_D.rx_head & RXBUF_MASK;
                rdataD_length = rdataD_length - 1;
            }
        }
    }
}
#endif

void Rx_Proc(void)
{
    unsigned char Frame_ID=0;
    unsigned char Address_ID = 0;
    unsigned int Frame_cnt = 0;
    unsigned char rcv_time = 0;
    int i = 0;
    int j = 0;
    int gro_i = 0;
    Frame_ID = data_tmp[i] & 0x00FF;        i = i + 1;
    Address_ID = data_tmp[i] & 0x00FF;      i = i + 1;
    i = 2;
    Frame_cnt = Byte_To_U16(data_tmp + i);        i = i + 2;
//    rcv_localtime = Byte_To_U16(data_tmp + i);       i = i + 2;
    switch( Frame_ID )
    {
#ifdef NODE_GCS
    case 0xC3:     // 主控节点帧
        i = 4;
        ComDataNode.Timer_100us = data_tmp[i] & 0x00FF + 80;        i = i + 1;
        ComDataNode.Tx_TimeCNT = data_tmp[i] & 0x00FF;        i = i + 1;
        if(ComDataNode.Timer_100us > 100)
        {
            ComDataNode.Timer_100us = ComDataNode.Timer_100us -100;
            ComDataNode.Tx_TimeCNT = ComDataNode.Tx_TimeCNT + 1;
        }
       ComDataNode.TimeSyn_Status = TIME_SYN;
       ComDataNode.C3.Driv_instruction[0]   = data_tmp[i];        i = i + 1;
       ComDataNode.C3.Driv_instruction[1]   = data_tmp[i];        i = i + 1;
       ComDataNode.C3.Driv_instruction[2]   = data_tmp[i];        i = i + 1;
       ComDataNode.C3.Speed_instruction[0]  = Byte_To_I16(data_tmp + i);               i = i + 2;
       ComDataNode.C3.Speed_instruction[1]  = Byte_To_I16(data_tmp + i);               i = i + 2;
       ComDataNode.C3.steer_instruction[0]  = Byte_To_U16(data_tmp + i);               i = i + 2;
       ComDataNode.C3.steer_instruction[1]  = Byte_To_U16(data_tmp + i);               i = i + 2;
       ComDataNode.C3.attack_instruction[0] = data_tmp[i];               i = i + 1;
       ComDataNode.C3.attack_instruction[1] = data_tmp[i];               i = i + 1;
       ComDataNode.C3.attack_instruction[2] = data_tmp[i];               i = i + 1;
       ComDataNode.C3.target_X              = Byte_To_I16(data_tmp + i);               i = i + 2;
       ComDataNode.C3.target_Y              = Byte_To_I16(data_tmp + i);               i = i + 2;
       ComDataNode.C3.target_delte_X        = Byte_To_U16(data_tmp + i);               i = i + 2;
       ComDataNode.C3.target_delte_Y        = Byte_To_U16(data_tmp + i);               i = i + 2;

       VarComDataNode.Var_C3.Driv_instruction[0]    = ComDataNode.C3.Driv_instruction[0];
       VarComDataNode.Var_C3.Driv_instruction[1]    = ComDataNode.C3.Driv_instruction[1];
       VarComDataNode.Var_C3.Driv_instruction[2]    = ComDataNode.C3.Driv_instruction[2];
       VarComDataNode.Var_C3.Speed_instruction[0]   = (float)(ComDataNode.C3.Speed_instruction[0] * 0.01);
       VarComDataNode.Var_C3.Speed_instruction[1]   = (float)(ComDataNode.C3.Speed_instruction[1] * 0.01);
       VarComDataNode.Var_C3.steer_instruction[0]   = (float)(ComDataNode.C3.steer_instruction[0] * 0.01);
       VarComDataNode.Var_C3.steer_instruction[1]   = (float)(ComDataNode.C3.steer_instruction[1] * 0.01);
       VarComDataNode.Var_C3.attack_instruction[0]  = ComDataNode.C3.attack_instruction[0];
       VarComDataNode.Var_C3.attack_instruction[1]  = ComDataNode.C3.attack_instruction[0];
       VarComDataNode.Var_C3.attack_instruction[2]  = ComDataNode.C3.attack_instruction[0];
       VarComDataNode.Var_C3.target_X               = (float)(ComDataNode.C3.target_X);
       VarComDataNode.Var_C3.target_Y               = (float)(ComDataNode.C3.target_Y);
       VarComDataNode.Var_C3.target_delte_X         = (float)(ComDataNode.C3.target_delte_X);
       VarComDataNode.Var_C3.target_delte_Y         = (float)(ComDataNode.C3.target_delte_Y);
       break;

   case 0xA5:     // 方向伺服指令
       i = 6;
       ComDataNode.A5.local_instruction = Byte_To_I32(data_tmp + i);          i = i + 4;
       ComDataNode.A5.before_fbk        = Byte_To_I32(data_tmp + i);          i = i + 4;
       ComDataNode.A5.acc_X             = Byte_To_I32(data_tmp + i);          i = i + 4;
       ComDataNode.A5.acc_Y             = Byte_To_I32(data_tmp + i);          i = i + 4;
       ComDataNode.A5.acc_Z             = Byte_To_I32(data_tmp + i);          i = i + 4;
       ComDataNode.A5.omega_X           = Byte_To_I32(data_tmp + i);          i = i + 4;
       ComDataNode.A5.omega_Y           = Byte_To_I32(data_tmp + i);          i = i + 4;
       ComDataNode.A5.omega_Z           = Byte_To_I32(data_tmp + i);          i = i + 4;

       VarComDataNode.Var_A5.local_instruction  = (long double)(ComDataNode.A5.local_instruction * 1e-4);
       VarComDataNode.Var_A5.before_fbk         = (long double)(ComDataNode.A5.before_fbk * 0.1);
       VarComDataNode.Var_A5.acc_X              = (long double)(ComDataNode.A5.acc_X * 1e-4);
       VarComDataNode.Var_A5.acc_Y              = (long double)(ComDataNode.A5.acc_Y * 1e-4);
       VarComDataNode.Var_A5.acc_Z              = (long double)(ComDataNode.A5.acc_Z * 1e-4);
       VarComDataNode.Var_A5.omega_X            = (long double)(ComDataNode.A5.omega_X * 1e-4);
       VarComDataNode.Var_A5.omega_Y            = (long double)(ComDataNode.A5.omega_Y * 1e-4);
       VarComDataNode.Var_A5.omega_Z            = (long double)(ComDataNode.A5.omega_Z * 1e-4);
        break;

    case 0xA6:     // 车体经纬高
        i = 6;
        ComDataNode.A6.lon              = Byte_To_I32(data_tmp + i);          i = i + 4;
        ComDataNode.A6.lat              = Byte_To_I32(data_tmp + i);          i = i + 4;
        ComDataNode.A6.alt              = Byte_To_I32(data_tmp + i);          i = i + 4;
        ComDataNode.A6.course           = Byte_To_I32(data_tmp + i);          i = i + 4;
        ComDataNode.A6.roll             = Byte_To_I32(data_tmp + i);          i = i + 4;
        ComDataNode.A6.pitch            = Byte_To_I32(data_tmp + i);          i = i + 4;
        ComDataNode.A6.speed_sample_X   = Byte_To_I32(data_tmp + i);          i = i + 4;
        ComDataNode.A6.speed_sample_Y   = Byte_To_I32(data_tmp + i);          i = i + 4;
        ComDataNode.A6.speed_sample_Z   = Byte_To_I32(data_tmp + i);          i = i + 4;

        VarComDataNode.Var_A6.lon = (long double)(ComDataNode.A6.lon * 1e-7);
        VarComDataNode.Var_A6.lat = (long double)(ComDataNode.A6.lat * 1e-7);
        VarComDataNode.Var_A6.alt = (long double)(ComDataNode.A6.alt * 1e-4);
        VarComDataNode.Var_A6.course = (long double)(ComDataNode.A6.course * 1e-4);
        VarComDataNode.Var_A6.roll = (long double)(ComDataNode.A6.roll * 1e-4);
        VarComDataNode.Var_A6.pitch = (long double)(ComDataNode.A6.pitch * 1e-4);
        VarComDataNode.Var_A6.speed_sample_X = (long double)(ComDataNode.A6.speed_sample_X * 1e-4);
        VarComDataNode.Var_A6.speed_sample_Y = (long double)(ComDataNode.A6.speed_sample_Y * 1e-4);
        VarComDataNode.Var_A6.speed_sample_Z = (long double)(ComDataNode.A6.speed_sample_Z * 1e-4);
        break;
#endif

#ifdef NODE_MASTER_standby_1
    case 0xC3:     // 主控节点帧
        i = 4;
        ComDataNode.Timer_100us = data_tmp[i] & 0x00FF + 80;        i = i + 1;
        ComDataNode.Tx_TimeCNT = data_tmp[i] & 0x00FF;        i = i + 1;
        if(ComDataNode.Timer_100us > 100)
        {
            ComDataNode.Timer_100us = ComDataNode.Timer_100us -100;
            ComDataNode.Tx_TimeCNT = ComDataNode.Tx_TimeCNT + 1;
        }

#endif
    case 0x9C:
        i = 6;
        ComDataNode.D_9C.radar_data[0]  = Byte_To_I16(data_tmp + i);               i = i + 2;
        ComDataNode.D_9C.radar_data[1]  = Byte_To_I16(data_tmp + i);               i = i + 2;
        ComDataNode.D_9C.radar_data[2]  = Byte_To_I16(data_tmp + i);               i = i + 2;
        ComDataNode.D_9C.radar_data[3]  = Byte_To_I16(data_tmp + i);               i = i + 2;
        ComDataNode.D_9C.radar_data[4]  = Byte_To_I16(data_tmp + i);               i = i + 2;
        ComDataNode.D_9C.radar_data[5]  = Byte_To_I16(data_tmp + i);               i = i + 2;
        ComDataNode.D_9C.radar_data[6]  = Byte_To_I16(data_tmp + i);               i = i + 2;
        ComDataNode.D_9C.radar_data[7]  = Byte_To_I16(data_tmp + i);               i = i + 2;
        ComDataNode.D_9C.radar_data[8]  = Byte_To_I16(data_tmp + i);               i = i + 2;
        ComDataNode.D_9C.radar_data[9]  = Byte_To_I16(data_tmp + i);               i = i + 2;
        ComDataNode.D_9C.radar_data[10] = Byte_To_I16(data_tmp + i);               i = i + 2;
        ComDataNode.D_9C.radar_data[11] = Byte_To_I16(data_tmp + i);               i = i + 2;
        ComDataNode.D_9C.milemeter_sig  = Byte_To_I16(data_tmp + i);               i = i + 2;

        for (j = 0; j <12;j++)
            VarComDataNode.Var_D_9C.radar_data[j] = (float)(ComDataNode.D_9C.radar_data[j] * 0.01);
        VarComDataNode.Var_D_9C.milemeter_sig = (float)(ComDataNode.D_9C.milemeter_sig * 0.1);
        break;

    case 0x60:
        i = 6;
        ComDataNode.motor_brake[0].pos_hub_mot      = Byte_To_I16(data_tmp + i);               i = i + 2;
        ComDataNode.motor_brake[0].speed_hub_mot    = Byte_To_I16(data_tmp + i);               i = i + 2;
        ComDataNode.motor_brake[0].vol_hub_mot      = Byte_To_I16(data_tmp + i);               i = i + 2;
        ComDataNode.motor_brake[0].cur_hub_mot      = Byte_To_I16(data_tmp + i);               i = i + 2;
        ComDataNode.motor_brake[0].Tem_hub_mot      = Byte_To_I16(data_tmp + i);               i = i + 2;
        ComDataNode.motor_brake[0].pos_brake        = Byte_To_I16(data_tmp + i);               i = i + 2;
        ComDataNode.motor_brake[0].pos_fbk_sus      = Byte_To_I16(data_tmp + i);               i = i + 2;
        ComDataNode.motor_brake[0].speed_sus        = Byte_To_I16(data_tmp + i);               i = i + 2;
        ComDataNode.motor_brake[0].pressure_sus     = Byte_To_I16(data_tmp + i);               i = i + 2;
        ComDataNode.motor_brake[0].vol_sus          = Byte_To_I16(data_tmp + i);               i = i + 2;
        ComDataNode.motor_brake[0].cur_sus          = Byte_To_I16(data_tmp + i);               i = i + 2;
        ComDataNode.motor_brake[0].Tem_sus          = Byte_To_I16(data_tmp + i);               i = i + 2;
        ComDataNode.motor_brake[0].stat_hub_mot     = (data_tmp[i]);               i = i + 1;
        ComDataNode.motor_brake[0].stat_brake       = (data_tmp[i]);               i = i + 1;
        ComDataNode.motor_brake[0].stat_sus         = (data_tmp[i]);               i = i + 1;

        VarComDataNode.Var_motor_brake[0].pos_hub_mot   = (float)(ComDataNode.motor_brake[0].pos_hub_mot * 0.01);
        VarComDataNode.Var_motor_brake[0].speed_hub_mot = (float)(ComDataNode.motor_brake[0].speed_hub_mot * 1);
        VarComDataNode.Var_motor_brake[0].vol_hub_mot   = (float)(ComDataNode.motor_brake[0].vol_hub_mot * 0.1);
        VarComDataNode.Var_motor_brake[0].cur_hub_mot   = (float)(ComDataNode.motor_brake[0].cur_hub_mot * 0.1);
        VarComDataNode.Var_motor_brake[0].Tem_hub_mot   = (float)(ComDataNode.motor_brake[0].Tem_hub_mot * 0.1);
        VarComDataNode.Var_motor_brake[0].pos_brake     = (float)(ComDataNode.motor_brake[0].pos_brake * 0.01);
        VarComDataNode.Var_motor_brake[0].pos_fbk_sus   = (float)(ComDataNode.motor_brake[0].pos_fbk_sus * 0.01);
        VarComDataNode.Var_motor_brake[0].speed_sus     = (float)(ComDataNode.motor_brake[0].speed_sus);
        VarComDataNode.Var_motor_brake[0].pressure_sus  = (float)(ComDataNode.motor_brake[0].pressure_sus * 0.01);
        VarComDataNode.Var_motor_brake[0].vol_sus       = (float)(ComDataNode.motor_brake[0].vol_sus * 0.1);
        VarComDataNode.Var_motor_brake[0].cur_sus       = (float)(ComDataNode.motor_brake[0].cur_sus * 0.1);
        VarComDataNode.Var_motor_brake[0].Tem_sus       = (float)(ComDataNode.motor_brake[0].Tem_sus * 0.1);
        VarComDataNode.Var_motor_brake[0].stat_hub_mot  = ComDataNode.motor_brake[0].stat_hub_mot;
        VarComDataNode.Var_motor_brake[0].stat_brake    = ComDataNode.motor_brake[0].stat_brake;
        VarComDataNode.Var_motor_brake[0].stat_sus      = ComDataNode.motor_brake[0].stat_sus;
        break;

    case 0x63:
            i = 6;
            ComDataNode.motor_brake[1].pos_hub_mot      = Byte_To_I16(data_tmp + i);               i = i + 2;
            ComDataNode.motor_brake[1].speed_hub_mot    = Byte_To_I16(data_tmp + i);               i = i + 2;
            ComDataNode.motor_brake[1].vol_hub_mot      = Byte_To_I16(data_tmp + i);               i = i + 2;
            ComDataNode.motor_brake[1].cur_hub_mot      = Byte_To_I16(data_tmp + i);               i = i + 2;
            ComDataNode.motor_brake[1].Tem_hub_mot      = Byte_To_I16(data_tmp + i);               i = i + 2;
            ComDataNode.motor_brake[1].pos_brake        = Byte_To_I16(data_tmp + i);               i = i + 2;
            ComDataNode.motor_brake[1].pos_fbk_sus      = Byte_To_I16(data_tmp + i);               i = i + 2;
            ComDataNode.motor_brake[1].speed_sus        = Byte_To_I16(data_tmp + i);               i = i + 2;
            ComDataNode.motor_brake[1].pressure_sus     = Byte_To_I16(data_tmp + i);               i = i + 2;
            ComDataNode.motor_brake[1].vol_sus          = Byte_To_I16(data_tmp + i);               i = i + 2;
            ComDataNode.motor_brake[1].cur_sus          = Byte_To_I16(data_tmp + i);               i = i + 2;
            ComDataNode.motor_brake[1].Tem_sus          = Byte_To_I16(data_tmp + i);               i = i + 2;
            ComDataNode.motor_brake[1].stat_hub_mot     = (data_tmp[i]);               i = i + 1;
            ComDataNode.motor_brake[1].stat_brake       = (data_tmp[i]);               i = i + 1;
            ComDataNode.motor_brake[1].stat_sus         = (data_tmp[i]);               i = i + 1;

            VarComDataNode.Var_motor_brake[1].pos_hub_mot   = (float)(ComDataNode.motor_brake[1].pos_hub_mot * 0.01);
            VarComDataNode.Var_motor_brake[1].speed_hub_mot = (float)(ComDataNode.motor_brake[1].speed_hub_mot * 1);
            VarComDataNode.Var_motor_brake[1].vol_hub_mot   = (float)(ComDataNode.motor_brake[1].vol_hub_mot * 0.1);
            VarComDataNode.Var_motor_brake[1].cur_hub_mot   = (float)(ComDataNode.motor_brake[1].cur_hub_mot * 0.1);
            VarComDataNode.Var_motor_brake[1].Tem_hub_mot   = (float)(ComDataNode.motor_brake[1].Tem_hub_mot * 0.1);
            VarComDataNode.Var_motor_brake[1].pos_brake     = (float)(ComDataNode.motor_brake[1].pos_brake * 0.01);
            VarComDataNode.Var_motor_brake[1].pos_fbk_sus   = (float)(ComDataNode.motor_brake[1].pos_fbk_sus * 0.01);
            VarComDataNode.Var_motor_brake[1].speed_sus     = (float)(ComDataNode.motor_brake[1].speed_sus);
            VarComDataNode.Var_motor_brake[1].pressure_sus  = (float)(ComDataNode.motor_brake[1].pressure_sus * 0.01);
            VarComDataNode.Var_motor_brake[1].vol_sus       = (float)(ComDataNode.motor_brake[1].vol_sus * 0.1);
            VarComDataNode.Var_motor_brake[1].cur_sus       = (float)(ComDataNode.motor_brake[1].cur_sus * 0.1);
            VarComDataNode.Var_motor_brake[1].Tem_sus       = (float)(ComDataNode.motor_brake[1].Tem_sus * 0.1);
            VarComDataNode.Var_motor_brake[1].stat_hub_mot  = ComDataNode.motor_brake[1].stat_hub_mot;
            VarComDataNode.Var_motor_brake[1].stat_brake    = ComDataNode.motor_brake[1].stat_brake;
            VarComDataNode.Var_motor_brake[1].stat_sus      = ComDataNode.motor_brake[1].stat_sus;
            break;

    case 0x65:
            i = 6;
            ComDataNode.motor_brake[2].pos_hub_mot      = Byte_To_I16(data_tmp + i);               i = i + 2;
            ComDataNode.motor_brake[2].speed_hub_mot    = Byte_To_I16(data_tmp + i);               i = i + 2;
            ComDataNode.motor_brake[2].vol_hub_mot      = Byte_To_I16(data_tmp + i);               i = i + 2;
            ComDataNode.motor_brake[2].cur_hub_mot      = Byte_To_I16(data_tmp + i);               i = i + 2;
            ComDataNode.motor_brake[2].Tem_hub_mot      = Byte_To_I16(data_tmp + i);               i = i + 2;
            ComDataNode.motor_brake[2].pos_brake        = Byte_To_I16(data_tmp + i);               i = i + 2;
            ComDataNode.motor_brake[2].pos_fbk_sus      = Byte_To_I16(data_tmp + i);               i = i + 2;
            ComDataNode.motor_brake[2].speed_sus        = Byte_To_I16(data_tmp + i);               i = i + 2;
            ComDataNode.motor_brake[2].pressure_sus     = Byte_To_I16(data_tmp + i);               i = i + 2;
            ComDataNode.motor_brake[2].vol_sus          = Byte_To_I16(data_tmp + i);               i = i + 2;
            ComDataNode.motor_brake[2].cur_sus          = Byte_To_I16(data_tmp + i);               i = i + 2;
            ComDataNode.motor_brake[2].Tem_sus          = Byte_To_I16(data_tmp + i);               i = i + 2;
            ComDataNode.motor_brake[2].stat_hub_mot     = (data_tmp[i]);               i = i + 1;
            ComDataNode.motor_brake[2].stat_brake       = (data_tmp[i]);               i = i + 1;
            ComDataNode.motor_brake[2].stat_sus         = (data_tmp[i]);               i = i + 1;

            VarComDataNode.Var_motor_brake[2].pos_hub_mot   = (float)(ComDataNode.motor_brake[2].pos_hub_mot * 0.01);
            VarComDataNode.Var_motor_brake[2].speed_hub_mot = (float)(ComDataNode.motor_brake[2].speed_hub_mot * 1);
            VarComDataNode.Var_motor_brake[2].vol_hub_mot   = (float)(ComDataNode.motor_brake[2].vol_hub_mot * 0.1);
            VarComDataNode.Var_motor_brake[2].cur_hub_mot   = (float)(ComDataNode.motor_brake[2].cur_hub_mot * 0.1);
            VarComDataNode.Var_motor_brake[2].Tem_hub_mot   = (float)(ComDataNode.motor_brake[2].Tem_hub_mot * 0.1);
            VarComDataNode.Var_motor_brake[2].pos_brake     = (float)(ComDataNode.motor_brake[2].pos_brake * 0.01);
            VarComDataNode.Var_motor_brake[2].pos_fbk_sus   = (float)(ComDataNode.motor_brake[2].pos_fbk_sus * 0.01);
            VarComDataNode.Var_motor_brake[2].speed_sus     = (float)(ComDataNode.motor_brake[2].speed_sus);
            VarComDataNode.Var_motor_brake[2].pressure_sus  = (float)(ComDataNode.motor_brake[2].pressure_sus * 0.01);
            VarComDataNode.Var_motor_brake[2].vol_sus       = (float)(ComDataNode.motor_brake[2].vol_sus * 0.1);
            VarComDataNode.Var_motor_brake[2].cur_sus       = (float)(ComDataNode.motor_brake[2].cur_sus * 0.1);
            VarComDataNode.Var_motor_brake[2].Tem_sus       = (float)(ComDataNode.motor_brake[2].Tem_sus * 0.1);
            VarComDataNode.Var_motor_brake[2].stat_hub_mot  = ComDataNode.motor_brake[2].stat_hub_mot;
            VarComDataNode.Var_motor_brake[2].stat_brake    = ComDataNode.motor_brake[2].stat_brake;
            VarComDataNode.Var_motor_brake[2].stat_sus      = ComDataNode.motor_brake[2].stat_sus;
            break;

    case 0x66:
            i = 6;
            ComDataNode.motor_brake[3].pos_hub_mot      = Byte_To_I16(data_tmp + i);               i = i + 2;
            ComDataNode.motor_brake[3].speed_hub_mot    = Byte_To_I16(data_tmp + i);               i = i + 2;
            ComDataNode.motor_brake[3].vol_hub_mot      = Byte_To_I16(data_tmp + i);               i = i + 2;
            ComDataNode.motor_brake[3].cur_hub_mot      = Byte_To_I16(data_tmp + i);               i = i + 2;
            ComDataNode.motor_brake[3].Tem_hub_mot      = Byte_To_I16(data_tmp + i);               i = i + 2;
            ComDataNode.motor_brake[3].pos_brake        = Byte_To_I16(data_tmp + i);               i = i + 2;
            ComDataNode.motor_brake[3].pos_fbk_sus      = Byte_To_I16(data_tmp + i);               i = i + 2;
            ComDataNode.motor_brake[3].speed_sus        = Byte_To_I16(data_tmp + i);               i = i + 2;
            ComDataNode.motor_brake[3].pressure_sus     = Byte_To_I16(data_tmp + i);               i = i + 2;
            ComDataNode.motor_brake[3].vol_sus          = Byte_To_I16(data_tmp + i);               i = i + 2;
            ComDataNode.motor_brake[3].cur_sus          = Byte_To_I16(data_tmp + i);               i = i + 2;
            ComDataNode.motor_brake[3].Tem_sus          = Byte_To_I16(data_tmp + i);               i = i + 2;
            ComDataNode.motor_brake[3].stat_hub_mot     = (data_tmp[i]);               i = i + 1;
            ComDataNode.motor_brake[3].stat_brake       = (data_tmp[i]);               i = i + 1;
            ComDataNode.motor_brake[3].stat_sus         = (data_tmp[i]);               i = i + 1;

            VarComDataNode.Var_motor_brake[3].pos_hub_mot   = (float)(ComDataNode.motor_brake[3].pos_hub_mot * 0.01);
            VarComDataNode.Var_motor_brake[3].speed_hub_mot = (float)(ComDataNode.motor_brake[3].speed_hub_mot * 1);
            VarComDataNode.Var_motor_brake[3].vol_hub_mot   = (float)(ComDataNode.motor_brake[3].vol_hub_mot * 0.1);
            VarComDataNode.Var_motor_brake[3].cur_hub_mot   = (float)(ComDataNode.motor_brake[3].cur_hub_mot * 0.1);
            VarComDataNode.Var_motor_brake[3].Tem_hub_mot   = (float)(ComDataNode.motor_brake[3].Tem_hub_mot * 0.1);
            VarComDataNode.Var_motor_brake[3].pos_brake     = (float)(ComDataNode.motor_brake[3].pos_brake * 0.01);
            VarComDataNode.Var_motor_brake[3].pos_fbk_sus   = (float)(ComDataNode.motor_brake[3].pos_fbk_sus * 0.01);
            VarComDataNode.Var_motor_brake[3].speed_sus     = (float)(ComDataNode.motor_brake[3].speed_sus);
            VarComDataNode.Var_motor_brake[3].pressure_sus  = (float)(ComDataNode.motor_brake[3].pressure_sus * 0.01);
            VarComDataNode.Var_motor_brake[3].vol_sus       = (float)(ComDataNode.motor_brake[3].vol_sus * 0.1);
            VarComDataNode.Var_motor_brake[3].cur_sus       = (float)(ComDataNode.motor_brake[3].cur_sus * 0.1);
            VarComDataNode.Var_motor_brake[3].Tem_sus       = (float)(ComDataNode.motor_brake[3].Tem_sus * 0.1);
            VarComDataNode.Var_motor_brake[3].stat_hub_mot  = ComDataNode.motor_brake[3].stat_hub_mot;
            VarComDataNode.Var_motor_brake[3].stat_brake    = ComDataNode.motor_brake[3].stat_brake;
            VarComDataNode.Var_motor_brake[3].stat_sus      = ComDataNode.motor_brake[3].stat_sus;
            break;

    case 0x69:
            i = 6;
            ComDataNode.motor_brake[4].pos_hub_mot      = Byte_To_I16(data_tmp + i);               i = i + 2;
            ComDataNode.motor_brake[4].speed_hub_mot    = Byte_To_I16(data_tmp + i);               i = i + 2;
            ComDataNode.motor_brake[4].vol_hub_mot      = Byte_To_I16(data_tmp + i);               i = i + 2;
            ComDataNode.motor_brake[4].cur_hub_mot      = Byte_To_I16(data_tmp + i);               i = i + 2;
            ComDataNode.motor_brake[4].Tem_hub_mot      = Byte_To_I16(data_tmp + i);               i = i + 2;
            ComDataNode.motor_brake[4].pos_brake        = Byte_To_I16(data_tmp + i);               i = i + 2;
            ComDataNode.motor_brake[4].pos_fbk_sus      = Byte_To_I16(data_tmp + i);               i = i + 2;
            ComDataNode.motor_brake[4].speed_sus        = Byte_To_I16(data_tmp + i);               i = i + 2;
            ComDataNode.motor_brake[4].pressure_sus     = Byte_To_I16(data_tmp + i);               i = i + 2;
            ComDataNode.motor_brake[4].vol_sus          = Byte_To_I16(data_tmp + i);               i = i + 2;
            ComDataNode.motor_brake[4].cur_sus          = Byte_To_I16(data_tmp + i);               i = i + 2;
            ComDataNode.motor_brake[4].Tem_sus          = Byte_To_I16(data_tmp + i);               i = i + 2;
            ComDataNode.motor_brake[4].stat_hub_mot     = (data_tmp[i]);               i = i + 1;
            ComDataNode.motor_brake[4].stat_brake       = (data_tmp[i]);               i = i + 1;
            ComDataNode.motor_brake[4].stat_sus         = (data_tmp[i]);               i = i + 1;

            VarComDataNode.Var_motor_brake[4].pos_hub_mot   = (float)(ComDataNode.motor_brake[4].pos_hub_mot * 0.01);
            VarComDataNode.Var_motor_brake[4].speed_hub_mot = (float)(ComDataNode.motor_brake[4].speed_hub_mot * 1);
            VarComDataNode.Var_motor_brake[4].vol_hub_mot   = (float)(ComDataNode.motor_brake[4].vol_hub_mot * 0.1);
            VarComDataNode.Var_motor_brake[4].cur_hub_mot   = (float)(ComDataNode.motor_brake[4].cur_hub_mot * 0.1);
            VarComDataNode.Var_motor_brake[4].Tem_hub_mot   = (float)(ComDataNode.motor_brake[4].Tem_hub_mot * 0.1);
            VarComDataNode.Var_motor_brake[4].pos_brake     = (float)(ComDataNode.motor_brake[4].pos_brake * 0.01);
            VarComDataNode.Var_motor_brake[4].pos_fbk_sus   = (float)(ComDataNode.motor_brake[4].pos_fbk_sus * 0.01);
            VarComDataNode.Var_motor_brake[4].speed_sus     = (float)(ComDataNode.motor_brake[4].speed_sus);
            VarComDataNode.Var_motor_brake[4].pressure_sus  = (float)(ComDataNode.motor_brake[4].pressure_sus * 0.01);
            VarComDataNode.Var_motor_brake[4].vol_sus       = (float)(ComDataNode.motor_brake[4].vol_sus * 0.1);
            VarComDataNode.Var_motor_brake[4].cur_sus       = (float)(ComDataNode.motor_brake[4].cur_sus * 0.1);
            VarComDataNode.Var_motor_brake[4].Tem_sus       = (float)(ComDataNode.motor_brake[4].Tem_sus * 0.1);
            VarComDataNode.Var_motor_brake[4].stat_hub_mot  = ComDataNode.motor_brake[4].stat_hub_mot;
            VarComDataNode.Var_motor_brake[4].stat_brake    = ComDataNode.motor_brake[4].stat_brake;
            VarComDataNode.Var_motor_brake[4].stat_sus      = ComDataNode.motor_brake[4].stat_sus;
            break;

    case 0x6A:
            i = 6;
            ComDataNode.motor_brake[5].pos_hub_mot      = Byte_To_I16(data_tmp + i);               i = i + 2;
            ComDataNode.motor_brake[5].speed_hub_mot    = Byte_To_I16(data_tmp + i);               i = i + 2;
            ComDataNode.motor_brake[5].vol_hub_mot      = Byte_To_I16(data_tmp + i);               i = i + 2;
            ComDataNode.motor_brake[5].cur_hub_mot      = Byte_To_I16(data_tmp + i);               i = i + 2;
            ComDataNode.motor_brake[5].Tem_hub_mot      = Byte_To_I16(data_tmp + i);               i = i + 2;
            ComDataNode.motor_brake[5].pos_brake        = Byte_To_I16(data_tmp + i);               i = i + 2;
            ComDataNode.motor_brake[5].pos_fbk_sus      = Byte_To_I16(data_tmp + i);               i = i + 2;
            ComDataNode.motor_brake[5].speed_sus        = Byte_To_I16(data_tmp + i);               i = i + 2;
            ComDataNode.motor_brake[5].pressure_sus     = Byte_To_I16(data_tmp + i);               i = i + 2;
            ComDataNode.motor_brake[5].vol_sus          = Byte_To_I16(data_tmp + i);               i = i + 2;
            ComDataNode.motor_brake[5].cur_sus          = Byte_To_I16(data_tmp + i);               i = i + 2;
            ComDataNode.motor_brake[5].Tem_sus          = Byte_To_I16(data_tmp + i);               i = i + 2;
            ComDataNode.motor_brake[5].stat_hub_mot     = (data_tmp[i]);               i = i + 1;
            ComDataNode.motor_brake[5].stat_brake       = (data_tmp[i]);               i = i + 1;
            ComDataNode.motor_brake[5].stat_sus         = (data_tmp[i]);               i = i + 1;

            VarComDataNode.Var_motor_brake[5].pos_hub_mot   = (float)(ComDataNode.motor_brake[5].pos_hub_mot * 0.01);
            VarComDataNode.Var_motor_brake[5].speed_hub_mot = (float)(ComDataNode.motor_brake[5].speed_hub_mot * 1);
            VarComDataNode.Var_motor_brake[5].vol_hub_mot   = (float)(ComDataNode.motor_brake[5].vol_hub_mot * 0.1);
            VarComDataNode.Var_motor_brake[5].cur_hub_mot   = (float)(ComDataNode.motor_brake[5].cur_hub_mot * 0.1);
            VarComDataNode.Var_motor_brake[5].Tem_hub_mot   = (float)(ComDataNode.motor_brake[5].Tem_hub_mot * 0.1);
            VarComDataNode.Var_motor_brake[5].pos_brake     = (float)(ComDataNode.motor_brake[5].pos_brake * 0.01);
            VarComDataNode.Var_motor_brake[5].pos_fbk_sus   = (float)(ComDataNode.motor_brake[5].pos_fbk_sus * 0.01);
            VarComDataNode.Var_motor_brake[5].speed_sus     = (float)(ComDataNode.motor_brake[5].speed_sus);
            VarComDataNode.Var_motor_brake[5].pressure_sus  = (float)(ComDataNode.motor_brake[5].pressure_sus * 0.01);
            VarComDataNode.Var_motor_brake[5].vol_sus       = (float)(ComDataNode.motor_brake[5].vol_sus * 0.1);
            VarComDataNode.Var_motor_brake[5].cur_sus       = (float)(ComDataNode.motor_brake[5].cur_sus * 0.1);
            VarComDataNode.Var_motor_brake[5].Tem_sus       = (float)(ComDataNode.motor_brake[5].Tem_sus * 0.1);
            VarComDataNode.Var_motor_brake[5].stat_hub_mot  = ComDataNode.motor_brake[5].stat_hub_mot;
            VarComDataNode.Var_motor_brake[5].stat_brake    = ComDataNode.motor_brake[5].stat_brake;
            VarComDataNode.Var_motor_brake[5].stat_sus      = ComDataNode.motor_brake[5].stat_sus;
            break;

    case 0x6C:
            i = 6;
            ComDataNode.motor_brake[6].pos_hub_mot      = Byte_To_I16(data_tmp + i);               i = i + 2;
            ComDataNode.motor_brake[6].speed_hub_mot    = Byte_To_I16(data_tmp + i);               i = i + 2;
            ComDataNode.motor_brake[6].vol_hub_mot      = Byte_To_I16(data_tmp + i);               i = i + 2;
            ComDataNode.motor_brake[6].cur_hub_mot      = Byte_To_I16(data_tmp + i);               i = i + 2;
            ComDataNode.motor_brake[6].Tem_hub_mot      = Byte_To_I16(data_tmp + i);               i = i + 2;
            ComDataNode.motor_brake[6].pos_brake        = Byte_To_I16(data_tmp + i);               i = i + 2;
            ComDataNode.motor_brake[6].pos_fbk_sus      = Byte_To_I16(data_tmp + i);               i = i + 2;
            ComDataNode.motor_brake[6].speed_sus        = Byte_To_I16(data_tmp + i);               i = i + 2;
            ComDataNode.motor_brake[6].pressure_sus     = Byte_To_I16(data_tmp + i);               i = i + 2;
            ComDataNode.motor_brake[6].vol_sus          = Byte_To_I16(data_tmp + i);               i = i + 2;
            ComDataNode.motor_brake[6].cur_sus          = Byte_To_I16(data_tmp + i);               i = i + 2;
            ComDataNode.motor_brake[6].Tem_sus          = Byte_To_I16(data_tmp + i);               i = i + 2;
            ComDataNode.motor_brake[6].stat_hub_mot     = (data_tmp[i]);               i = i + 1;
            ComDataNode.motor_brake[6].stat_brake       = (data_tmp[i]);               i = i + 1;
            ComDataNode.motor_brake[6].stat_sus         = (data_tmp[i]);               i = i + 1;

            VarComDataNode.Var_motor_brake[6].pos_hub_mot   = (float)(ComDataNode.motor_brake[6].pos_hub_mot * 0.01);
            VarComDataNode.Var_motor_brake[6].speed_hub_mot = (float)(ComDataNode.motor_brake[6].speed_hub_mot * 1);
            VarComDataNode.Var_motor_brake[6].vol_hub_mot   = (float)(ComDataNode.motor_brake[6].vol_hub_mot * 0.1);
            VarComDataNode.Var_motor_brake[6].cur_hub_mot   = (float)(ComDataNode.motor_brake[6].cur_hub_mot * 0.1);
            VarComDataNode.Var_motor_brake[6].Tem_hub_mot   = (float)(ComDataNode.motor_brake[6].Tem_hub_mot * 0.1);
            VarComDataNode.Var_motor_brake[6].pos_brake     = (float)(ComDataNode.motor_brake[6].pos_brake * 0.01);
            VarComDataNode.Var_motor_brake[6].pos_fbk_sus   = (float)(ComDataNode.motor_brake[6].pos_fbk_sus * 0.01);
            VarComDataNode.Var_motor_brake[6].speed_sus     = (float)(ComDataNode.motor_brake[6].speed_sus);
            VarComDataNode.Var_motor_brake[6].pressure_sus  = (float)(ComDataNode.motor_brake[6].pressure_sus * 0.01);
            VarComDataNode.Var_motor_brake[6].vol_sus       = (float)(ComDataNode.motor_brake[6].vol_sus * 0.1);
            VarComDataNode.Var_motor_brake[6].cur_sus       = (float)(ComDataNode.motor_brake[6].cur_sus * 0.1);
            VarComDataNode.Var_motor_brake[6].Tem_sus       = (float)(ComDataNode.motor_brake[6].Tem_sus * 0.1);
            VarComDataNode.Var_motor_brake[6].stat_hub_mot  = ComDataNode.motor_brake[6].stat_hub_mot;
            VarComDataNode.Var_motor_brake[6].stat_brake    = ComDataNode.motor_brake[6].stat_brake;
            VarComDataNode.Var_motor_brake[6].stat_sus      = ComDataNode.motor_brake[6].stat_sus;
            break;

    case 0x6F:
            i = 6;
            ComDataNode.motor_brake[7].pos_hub_mot      = Byte_To_I16(data_tmp + i);               i = i + 2;
            ComDataNode.motor_brake[7].speed_hub_mot    = Byte_To_I16(data_tmp + i);               i = i + 2;
            ComDataNode.motor_brake[7].vol_hub_mot      = Byte_To_I16(data_tmp + i);               i = i + 2;
            ComDataNode.motor_brake[7].cur_hub_mot      = Byte_To_I16(data_tmp + i);               i = i + 2;
            ComDataNode.motor_brake[7].Tem_hub_mot      = Byte_To_I16(data_tmp + i);               i = i + 2;
            ComDataNode.motor_brake[7].pos_brake        = Byte_To_I16(data_tmp + i);               i = i + 2;
            ComDataNode.motor_brake[7].pos_fbk_sus      = Byte_To_I16(data_tmp + i);               i = i + 2;
            ComDataNode.motor_brake[7].speed_sus        = Byte_To_I16(data_tmp + i);               i = i + 2;
            ComDataNode.motor_brake[7].pressure_sus     = Byte_To_I16(data_tmp + i);               i = i + 2;
            ComDataNode.motor_brake[7].vol_sus          = Byte_To_I16(data_tmp + i);               i = i + 2;
            ComDataNode.motor_brake[7].cur_sus          = Byte_To_I16(data_tmp + i);               i = i + 2;
            ComDataNode.motor_brake[7].Tem_sus          = Byte_To_I16(data_tmp + i);               i = i + 2;
            ComDataNode.motor_brake[7].stat_hub_mot     = (data_tmp[i]);               i = i + 1;
            ComDataNode.motor_brake[7].stat_brake       = (data_tmp[i]);               i = i + 1;
            ComDataNode.motor_brake[7].stat_sus         = (data_tmp[i]);               i = i + 1;

            VarComDataNode.Var_motor_brake[7].pos_hub_mot   = (float)(ComDataNode.motor_brake[7].pos_hub_mot * 0.01);
            VarComDataNode.Var_motor_brake[7].speed_hub_mot = (float)(ComDataNode.motor_brake[7].speed_hub_mot * 1);
            VarComDataNode.Var_motor_brake[7].vol_hub_mot   = (float)(ComDataNode.motor_brake[7].vol_hub_mot * 0.1);
            VarComDataNode.Var_motor_brake[7].cur_hub_mot   = (float)(ComDataNode.motor_brake[7].cur_hub_mot * 0.1);
            VarComDataNode.Var_motor_brake[7].Tem_hub_mot   = (float)(ComDataNode.motor_brake[7].Tem_hub_mot * 0.1);
            VarComDataNode.Var_motor_brake[7].pos_brake     = (float)(ComDataNode.motor_brake[7].pos_brake * 0.01);
            VarComDataNode.Var_motor_brake[7].pos_fbk_sus   = (float)(ComDataNode.motor_brake[7].pos_fbk_sus * 0.01);
            VarComDataNode.Var_motor_brake[7].speed_sus     = (float)(ComDataNode.motor_brake[7].speed_sus);
            VarComDataNode.Var_motor_brake[7].pressure_sus  = (float)(ComDataNode.motor_brake[7].pressure_sus * 0.01);
            VarComDataNode.Var_motor_brake[7].vol_sus       = (float)(ComDataNode.motor_brake[7].vol_sus * 0.1);
            VarComDataNode.Var_motor_brake[7].cur_sus       = (float)(ComDataNode.motor_brake[7].cur_sus * 0.1);
            VarComDataNode.Var_motor_brake[7].Tem_sus       = (float)(ComDataNode.motor_brake[7].Tem_sus * 0.1);
            VarComDataNode.Var_motor_brake[7].stat_hub_mot  = ComDataNode.motor_brake[7].stat_hub_mot;
            VarComDataNode.Var_motor_brake[7].stat_brake    = ComDataNode.motor_brake[7].stat_brake;
            VarComDataNode.Var_motor_brake[7].stat_sus      = ComDataNode.motor_brake[7].stat_sus;
            break;
    case 0x5A:
        i = 6;
        ComDataNode.D_5A.oil_tank   = Byte_To_U16(data_tmp + i);               i = i + 2;
        ComDataNode.D_5A.speed_Eng  = Byte_To_I16(data_tmp + i);               i = i + 2;
        ComDataNode.D_5A.Tem_Eng    = Byte_To_I16(data_tmp + i);               i = i + 2;
        ComDataNode.D_5A.tor_Eng    = Byte_To_I16(data_tmp + i);               i = i + 2;
        ComDataNode.D_5A.speed_Gen  = Byte_To_I16(data_tmp + i);               i = i + 2;
        ComDataNode.D_5A.cur_Gen    = Byte_To_I16(data_tmp + i);               i = i + 2;
        ComDataNode.D_5A.vol_Gen    = Byte_To_I16(data_tmp + i);               i = i + 2;
        ComDataNode.D_5A.Tem_Gen    = Byte_To_I16(data_tmp + i);               i = i + 2;
        ComDataNode.D_5A.stat_Gen   = (data_tmp[i]);               i = i + 1;
        ComDataNode.D_5A.SOC_bat_1  = Byte_To_I16(data_tmp + i);               i = i + 2;
        ComDataNode.D_5A.vol_bat_1  = Byte_To_I16(data_tmp + i);               i = i + 2;
        ComDataNode.D_5A.cur_bat_1  = Byte_To_I16(data_tmp + i);               i = i + 2;
        ComDataNode.D_5A.Tem_bat_1  = Byte_To_I16(data_tmp + i);               i = i + 2;
        ComDataNode.D_5A.stat_bat_1 = (data_tmp[i]);               i = i + 1;
        ComDataNode.D_5A.SOC_bat_2  = Byte_To_I16(data_tmp + i);               i = i + 2;
        ComDataNode.D_5A.cur_bat_2  = Byte_To_I16(data_tmp + i);               i = i + 2;
        ComDataNode.D_5A.vol_bat_2  = Byte_To_I16(data_tmp + i);               i = i + 2;
        ComDataNode.D_5A.Tem_bat_2  = Byte_To_I16(data_tmp + i);               i = i + 2;
        ComDataNode.D_5A.stat_bat_2 = (data_tmp[i]);               i = i + 1;

//        VarComDataNode.Var_D_5A.oil_tank = (float)(ComDataNode.D_5A.oil_tank * 1);
//        VarComDataNode.Var_D_5A.speed_Eng = (float)(ComDataNode.D_5A.speed_Eng * 1);
//        VarComDataNode.Var_D_5A.Tem_Eng = (float)(ComDataNode.D_5A.Tem_Eng * 0.1);
//        VarComDataNode.Var_D_5A.tor_Eng = (float)(ComDataNode.D_5A.tor_Eng);
//        VarComDataNode.Var_D_5A.control_speed_Eng = (float)(ComDataNode.D_5A.speed_Gen * 1);
//        VarComDataNode.Var_D_5A.cur_Gen = (float)(ComDataNode.D_5A.cur_Gen * 0.1);
//        VarComDataNode.Var_D_5A.vol_Gen = (float)(ComDataNode.D_5A.vol_Gen * 0.1);
//        VarComDataNode.Var_D_5A.Tem_Gen = (float)(ComDataNode.D_5A.Tem_Gen * 0.1);
//        VarComDataNode.Var_D_5A.stat_Gen = ComDataNode.D_5A.stat_Gen;
//        VarComDataNode.Var_D_5A.SOC_bat_1 = (float)(ComDataNode.D_5A.SOC_bat_1 * 0.1);
//        VarComDataNode.Var_D_5A.vol_bat_1 = (float)(ComDataNode.D_5A.vol_bat_1 * 0.1);
//        VarComDataNode.Var_D_5A.cur_bat_1 = (float)(ComDataNode.D_5A.cur_bat_1 * 0.1);
//        VarComDataNode.Var_D_5A.Tem_bat_1 = (float)(ComDataNode.D_5A.Tem_bat_1 * 0.1);
//        VarComDataNode.Var_D_5A.stat_bat_1 = ComDataNode.D_5A.stat_bat_1;
//        VarComDataNode.Var_D_5A.SOC_bat_2 = (float)(ComDataNode.D_5A.SOC_bat_2 * 0.1);
//        VarComDataNode.Var_D_5A.vol_bat_2 = (float)(ComDataNode.D_5A.vol_bat_2 * 0.1);
//        VarComDataNode.Var_D_5A.cur_bat_2 = (float)(ComDataNode.D_5A.cur_bat_2 * 0.1);
//        VarComDataNode.Var_D_5A.Tem_bat_2 = (float)(ComDataNode.D_5A.Tem_bat_2 * 0.1);
//        VarComDataNode.Var_D_5A.stat_bat_2 = ComDataNode.D_5A.stat_bat_2;
        break;

    case 0x3C:
        i = 6;
        ComDataNode.D_3C.pos_fbk    = Byte_To_I16(data_tmp + i);               i = i + 2;
        ComDataNode.D_3C.speed      = Byte_To_I16(data_tmp + i);               i = i + 2;
        ComDataNode.D_3C.vol        = Byte_To_I16(data_tmp + i);               i = i + 2;
        ComDataNode.D_3C.cur        = Byte_To_I16(data_tmp + i);               i = i + 2;
        ComDataNode.D_3C.Tem        = Byte_To_I16(data_tmp + i);               i = i + 2;
        ComDataNode.D_3C.stat       = (data_tmp[i]);               i = i + 1;

        VarComDataNode.Var_D_3C.pos_fbk = (float)(ComDataNode.D_3C.pos_fbk * 0.01);
        VarComDataNode.Var_D_3C.speed_fbk = (float)(ComDataNode.D_3C.speed * 0.1);
        VarComDataNode.Var_D_3C.vol = (float)(ComDataNode.D_3C.vol * 0.1);
        VarComDataNode.Var_D_3C.cur = (float)(ComDataNode.D_3C.cur * 0.1);
        VarComDataNode.Var_D_3C.Tem = (float)(ComDataNode.D_3C.Tem *0.1);
        VarComDataNode.Var_D_3C.stat = ComDataNode.D_3C.stat;
        break;

        default:
            break;
    }
}

void Tx_Proc(unsigned int ch)
{
    unsigned int i;
    unsigned int j = 0;
    static unsigned long CRC32C_T = 0;
    unsigned char frameID_buf[7]={0xC3, 0xA5, 0xA6, 0x9C,0x60, 0x5A, 0x3C};
    static unsigned int ID_i = 0;
    unsigned char frameID;
    frameID =  0xC3;//frameID_buf[ID_i];

#ifdef NODE_MASTER_standby_1
    if(send_ini_frame_sig)
    {

    }
    unsigned char frameID_buf[7]={0xC3, 0xA5, 0xA6, 0x9C,0x60, 0x5A, 0x3C};

#endif

/*
    //----------------------------下面是节点之间通信的Var_To_Data--------------------------------------------
        //----------------给定测试时用到的初始值----------------------
        VarComDataNode.Var_C3.Driv_instruction[0] = 0x6A;
        VarComDataNode.Var_C3.Driv_instruction[1] = 0x6A;
        VarComDataNode.Var_C3.Driv_instruction[2] = 0x6A;
        VarComDataNode.Var_C3.Speed_instruction[0] = 6.25;
        VarComDataNode.Var_C3.Speed_instruction[1] = 6.25;
        VarComDataNode.Var_C3.steer_instruction[0] = 0.12;
        VarComDataNode.Var_C3.steer_instruction[1] = 0.12;
        VarComDataNode.Var_C3.attack_instruction[0] = 0x55;
        VarComDataNode.Var_C3.attack_instruction[1] = 0x55;
        VarComDataNode.Var_C3.attack_instruction[2] = 0x55;
        VarComDataNode.Var_C3.target_X = 26;
        VarComDataNode.Var_C3.target_Y = 147;
        VarComDataNode.Var_C3.target_delte_X = 13;
        VarComDataNode.Var_C3.target_delte_Y = 9;
        VarComDataNode.Var_A5.local_instruction = 21.6789;
        VarComDataNode.Var_A5.before_fbk = 55.6;
        VarComDataNode.Var_A5.acc_X = -1.2345;
        VarComDataNode.Var_A5.acc_Y = -2.5789;
        VarComDataNode.Var_A5.acc_Z = -3.7788;
        VarComDataNode.Var_A5.omega_X = -8.5579;
        VarComDataNode.Var_A5.omega_Y = -36.7469;
        VarComDataNode.Var_A5.omega_Z = -79.5763;
        VarComDataNode.Var_A6.lon = 116.30;
        VarComDataNode.Var_A6.lat = 39.95;
        VarComDataNode.Var_A6.alt = -16.25;
        VarComDataNode.Var_A6.course = 124.556;
        VarComDataNode.Var_A6.pitch = -15.8848;
        VarComDataNode.Var_A6.roll = -36.667;
        VarComDataNode.Var_A6.speed_sample_X = -29.9987;
        VarComDataNode.Var_A6.speed_sample_Y = 29.9987;
        VarComDataNode.Var_A6.speed_sample_Z = -5.9167;
        for(j = 0;j < 12;j++)
            VarComDataNode.Var_D_9C.radar_data[j] = 1.21+0.01*j;
        VarComDataNode.Var_D_9C.milemeter_sig = 2.1;
        VarComDataNode.Var_motor_brake[0].pos_hub_mot = 3.28;
        VarComDataNode.Var_motor_brake[0].speed_hub_mot = 302;
        VarComDataNode.Var_motor_brake[0].vol_hub_mot = 28.6;
        VarComDataNode.Var_motor_brake[0].cur_hub_mot = 3.6;
        VarComDataNode.Var_motor_brake[0].Tem_hub_mot = 48.6;
        VarComDataNode.Var_motor_brake[0].pos_brake = 2.59;
        VarComDataNode.Var_motor_brake[0].pos_fbk_sus = 4.59;
        VarComDataNode.Var_motor_brake[0].speed_sus = 84;
        VarComDataNode.Var_motor_brake[0].pressure_sus = 55.968;
        VarComDataNode.Var_motor_brake[0].vol_sus = 28.6;
        VarComDataNode.Var_motor_brake[0].cur_sus = 4.8;
        VarComDataNode.Var_motor_brake[0].Tem_sus = 40.5;
        VarComDataNode.Var_motor_brake[0].stat_hub_mot = 0xA3;
        VarComDataNode.Var_motor_brake[0].stat_brake = 0xA3;
        VarComDataNode.Var_motor_brake[0].stat_sus = 0x55;
        VarComDataNode.Var_D_5A.oil_tank = 21;
        VarComDataNode.Var_D_5A.speed_Eng = 1205;
        VarComDataNode.Var_D_5A.Tem_Eng = 112.7;
        VarComDataNode.Var_D_5A.tor_Eng = 1147;
        VarComDataNode.Var_D_5A.speed_Gen = 1500;
        VarComDataNode.Var_D_5A.cur_Gen = 4.7;
        VarComDataNode.Var_D_5A.vol_Gen = 23.4;
        VarComDataNode.Var_D_5A.Tem_Gen = 52.4;
        VarComDataNode.Var_D_5A.stat_Gen = 0x55;
        VarComDataNode.Var_D_5A.SOC_bat_1 = 25.5;
        VarComDataNode.Var_D_5A.vol_bat_1 = 19.3;
        VarComDataNode.Var_D_5A.cur_bat_1 = 6.3;
        VarComDataNode.Var_D_5A.Tem_bat_1 = 49.6;
        VarComDataNode.Var_D_5A.stat_bat_1 = 0xA3;
        VarComDataNode.Var_D_5A.SOC_bat_2 = 25.5;
        VarComDataNode.Var_D_5A.vol_bat_2 = 19.4;
        VarComDataNode.Var_D_5A.cur_bat_2 = 6.8;
        VarComDataNode.Var_D_5A.Tem_bat_2 = 49.2;
        VarComDataNode.Var_D_5A.stat_bat_2 = 0x55;
        VarComDataNode.Var_D_3C.pos_fbk = 1.38;
        VarComDataNode.Var_D_3C.speed = 158.6;
        VarComDataNode.Var_D_3C.vol = 24.3;
        VarComDataNode.Var_D_3C.cur = 6.7;
        VarComDataNode.Var_D_3C.Tem = 51.7;
        VarComDataNode.Var_D_3C.stat = 0x55;

        //----------------------测试赋初值结束----------------------------------------
//--------------------------Var To Data---------------------------------------------------
        //---------------C3 帧--------------
        ComDataNode.C3.Driv_instruction[0] = VarComDataNode.Var_C3.Driv_instruction[0];
        ComDataNode.C3.Driv_instruction[1] = VarComDataNode.Var_C3.Driv_instruction[1];
        ComDataNode.C3.Driv_instruction[2] = VarComDataNode.Var_C3.Driv_instruction[2];
        ComDataNode.C3.Speed_instruction[0] = (int16)(VarComDataNode.Var_C3.Speed_instruction[0] * 100);
        ComDataNode.C3.Speed_instruction[1] = (int16)(VarComDataNode.Var_C3.Speed_instruction[1] * 100);
        ComDataNode.C3.steer_instruction[0] = (Uint16)(VarComDataNode.Var_C3.steer_instruction[0] * 100);
        ComDataNode.C3.steer_instruction[1] = (Uint16)(VarComDataNode.Var_C3.steer_instruction[1] * 100);
        ComDataNode.C3.attack_instruction[0] = VarComDataNode.Var_C3.attack_instruction[0];
        ComDataNode.C3.attack_instruction[1] = VarComDataNode.Var_C3.attack_instruction[1];
        ComDataNode.C3.attack_instruction[2] = VarComDataNode.Var_C3.attack_instruction[2];
        ComDataNode.C3.target_X = (int16)(VarComDataNode.Var_C3.target_X);
        ComDataNode.C3.target_Y = (int16)(VarComDataNode.Var_C3.target_Y);
        ComDataNode.C3.target_delte_X = (Uint16)(VarComDataNode.Var_C3.target_delte_X);
        ComDataNode.C3.target_delte_Y = (Uint16)(VarComDataNode.Var_C3.target_delte_Y);

        //---------------A5 帧----------------------
        ComDataNode.A5.local_instruction = (int32)(VarComDataNode.Var_A5.local_instruction * 10000);
        ComDataNode.A5.before_fbk = (int32)(VarComDataNode.Var_A5.before_fbk * 10);
        ComDataNode.A5.acc_X = (int32)(VarComDataNode.Var_A5.acc_X * 10000);
        ComDataNode.A5.acc_Y = (int32)(VarComDataNode.Var_A5.acc_Y * 10000);
        ComDataNode.A5.acc_Z = (int32)(VarComDataNode.Var_A5.acc_Z * 10000);
        ComDataNode.A5.omega_X = (int32)(VarComDataNode.Var_A5.omega_X * 10000);
        ComDataNode.A5.omega_Y = (int32)(VarComDataNode.Var_A5.omega_Y * 10000);
        ComDataNode.A5.omega_Z = (int32)(VarComDataNode.Var_A5.omega_Z * 10000);

        //---------------A6 帧------------------
        ComDataNode.A6.lon = (int32)(VarComDataNode.Var_A6.lon * 1e7);
        ComDataNode.A6.lat = (int32)(VarComDataNode.Var_A6.lat * 1e7);
        ComDataNode.A6.alt = (int32)(VarComDataNode.Var_A6.alt * 1e4);
        ComDataNode.A6.course = (int32)(VarComDataNode.Var_A6.course * 1e4);
        ComDataNode.A6.pitch = (int32)(VarComDataNode.Var_A6.pitch * 1e4);
        ComDataNode.A6.roll = (int32)(VarComDataNode.Var_A6.roll * 1e4);
        ComDataNode.A6.speed_sample_X = (int32)(VarComDataNode.Var_A6.speed_sample_X * 1e4);
        ComDataNode.A6.speed_sample_Y = (int32)(VarComDataNode.Var_A6.speed_sample_Y * 1e4);
        ComDataNode.A6.speed_sample_Z = (int32)(VarComDataNode.Var_A6.speed_sample_Z * 1e4);

        //----------------9C 帧-------------------
        ComDataNode.D_9C.radar_data[0] = (int16)(VarComDataNode.Var_D_9C.radar_data[0] * 100);
        ComDataNode.D_9C.radar_data[1] = (int16)(VarComDataNode.Var_D_9C.radar_data[1] * 100);
        ComDataNode.D_9C.radar_data[2] = (int16)(VarComDataNode.Var_D_9C.radar_data[2] * 100);
        ComDataNode.D_9C.radar_data[3] = (int16)(VarComDataNode.Var_D_9C.radar_data[3] * 100);
        ComDataNode.D_9C.radar_data[4] = (int16)(VarComDataNode.Var_D_9C.radar_data[4] * 100);
        ComDataNode.D_9C.radar_data[5] = (int16)(VarComDataNode.Var_D_9C.radar_data[5] * 100);
        ComDataNode.D_9C.radar_data[6] = (int16)(VarComDataNode.Var_D_9C.radar_data[6] * 100);
        ComDataNode.D_9C.radar_data[7] = (int16)(VarComDataNode.Var_D_9C.radar_data[7] * 100);
        ComDataNode.D_9C.radar_data[8] = (int16)(VarComDataNode.Var_D_9C.radar_data[8] * 100);
        ComDataNode.D_9C.radar_data[9] = (int16)(VarComDataNode.Var_D_9C.radar_data[9] * 100);
        ComDataNode.D_9C.radar_data[10] = (int16)(VarComDataNode.Var_D_9C.radar_data[10] * 100);
        ComDataNode.D_9C.radar_data[11] = (int16)(VarComDataNode.Var_D_9C.radar_data[11] * 100);
        ComDataNode.D_9C.milemeter_sig = (int16)(VarComDataNode.Var_D_9C.milemeter_sig * 10);

        //------------------60 帧------------------------
        ComDataNode.motor_brake[0].pos_hub_mot = (int16)(VarComDataNode.Var_motor_brake[0].pos_hub_mot * 100);
        ComDataNode.motor_brake[0].speed_hub_mot = (int16)(VarComDataNode.Var_motor_brake[0].speed_hub_mot * 1);
        ComDataNode.motor_brake[0].vol_hub_mot = (int16)(VarComDataNode.Var_motor_brake[0].vol_hub_mot * 10);
        ComDataNode.motor_brake[0].cur_hub_mot = (int16)(VarComDataNode.Var_motor_brake[0].cur_hub_mot * 10);
        ComDataNode.motor_brake[0].Tem_hub_mot = (int16)(VarComDataNode.Var_motor_brake[0].Tem_hub_mot *10);
        ComDataNode.motor_brake[0].pos_brake = (int16)(VarComDataNode.Var_motor_brake[0].pos_brake * 100);
        ComDataNode.motor_brake[0].pos_fbk_sus = (int16)(VarComDataNode.Var_motor_brake[0].pos_fbk_sus * 100);
        ComDataNode.motor_brake[0].speed_sus = (int16)(VarComDataNode.Var_motor_brake[0].speed_sus * 1);
        ComDataNode.motor_brake[0].pressure_sus = (int16)(VarComDataNode.Var_motor_brake[0].pressure_sus * 100);
        ComDataNode.motor_brake[0].vol_sus = (int16)(VarComDataNode.Var_motor_brake[0].vol_sus * 10);
        ComDataNode.motor_brake[0].cur_sus = (int16)(VarComDataNode.Var_motor_brake[0].cur_sus * 10);
        ComDataNode.motor_brake[0].Tem_sus = (int16)(VarComDataNode.Var_motor_brake[0].Tem_sus * 10);
        ComDataNode.motor_brake[0].stat_hub_mot = VarComDataNode.Var_motor_brake[0].stat_hub_mot;
        ComDataNode.motor_brake[0].stat_brake = VarComDataNode.Var_motor_brake[0].stat_brake;
        ComDataNode.motor_brake[0].stat_sus = VarComDataNode.Var_motor_brake[0].stat_sus;

        //------------------5A 帧---------------------------
        ComDataNode.D_5A.oil_tank = (Uint16)(VarComDataNode.Var_D_5A.oil_tank * 1);
        ComDataNode.D_5A.speed_Eng = (int16)(VarComDataNode.Var_D_5A.speed_Eng * 1);
        ComDataNode.D_5A.Tem_Eng = (int16)(VarComDataNode.Var_D_5A.Tem_Eng * 10);
        ComDataNode.D_5A.tor_Eng = (int16)(VarComDataNode.Var_D_5A.tor_Eng * 1);
        ComDataNode.D_5A.speed_Gen = (int16)(VarComDataNode.Var_D_5A.speed_Gen * 1);
        ComDataNode.D_5A.cur_Gen = (int16)(VarComDataNode.Var_D_5A.cur_Gen * 10);
        ComDataNode.D_5A.vol_Gen = (int16)(VarComDataNode.Var_D_5A.vol_Gen * 10);
        ComDataNode.D_5A.Tem_Gen = (int16)(VarComDataNode.Var_D_5A.Tem_Gen * 10);
        ComDataNode.D_5A.stat_Gen = (VarComDataNode.Var_D_5A.stat_Gen );
        ComDataNode.D_5A.SOC_bat_1 = (int16)(VarComDataNode.Var_D_5A.SOC_bat_1 * 10);
        ComDataNode.D_5A.vol_bat_1 = (int16)(VarComDataNode.Var_D_5A.vol_bat_1 * 10);
        ComDataNode.D_5A.cur_bat_1 = (int16)(VarComDataNode.Var_D_5A.cur_bat_1 * 10);
        ComDataNode.D_5A.Tem_bat_1 = (int16)(VarComDataNode.Var_D_5A.Tem_bat_1 * 10);
        ComDataNode.D_5A.stat_bat_1 = (VarComDataNode.Var_D_5A.stat_bat_1 );
        ComDataNode.D_5A.SOC_bat_2 = (int16)(VarComDataNode.Var_D_5A.SOC_bat_2 * 10);
        ComDataNode.D_5A.cur_bat_2 = (int16)(VarComDataNode.Var_D_5A.cur_bat_2 * 10);
        ComDataNode.D_5A.vol_bat_2 = (int16)(VarComDataNode.Var_D_5A.vol_bat_2 * 10);
        ComDataNode.D_5A.Tem_bat_2 = (int16)(VarComDataNode.Var_D_5A.Tem_bat_2 * 10);
        ComDataNode.D_5A.stat_bat_2 = (VarComDataNode.Var_D_5A.stat_bat_2 );

        //---------------3C 帧 ---------------------------
        ComDataNode.D_3C.pos_fbk = (int16)(VarComDataNode.Var_D_3C.pos_fbk * 100);
        ComDataNode.D_3C.speed = (int16)(VarComDataNode.Var_D_3C.speed * 10);
        ComDataNode.D_3C.vol = (int16)(VarComDataNode.Var_D_3C.vol * 10);
        ComDataNode.D_3C.cur = (int16)(VarComDataNode.Var_D_3C.cur * 10);
        ComDataNode.D_3C.Tem = (int16)(VarComDataNode.Var_D_3C.Tem * 10);
        ComDataNode.D_3C.stat = (VarComDataNode.Var_D_3C.stat );
*/

    //============================Data To Buf=============================================
        TX_A.len = 48;
        TX_A.buf[0] = 0xEB;
        TX_A.buf[1] = 0x90;
        TX_A.buf[2] = frameID;
//        i = 0;
//        for(i = 0;i <41; i++)
//            TX_A.buf[3+i] = i;

        TX_A.buf[3] = ComDataNode.address_ID;
        i = 4;
        U16_To_Byte(TX_A.buf + i ,ComDataNode.frame_cnt);        i= i + 2;
        TX_A.buf[i] = ComDataNode.Timer_100us;       i = i + 1;
        TX_A.buf[i] = ComDataNode.Tx_TimeCNT;       i = i + 1;
//        U16_To_Byte(TX_A.buf + i ,ComDataNode.local_time);       i= i + 2;
        switch(frameID)
        {
            case 0xC3:
                i = 8;
//                ComDataNode.frame_cnt = ComDataNode.frame_cnt + 1;
                TX_A.buf[i] = ComDataNode.C3.Driv_instruction[0];       i = i + 1;
                TX_A.buf[i] = ComDataNode.C3.Driv_instruction[1];       i = i + 1;
                TX_A.buf[i] = ComDataNode.C3.Driv_instruction[2];       i = i + 1;
                I16_To_Byte(TX_A.buf + i ,ComDataNode.C3.Speed_instruction[0]);       i = i + 2;
                I16_To_Byte(TX_A.buf + i ,ComDataNode.C3.Speed_instruction[1]);       i = i + 2;
                U16_To_Byte(TX_A.buf + i ,ComDataNode.C3.steer_instruction[0]);       i = i + 2;
                U16_To_Byte(TX_A.buf + i ,ComDataNode.C3.steer_instruction[1]);       i = i + 2;
                TX_A.buf[i] = ComDataNode.C3.attack_instruction[0];       i = i + 1;
                TX_A.buf[i] = ComDataNode.C3.attack_instruction[1];       i = i + 1;
                TX_A.buf[i] = ComDataNode.C3.attack_instruction[2];       i = i + 1;
                I16_To_Byte(TX_A.buf + i ,ComDataNode.C3.target_X);                   i = i + 2;
                I16_To_Byte(TX_A.buf + i ,ComDataNode.C3.target_Y);                   i = i + 2;
                I16_To_Byte(TX_A.buf + i ,ComDataNode.C3.target_delte_X);             i = i + 2;
                I16_To_Byte(TX_A.buf + i ,ComDataNode.C3.target_delte_Y);             i = i + 2;
                break;

            case 0xA5:
                i = 8;
                I32_To_Byte(TX_A.buf + i ,ComDataNode.A5.local_instruction);      i = i + 4;
                I32_To_Byte(TX_A.buf + i ,ComDataNode.A5.before_fbk);             i = i + 4;
                I32_To_Byte(TX_A.buf + i ,ComDataNode.A5.acc_X);                  i = i + 4;
                I32_To_Byte(TX_A.buf + i ,ComDataNode.A5.acc_Y);                  i = i + 4;
                I32_To_Byte(TX_A.buf + i ,ComDataNode.A5.acc_Z);                  i = i + 4;
                I32_To_Byte(TX_A.buf + i ,ComDataNode.A5.omega_X);                i = i + 4;
                I32_To_Byte(TX_A.buf + i ,ComDataNode.A5.omega_Y);                i = i + 4;
                I32_To_Byte(TX_A.buf + i ,ComDataNode.A5.omega_Z);                i = i + 4;
                break;

            case 0xA6:
                i = 8;
                I32_To_Byte(TX_A.buf + i ,ComDataNode.A6.lon);                            i = i + 4;
                I32_To_Byte(TX_A.buf + i ,ComDataNode.A6.lat);                            i = i + 4;
                I32_To_Byte(TX_A.buf + i ,ComDataNode.A6.alt);                            i = i + 4;
                I32_To_Byte(TX_A.buf + i ,ComDataNode.A6.course);                         i = i + 4;
                I32_To_Byte(TX_A.buf + i ,ComDataNode.A6.roll);                           i = i + 4;
                I32_To_Byte(TX_A.buf + i ,ComDataNode.A6.pitch);                          i = i + 4;
                I32_To_Byte(TX_A.buf + i ,ComDataNode.A6.speed_sample_X);                 i = i + 4;
                I32_To_Byte(TX_A.buf + i ,ComDataNode.A6.speed_sample_Y);                 i = i + 4;
                I32_To_Byte(TX_A.buf + i ,ComDataNode.A6.speed_sample_Z);                 i = i + 4;
                break;
#ifndef NODE_GCS
            case 0x9C:
                i = 8;
                I16_To_Byte(TX_A.buf + i ,ComDataNode.D_9C.radar_data[0]);                 i = i + 2;
                I16_To_Byte(TX_A.buf + i ,ComDataNode.D_9C.radar_data[1]);                 i = i + 2;
                I16_To_Byte(TX_A.buf + i ,ComDataNode.D_9C.radar_data[2]);                 i = i + 2;
                I16_To_Byte(TX_A.buf + i ,ComDataNode.D_9C.radar_data[3]);                 i = i + 2;
                I16_To_Byte(TX_A.buf + i ,ComDataNode.D_9C.radar_data[4]);                 i = i + 2;
                I16_To_Byte(TX_A.buf + i ,ComDataNode.D_9C.radar_data[5]);                 i = i + 2;
                I16_To_Byte(TX_A.buf + i ,ComDataNode.D_9C.radar_data[6]);                 i = i + 2;
                I16_To_Byte(TX_A.buf + i ,ComDataNode.D_9C.radar_data[7]);                 i = i + 2;
                I16_To_Byte(TX_A.buf + i ,ComDataNode.D_9C.radar_data[8]);                 i = i + 2;
                I16_To_Byte(TX_A.buf + i ,ComDataNode.D_9C.radar_data[9]);                 i = i + 2;
                I16_To_Byte(TX_A.buf + i ,ComDataNode.D_9C.radar_data[10]);                 i = i + 2;
                I16_To_Byte(TX_A.buf + i ,ComDataNode.D_9C.radar_data[11]);                 i = i + 2;
                I16_To_Byte(TX_A.buf + i ,ComDataNode.D_9C.milemeter_sig);                 i = i + 2;
                break;

            case 0x60:
                i = 8;
                I16_To_Byte(TX_A.buf + i ,ComDataNode.motor_brake[0].pos_hub_mot);                  i = i + 2;
                I16_To_Byte(TX_A.buf + i ,ComDataNode.motor_brake[0].speed_hub_mot);                i = i + 2;
                I16_To_Byte(TX_A.buf + i ,ComDataNode.motor_brake[0].vol_hub_mot);                  i = i + 2;
                I16_To_Byte(TX_A.buf + i ,ComDataNode.motor_brake[0].cur_hub_mot);                  i = i + 2;
                I16_To_Byte(TX_A.buf + i ,ComDataNode.motor_brake[0].Tem_hub_mot);                  i = i + 2;
                I16_To_Byte(TX_A.buf + i ,ComDataNode.motor_brake[0].pos_brake);                    i = i + 2;
                I16_To_Byte(TX_A.buf + i ,ComDataNode.motor_brake[0].pos_fbk_sus);                  i = i + 2;
                I16_To_Byte(TX_A.buf + i ,ComDataNode.motor_brake[0].speed_sus);                    i = i + 2;
                I16_To_Byte(TX_A.buf + i ,ComDataNode.motor_brake[0].pressure_sus);                 i = i + 2;
                I16_To_Byte(TX_A.buf + i ,ComDataNode.motor_brake[0].vol_sus);                      i = i + 2;
                I16_To_Byte(TX_A.buf + i ,ComDataNode.motor_brake[0].cur_sus);                      i = i + 2;
                I16_To_Byte(TX_A.buf + i ,ComDataNode.motor_brake[0].Tem_sus);                      i = i + 2;
                TX_A.buf[i] = ComDataNode.motor_brake[0].stat_hub_mot;                              i = i + 1;
                TX_A.buf[i] = ComDataNode.motor_brake[0].stat_brake;                                i = i + 1;
                TX_A.buf[i] = ComDataNode.motor_brake[0].stat_sus;                                  i = i + 1;
                break;

            case 0x5A:
                i = 8;
                U16_To_Byte(TX_A.buf + i ,ComDataNode.D_5A.oil_tank);       i = i + 2;
                I16_To_Byte(TX_A.buf + i ,ComDataNode.D_5A.speed_Eng);      i = i + 2;
                I16_To_Byte(TX_A.buf + i ,ComDataNode.D_5A.Tem_Eng);        i = i + 2;
                I16_To_Byte(TX_A.buf + i ,ComDataNode.D_5A.tor_Eng);        i = i + 2;
                I16_To_Byte(TX_A.buf + i ,ComDataNode.D_5A.speed_Gen);      i = i + 2;
                I16_To_Byte(TX_A.buf + i ,ComDataNode.D_5A.cur_Gen);        i = i + 2;
                I16_To_Byte(TX_A.buf + i ,ComDataNode.D_5A.vol_Gen);        i = i + 2;
                I16_To_Byte(TX_A.buf + i ,ComDataNode.D_5A.Tem_Gen);        i = i + 2;
                TX_A.buf[i] = ComDataNode.D_5A.stat_Gen;            i = i + 1;
                I16_To_Byte(TX_A.buf + i ,ComDataNode.D_5A.SOC_bat_1);      i = i + 2;
                I16_To_Byte(TX_A.buf + i ,ComDataNode.D_5A.vol_bat_1);      i = i + 2;
                I16_To_Byte(TX_A.buf + i ,ComDataNode.D_5A.cur_bat_1);      i = i + 2;
                I16_To_Byte(TX_A.buf + i ,ComDataNode.D_5A.Tem_bat_1);      i = i + 2;
                TX_A.buf[i] = ComDataNode.D_5A.stat_bat_1;          i = i + 1;
                I16_To_Byte(TX_A.buf + i ,ComDataNode.D_5A.SOC_bat_2);      i = i + 2;
                I16_To_Byte(TX_A.buf + i ,ComDataNode.D_5A.vol_bat_2);      i = i + 2;
                I16_To_Byte(TX_A.buf + i ,ComDataNode.D_5A.cur_bat_2);      i = i + 2;
                I16_To_Byte(TX_A.buf + i ,ComDataNode.D_5A.Tem_bat_2);      i = i + 2;
                TX_A.buf[i] = ComDataNode.D_5A.stat_bat_2;          i = i + 1;
                break;

            case 0x3C:
                i = 8;
                I16_To_Byte(TX_A.buf + i ,ComDataNode.D_3C.pos_fbk);        i = i + 2;
                I16_To_Byte(TX_A.buf + i ,ComDataNode.D_3C.speed);          i = i + 2;
                I16_To_Byte(TX_A.buf + i ,ComDataNode.D_3C.vol);            i = i + 2;
                I16_To_Byte(TX_A.buf + i ,ComDataNode.D_3C.cur);            i = i + 2;
                I16_To_Byte(TX_A.buf + i ,ComDataNode.D_3C.Tem);            i = i + 2;
                I16_To_Byte(TX_A.buf + i ,ComDataNode.D_3C.stat);           i = i + 2;
                break;
#endif
            default: break;
        }

    //    frame_ID = 0xC5;

        CRC32C_T = crc32(TX_A.buf + 2, TX_A.len-6);
        U32_To_Byte(TX_A.buf + TX_A.len - 4, CRC32C_T);            i = i + 4;


//============================Trigger ISR=============================================
    switch(ch)
    {

    case 0:
        ID_i = (ID_i + 1)%7;
        RX_A_DIS;
        TX_A_EN;
        GpioDataRegs.GPCDAT.bit.GPIO94 = 1;
        Com_A.Tx_State = 1;
        Com_A.Timer_Tx = 0;
        GpioDataRegs.GPDDAT.bit.GPIO115 = ~GpioDataRegs.GPDDAT.bit.GPIO115;
        /*    清错误标志，串口发送前进行软复位，防止复位影响发送缓存数据         */
        if( SciaRegs.SCIRXST.bit.RXERROR )
        {
            SciaRegs.SCICTL1.bit.SWRESET = 0;
            SciaRegs.SCICTL1.bit.SWRESET = 1;
        }

        // 清串口发送中断标志
        SciaRegs.SCIFFTX.bit.TXFIFORESET = 1;
        SciaRegs.SCIFFTX.bit.TXFFINTCLR = 1;                    // Clear SCI Interrupt Flag 清串口发送中断标志，发送
        break;
    case 1:
        RX_B_DIS;
        TX_B_EN;
        Com_B.Tx_State = 1;
        Com_B.Timer_Tx = 0;
        /*    清错误标志，串口发送前进行软复位，防止复位影响发送缓存数据         */
        if( ScibRegs.SCIRXST.bit.RXERROR )
        {
            ScibRegs.SCICTL1.bit.SWRESET = 0;
            ScibRegs.SCICTL1.bit.SWRESET = 1;
        }

        // 清串口发送中断标志
        ScibRegs.SCIFFTX.bit.TXFIFORESET = 1;
        ScibRegs.SCIFFTX.bit.TXFFINTCLR = 1;                    // Clear SCI Interrupt Flag 清串口发送中断标志，发送
        break;
    case 2:
        RX_C_DIS;
        TX_C_EN;
        Com_C.Tx_State = 1;
        Com_C.Timer_Tx = 0;
        /*    清错误标志，串口发送前进行软复位，防止复位影响发送缓存数据         */
        if( ScicRegs.SCIRXST.bit.RXERROR )
        {
            ScicRegs.SCICTL1.bit.SWRESET = 0;
            ScicRegs.SCICTL1.bit.SWRESET = 1;
        }

        // 清串口发送中断标志
        ScicRegs.SCIFFTX.bit.TXFIFORESET = 1;
        ScicRegs.SCIFFTX.bit.TXFFINTCLR = 1;                    // Clear SCI Interrupt Flag 清串口发送中断标志，发送
        break;
    case 3:
#ifdef NODE_MASTER
        RX_D_DIS;
        TX_D_EN;
        Com_D.Tx_State = 1;
        Com_D.Timer_Tx = 0;
        /*    清错误标志，串口发送前进行软复位，防止复位影响发送缓存数据         */
        if( ScidRegs.SCIRXST.bit.RXERROR )
        {
            ScidRegs.SCICTL1.bit.SWRESET = 0;
            ScidRegs.SCICTL1.bit.SWRESET = 1;
        }

        // 清串口发送中断标志
        ScidRegs.SCIFFTX.bit.TXFIFORESET = 1;
        ScidRegs.SCIFFTX.bit.TXFFINTCLR = 1;                    // Clear SCI Interrupt Flag 清串口发送中断标志，发送
        break;
#endif
    default:
        break;


    }

}


Uint32 crc32(unsigned char *data, unsigned char len)
{
	unsigned int i = 0;
    Uint32 crc = 0xFFFFFFFF;

    for(i = 0; i < len; i++)
        crc = (crc << 8) ^ crc32tab[((crc >> 24) ^ *data++) & 0xFF];

    return crc;
}
#endif
////===========================================================================
//// End of file.
////===========================================================================
