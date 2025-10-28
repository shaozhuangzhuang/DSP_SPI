/*
 * SCI.c
 *
 *  Created on: 2025年10月28日
 *      Author: szz
 */
#include "F2837xD_device.h"
#include "F2837xD_Examples.h"
#include "Sci.h"


volatile struct RX_BUF Rx_A;


unsigned int SCIA_Txdate[8] = {0};

void InitSci(void)
{
    // Initialize SCI-A:
    SciaRegs.SCICCR.all =0x0007;    // 1 stop bit,  No loopback
    // No parity,8 char bits,
    // async mode, idle-line protocol
    SciaRegs.SCICTL1.all =0x0003;  // enable TX, RX, internal SCICLK,
    // Disable RX ERR, SLEEP, TXWAKE
    SciaRegs.SCICTL2.bit.TXINTENA =1;
    SciaRegs.SCICTL2.bit.RXBKINTENA =1;
    //波特率 921600
    SciaRegs.SCIHBAUD.all    =0x0000;
    SciaRegs.SCILBAUD.all    =0x001A;

    SciaRegs.SCIFFTX.all=0xC020; //0x0530
    SciaRegs.SCIFFRX.all=0x0030;

    SciaRegs.SCIFFCT.all=0x0;

    SciaRegs.SCICTL1.all =0x0023;  // Relinquish SCI from Reset
    SciaRegs.SCIFFTX.bit.TXFIFORESET=1;
    SciaRegs.SCIFFRX.bit.RXFIFORESET=1;

}



void InitSciGpio(void)
{
    //串口A GPIO初始化
    GPIO_SetupPinMux(84, GPIO_MUX_CPU1, 5); // GPIO65 - (SCITXDA)
    GPIO_SetupPinMux(85, GPIO_MUX_CPU1, 5); // GPIO43 - (SCIRXDA)
    GPIO_SetupPinOptions(43, GPIO_INPUT, GPIO_ASYNC);
    GPIO_SetupPinOptions(65, GPIO_OUTPUT, GPIO_PUSHPULL);

    GPIO_SetupPinMux(73,GPIO_MUX_CPU1,0);     // DE_A2
    GPIO_SetupPinOptions(73, GPIO_OUTPUT,1);
    GpioDataRegs.GPCSET.bit.GPIO73 = 1;
    GPIO_SetupPinMux(72,GPIO_MUX_CPU1,0);     // /RE_A2
    GPIO_SetupPinOptions(72, GPIO_OUTPUT,0);
    GpioDataRegs.GPCCLEAR.bit.GPIO72 = 1;

}


void SCI_ReadFIFO(void)
{
    if( SciaRegs.SCIRXST.bit.RXERROR )
    {
        SciaRegs.SCICTL1.bit.SWRESET = 0;
        SciaRegs.SCICTL1.bit.SWRESET = 1;
    }
    while(SciaRegs.SCIFFRX.bit.RXFFST != 0)
    {
        Rx_A.rx_buf[Rx_A.rx_tail] = SciaRegs.SCIRXBUF.all & RXBUF_MASK; // 读取数据
        Rx_A.rx_tail = Rx_A.rx_tail + 1;
        Rx_A.rx_tail = Rx_A.rx_tail & RXBUF_MASK;                       // 每存一个字节，循环队列尾指针循环+1
    }
}


void SCI_TO_Sim(float u_res)
{
    i32_Byte Cal_Result;
    SCIA_Txdate[0] = 0xEB;
    SCIA_Txdate[1] = 0x90;


    Cal_Result.i32 = u_res * 1e7;
    SCIA_Txdate[2] = Cal_Result.BYTE.Byte_0;
    SCIA_Txdate[3] = Cal_Result.BYTE.Byte_1;
    SCIA_Txdate[4] = Cal_Result.BYTE.Byte_2;
    SCIA_Txdate[5] = Cal_Result.BYTE.Byte_3;


    SCIA_Txdate[6] = 0xAA;
    SCIA_Txdate[7] = 0x55;
    SciaRegs.SCIFFTX.bit.TXFFINTCLR = 1;
}
extern double Rec_Vel1;
extern double Rec_Vel2;
extern double Rec_Vel3;
extern uint16_t SCI_Connect;
uint32_t SCI_RecCnt;
void SCI_RX(void)
{
    static int Rdata_length = 0;
    i32_Byte Rec_Data;

    if( ScibRegs.SCIRXST.bit.RXERROR )
     {
        ScibRegs.SCICTL1.bit.SWRESET = 0;
        ScibRegs.SCICTL1.bit.SWRESET = 1;
     }


    if( Rx_A.rx_tail >= Rx_A.rx_head )
        Rdata_length = Rx_A.rx_tail - Rx_A.rx_head;
    else
        Rdata_length = Rx_A.rx_tail + 256 - Rx_A.rx_head;



    // 如果接收缓存数据数有42个或更多，则可能有一帧完整数据
    if( Rdata_length >= 16 )
    {
        // 如果接收缓存数据有超过83字节,正常情况下应有一帧完整数据，直接从倒数第83字节开始搜寻最新帧
        if( Rdata_length > 31 )
        {
            Rx_A.rx_head = Rx_A.rx_head + Rdata_length - 31;
            Rx_A.rx_head = Rx_A.rx_head & RXBUF_MASK;
            Rdata_length = 31;
        }

        while( Rdata_length >= 16 )
        {
            // 如果当前数据和同步帧数据吻合，则判断帧信息是否完整
            if( Rx_A.rx_buf[Rx_A.rx_head] == 0xEB && Rx_A.rx_buf[Rx_A.rx_head + 1] == 0x90 )
            {
                // 接收处理,CRC判断
                if((Rx_A.rx_buf[(Rx_A.rx_head + 15) & RXBUF_MASK] == 0X55)
                               &&(Rx_A.rx_buf[(Rx_A.rx_head + 14) & RXBUF_MASK] == 0XAA))
                {
                    SCI_Connect = 1;
                    Rec_Data.BYTE.Byte_0 = Rx_A.rx_buf[(Rx_A.rx_head + 2) & RXBUF_MASK];
                    Rec_Data.BYTE.Byte_1 = Rx_A.rx_buf[(Rx_A.rx_head + 3) & RXBUF_MASK];
                    Rec_Data.BYTE.Byte_2 = Rx_A.rx_buf[(Rx_A.rx_head + 4) & RXBUF_MASK];
                    Rec_Data.BYTE.Byte_3 = Rx_A.rx_buf[(Rx_A.rx_head + 5) & RXBUF_MASK];
                    Rec_Vel1 = Rec_Data.i32 * 1e-7;

                    Rec_Data.BYTE.Byte_0 = Rx_A.rx_buf[(Rx_A.rx_head + 6) & RXBUF_MASK];
                    Rec_Data.BYTE.Byte_1 = Rx_A.rx_buf[(Rx_A.rx_head + 7) & RXBUF_MASK];
                    Rec_Data.BYTE.Byte_2 = Rx_A.rx_buf[(Rx_A.rx_head + 8) & RXBUF_MASK];
                    Rec_Data.BYTE.Byte_3 = Rx_A.rx_buf[(Rx_A.rx_head + 9) & RXBUF_MASK];
                    Rec_Vel2 = Rec_Data.i32 * 1e-7;

                    Rec_Data.BYTE.Byte_0 = Rx_A.rx_buf[(Rx_A.rx_head + 10) & RXBUF_MASK];
                    Rec_Data.BYTE.Byte_1 = Rx_A.rx_buf[(Rx_A.rx_head + 11) & RXBUF_MASK];
                    Rec_Data.BYTE.Byte_2 = Rx_A.rx_buf[(Rx_A.rx_head + 12) & RXBUF_MASK];
                    Rec_Data.BYTE.Byte_3 = Rx_A.rx_buf[(Rx_A.rx_head + 13) & RXBUF_MASK];
                    Rec_Vel3 = Rec_Data.i32 * 1e-7;

                    SCI_RecCnt++;
                    //解算后字节向后增加46
                    Rx_A.rx_head = Rx_A.rx_head + 16;
                    Rx_A.rx_head = Rx_A.rx_head & RXBUF_MASK;
                    Rdata_length = Rdata_length - 16;
                }
                else
                {
                    Rx_A.rx_head = Rx_A.rx_head + 1;
                    Rx_A.rx_head = Rx_A.rx_head & RXBUF_MASK;
                    Rdata_length = Rdata_length - 1;
                }

            }
            else
            {
                Rx_A.rx_head = Rx_A.rx_head + 1;
                Rx_A.rx_head = Rx_A.rx_head & RXBUF_MASK;
                Rdata_length = Rdata_length - 1;
            }
        }
    }
}
