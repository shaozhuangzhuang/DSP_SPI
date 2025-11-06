/*
 * Sci_D.c
 *
 *  Created on: 2022年8月2日
 *      Author: 110
 */

#include "F2837xD_device.h"         // F2837xD Headerfile Include File
#include "F2837xD_Examples.h"       // F2837xD Examples Include File
#include "Sci_D.h"

volatile struct RX_BUF Rx_D;
extern volatile struct TX_FRAME_Integrated_Control                  TX_FRAME_1SCIA;   // 仅#1使用
extern volatile struct RX_FRAME_Integrated_Control_12SCIA           RX_FRAME_12SCIA;  // #1、2使用
extern volatile struct COM_VAR McomData;
extern unsigned int RXA_flag;
extern unsigned long TX_cnt, RX_cnt;
extern float test_theta1,test_omega1,test_theta2,test_omega2,test_theta3,test_omega3,test_U,test_1,test_2;

unsigned int rdataA[128], rdataB[128];                                               // Receive Buffer for SCI-A
unsigned int rdataA_head = 0, rdataA_tail = 0, rdataB_head = 0, rdataB_tail = 0;     // Used for checking the received data
unsigned int tdataA[36];

void InitScid(void)
{
    static int i = 0;
    // Initialize SCI-A:
    ScidRegs.SCICCR.all =0x0007;    // 1 stop bit,  No loopback
    // No parity,8 char bits,
    // async mode, idle-line protocol
    ScidRegs.SCICTL1.all =0x0003;  // enable TX, RX, internal SCICLK,
    // Disable RX ERR, SLEEP, TXWAKE
//    SciaRegs.SCICTL2.all = 0x0003;
    ScidRegs.SCICTL2.bit.TXINTENA =0;
    ScidRegs.SCICTL2.bit.RXBKINTENA =0;
    //波特率 614400
    ScidRegs.SCIHBAUD.all    =0x0000;
    ScidRegs.SCILBAUD.all    =0x0013;


    //FIFO INIT
    ScidRegs.SCIFFTX.all=0xE040;//发送查询模式
//    ScidRegs.SCIFFTX.all=0xC020;    //发送中断模式
    ScidRegs.SCIFFRX.all=0x2040;      //接收查询模式
//    ScidRegs.SCIFFRX.all=0x002A;//接收中断模式

    ScidRegs.SCIFFCT.all=0x0;

    ScidRegs.SCICTL1.all =0x0023;  // Relinquish SCI from Reset
    ScidRegs.SCIFFTX.bit.TXFIFORESET=1;
    ScidRegs.SCIFFRX.bit.RXFIFORESET=1;

    // Initialize frame data
        TX_FRAME_1SCIA.SYNC[0]  = 0x55;                       // 将要发送的数据存入发送结构体中
        TX_FRAME_1SCIA.SYNC[1]  = 0xAA;
        TX_FRAME_1SCIA.LONGTH   = 0x20;                       // 综控帧的数据长度，等于36-4=32=0x20

        for(i = 0; i < 4; i++)
            TX_FRAME_1SCIA.RXCNT[i] = 0;

        for(i = 0; i < 4; i++)
            TX_FRAME_1SCIA.TXCNT[i] = 0;

        for(i = 0; i < 2; i++)
            TX_FRAME_1SCIA.STATE[i] = 0;

        for(i = 0; i < 2; i++)
            TX_FRAME_1SCIA.FUNC[i] = 0;

        for(i = 0; i < 20; i++)
            TX_FRAME_1SCIA.DATA[i] = 0;

        TX_FRAME_1SCIA.CSUM = 0;                              // 综控帧初始化配置

        for(i=3; i < 36; i++)                                 // 将发送帧中3-47字节的数据清零
            tdataA[i] = 0;
}


unsigned char SCID_SEND_DATA[80];
void SCID_TX(void)
{
    unsigned int i, i_Check;
    unsigned int CheckSum1 = 0, CheckSum2 = 0, CheckSum3 = 0, CheckSum4 = 0, CheckSum5 = 0, CheckSum = 0;

    TX_FRAME_1SCIA.RXCNT[0] = (unsigned int)(RX_cnt & 0x000000FF);                       // 接收帧计数第1字节
    TX_FRAME_1SCIA.RXCNT[1] = (unsigned int)((RX_cnt & 0x0000FF00) >> 8);                // 接收帧计数第2字节
    TX_FRAME_1SCIA.RXCNT[2] = (unsigned int)((RX_cnt & 0x00FF0000) >> 16);               // 接收帧计数第3字节
    TX_FRAME_1SCIA.RXCNT[3] = (unsigned int)((RX_cnt & 0xFF000000) >> 24);               // 接收帧计数第4字节
    TX_FRAME_1SCIA.TXCNT[0] = (unsigned int)(TX_cnt & 0x000000FF);                       // 发送帧计数第1字节
    TX_FRAME_1SCIA.TXCNT[1] = (unsigned int)((TX_cnt & 0x0000FF00) >> 8);                // 发送帧计数第2字节
    TX_FRAME_1SCIA.TXCNT[2] = (unsigned int)((TX_cnt & 0x00FF0000) >> 16);               // 发送帧计数第3字节
    TX_FRAME_1SCIA.TXCNT[3] = (unsigned int)((TX_cnt & 0xFF000000) >> 24);               // 发送帧计数第4字节
    TX_FRAME_1SCIA.STATE[0] = 0;                                                         // 执行状态/结果第1字节
    TX_FRAME_1SCIA.STATE[1] = 0;                                                         // 执行状态/结果第2字节
    TX_FRAME_1SCIA.FUNC[0]  = 0;                                                         // 工作状态字第1字节
    TX_FRAME_1SCIA.FUNC[1]  = 0;                                                         // 工作状态字第2字节

    TX_FRAME_1SCIA.DATA[0]  = ((int)(100.0 * test_1 )) & 0x00FF;                          // I号舵机位置指令低字节
    TX_FRAME_1SCIA.DATA[1]  = (((int)(100.0 * test_1 )) & 0xFF00) >> 8;                   // I号舵机位置指令高字节
    TX_FRAME_1SCIA.DATA[2]  = ((int)(100.0 * test_theta1  )) & 0x00FF;                              // II号舵机位置指令低字节
    TX_FRAME_1SCIA.DATA[3]  = (((int)(100.0 * test_theta1)) & 0xFF00) >> 8;                        // II号舵机位置指令高字节
    TX_FRAME_1SCIA.DATA[4]  = ((int)( 100.0 * test_theta2 )) & 0x00FF;                             // III号舵机位置指令低字节
    TX_FRAME_1SCIA.DATA[5]  = (((int)(100.0 * test_theta2 )) & 0xFF00) >> 8;                       // III号舵机位置指令高字节
    TX_FRAME_1SCIA.DATA[6]  = ((int)( 100.0 * test_theta3 )) & 0x00FF;                              // IV号舵机位置指令低字节* 57.29577951
    TX_FRAME_1SCIA.DATA[7]  = (((int)(100.0 * test_theta3 )) & 0xFF00) >> 8;                        // IV号舵机位置指令高字节
    TX_FRAME_1SCIA.DATA[8]  = ((int)( 100.0 * test_2 )) & 0x00FF;                             // I号舵机位置反馈低字节
    TX_FRAME_1SCIA.DATA[9]  = (((int)(100.0 * test_2 )) & 0xFF00) >> 8;                       // I号舵机位置反馈高字节
    TX_FRAME_1SCIA.DATA[10] = ((int)(  100.0 * test_omega1 )) & 0x00FF;                            // II号舵机位置反馈低字节
    TX_FRAME_1SCIA.DATA[11] = (((int)( 100.0 * test_omega1 )) & 0xFF00) >> 8;                      // II号舵机位置反馈高字节
    TX_FRAME_1SCIA.DATA[12] = ((int)( 100.0 * test_omega2)) & 0x00FF;                              // III号舵机位置反馈低字节
    TX_FRAME_1SCIA.DATA[13] = (((int)(100.0 * test_omega2)) & 0xFF00) >> 8;                        // III号舵机位置反馈高字节
    TX_FRAME_1SCIA.DATA[14] = ((int)( 100.0 * test_omega3 )) & 0x00FF;                             // IV号舵机位置反馈低字节
    TX_FRAME_1SCIA.DATA[15] = (((int)(100.0 * test_omega3 )) & 0xFF00) >> 8;                       // IV号舵机位置反馈高字节
    TX_FRAME_1SCIA.DATA[16] = 0;                                                         // 控制器软件版本号低字节
    TX_FRAME_1SCIA.DATA[17] = 0;                                                         // 控制器I软件版本号高字节
    TX_FRAME_1SCIA.DATA[18] = 0;                                                         // 备用字节
    TX_FRAME_1SCIA.DATA[19] = 0;                                                         // 备用字节

    //Count checksum
    for (i = 0; i < 4; i++)
        CheckSum1 = CheckSum1 + TX_FRAME_1SCIA.RXCNT[i];

    for (i = 0; i < 4; i++)
        CheckSum2 = CheckSum2 + TX_FRAME_1SCIA.TXCNT[i];

    for (i = 0; i < 2; i++)
        CheckSum3 = CheckSum3 + TX_FRAME_1SCIA.STATE[i];

    for (i = 0; i < 2; i++)
        CheckSum4 = CheckSum4 + TX_FRAME_1SCIA.FUNC[i];

    for(i_Check = 0; i_Check < 20; i_Check++)
        CheckSum5 = CheckSum5 + TX_FRAME_1SCIA.DATA[i_Check];

    CheckSum = TX_FRAME_1SCIA.LONGTH + CheckSum1 + CheckSum2 + CheckSum3 + CheckSum4 + CheckSum5;

    TX_FRAME_1SCIA.CSUM = (unsigned int)(CheckSum & 0x00FF);                                             // 校验和

    // 将综控帧载入至发送缓存
    tdataA[0] = TX_FRAME_1SCIA.SYNC[0];  // 发送数据帧头1
    tdataA[1] = TX_FRAME_1SCIA.SYNC[1];  // 发送数据帧头2
    tdataA[2] = TX_FRAME_1SCIA.LONGTH;  // 发送数据总长度-4

    for (i = 0; i < 4; i++)
        tdataA[i + 3] = TX_FRAME_1SCIA.RXCNT[i];

    for (i = 0; i < 4; i++)
        tdataA[i + 7] = TX_FRAME_1SCIA.TXCNT[i];

    for (i = 0; i < 2; i++)
        tdataA[i + 11] = TX_FRAME_1SCIA.STATE[i];

    for (i = 0; i < 2; i++)
        tdataA[i + 13] = TX_FRAME_1SCIA.FUNC[i];

    for (i = 0; i < 20; i++)
        tdataA[i + 15]= TX_FRAME_1SCIA.DATA[i];

    tdataA[35] = TX_FRAME_1SCIA.CSUM;

    for(i = 0; i < 36; i++)
    {
        scid_xmit(tdataA[i]);
    }


    /*    清错误标志，串口发送前进行软复位，防止复位影响发送缓存数据           */
    if( ScidRegs.SCIRXST.bit.RXERROR )
    {
        ScidRegs.SCICTL1.bit.SWRESET = 0;
        ScidRegs.SCICTL1.bit.SWRESET = 1;
    }

    // 清串口发送中断标志
    ScidRegs.SCIFFTX.bit.TXFIFORESET = 1;
//    ScidRegs.SCIFFTX.bit.TXFIFOXRESET = 1;
    ScidRegs.SCIFFTX.bit.TXFFINTCLR = 1;                    // Clear SCI Interrupt Flag 清串口发送中断标志，发送
}

//发送任意数据程序
void scid_xmit(int a)
{
    while (ScidRegs.SCIFFTX.bit.TXFFST != 0) {}
    ScidRegs.SCITXBUF.all =a;
}

void SCID_ReadFIFO(void)
{
//    if( SciaRegs.SCIRXST.bit.RXERROR )
//    {
//        SciaRegs.SCICTL1.bit.SWRESET = 0;
//        SciaRegs.SCICTL1.bit.SWRESET = 1;
//    }
    while(ScidRegs.SCIFFRX.bit.RXFFST != 0)
    {
        rdataA[rdataA_tail] = ScidRegs.SCIRXBUF.all & 0x00FF;   // 读取数据

        rdataA_tail = rdataA_tail + 1;
        rdataA_tail = rdataA_tail & 0xFF;                       // 每存一个字节，循环队列尾指针循环+1
    }
    SCID_RX();
}

void SCID_RX(void)
{
    static unsigned int rdata_n = 0;
    static unsigned int rdata_length = 0;
    static unsigned int CheckSum = 0;

    if( rdataA_tail >= rdataA_head )
        rdata_length = rdataA_tail - rdataA_head;
    else
        rdata_length = rdataA_tail + 128 - rdataA_head;

    // 如果接收缓存数据数有8个或更多，则可能有一帧完整数据
    if( rdata_length >= 20 )
    {
        // 如果接收缓存数据有超过39字节,正常情况下应有一帧完整数据，直接从倒数第15字节开始搜寻最新帧
        if( rdata_length > 39 )
        {
            rdataA_head = rdataA_head + rdata_length -39;
            rdataA_head = rdataA_head & 0x7F;
            rdata_length = 39;
        }

        while( rdata_length >= 20 )
        {
            // 如果当前数据和同步帧数据吻合，则判断帧信息是否完整
            if( rdataA[rdataA_head] == 0x55 && rdataA[( rdataA_head + 1 ) & 0x7F] == 0xAA )
            {
                CheckSum = 0;
                for( rdata_n = 2; rdata_n < 19; rdata_n++ )
                    CheckSum = CheckSum + (rdataA[(rdataA_head + rdata_n) & 0x7F] & 0x00FF);

                if( ( CheckSum & 0x00FF ) == (rdataA[(rdataA_head + 19) & 0x7F] & 0x00FF) )
                {
                    RX_FRAME_12SCIA.SYNC[0] = rdataA[(rdataA_head & 0x7F)] & 0x00FF;
                    RX_FRAME_12SCIA.SYNC[1] = rdataA[(rdataA_head + 1) & 0x7F] & 0x00FF;
                    RX_FRAME_12SCIA.LONGTH  = rdataA[(rdataA_head + 2) & 0x7F] & 0x00FF;

                    for (rdata_n = 0; rdata_n < 4; rdata_n++)
                        RX_FRAME_12SCIA.FRAMECNT[rdata_n] = rdataA[(rdataA_head + rdata_n + 3) & 0x7F] & 0x00FF;

                    for (rdata_n = 0; rdata_n < 12; rdata_n++)
                        RX_FRAME_12SCIA.DATA[rdata_n] = rdataA[(rdataA_head + rdata_n + 7) & 0x7F] & 0x00FF;

                    RX_FRAME_12SCIA.CSUM = rdataA[(rdataA_head + 19) & 0x7F] & 0x00FF;

                    // 接收飞行零点位置
                    for (rdata_n = 0; rdata_n < 2; rdata_n++)
                        McomData.flightzero_cmd[rdata_n] = RX_FRAME_12SCIA.DATA[rdata_n];

                    // 接收I号舵机位置指令
                    for (rdata_n = 0; rdata_n < 2; rdata_n++)
                        McomData.position_cmd1[rdata_n] = RX_FRAME_12SCIA.DATA[rdata_n + 2] & 0x00FF;

                    // 接收标志置位
                    RXA_flag = 1;

                    rdataA_head = rdataA_head + 20;
                    rdataA_head = rdataA_head & 0x7F;
                    rdata_length = rdata_length - 20;
                }
                else
                {
                    rdataA_head = rdataA_head + 1;
                    rdataA_head = rdataA_head & 0x7F;
                    rdata_length = rdata_length - 1;
                }
            }
            else
            {
                rdataA_head = rdataA_head + 1;
                rdataA_head = rdataA_head & 0x7F;
                rdata_length = rdata_length - 1;
            }
        }
    }
}

Uint16 crc16_IBM(unsigned char *data, Uint32 length)
{
    unsigned char  i;
    Uint16 crc = 0;        // Initial value
    while(length--)
    {
        crc ^= *data++;        // crc ^= *data; data++;
        for (i = 0; i < 8; ++i)
        {
            if (crc & 1)
                crc = (crc >> 1) ^ 0xA001;        // 0xA001 = reverse 0x8005
            else
                crc = (crc >> 1);
        }
    }
    return crc;
}
