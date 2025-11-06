
/*
 * McBsp_USART.c
 *
 *  Created on: 2021年11月22日
 *      Author: SZZ
 */
#include "MCBSP.h"
#include "COMMON.h"

#define CPU_SPD              200E6
#define MCBSP_SRG_FREQ       CPU_SPD/4   // SRG input is LSPCLK (SYSCLKOUT/4) for examples

#define CLKGDV_VAL           1
#define MCBSP_INIT_DELAY     2*(CPU_SPD/MCBSP_SRG_FREQ)                  // # of CPU cycles in 2 SRG cycles-init delay

#pragma DATA_SECTION(McBSP_Sdata, "ramgs0") // Place sdata and rdata buffers in
#pragma DATA_SECTION(McBSP_Rdata, "ramgs1") // DMA-accessible RAM
Uint16 McBSP_Sdata[128];                    // Sent Data
Uint16 McBSP_Sdata_NoEncode[128];                    // Sent Data
Uint16 McBSP_Rdata[128];                    // Received Data
Uint16 McBSP_Rdata_use[128];                    // Received Data

Uint32 SCI_TX_Cnt;
int TransFinish_Flag;

char sumcheck_F1 = 0;
char addcheck_F1 = 0;
char sumcheck_F2 = 0;
char addcheck_F2 = 0;

extern float testdata3,testdata4;
extern double latitude;
extern double longitude;
extern int longitude_Proportion;
extern double longitude0;
extern double latitude0;
float Servo_Deflection1,Servo_Deflection2,Servo_Deflection3,Servo_Deflection4;

unsigned int DJ_fire_test = 0;

union HexData McBSP_Rx_SCI[128];                    // McBSP接收到串口的数据,转换后的
union HexData McBSP_Rx_SCI_Interim[128];            // McBSP接收到串口的数据,转换中

unsigned int rdataB[256];                           // Receive Buffer for mcbsp-B
unsigned int rdataB_head = 0, rdataB_tail = 0;      // Used for checking the received data

int i = 0;

//inline Uint16 Byte_To_U16( unsigned char *buf)
//{
//    Uint16 data;
//
//    data = (unsigned long)( (((unsigned long)(*(buf + 3)) & 0x000000FF) << 24) | (((unsigned long)(*(buf + 2)) & 0x000000FF) << 16) | (((unsigned long)(*(buf + 1)) & 0x000000FF) << 8) | ((unsigned long)(*(buf)) & 0x000000FF) );
//
//}

void McBsp_GPIO_Init(void)
{
    EALLOW;
    // MDXA
//    GpioCtrlRegs.GPAMUX2.bit.GPIO20 = 2;
//    // MDRA
//    GpioCtrlRegs.GPAMUX2.bit.GPIO21 = 2;
//    GpioCtrlRegs.GPAQSEL2.bit.GPIO21 = 3;
//
//    // MCLKXA
//    GpioCtrlRegs.GPAMUX2.bit.GPIO22 = 2;
//
//    // MCLKRA
//    GpioCtrlRegs.GPAMUX1.bit.GPIO7 = 2;
//    GpioCtrlRegs.GPAQSEL1.bit.GPIO7 = 3;
//    // MFSXA
//    GpioCtrlRegs.GPAMUX2.bit.GPIO23 = 2;
//    // MFSRA
//    GpioCtrlRegs.GPBMUX2.bit.GPIO59 = 1;
//    GpioCtrlRegs.GPBQSEL2.bit.GPIO59 = 3;

    // Select one of the following for MDXB
    // GPIO24
    // GPIO84
    //
    //GpioCtrlRegs.GPAMUX2.bit.GPIO24 = 3;
    GpioCtrlRegs.GPAGMUX1.bit.GPIO12 = 0;
    GpioCtrlRegs.GPAMUX1.bit.GPIO12 = 3;

    //
    // MDRB
    // GPIO13 with asynchronous qualification
    // GPIO25 with asynchronous qualification
    // GPIO85 with asynchronous qualification
    //
    //GpioCtrlRegs.GPAMUX1.bit.GPIO13 = 3;
    //GpioCtrlRegs.GPAQSEL1.bit.GPIO13 = 3;
    //GpioCtrlRegs.GPAMUX2.bit.GPIO25 = 3;
    //GpioCtrlRegs.GPAQSEL2.bit.GPIO25 = 3;
    GpioCtrlRegs.GPCGMUX2.bit.GPIO85 = 1;
    GpioCtrlRegs.GPCMUX2.bit.GPIO85 = 2;
//    GpioCtrlRegs.GPCQSEL2.bit.GPIO85 = 3;
//    GPIO_SetupPinMux(85,GPIO_MUX_CPU1,6);     // DE_A2
//    GPIO_SetupPinOptions(85, GPIO_INPUT,GPIO_PULLUP);

    //
    // MCLKXB
    // GPIO14 with asynchronous qualification
    // GPIO26 with asynchronous qualification
    // GPIO86 with asynchronous qualification
    //
    //GpioCtrlRegs.GPAMUX1.bit.GPIO14 = 3;
    //GpioCtrlRegs.GPAQSEL1.bit.GPIO14 = 3;
    //GpioCtrlRegs.GPAMUX2.bit.GPIO26 = 3;
    //GpioCtrlRegs.GPAQSEL2.bit.GPIO26 = 3;
//    GpioCtrlRegs.GPCGMUX2.bit.GPIO86 = 1;X
//    GpioCtrlRegs.GPCMUX2.bit.GPIO86 = 2;
//    GpioCtrlRegs.GPCQSEL2.bit.GPIO86= 3;X

    //
    // MCLKRB
    // Select one of the following
    // GPIO3 with asynchronous qualification
    // GPIO60 with asynchronous qualification
    //
    //GpioCtrlRegs.GPAMUX1.bit.GPIO3 = 3;
    //GpioCtrlRegs.GPAQSEL1.bit.GPIO3 = 3;
//    GpioCtrlRegs.GPBMUX2.bit.GPIO60 = 1;
//    GpioCtrlRegs.GPBQSEL2.bit.GPIO60 = 3;

    //
    // MFSXB
    // GPIO15 with asynchronous qualification
    // GPIO27 with asynchronous qualification
    // GPIO87 with asynchronous qualification
    //
    //GpioCtrlRegs.GPAMUX1.bit.GPIO15 = 3;
    //GpioCtrlRegs.GPAQSEL1.bit.GPIO15 = 3;
    //GpioCtrlRegs.GPAMUX2.bit.GPIO27 = 3;
    //GpioCtrlRegs.GPAQSEL2.bit.GPIO27 = 3;
//    GpioCtrlRegs.GPCGMUX2.bit.GPIO87 = 1;X
//    GpioCtrlRegs.GPCMUX2.bit.GPIO87 = 2;
//    GpioCtrlRegs.GPCQSEL2.bit.GPIO87= 3;X

    //
    // MFSRB
    // Select one of the following
    // GPIO1 with asynchronous qualification
    // GPIO61 with asynchronous qualification
    //
    //GpioCtrlRegs.GPAMUX1.bit.GPIO1 = 3;
    //GpioCtrlRegs.GPAQSEL1.bit.GPIO1 = 3;
    GpioCtrlRegs.GPBMUX2.bit.GPIO61 = 1;
    GpioCtrlRegs.GPBQSEL2.bit.GPIO61 = 3;

    EDIS;
}
void McBsp_REG_Init(void)
{
    McbspbRegs.SPCR2.all=0x0000;        // Reset FS generator, sample rate
                                        // generator & transmitter
    McbspbRegs.SPCR1.all=0x0000;        // Reset Receiver, Right justify word

//    McbspaRegs.SPCR1.bit.DLB        = 0;    //关闭数据回环模式
//    McbspaRegs.SPCR1.bit.RJUST      = 2;    //接收数据的是LSB或MSB,使用LSB,左移补零
//    McbspaRegs.SPCR1.bit.CLKSTP     = 0;    //禁用时钟停止模式,使用SPI模式时才开启

//    McbspaRegs.SPCR1.bit.DXENA      = 0;    //DX引脚延迟使能位,禁止
//    McbspaRegs.SPCR1.bit.RINTM      = 2;    //接收中断标志位
//    McbspaRegs.SPCR1.bit.RSYNCERR   = 0;    //接收帧同步标志位,0为无错误
//    McbspaRegs.SPCR1.bit.RFULL      = 0;    //接收溢出标志位,0为未溢出
//    McbspaRegs.SPCR1.bit.RRDY       = 0;    //接收延迟标志位,0禁止
//    McbspaRegs.SPCR1.bit.RRST       = 0;    //接收复位,0使能

//    McbspaRegs.SPCR2.bit.FREE       = 0;    //
//    McbspaRegs.SPCR2.bit.SOFT       = 0;    //
//    McbspaRegs.SPCR2.bit.FRST       = 0;    //帧同步逻辑复位标志位,0复位
//    McbspaRegs.SPCR2.bit.GRST       = 0;    //采样速率发生器复位标志位,0复位
//    McbspaRegs.SPCR2.bit.XINTM      = 0;    //发送中断标志位
//    McbspaRegs.SPCR2.bit.XSYNCERR   = 0;    //发送帧同步错误标志位,
//    McbspaRegs.SPCR2.bit.XEMPTY     = 0;    //发送空标志位
//    McbspaRegs.SPCR2.bit.XRDY       = 0;    //发送就绪标志位
//    McbspaRegs.SPCR2.bit.XRST       = 0;    //发送复位标志位,0复位

    McbspbRegs.RCR1.bit.RFRLEN1     = 0;    //接收帧长度1
    McbspbRegs.RCR1.bit.RWDLEN1     = 2;    //接收字长1,选择bit数

    McbspbRegs.RCR2.bit.RPHASE      = 0;    //接收相位数,0为一帧,1为两帧
    McbspbRegs.RCR2.bit.RFRLEN2     = 0;    //接收帧长度2
    McbspbRegs.RCR2.bit.RWDLEN2     = 0;    //接收字长2,选择bit数
    McbspbRegs.RCR2.bit.RCOMPAND    = 0;    //接收扩展模式位,0:无扩展,任何bit数，MSB方式接收
    McbspbRegs.RCR2.bit.RFIG        = 1;    //接收帧同步忽略位,
    McbspbRegs.RCR2.bit.RDATDLY     = 1;    //接收数据延迟位,0为不延迟

    McbspbRegs.XCR1.bit.XFRLEN1     = 9;    //发送帧字长1,
    McbspbRegs.XCR1.bit.XWDLEN1     = 2;    //发送字长1,bit数

//    McbspaRegs.XCR2.bit.XPHASE      = 0;    //发送相位数,0为一帧,1为两帧
//    McbspaRegs.XCR2.bit.XFRLEN2     = 0;    //发送帧字长2,
//    McbspaRegs.XCR2.bit.XWDLEN2     = 0;    //发送字长2,bit数
//    McbspaRegs.XCR2.bit.XCOMPAND    = 0;    //发送扩展模式位,0:无扩展,任何bit数，MSB方式发送
//    McbspaRegs.XCR2.bit.XFIG        = 1;    //发送帧同步忽略位
//    McbspaRegs.XCR2.bit.XDATDLY     = 0;    //发送数据延迟位,0为不延迟
    //波特率1250000
    McbspbRegs.SRGR1.bit.FWID       = 0;    //FSG的帧同步脉冲宽度位
    McbspbRegs.SRGR1.bit.CLKGDV     = 216;    //CLKG的分频值
    delay_loop();


    McbspbRegs.SRGR2.bit.GSYNC      = 1;    //CLKG的时钟同步模式位,0不同步
    McbspbRegs.SRGR2.bit.CLKSM      = 1;    //采样速率发生器输入时钟模式位,
    McbspbRegs.SRGR2.bit.FSGM       = 1;    //采样速率发生器传输帧同步模式位,
    McbspbRegs.SRGR2.bit.FPER       = 431;   //FSG的帧同步周期位,设置FSG帧同步的周期

    McbspbRegs.PCR.bit.FSXM         = 1;    //发送帧同步模式位,0代表发送帧同步脉冲由FSX引脚提供;1代表由采样速率生成器在内部生成
    McbspbRegs.PCR.bit.FSRM         = 0;    //接收帧同步模式位,0代表接收帧同步脉冲由FSR引脚提供;1代表由采样速率生成器在内部生成
    McbspbRegs.PCR.bit.CLKXM        = 1;    //发送时钟模式位,0代表从MCLKX引脚获取时钟信号,1代表由采样速率生成器在内部生成
    McbspbRegs.PCR.bit.CLKRM        = 1;    //接收时钟模式位,0代表从MCLKR引脚获取时钟信号,1代表由采样速率生成器在内部生成
    McbspbRegs.PCR.bit.SCLKME       = 0;    //采样速率发生器输入时钟模式位
    McbspbRegs.PCR.bit.FSRP         = 1;    //接收帧同步极性位,1:低电平有效
    McbspbRegs.PCR.bit.CLKRP        = 0;    //接收时钟极性位

    //
    // Enable TX/RX unit
    //
//    McbspbRegs.MFFINT.bit.XINT = 1; // Enable Transmit Interrupts
//    McbspbRegs.MFFINT.bit.RINT = 1; // Enable Receive Interrupts
    McbspbRegs.SPCR2.bit.GRST = 1;
    delay_loop();

    McbspbRegs.SPCR2.bit.XRST = 1;
    McbspbRegs.SPCR1.bit.RRST = 1;
    McbspbRegs.SPCR2.bit.FRST = 1;
}

// McBsp_A的DMA初始化
void McBSP_A_DMA_Init()
{
  EALLOW;
  //硬件复位DMA
  DmaRegs.DMACTRL.bit.HARDRESET = 1;
  __asm(" NOP");                        // Only 1 NOP needed per Design

  //DMA通道中断启用位
  DmaRegs.CH1.MODE.bit.CHINTE = 0;

  // Channel 1, McBSPA transmit
  //突发传输的位数 = 0 + 1
  DmaRegs.CH1.BURST_SIZE.all = 0;       // 1 word/burst
  DmaRegs.CH1.SRC_BURST_STEP = 0;       // no effect when using 1 word/burst
  DmaRegs.CH1.DST_BURST_STEP = 0;       // no effect when using 1 word/burst

  DmaRegs.CH1.TRANSFER_SIZE = 43;      // Interrupt every frame
                                        // (127 bursts/transfer)
  DmaRegs.CH1.SRC_TRANSFER_STEP = 1;    // Move to next word in buffer after  SRC_WRAP_SIZE  SRC_TRANSFER_STEP
                                        // each word in a burst
  DmaRegs.CH1.DST_TRANSFER_STEP = 0;    // Don't move destination address
  DmaRegs.CH1.SRC_ADDR_SHADOW = (Uint32) &McBSP_Sdata[0];   // Start address = buffer
  DmaRegs.CH1.SRC_BEG_ADDR_SHADOW = (Uint32) &McBSP_Sdata[0];  // Not needed unless
                                                         // using wrap function
  DmaRegs.CH1.DST_ADDR_SHADOW = (Uint32) &McbspbRegs.DXR1.all; // Start address
                                                               // = McBSPA DXR
  //
  // Not needed unless using wrap function
  //
  DmaRegs.CH1.DST_BEG_ADDR_SHADOW = (Uint32) &McbspbRegs.DXR1.all;
  DmaRegs.CH1.CONTROL.bit.PERINTCLR = 1;   // Clear peripheral interrupt event
                                           // flag.
  DmaRegs.CH1.CONTROL.bit.ERRCLR = 1;      // Clear sync error flag
  DmaRegs.CH1.DST_WRAP_SIZE = 0xFFFF;      // Put to maximum - don't want
                                           // destination wrap.
  DmaRegs.CH1.SRC_WRAP_SIZE = 0xFFFF;      // Put to maximum - don't want
                                           // source wrap.
  DmaRegs.CH1.MODE.bit.CHINTE = 1;         // Enable channel interrupt
  DmaRegs.CH1.MODE.bit.CHINTMODE = 1;      // Interrupt at end of transfer
  DmaRegs.CH1.MODE.bit.PERINTE = 1;        // Enable peripheral interrupt event
  DmaRegs.CH1.MODE.bit.PERINTSEL = 1;      // Peripheral interrupt select =
                                           // McBSP MXSYNCA
  DmaClaSrcSelRegs.DMACHSRCSEL1.bit.CH1 = DMA_MXEVTB;
  DmaRegs.CH1.CONTROL.bit.PERINTCLR = 1;   // Clear any spurious interrupt flags

  //
  // Channel 2, McBSPA Receive
  //
  DmaRegs.CH2.MODE.bit.CHINTE = 0;
  DmaRegs.CH2.BURST_SIZE.all = 0;        // 1 word/burst
  DmaRegs.CH2.SRC_BURST_STEP = 0;        // no effect when using 1 word/burst
  DmaRegs.CH2.DST_BURST_STEP = 0;        // no effect when using 1 word/burst
  DmaRegs.CH2.TRANSFER_SIZE = 47;       // Interrupt every 127 bursts/transfer
  DmaRegs.CH2.SRC_TRANSFER_STEP = 0;     // Don't move source address
  DmaRegs.CH2.DST_TRANSFER_STEP = 1;     // Move to next word in buffer after
                                         // each word in a burst
  DmaRegs.CH2.SRC_ADDR_SHADOW = (Uint32) &McbspbRegs.DRR1.all; // Start address
                                                               // = McBSPA DRR
  //
  // Not needed unless using wrap function
  //
  DmaRegs.CH2.SRC_BEG_ADDR_SHADOW = (Uint32) &McbspbRegs.DRR1.all;
  DmaRegs.CH2.DST_ADDR_SHADOW = (Uint32) &McBSP_Rdata[0];      // Start address =
                                                         // Receive buffer
                                                         // (for McBSP-A)
  DmaRegs.CH2.DST_BEG_ADDR_SHADOW = (Uint32) &McBSP_Rdata[0];  // Not needed unless
                                                         // using wrap function
  DmaRegs.CH2.CONTROL.bit.PERINTCLR = 1; // Clear peripheral interrupt event
                                         // flag.
  DmaRegs.CH2.CONTROL.bit.ERRCLR = 1;    // Clear sync error flag
  DmaRegs.CH2.DST_WRAP_SIZE = 0xFFFF;    // Put to maximum - don't want
                                         // destination wrap.
  DmaRegs.CH2.SRC_WRAP_SIZE = 0xFFFF;    // Put to maximum - don't want
                                         // source wrap.
  DmaRegs.CH2.MODE.bit.CHINTE = 1;       // Enable channel interrupt
//  DmaRegs.CH2.MODE.bit.CONTINUOUS = 1;   // DMA连续模式
  DmaRegs.CH2.MODE.bit.CHINTMODE = 1;    // Interrupt at end of transfer
  DmaRegs.CH2.MODE.bit.PERINTE = 1;      // Enable peripheral interrupt event
  DmaRegs.CH2.MODE.bit.PERINTSEL = 2;    // Peripheral interrupt select =
                                         // McBSP MRSYNCA
  DmaClaSrcSelRegs.DMACHSRCSEL1.bit.CH2 = DMA_MREVTB;
  DmaRegs.CH2.CONTROL.bit.PERINTCLR = 1; // Clear any spurious interrupt flags
  EDIS;
}
void start_dma (void)
{
  EALLOW;
  DmaRegs.CH1.CONTROL.bit.RUN = 1;      // Start DMA Transmit from McBSP-A
  DmaRegs.CH2.CONTROL.bit.RUN = 1;      // Start DMA Receive from McBSP-A
  EDIS;
}

void McBsp_USART_Init(void)
{
//    McBsp_GPIO_Init();
    McBsp_REG_Init();
}

Uint64 Txcnt = 0;
interrupt void McBsp_DMA_Tx(void)
{
    EALLOW;
    // 停止McBSP的DMA发送通道
    DmaRegs.CH1.CONTROL.bit.HALT = 1;
    Txcnt++;
//    McbspbRegs.DXR1.all = 0xFFFF;
//    McbspbRegs.SPCR2.bit.XRST = 0;
    TransFinish_Flag = 2;
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP7;
    EDIS;
}

Uint32 Rexcnt = 0;
int McBsp_Rec_Flag = 0;
interrupt void McBsp_DMA_Rx(void)
{
    EALLOW;
    DmaRegs.CH2.CONTROL.bit.HALT = 1;
    // 将接收到的串口数据进行大小端调换,转为正确的数据
    // 一帧帧长,McBSP接收到的字节数
    int frame_len = 48;
    for(i = 0;i < frame_len;i++)
    {
        McBSP_Rx_SCI_Interim[i].Hexall = (McBSP_Rdata[i] & 0xFF00) >> 8;
//        McBSP_Rx_SCI[i].bit.bit7       = McBSP_Rx_SCI_Interim[i].bit.bit0;
//        McBSP_Rx_SCI[i].bit.bit6       = McBSP_Rx_SCI_Interim[i].bit.bit1;
//        McBSP_Rx_SCI[i].bit.bit5       = McBSP_Rx_SCI_Interim[i].bit.bit2;
//        McBSP_Rx_SCI[i].bit.bit4       = McBSP_Rx_SCI_Interim[i].bit.bit3;
//        McBSP_Rx_SCI[i].bit.bit3       = McBSP_Rx_SCI_Interim[i].bit.bit4;
//        McBSP_Rx_SCI[i].bit.bit2       = McBSP_Rx_SCI_Interim[i].bit.bit5;
//        McBSP_Rx_SCI[i].bit.bit1       = McBSP_Rx_SCI_Interim[i].bit.bit6;
//        McBSP_Rx_SCI[i].bit.bit0       = McBSP_Rx_SCI_Interim[i].bit.bit7;
    }
    McBsp_Rec_Flag = 1;
    Rexcnt++;
    DmaRegs.CH2.CONTROL.bit.RUN = 1;      // Start DMA Receive from McBSP-A
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP7;
    EDIS;

}

// McBSP发送函数
void McBSP_xmit(int a, int b)
{
    McbspbRegs.DXR2.all=b;
    McbspbRegs.DXR1.all=a;
}

//仿真机结构体实例化
//struct Rx_SimToFlight             Rx_SimToFlight_Rx;      //飞控接收到仿真的数据帧
//struct Rx_ServoToFlight           Rx_ServoToFlight_Rx;    //飞控接收到舵机的数据帧
//struct Rx_SeekerToFlight          SeekerToFlight_Rx;      //飞控接收到导引头的数据帧

extern volatile struct can_rx CanDaTa_rx;
unsigned int Mcbsp_Txdate[42];
extern float real_Pitch_angle;
extern float real_Roll_angle;
extern float Yaw_VelAng_Sim;
extern float Pitch_VelAng_Sim;
uint32_t McBSP_TX_Counter = 0;
float McBSP_TX_Counter_Test = 0;
extern float Angle_cmd1;
extern float Angle_cmd2;
extern float Angle_cmd3;
extern float Angle_cmd4;
extern uint32_t SCIA_Counter_Servo;
extern Uint32 SCIA_RX_Cnt;
float SCIA_Counter_Servo_Test;
float SCIA_RX_Cnt_Test;
extern float get_Code_Cnt;


float Tra_acc[3] = {0};
float Tra_a_p[3] = {0};

Uint16 switch1 = 0;

void McBSP_TX(void)
{
    F32_Byte F1_data1;
    F32_Byte F1_data2;
    F32_Byte F1_data3;
    F32_Byte F1_data4;
    F32_Byte F1_data5;
    F32_Byte F1_data6;
    F32_Byte F1_data7;
    F32_Byte F1_data8;
    F32_Byte F1_data9;


    Mcbsp_Txdate[0] = 0xAA;
    Mcbsp_Txdate[1] = 0xFF;

    switch(switch1)
    {
    case 0:

            Mcbsp_Txdate[2] = 0xF1;
            Mcbsp_Txdate[3] = 0X24;

            F1_data1.I32 = IPC_Data_Cpu2.AttitudeSolution_Longitude;
            F1_data2.I32 = IPC_Data_Cpu2.AttitudeSolution_Latitude;
            F1_data3.I32 = IPC_Data_Cpu2.AttitudeSolution_Height;
            F1_data4.I32 = IPC_Data_Cpu2.AttitudeSolution_X;
            F1_data5.I32 = IPC_Data_Cpu2.AttitudeSolution_Y;
            F1_data6.I32 = IPC_Data_Cpu2.AttitudeSolution_Z;
            F1_data7.I32 = IPC_Data_Cpu2.AttitudeSolution_VN;
            F1_data8.I32 = IPC_Data_Cpu2.AttitudeSolution_VE;
            F1_data9.I32 = IPC_Data_Cpu2.AttitudeSolution_VD;
            switch1 = 1;
        break;
    case 1:

            Mcbsp_Txdate[2] = 0xF2;
            Mcbsp_Txdate[3] = 0X24;

            F1_data1.I32 = IPC_Data_Cpu2.AttitudeSolution_Yaw *1e7;
            F1_data2.I32 = IPC_Data_Cpu2.AttitudeSolution_Pitch *1e7;
            F1_data3.I32 = IPC_Data_Cpu2.AttitudeSolution_Roll *1e7;
            F1_data4.I32 = 44335.5;
            F1_data5.I32 = 22113.3;
            F1_data6.I32 = 77668.8;
            F1_data7.I32 = 44221.1;
            F1_data8.I32 = 55332.2;
            F1_data9.I32 = 44337.4;
            switch1 = 0;
        break;
    }

            Mcbsp_Txdate[4]  = F1_data1.BYTE.Byte_0 & 0xFF;
            Mcbsp_Txdate[5]  = F1_data1.BYTE.Byte_1 & 0xFF;
            Mcbsp_Txdate[6]  = F1_data1.BYTE.Byte_2 & 0xFF;
            Mcbsp_Txdate[7]  = F1_data1.BYTE.Byte_3 & 0xFF;


            Mcbsp_Txdate[8]  = F1_data2.BYTE.Byte_0 & 0xFF;
            Mcbsp_Txdate[9]  = F1_data2.BYTE.Byte_1 & 0xFF;
            Mcbsp_Txdate[10] = F1_data2.BYTE.Byte_2 & 0xFF;
            Mcbsp_Txdate[11] = F1_data2.BYTE.Byte_3 & 0xFF;


            Mcbsp_Txdate[12] = F1_data3.BYTE.Byte_0 & 0xFF;
            Mcbsp_Txdate[13] = F1_data3.BYTE.Byte_1 & 0xFF;
            Mcbsp_Txdate[14] = F1_data3.BYTE.Byte_2 & 0xFF;
            Mcbsp_Txdate[15] = F1_data3.BYTE.Byte_3 & 0xFF;


            Mcbsp_Txdate[16] = F1_data4.BYTE.Byte_0 & 0xFF;
            Mcbsp_Txdate[17] = F1_data4.BYTE.Byte_1 & 0xFF;
            Mcbsp_Txdate[18] = F1_data4.BYTE.Byte_2 & 0xFF;
            Mcbsp_Txdate[19] = F1_data4.BYTE.Byte_3 & 0xFF;


            Mcbsp_Txdate[20] = F1_data5.BYTE.Byte_0 & 0xFF;
            Mcbsp_Txdate[21] = F1_data5.BYTE.Byte_1 & 0xFF;
            Mcbsp_Txdate[22] = F1_data5.BYTE.Byte_2 & 0xFF;
            Mcbsp_Txdate[23] = F1_data5.BYTE.Byte_3 & 0xFF;


            Mcbsp_Txdate[24] = F1_data6.BYTE.Byte_0 & 0xFF;
            Mcbsp_Txdate[25] = F1_data6.BYTE.Byte_1 & 0xFF;
            Mcbsp_Txdate[26] = F1_data6.BYTE.Byte_2 & 0xFF;
            Mcbsp_Txdate[27] = F1_data6.BYTE.Byte_3 & 0xFF;


            Mcbsp_Txdate[28] = F1_data7.BYTE.Byte_0 & 0xFF;
            Mcbsp_Txdate[29] = F1_data7.BYTE.Byte_1 & 0xFF;
            Mcbsp_Txdate[30] = F1_data7.BYTE.Byte_2 & 0xFF;
            Mcbsp_Txdate[31] = F1_data7.BYTE.Byte_3 & 0xFF;


            Mcbsp_Txdate[32] = F1_data8.BYTE.Byte_0 & 0xFF;
            Mcbsp_Txdate[33] = F1_data8.BYTE.Byte_1 & 0xFF;
            Mcbsp_Txdate[34] = F1_data8.BYTE.Byte_2 & 0xFF;
            Mcbsp_Txdate[35] = F1_data8.BYTE.Byte_3 & 0xFF;


            Mcbsp_Txdate[36] = F1_data9.BYTE.Byte_0 & 0xFF;
            Mcbsp_Txdate[37] = F1_data9.BYTE.Byte_1 & 0xFF;
            Mcbsp_Txdate[38] = F1_data9.BYTE.Byte_2 & 0xFF;
            Mcbsp_Txdate[39] = F1_data9.BYTE.Byte_3 & 0xFF;

            Cal_Sum_F1(Mcbsp_Txdate);
            Mcbsp_Txdate[40] = sumcheck_F1 & 0XFF;
            Mcbsp_Txdate[41] = addcheck_F1 & 0XFF;


    McBSP_TX_Counter++;
}

void Cal_Sum_F1(Uint16 data_buf[])
{
    char sumcheck = 0;
    char addcheck = 0;
    int i;
    for( i = 0; i < (data_buf[3] + 4); i++)
    {
        sumcheck += data_buf[i] & 0x00FF; //从帧头开始，对每一字节进行求和，直到DATA区结束
        addcheck += sumcheck & 0x00FF; //每一字节的求和操作，进行一次sumcheck的累加
    }
    sumcheck_F1 = sumcheck;
    addcheck_F1 = addcheck;
}


void Cal_Sum_F2(Uint16 data_buf[])
{
    char sumcheck = 0;
    char addcheck = 0;
    int i;
    for (i = 0; i < (data_buf[3] + 4); i++)
    {
        sumcheck += data_buf[i]; //从帧头开始，对每一字节进行求和，直到DATA区结束
        addcheck += sumcheck; //每一字节的求和操作，进行一次sumcheck的累加
    }
    sumcheck_F2 = sumcheck;
    addcheck_F2 = addcheck;
}

struct RX_McbspData RX_MCBSP;
//struct RX_Data RX_HUOKONG;
//volatile struct RX_BUF Rx_A;
volatile struct RX_B_FRAME_REG RX_B_FRAME;
Uint32 McBSP_RX_Cnt = 0;
void McBSP_RX(void)
{
    static unsigned int rdataB_n = 0;
    static int rdataB_length = 0;
    static unsigned long CRC16B_R = 0;
    static unsigned char buf32B[46];


    U16_Byte C16T8_C_receive;
    U32_Byte DataLink_U32_U8;
    i32_Byte DataLink_i32_U8;
    i16_Byte DataLink_i16_U8;
    U16_Byte DataLink_U16_U8;


    if( rdataB_tail >= rdataB_head )
        rdataB_length = rdataB_tail - rdataB_head;
    else
        rdataB_length = rdataB_tail + 256 - rdataB_head;

    // 如果接收缓存数据数有48个或更多，则可能有一帧完整数据
    if( rdataB_length >= 48 )
    {
        // 如果接收缓存数据有超过95字节,正常情况下应有一帧完整数据，直接从倒数第95字节开始搜寻最新帧
        if( rdataB_length > 95 )
        {
            rdataB_head = rdataB_head + rdataB_length - 95;
            rdataB_head = rdataB_head & 0xFF;
            rdataB_length = 95;
        }

        while( rdataB_length >= 48 )
        {
            // 如果当前数据和同步帧数据吻合，则判断帧信息是否完整
            if( rdataB[rdataB_head] == 0xEB && rdataB[( rdataB_head + 1 ) & 0xFF] == 0x90 )
            {
                fireCtrlFlag = 1;//用于判断串口连接状态
                // CRC校验
                for(rdataB_n = 0; rdataB_n < 44; rdataB_n++)                                              //
                    buf32B[rdataB_n] = rdataB[(rdataB_head + rdataB_n + 2) & 0xFF];

                CRC16B_R =  crc16_Ibm( buf32B, 44 );
                C16T8_C_receive.U16 = CRC16B_R;
//                rcv_crc32 = Byte_To_U16(buf32B + 42 );
                // 接收处理
                //C16T8_C_receive.Byte.Byte_1 == rdataB[(rdataB_head + 47) & RXBUF_MASK] && C16T8_C_receive.Byte.Byte_0 == rdataB[(rdataB_head + 46) & RXBUF_MASK]
                if( C16T8_C_receive.Byte.Byte_1 == rdataB[(rdataB_head + 47) & RXBUF_MASK] && C16T8_C_receive.Byte.Byte_0 == rdataB[(rdataB_head + 46) & RXBUF_MASK] )//CRC16B_R == rcv_crc32
                {
                    for(rdataB_n = 0; rdataB_n < 46; rdataB_n++)
                        RX_B_FRAME.DATA[rdataB_n] = rdataB[(rdataB_head + rdataB_n + 2) & 0xFF];

                    // 获取指令数据

                    //帧功能识别码
                           RX_HUOKONG.FrameID = RX_B_FRAME.DATA[0] & 0x00FF;//功能码
                           McBSP_RX_Cnt++;
                           switch(RX_HUOKONG.FrameID)
                           {
                           case 0x5c:
                               //飞行器编号
                               RX_HUOKONG.FlightNum = RX_B_FRAME.DATA[1] & 0x00FF;//功能码

                               //帧计数
                               DataLink_U32_U8.BYTE.Byte_0 =  RX_B_FRAME.DATA[2] & 0x00FF;
                               DataLink_U32_U8.BYTE.Byte_1 =  RX_B_FRAME.DATA[3] & 0x00FF;
                               DataLink_U32_U8.BYTE.Byte_2 =  RX_B_FRAME.DATA[4] & 0x00FF;
                               DataLink_U32_U8.BYTE.Byte_3 =  RX_B_FRAME.DATA[5] & 0x00FF;
                               RX_HUOKONG.FrameCnt = DataLink_U32_U8.U32;

                               //发射筒编号
                               RX_HUOKONG.TargetNum = RX_B_FRAME.DATA[6] & 0x00FF;

                               //查询准备状态指令
                               RX_HUOKONG.Ready_cmd = RX_B_FRAME.DATA[7] & 0x00FF;

                               //准备指令-电池上电
                               RX_HUOKONG.Battery_cmd = RX_B_FRAME.DATA[8] & 0x00FF;

                               //发动机点火
                               RX_HUOKONG.Engine_fire = RX_B_FRAME.DATA[9] & 0x00FF;

                               //仿真程序指令
                               RX_HUOKONG.Emulator_cmd = RX_B_FRAME.DATA[10] & 0x00FF;

                               //发射指令
                               RX_HUOKONG.Launch_cmd = RX_B_FRAME.DATA[11] & 0x00FF;

                               //开始对准
                               RX_HUOKONG.Alignment_cmd = RX_B_FRAME.DATA[12] & 0x00FF;
                               //导引头重置-北理工联调用
                               RX_HUOKONG.DYT_RESTE = RX_B_FRAME.DATA[13] & 0x00FF;
                               //拦截弹经度
                               DataLink_i32_U8.BYTE.Byte_0 = RX_B_FRAME.DATA[14] & 0x00FF;
                               DataLink_i32_U8.BYTE.Byte_1 = RX_B_FRAME.DATA[15] & 0x00FF;
                               DataLink_i32_U8.BYTE.Byte_2 = RX_B_FRAME.DATA[16] & 0x00FF;
                               DataLink_i32_U8.BYTE.Byte_3 = RX_B_FRAME.DATA[17] & 0x00FF;
                               RX_HUOKONG.Carrire_Longitude = (float32)(DataLink_i32_U8.i32 * 1e-7);

                               //拦截弹纬度
                               DataLink_i32_U8.BYTE.Byte_0 = RX_B_FRAME.DATA[18] & 0x00FF;
                               DataLink_i32_U8.BYTE.Byte_1 = RX_B_FRAME.DATA[19] & 0x00FF;
                               DataLink_i32_U8.BYTE.Byte_2 = RX_B_FRAME.DATA[20]& 0x00FF;
                               DataLink_i32_U8.BYTE.Byte_3 = RX_B_FRAME.DATA[21] & 0x00FF;
                               RX_HUOKONG.Carrire_Latitude = (float32)(DataLink_i32_U8.i32) * 1e-7;

                               //拦截弹高度
                               DataLink_i16_U8.Byte.Byte_0 = RX_B_FRAME.DATA[22] & 0x00FF;
                               DataLink_i16_U8.Byte.Byte_1 = RX_B_FRAME.DATA[23] & 0x00FF;
                               RX_HUOKONG.Carrire_Altitude = (float32)(DataLink_i16_U8.i16t) * 0.2;
                               break;
                        default:
                            break;
                    }

                    rdataB_head = rdataB_head + 46;//48
                    rdataB_head = rdataB_head & 0xFF;
                    rdataB_length = rdataB_length - 46;
                }
                else
                {
                    rdataB_head = rdataB_head + 1;
                    rdataB_head = rdataB_head & 0xFF;
                    rdataB_length = rdataB_length - 1;
                }
            }
            else
            {
                rdataB_head = rdataB_head + 1;
                rdataB_head = rdataB_head & 0xFF;
                rdataB_length = rdataB_length - 1;
            }
        }
    }

}

void McBSP_Encode(Uint16 *Dat,Uint16 len)
{
    union HexData Hex_Dat[128];                    // McBSP接收到串口的数据,转换后的
//    union HexData McBSP_Tx[128];                    // McBSP接收到串口的数据,转换后的
    int i = 0;

    for(i = 0;i < len;i++)
    {
        Hex_Dat[i].Hexall = Dat[i];
    }
    for(i = 0;i < len;i++)
    {
        McBSP_Rx_SCI[i].bit.bit7       = Hex_Dat[i].bit.bit0;
        McBSP_Rx_SCI[i].bit.bit6       = Hex_Dat[i].bit.bit1;
        McBSP_Rx_SCI[i].bit.bit5       = Hex_Dat[i].bit.bit2;
        McBSP_Rx_SCI[i].bit.bit4       = Hex_Dat[i].bit.bit3;
        McBSP_Rx_SCI[i].bit.bit3       = Hex_Dat[i].bit.bit4;
        McBSP_Rx_SCI[i].bit.bit2       = Hex_Dat[i].bit.bit5;
        McBSP_Rx_SCI[i].bit.bit1       = Hex_Dat[i].bit.bit6;
        McBSP_Rx_SCI[i].bit.bit0       = Hex_Dat[i].bit.bit7;
    }
    for(i = 0;i < len;i++)
    {
        McBSP_Sdata[i] = McBSP_Rx_SCI[i].Hexall << 4;
        McBSP_Sdata[i] = McBSP_Sdata[i] | 0xE00F;
    }
    for(i = len;i < len + 2;i++)
    {
        McBSP_Sdata[i] = 0xFFFF;
    }

}
void McBSP_Decode(union HexData* Src, Uint16* tar,int len)
{
    Uint16 j = 0;
    for(j = 0;j < len;j++)
    {
        tar[j] = ((Uint16)(Src[j].bit.bit7))  | (((Uint16)(Src[j].bit.bit6))<<1) | (((Uint16)(Src[j].bit.bit5)) << 2)
                                              | (((Uint16)(Src[j].bit.bit4))<<3) | (((Uint16)(Src[j].bit.bit3)) << 4)
                                              | (((Uint16)(Src[j].bit.bit2))<<5) | (((Uint16)(Src[j].bit.bit1)) << 6)
                                              | (((Uint16)(Src[j].bit.bit0))<<7);
//        Rx_A.rx_buf[j] = tar[j];
        rdataB[rdataB_tail] = tar[j];   // 读取数据

        rdataB_tail = rdataB_tail + 1;
        rdataB_tail = rdataB_tail & 0xFF;                       // 每存一个字节，循环队列尾指针循环+1
    }
}
//void McBSP_Decode1(void)
//{
//    Uint16 j = 0;
//    for(j = 0;j < 48;j++)
//    {
//        McBSP_Rdata_use[j] = ((Uint16)(McBSP_Rx_SCI_Interim[j].bit.bit7))  | (((Uint16)(McBSP_Rx_SCI_Interim[j].bit.bit6))<<1) | (((Uint16)(McBSP_Rx_SCI_Interim[j].bit.bit5)) << 2)
//                                              | (((Uint16)(McBSP_Rx_SCI_Interim[j].bit.bit4))<<3) | (((Uint16)(McBSP_Rx_SCI_Interim[j].bit.bit3)) << 4)
//                                              | (((Uint16)(McBSP_Rx_SCI_Interim[j].bit.bit2))<<5) | (((Uint16)(McBSP_Rx_SCI_Interim[j].bit.bit1)) << 6)
//                                              | (((Uint16)(McBSP_Rx_SCI_Interim[j].bit.bit0))<<7);
//        rdataB[rdataB_tail] = McBSP_Rdata_use[j];   // 读取数据
//
//        rdataB_tail = rdataB_tail + 1;
//        rdataB_tail = rdataB_tail & 0xFF;                       // 每存一个字节，循环队列尾指针循环+1
//    }
//}
Uint16 crc16_Ibm(unsigned char *data, Uint32 length)
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

void delay_loop(void)
{
    long i;
    for (i = 0; i < MCBSP_INIT_DELAY; i++) {}
}

//*****************************************************************************
// Delay function: at least 2 CLKG cycles
// Required in McBSP init
//*****************************************************************************

