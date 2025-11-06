/*
 * Sci_A.c
 *
 *  Created on: 2022年8月1日
 *      Author: 110
 */
#include "F2837xD_device.h"         // F2837xD Headerfile Include File
#include "F2837xD_Examples.h"       // F2837xD Examples Include File
//#include "Sci_A.h"
#include"COMMON.h"

struct RX_Datalink RX_HUOKONG;
volatile struct RX_BUF Rx_A;
unsigned int SCIA_Txdate[48];
struct State_SCI_Tx SCIA_Tx_State;
//帧计数
uint32_t SCIA_Counter = 0;
struct RX_Sim RX_SimData;

struct RX_McbspData RX_SteeringEngine;
volatile struct RX_B_FRAME_REG RX_A_FRAME;


void InitScia(void)
{
    // Initialize SCI-A:
    SciaRegs.SCICCR.all =0x0007;    // 1 stop bit,  No loopback
    // No parity,8 char bits,
    // async mode, idle-line protocol
    SciaRegs.SCICTL1.all =0x0003;  // enable TX, RX, internal SCICLK,
    // Disable RX ERR, SLEEP, TXWAKE
//    SciaRegs.SCICTL2.all = 0x0003;
    SciaRegs.SCICTL2.bit.TXINTENA =1;
    SciaRegs.SCICTL2.bit.RXBKINTENA =1;
#ifdef Servo_TEST
    //波特率 921600
    SciaRegs.SCIHBAUD.all    =0x0000;
    SciaRegs.SCILBAUD.all    =0x001A;
#endif
#ifdef FZJ_TEST
    //波特率 1250000
    SciaRegs.SCIHBAUD.all    =0x0000;
    SciaRegs.SCILBAUD.all    =0x0013;
#endif



    //FIFO INIT
    SciaRegs.SCIFFTX.all=0xC020; //0x0530
    SciaRegs.SCIFFRX.all=0x0030;

    SciaRegs.SCIFFCT.all=0x0;

    SciaRegs.SCICTL1.all =0x0023;  // Relinquish SCI from Reset
    SciaRegs.SCIFFTX.bit.TXFIFORESET=1;
    SciaRegs.SCIFFRX.bit.RXFIFORESET=1;

}

void SCIA_ReadFIFO(void)
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


#ifdef Servo_TEST
Uint32 SCIA_RX_Cnt = 0;



/*
 * 机械舵面编号与“X”型分布时有差异
 *
 * 机械1号舵面 == “X”型3号舵面
 * 机械2号舵面 == “X”型2号舵面
 * 机械3号舵面 == “X”型1号舵面
 * 机械4号舵面 == “X”型4号舵面
 *
 * “X”型1号舵面 == 机械3号舵面
 * “X”型2号舵面 == 机械2号舵面
 * “X”型3号舵面 == 机械1号舵面
 * “X”型4号舵面 == 机械4号舵面
 *
 * Angle_cmd[0] == Angle_cmd3
 * Angle_cmd[1] == Angle_cmd2
 * Angle_cmd[2] == Angle_cmd1
 * Angle_cmd[3] == Angle_cmd4
 *
 */
void SCIA_RX(void)
{
    //用于CRC循环存数组
    unsigned int rdataA_n = 0;
    //判断接收新数据的数组长度
    static int rdataA_length = 0;
    //暂存CRC结果，用于比对是否正确
    static Uint16 CRC16C_R = 0;
    //数组长度等于接受数据的长度减4，用于CRC校验
    static unsigned char buf16A[30];
    //帧识别码
//    unsigned char FrameID_BUS;


    //拆分CRC的结构体
    U16_Byte C16T8_C_receive;
    U32_Byte DataLink_U32_U8;
    i32_Byte DataLink_i32_U8;

    i16_Byte DataLink_i16_U8;
    U16_Byte DataLink_U16_U8;

    if( SciaRegs.SCIRXST.bit.RXERROR )
    {
        SciaRegs.SCICTL1.bit.SWRESET = 0;
        SciaRegs.SCICTL1.bit.SWRESET = 1;
    }

    if( Rx_A.rx_tail >= Rx_A.rx_head )    rdataA_length = Rx_A.rx_tail - Rx_A.rx_head;
    else                                  rdataA_length = Rx_A.rx_tail + 256 - Rx_A.rx_head;
    // 如果接收缓存数据数有48个或更多，则可能有一帧完整数据
    if( rdataA_length >= 48 )
    {
        // 如果接收缓存数据有超过96 - 1字节,正常情况下应有一帧完整数据，直接从倒数第96字节开始搜寻最新帧
        if( rdataA_length > 95 )
        {
            Rx_A.rx_head = Rx_A.rx_head + rdataA_length - 95;
            Rx_A.rx_head = Rx_A.rx_head & RXBUF_MASK;
            rdataA_length = 95;
        }

        while( rdataA_length >= 48 )
        {
            // 如果当前数据和同步帧数据吻合，则判断帧信息是否完整
            if( Rx_A.rx_buf[Rx_A.rx_head] == 0xEB && Rx_A.rx_buf[( Rx_A.rx_head + 1 ) & RXBUF_MASK] == 0x90 )
            {
//                // CRC校验
                // CRC校验
                for(rdataA_n = 0; rdataA_n < 44; rdataA_n++)//校验第2到第45字节的数据
                {
                    buf16A[rdataA_n] = Rx_A.rx_buf[(Rx_A.rx_head + rdataA_n + 2) & RXBUF_MASK];
                }
                CRC16C_R =  crc16_IBM( buf16A, 44);
                C16T8_C_receive.U16 = CRC16C_R;
                //接收处理
                if( C16T8_C_receive.Byte.Byte_1 == Rx_A.rx_buf[(Rx_A.rx_head + 47) & RXBUF_MASK] && C16T8_C_receive.Byte.Byte_0 == Rx_A.rx_buf[(Rx_A.rx_head + 46) & RXBUF_MASK])
                {
                    for(rdataA_n = 0; rdataA_n < 46; rdataA_n++)
                        RX_A_FRAME.DATA[rdataA_n] = Rx_A.rx_buf[(Rx_A.rx_head + rdataA_n + 2) & 0xFF];
                    SCIA_RX_Cnt++;
                    //帧功能识别码
                    RX_SteeringEngine.FrameID = RX_A_FRAME.DATA[0] & 0x00FF;//功能码
                    switch( RX_SteeringEngine.FrameID )
                   {
                      case 0x33:     // 主控制器1
                          //先高后低传输的
                          DataLink_i16_U8.Byte.Byte_1 = RX_A_FRAME.DATA[3] & 0x00FF;
                          DataLink_i16_U8.Byte.Byte_0 = RX_A_FRAME.DATA[2] & 0x00FF;
                          Angle_Back[0] = (float32)(DataLink_i16_U8.i16t) * 0.001;

                          DataLink_i16_U8.Byte.Byte_1 = RX_A_FRAME.DATA[5] & 0x00FF;
                          DataLink_i16_U8.Byte.Byte_0 = RX_A_FRAME.DATA[4] & 0x00FF;
                          Angle_Back[1] = (float32)(DataLink_i16_U8.i16t) * 0.001;

                          DataLink_i16_U8.Byte.Byte_1 = RX_A_FRAME.DATA[7] & 0x00FF;
                          DataLink_i16_U8.Byte.Byte_0 = RX_A_FRAME.DATA[6] & 0x00FF;
                          Angle_Back[2] = (float32)(DataLink_i16_U8.i16t) * 0.001;

                          DataLink_i16_U8.Byte.Byte_1 = RX_A_FRAME.DATA[9] & 0x00FF;
                          DataLink_i16_U8.Byte.Byte_0 = RX_A_FRAME.DATA[8] & 0x00FF;
                          Angle_Back[3] = (float32)(DataLink_i16_U8.i16t) * 0.001;

                          DataLink_i16_U8.Byte.Byte_1 = RX_A_FRAME.DATA[11] & 0x00FF;
                          DataLink_i16_U8.Byte.Byte_0 = RX_A_FRAME.DATA[10] & 0x00FF;
                          Angle_Back[4] = (float32)(DataLink_i16_U8.i16t) * 0.001;

                          DataLink_i16_U8.Byte.Byte_1 = RX_A_FRAME.DATA[13] & 0x00FF;
                          DataLink_i16_U8.Byte.Byte_0 = RX_A_FRAME.DATA[12] & 0x00FF;
                          Angle_Back[5] = (float32)(DataLink_i16_U8.i16t) * 0.001;

                          DataLink_i16_U8.Byte.Byte_1 = RX_A_FRAME.DATA[15] & 0x00FF;
                          DataLink_i16_U8.Byte.Byte_0 = RX_A_FRAME.DATA[14] & 0x00FF;
                          Angle_Back[6] = (float32)(DataLink_i16_U8.i16t) * 0.001;

                          DataLink_i16_U8.Byte.Byte_1 = RX_A_FRAME.DATA[17] & 0x00FF;
                          DataLink_i16_U8.Byte.Byte_0 = RX_A_FRAME.DATA[16] & 0x00FF;
                          Angle_Back[7] = (float32)(DataLink_i16_U8.i16t) * 0.001;

                          DataLink_i16_U8.Byte.Byte_1 = RX_A_FRAME.DATA[18] & 0x00FF;
                          DataLink_i16_U8.Byte.Byte_0 = RX_A_FRAME.DATA[19] & 0x00FF;
                          Angle_Back[8] = (float32)(DataLink_i16_U8.i16t) * 0.001;

                          DataLink_i16_U8.Byte.Byte_1 = RX_A_FRAME.DATA[20] & 0x00FF;
                          DataLink_i16_U8.Byte.Byte_0 = RX_A_FRAME.DATA[21] & 0x00FF;
                          Angle_Back[9] = (float32)(DataLink_i16_U8.i16t) * 0.001;

                          DataLink_i16_U8.Byte.Byte_1 = RX_A_FRAME.DATA[22] & 0x00FF;
                          DataLink_i16_U8.Byte.Byte_0 = RX_A_FRAME.DATA[23] & 0x00FF;
                          Angle_Back[10] = (float32)(DataLink_i16_U8.i16t) * 0.001;

                          DataLink_i16_U8.Byte.Byte_1 = RX_A_FRAME.DATA[24] & 0x00FF;
                          DataLink_i16_U8.Byte.Byte_0 = RX_A_FRAME.DATA[25] & 0x00FF;
                          Angle_Back[11] = (float32)(DataLink_i16_U8.i16t) * 0.001;

                          RX_SteeringEngine.Angle_1 = Angle_Back[2];
                          RX_SteeringEngine.Angle_2 = Angle_Back[1];
                          RX_SteeringEngine.Angle_3 = Angle_Back[0];
                          RX_SteeringEngine.Angle_4 = Angle_Back[3];
                          RX_SteeringEngine.Angle_speed_1 = Angle_Back[6];
                          RX_SteeringEngine.Angle_speed_2 = Angle_Back[5];
                          RX_SteeringEngine.Angle_speed_3 = Angle_Back[4];
                          RX_SteeringEngine.Angle_speed_4 = Angle_Back[7];
                          RX_SteeringEngine.Angle_cmd_1 = Angle_Back[10];
                          RX_SteeringEngine.Angle_cmd_2 = Angle_Back[9];
                          RX_SteeringEngine.Angle_cmd_3 = Angle_Back[8];
                          RX_SteeringEngine.Angle_cmd_4 = Angle_Back[11];
                          Servo_Deflection1 = RX_SteeringEngine.Angle_1;
                          Servo_Deflection2 = RX_SteeringEngine.Angle_2;
                          Servo_Deflection3 = RX_SteeringEngine.Angle_3;
                          Servo_Deflection4 = RX_SteeringEngine.Angle_4;

                          break;
                    default:
                        break;
                    }
                    Rx_A.rx_head = Rx_A.rx_head + 48;
                    Rx_A.rx_head = Rx_A.rx_head & RXBUF_MASK;
                    rdataA_length = rdataA_length - 48;
                }
                else
                {
                    Rx_A.rx_head = Rx_A.rx_head + 1;
                    Rx_A.rx_head = Rx_A.rx_head & RXBUF_MASK;
                    rdataA_length = rdataA_length - 1;
                }
            }
            else
            {
                Rx_A.rx_head = Rx_A.rx_head + 1;
                Rx_A.rx_head = Rx_A.rx_head & RXBUF_MASK;
                rdataA_length = rdataA_length - 1;
            }
        }

    }
}

unsigned char SCIA_SEND_HUOKONG[80];
extern float PID_Result_GyroX_1;
extern float PID_Result_GyroY_1;
extern float PID_Result_GyroZ_1;
extern float InsOverload_aY;    //弹指令过载aY
extern float InsOverload_aZ;    //弹指令过载aZ
extern float pitch_f,roll_f,yaw_f;
float testdata3,testdata4;
float Angle_cmd1 = 0;
float Angle_cmd2 = 0;
float Angle_cmd3 = 0;
float Angle_cmd4 = 0;
float Angle_cmd[4] = {0};
float Angle_Back[12] = {0};
uint32_t SCIA_Counter_Servo = 0;
void SCIA_TO_Servo(void)
{

    //CRC
    static unsigned char buf16_crc[76];//CRC缓存数组
    Uint16 send_crc = 0;//CEC最终结果
    U16_Byte C16T8_C_send;//CRC结构体转换
    unsigned int CRC_count = 0;//CRC缓存遍历使用
    //结构体实例化
    i16_Byte i16_DATA;


    SCIA_Txdate[0] = 0xEB;
    SCIA_Txdate[1] = 0x90;
    SCIA_Txdate[2] = 0xCA;

    //舵机识别码
    SCIA_Txdate[3] = 0X33;


    Angle_cmd3 = Angle_cmd[0];
    Angle_cmd2 = Angle_cmd[1];
    Angle_cmd1 = Angle_cmd[2];
    Angle_cmd4 = Angle_cmd[3];

    //舵机舵量1
//    Angle_cmd1 = 10;
    i16_DATA.i16t = (int16)(Angle_cmd1 * 1000);//分辨率0.001
    SCIA_Txdate[4] = i16_DATA.Byte.Byte_0;
    SCIA_Txdate[5] = i16_DATA.Byte.Byte_1;
    //舵机舵量2
//    Angle_cmd2 = -10;
    i16_DATA.i16t = (int16)(Angle_cmd2 * 1000);//分辨率0.001
    SCIA_Txdate[6] = i16_DATA.Byte.Byte_0;
    SCIA_Txdate[7] = i16_DATA.Byte.Byte_1;
    //舵机舵量3
//    Angle_cmd3 = -5;
    i16_DATA.i16t = (int16)(Angle_cmd3 * 1000);//分辨率0.001
    SCIA_Txdate[8] = i16_DATA.Byte.Byte_0;
    SCIA_Txdate[9] = i16_DATA.Byte.Byte_1;
    //舵机舵量4
//    Angle_cmd4 = 5;
    i16_DATA.i16t = (int16)(Angle_cmd4 * 1000);//分辨率0.001
    SCIA_Txdate[10] = i16_DATA.Byte.Byte_0;
    SCIA_Txdate[11] = i16_DATA.Byte.Byte_1;


    for(CRC_count = 0; CRC_count < 44; CRC_count++)
    {
        buf16_crc[CRC_count] = SCIA_Txdate[CRC_count +2];
    }
    send_crc = crc16_IBM(buf16_crc,44);
    C16T8_C_send.U16 = send_crc;
    SCIA_Txdate[46] = C16T8_C_send.Byte.Byte_0;
    SCIA_Txdate[47] = C16T8_C_send.Byte.Byte_1;

    SCIA_Counter_Servo++;

    SCIA_Tx_State.State = Start_Tx;
    SCIA_Tx_State.Field_Number = One;
    SciaRegs.SCIFFTX.bit.TXFFINTCLR = 1;
}
#endif




#ifdef FZJ_TEST
Uint32 SCIA_RX_Cnt;

void SCIA_RX_FZJ(void)
{
    //用于CRC循环存数组
    unsigned int rdataA_n = 0;
    //判断接收新数据的数组长度
    static int rdataA_length = 0;
    //暂存CRC结果，用于比对是否正确
    static Uint16 CRC16C_R = 0;
    //数组长度等于接受数据的长度减4，用于CRC校验
    static unsigned char buf16A[30];
    //帧识别码
//    unsigned char FrameID_BUS;


    //拆分CRC的结构体
    U16_Byte C16T8_C_receive;
    U32_Byte DataLink_U32_U8;
    i32_Byte DataLink_i32_U8;

    i16_Byte DataLink_i16_U8;
    U16_Byte DataLink_U16_U8;

    if( SciaRegs.SCIRXST.bit.RXERROR )
    {
        SciaRegs.SCICTL1.bit.SWRESET = 0;
        SciaRegs.SCICTL1.bit.SWRESET = 1;
    }

    if( Rx_A.rx_tail >= Rx_A.rx_head )    rdataA_length = Rx_A.rx_tail - Rx_A.rx_head;
    else                                  rdataA_length = Rx_A.rx_tail + 256 - Rx_A.rx_head;
    // 如果接收缓存数据数有48个或更多，则可能有一帧完整数据
    if( rdataA_length >= 48 )
    {
        // 如果接收缓存数据有超过96 - 1字节,正常情况下应有一帧完整数据，直接从倒数第96字节开始搜寻最新帧
        if( rdataA_length > 95 )
        {
            Rx_A.rx_head = Rx_A.rx_head + rdataA_length - 95;
            Rx_A.rx_head = Rx_A.rx_head & RXBUF_MASK;
            rdataA_length = 95;
        }

        while( rdataA_length >= 48 )
        {
            // 如果当前数据和同步帧数据吻合，则判断帧信息是否完整
            if( Rx_A.rx_buf[Rx_A.rx_head] == 0xEB && Rx_A.rx_buf[( Rx_A.rx_head + 1 ) & RXBUF_MASK] == 0x90 )
            {
//                // CRC校验
                // CRC校验
                for(rdataA_n = 0; rdataA_n < 44; rdataA_n++)//校验第2到第45字节的数据
                {
                    buf16A[rdataA_n] = Rx_A.rx_buf[(Rx_A.rx_head + rdataA_n + 2) & RXBUF_MASK];
                }
                CRC16C_R =  crc16_IBM( buf16A, 44);
                C16T8_C_receive.U16 = CRC16C_R;
                //接收处理
                if( C16T8_C_receive.Byte.Byte_1 == Rx_A.rx_buf[(Rx_A.rx_head + 47) & RXBUF_MASK] && C16T8_C_receive.Byte.Byte_0 == Rx_A.rx_buf[(Rx_A.rx_head + 46) & RXBUF_MASK])
                {
                    for(rdataA_n = 0; rdataA_n < 46; rdataA_n++)
                        RX_A_FRAME.DATA[rdataA_n] = Rx_A.rx_buf[(Rx_A.rx_head + rdataA_n + 2) & 0xFF];
                    SCIA_RX_Cnt++;
                    //帧功能识别码
                    RX_SimData.FrameID = RX_A_FRAME.DATA[0] & 0x00FF;//功能码
                    switch( RX_SimData.FrameID )
                   {
                      case 0x36:     // 主控制器1
                          DataLink_U32_U8.BYTE.Byte_0 = RX_A_FRAME.DATA[2] & 0x00FF;
                          DataLink_U32_U8.BYTE.Byte_1 = RX_A_FRAME.DATA[3] & 0x00FF;
                          DataLink_U32_U8.BYTE.Byte_2 = RX_A_FRAME.DATA[4] & 0x00FF;
                          DataLink_U32_U8.BYTE.Byte_3 = RX_A_FRAME.DATA[5] & 0x00FF;
                          RX_SimData.FrameCnt = DataLink_U32_U8.U32;

                          DataLink_i32_U8.BYTE.Byte_0 = RX_A_FRAME.DATA[6] & 0x00FF;
                          DataLink_i32_U8.BYTE.Byte_1 = RX_A_FRAME.DATA[7] & 0x00FF;
                          DataLink_i32_U8.BYTE.Byte_2 = RX_A_FRAME.DATA[8] & 0x00FF;
                          DataLink_i32_U8.BYTE.Byte_3 = RX_A_FRAME.DATA[9] & 0x00FF;
                          RX_SimData.FZJ_Acc_X = (float32)(DataLink_i32_U8.i32) * 1e-6;

                          DataLink_i32_U8.BYTE.Byte_0 = RX_A_FRAME.DATA[10] & 0x00FF;
                          DataLink_i32_U8.BYTE.Byte_1 = RX_A_FRAME.DATA[11] & 0x00FF;
                          DataLink_i32_U8.BYTE.Byte_2 = RX_A_FRAME.DATA[12] & 0x00FF;
                          DataLink_i32_U8.BYTE.Byte_3 = RX_A_FRAME.DATA[13] & 0x00FF;
                          RX_SimData.FZJ_Acc_Y = (float32)(DataLink_i32_U8.i32) * 1e-6;

                          DataLink_i32_U8.BYTE.Byte_0 = RX_A_FRAME.DATA[14] & 0x00FF;
                          DataLink_i32_U8.BYTE.Byte_1 = RX_A_FRAME.DATA[15] & 0x00FF;
                          DataLink_i32_U8.BYTE.Byte_2 = RX_A_FRAME.DATA[16] & 0x00FF;
                          DataLink_i32_U8.BYTE.Byte_3 = RX_A_FRAME.DATA[17] & 0x00FF;
                          RX_SimData.FZJ_Acc_Z = (float32)(DataLink_i32_U8.i32) * 1e-6;

                          DataLink_i32_U8.BYTE.Byte_0 = RX_A_FRAME.DATA[18] & 0x00FF;
                          DataLink_i32_U8.BYTE.Byte_1 = RX_A_FRAME.DATA[19] & 0x00FF;
                          DataLink_i32_U8.BYTE.Byte_2 = RX_A_FRAME.DATA[20] & 0x00FF;
                          DataLink_i32_U8.BYTE.Byte_3 = RX_A_FRAME.DATA[21] & 0x00FF;
                          RX_SimData.FZJ_Pitch_VelAng = (float32)(DataLink_i32_U8.i32) * 1e-6;

                          DataLink_i32_U8.BYTE.Byte_0 = RX_A_FRAME.DATA[22] & 0x00FF;
                          DataLink_i32_U8.BYTE.Byte_1 = RX_A_FRAME.DATA[23] & 0x00FF;
                          DataLink_i32_U8.BYTE.Byte_2 = RX_A_FRAME.DATA[24] & 0x00FF;
                          DataLink_i32_U8.BYTE.Byte_3 = RX_A_FRAME.DATA[25] & 0x00FF;
                          RX_SimData.FZJ_Yaw_VelAng = (float32)(DataLink_i32_U8.i32) * 1e-6;

                          DataLink_i32_U8.BYTE.Byte_0 = RX_A_FRAME.DATA[26] & 0x00FF;
                          DataLink_i32_U8.BYTE.Byte_1 = RX_A_FRAME.DATA[27] & 0x00FF;
                          DataLink_i32_U8.BYTE.Byte_2 = RX_A_FRAME.DATA[28] & 0x00FF;
                          DataLink_i32_U8.BYTE.Byte_3 = RX_A_FRAME.DATA[29] & 0x00FF;
                          RX_SimData.FZJ_Roll_VelAng = (float32)(DataLink_i32_U8.i32) * 1e-6;

                          DataLink_i16_U8.Byte.Byte_0 = RX_A_FRAME.DATA[30] & 0x00FF;
                          DataLink_i16_U8.Byte.Byte_1 = RX_A_FRAME.DATA[31] & 0x00FF;
                          RX_SimData.Yaw_VelAng_Sim = (float32)(DataLink_i16_U8.i16t) * 0.01;

                          DataLink_i16_U8.Byte.Byte_0 = RX_A_FRAME.DATA[32] & 0x00FF;
                          DataLink_i16_U8.Byte.Byte_1 = RX_A_FRAME.DATA[33] & 0x00FF;
                          RX_SimData.Pitch_VelAng_Sim = (float32)(DataLink_i16_U8.i16t) * 0.01;
                          break;
                    default:
                        break;
                    }
                    Rx_A.rx_head = Rx_A.rx_head + 48;
                    Rx_A.rx_head = Rx_A.rx_head & RXBUF_MASK;
                    rdataA_length = rdataA_length - 48;
                }
                else
                {
                    Rx_A.rx_head = Rx_A.rx_head + 1;
                    Rx_A.rx_head = Rx_A.rx_head & RXBUF_MASK;
                    rdataA_length = rdataA_length - 1;
                }
            }
            else
            {
                Rx_A.rx_head = Rx_A.rx_head + 1;
                Rx_A.rx_head = Rx_A.rx_head & RXBUF_MASK;
                rdataA_length = rdataA_length - 1;
            }
        }

    }
}

unsigned char SCIA_SEND_HUOKONG[80];
extern float PID_Result_GyroX_1;
extern float PID_Result_GyroY_1;
extern float PID_Result_GyroZ_1;
extern float InsOverload_aY;    //弹指令过载aY
extern float InsOverload_aZ;    //弹指令过载aZ
extern float pitch_f,roll_f,yaw_f;
float testdata3,testdata4;
float Angle_cmd1 = 0;
float Angle_cmd2 = 0;
float Angle_cmd3 = 0;
float Angle_cmd4 = 0;
uint32_t SCIA_Counter_Servo = 0;
void SCIA_TO_FZJ(void)
{

    //CRC
    static unsigned char buf16_crc[76];//CRC缓存数组
    Uint16 send_crc = 0;//CEC最终结果
    U16_Byte C16T8_C_send;//CRC结构体转换
    unsigned int CRC_count = 0;//CRC缓存遍历使用
    //结构体实例化
    U32_Byte U32_DATA;
    i16_Byte i16_DATA;


    SCIA_Txdate[0] = 0xEB;
    SCIA_Txdate[1] = 0x90;
    SCIA_Txdate[2] = 0x37;

    //飞行器编号
    SCIA_Txdate[3] = 0X01;



    //帧计数
    U32_DATA.U32 = SCIA_Counter;
    SCIA_Txdate[4] = U32_DATA.BYTE.Byte_0;
    SCIA_Txdate[5] = U32_DATA.BYTE.Byte_1;
    SCIA_Txdate[6] = U32_DATA.BYTE.Byte_2;
    SCIA_Txdate[7] = U32_DATA.BYTE.Byte_3;



//    //舵机舵量1
////    Angle_cmd1 = 10;
//    i16_DATA.i16t = (int16)(Angle_cmd1 * 1000);//分辨率0.001
//    SCIA_Txdate[4] = i16_DATA.Byte.Byte_0;
//    SCIA_Txdate[5] = i16_DATA.Byte.Byte_1;
//    //舵机舵量2
////    Angle_cmd2 = -10;
//    i16_DATA.i16t = (int16)(Angle_cmd2 * 1000);//分辨率0.001
//    SCIA_Txdate[6] = i16_DATA.Byte.Byte_0;
//    SCIA_Txdate[7] = i16_DATA.Byte.Byte_1;
//    //舵机舵量3
////    Angle_cmd3 = -5;
//    i16_DATA.i16t = (int16)(Angle_cmd3 * 1000);//分辨率0.001
//    SCIA_Txdate[8] = i16_DATA.Byte.Byte_0;
//    SCIA_Txdate[9] = i16_DATA.Byte.Byte_1;
//    //舵机舵量4
////    Angle_cmd4 = 5;
//    i16_DATA.i16t = (int16)(Angle_cmd4 * 1000);//分辨率0.001
//    SCIA_Txdate[10] = i16_DATA.Byte.Byte_0;
//    SCIA_Txdate[11] = i16_DATA.Byte.Byte_1;


    for(CRC_count = 0; CRC_count < 44; CRC_count++)
    {
        buf16_crc[CRC_count] = SCIA_Txdate[CRC_count +2];
    }
    send_crc = crc16_IBM(buf16_crc,44);
    C16T8_C_send.U16 = send_crc;
    SCIA_Txdate[46] = C16T8_C_send.Byte.Byte_0;
    SCIA_Txdate[47] = C16T8_C_send.Byte.Byte_1;

    SCIA_Counter++;

    SCIA_Tx_State.State = Start_Tx;
    SCIA_Tx_State.Field_Number = One;
    SciaRegs.SCIFFTX.bit.TXFFINTCLR = 1;
}
#endif

//发送任意数据程序
void scia_xmit(int a)
{
    while (SciaRegs.SCIFFTX.bit.TXFFST != 0) {}
    SciaRegs.SCITXBUF.all =a;
}

void scia_msg(char * msg)
{
    int i;
    i = 0;
    while(msg[i] != '\0')
    {
        scia_xmit(msg[i]);
        i++;
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

