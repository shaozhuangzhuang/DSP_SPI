
#include"COMMON.h"
//#include"Sci_C.h"
#include "F28x_Project.h"


volatile struct RX_BUF Rx_C;


int last_frameCount;
//TEST USES
unsigned int send_counter = 0;

struct State_SCI_Tx SCIC_Tx_State;

const uint16_t crc_ibm_table[256] = {
                                     0x0000, 0xc0c1, 0xc181, 0x0140, 0xc301, 0x03c0, 0x0280, 0xc241,
                                     0xc601, 0x06c0, 0x0780, 0xc741, 0x0500, 0xc5c1, 0xc481, 0x0440,
                                     0xcc01, 0x0cc0, 0x0d80, 0xcd41, 0x0f00, 0xcfc1, 0xce81, 0x0e40,
                                     0x0a00, 0xcac1, 0xcb81, 0x0b40, 0xc901, 0x09c0, 0x0880, 0xc841,
                                     0xd801, 0x18c0, 0x1980, 0xd941, 0x1b00, 0xdbc1, 0xda81, 0x1a40,
                                     0x1e00, 0xdec1, 0xdf81, 0x1f40, 0xdd01, 0x1dc0, 0x1c80, 0xdc41,
                                     0x1400, 0xd4c1, 0xd581, 0x1540, 0xd701, 0x17c0, 0x1680, 0xd641,
                                     0xd201, 0x12c0, 0x1380, 0xd341, 0x1100, 0xd1c1, 0xd081, 0x1040,
                                     0xf001, 0x30c0, 0x3180, 0xf141, 0x3300, 0xf3c1, 0xf281, 0x3240,
                                     0x3600, 0xf6c1, 0xf781, 0x3740, 0xf501, 0x35c0, 0x3480, 0xf441,
                                     0x3c00, 0xfcc1, 0xfd81, 0x3d40, 0xff01, 0x3fc0, 0x3e80, 0xfe41,
                                     0xfa01, 0x3ac0, 0x3b80, 0xfb41, 0x3900, 0xf9c1, 0xf881, 0x3840,
                                     0x2800, 0xe8c1, 0xe981, 0x2940, 0xeb01, 0x2bc0, 0x2a80, 0xea41,
                                     0xee01, 0x2ec0, 0x2f80, 0xef41, 0x2d00, 0xedc1, 0xec81, 0x2c40,
                                     0xe401, 0x24c0, 0x2580, 0xe541, 0x2700, 0xe7c1, 0xe681, 0x2640,
                                     0x2200, 0xe2c1, 0xe381, 0x2340, 0xe101, 0x21c0, 0x2080, 0xe041,
                                     0xa001, 0x60c0, 0x6180, 0xa141, 0x6300, 0xa3c1, 0xa281, 0x6240,
                                     0x6600, 0xa6c1, 0xa781, 0x6740, 0xa501, 0x65c0, 0x6480, 0xa441,
                                     0x6c00, 0xacc1, 0xad81, 0x6d40, 0xaf01, 0x6fc0, 0x6e80, 0xae41,
                                     0xaa01, 0x6ac0, 0x6b80, 0xab41, 0x6900, 0xa9c1, 0xa881, 0x6840,
                                     0x7800, 0xb8c1, 0xb981, 0x7940, 0xbb01, 0x7bc0, 0x7a80, 0xba41,
                                     0xbe01, 0x7ec0, 0x7f80, 0xbf41, 0x7d00, 0xbdc1, 0xbc81, 0x7c40,
                                     0xb401, 0x74c0, 0x7580, 0xb541, 0x7700, 0xb7c1, 0xb681, 0x7640,
                                     0x7200, 0xb2c1, 0xb381, 0x7340, 0xb101, 0x71c0, 0x7080, 0xb041,
                                     0x5000, 0x90c1, 0x9181, 0x5140, 0x9301, 0x53c0, 0x5280, 0x9241,
                                     0x9601, 0x56c0, 0x5780, 0x9741, 0x5500, 0x95c1, 0x9481, 0x5440,
                                     0x9c01, 0x5cc0, 0x5d80, 0x9d41, 0x5f00, 0x9fc1, 0x9e81, 0x5e40,
                                     0x5a00, 0x9ac1, 0x9b81, 0x5b40, 0x9901, 0x59c0, 0x5880, 0x9841,
                                     0x8801, 0x48c0, 0x4980, 0x8941, 0x4b00, 0x8bc1, 0x8a81, 0x4a40,
                                     0x4e00, 0x8ec1, 0x8f81, 0x4f40, 0x8d01, 0x4dc0, 0x4c80, 0x8c41,
                                     0x4400, 0x84c1, 0x8581, 0x4540, 0x8701, 0x47c0, 0x4680, 0x8641,
                                     0x8201, 0x42c0, 0x4380, 0x8341, 0x4100, 0x81c1, 0x8081, 0x4040,
};


/******************************************************************************
 * Name:    CRC-16/IBM          x16+x15+x2+1
 * Poly:    0x8005
 * Init:    0x0000
 * Refin:   True
 * Refout:  True
 * Xorout:  0x0000
 * Alias:   CRC-16,CRC-16/ARC,CRC-16/LHA
 *****************************************************************************/
Uint16 crc16_ibm(unsigned char *data, Uint32 length)
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



//总线串口寄存器配置初始化
void SCIC_Reg_Init(void)
{
    ScicRegs.SCICCR.all =0x0007;    // 1 stop bit,  No loopback
    // No parity,8 char bits,
    // async mode, idle-line protocol
    ScicRegs.SCICTL1.all =0x0003;  // enable TX, RX, internal SCICLK,
    // Disable RX ERR, SLEEP, TXWAKE
    ScicRegs.SCICTL2.bit.TXINTENA =1;
    ScicRegs.SCICTL2.bit.RXBKINTENA =1;
    //波特率 921600
    ScicRegs.SCIHBAUD.all    =0x0000;
    ScicRegs.SCILBAUD.all    =0x001A;
#ifdef FZJ
    ScicRegs.SCILBAUD.all    =0x0009;//  0c:921600  09:1250000
#endif
#ifdef DATALINK
    ScicRegs.SCILBAUD.all    =0x000c;//  0c:921600  09:1250000
#endif
//#ifdef DATA_TEST
//    ScicRegs.SCILBAUD.all    =0x0009;
//#endif
    //FIFO INIT
    ScicRegs.SCIFFTX.all=0xC020;      //发送中断模式
    //ScicRegs.SCIFFTX.all=0xE040;    //发送查询模式
    //    ScicRegs.SCIFFRX.all=0x2040;      //接收查询模式
    ScicRegs.SCIFFRX.all=0x0030;   //接收中断模式
    ScicRegs.SCIFFRX.bit.RXFFST = 0x10;


    ScicRegs.SCIFFCT.all=0x0;

    ScicRegs.SCICTL1.all =0x0023;  // Relinquish SCI from Reset
    ScicRegs.SCIFFTX.bit.TXFIFORESET=1;
    ScicRegs.SCIFFRX.bit.RXFIFORESET=1;

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
        Rx_C.rx_buf[Rx_C.rx_tail] = ScicRegs.SCIRXBUF.all & RXBUF_MASK; // 读取数据
        Rx_C.rx_tail = Rx_C.rx_tail + 1;
        Rx_C.rx_tail = Rx_C.rx_tail & RXBUF_MASK;                       // 每存一个字节，循环队列尾指针循环+1
    }
}




//TEST USE
#ifdef FZJ

unsigned char FrameID_BUS;
int receive_ocunter = 0;
void SCIC_RX(void)
{
    //用于CRC循环存数组
    unsigned int rdataC_n = 0;
    //判断接收新数据的数组长度
    static int rdataC_length = 0;
    //暂存CRC结果，用于比对是否正确
    static Uint16 CRC16C_R = 0;
    //数组长度等于接受数据的长度减4，用于CRC校验
    static unsigned char buf16C[44];
    //帧识别码
    int i;
    //拆分CRC的结构体
    U16_Byte C16T8_C_receive;

    //用于解包导引头三字节长度的加速度
    uint8_i32 Seek_depack;

    //仿真机解包用
    U32_Byte SIMU_U32_U8;
    i32_Byte SIMU_i32_U8;
    i16_Byte SIMU_i16_U8;
    if( Rx_C.rx_tail >= Rx_C.rx_head )
        rdataC_length = Rx_C.rx_tail - Rx_C.rx_head;
    else
        rdataC_length = Rx_C.rx_tail + 256 - Rx_C.rx_head;
    // 如果接收缓存数据数有48个或更多，则可能有一帧完整数据
    if( rdataC_length >= 48 )
    {
        if(rdataC_length > 255)
        {
            asm ( "ESTOP0");
        }
        // 如果接收缓存数据有超过96 - 1字节,正常情况下应有一帧完整数据，直接从倒数第96字节开始搜寻最新帧
        if( rdataC_length > 95 )
        {
            Rx_C.rx_head = Rx_C.rx_head + rdataC_length - 95;
            Rx_C.rx_head = Rx_C.rx_head & RXBUF_MASK;
            rdataC_length = 95;
        }

        while( rdataC_length >= 48 )
        {
            // 如果当前数据和同步帧数据吻合，则判断帧信息是否完整
            if( Rx_C.rx_buf[Rx_C.rx_head] == 0xEB && Rx_C.rx_buf[( Rx_C.rx_head + 1 ) & RXBUF_MASK] == 0x90 )
            {

                // CRC校验
                for(rdataC_n = 0; rdataC_n < 44; rdataC_n++)//校验第2到第45字节的数据
                {
                    buf16C[rdataC_n] = Rx_C.rx_buf[(Rx_C.rx_head + rdataC_n + 2) & RXBUF_MASK];
                }
                CRC16C_R =  crc16_ibm( buf16C, 44 );
                C16T8_C_receive.U16 = CRC16C_R;
                // 接收处理
                if(1)// C16T8_C_receive.Byte.Byte_1 == Rx_C.rx_buf[(Rx_C.rx_head + 47) & RXBUF_MASK] && C16T8_C_receive.Byte.Byte_0 == Rx_C.rx_buf[(Rx_C.rx_head + 46) & RXBUF_MASK])
                {
                    FrameID_BUS = Rx_C.rx_buf[(Rx_C.rx_head + 2) & RXBUF_MASK];
                    switch(FrameID_BUS)
                    {
                        //仿真机数据1
                    case 0x36:
                        //飞行器编号
                        Rx_SimToFlight_Rx.FlightNum = Rx_C.rx_buf[(Rx_C.rx_head + 3) & 0x00FF];
                        //帧计数
                        SIMU_U32_U8.BYTE.Byte_0 = Rx_C.rx_buf[(Rx_C.rx_head + 4) & 0x00FF];
                        SIMU_U32_U8.BYTE.Byte_1 = Rx_C.rx_buf[(Rx_C.rx_head + 5) & 0x00FF];
                        SIMU_U32_U8.BYTE.Byte_2 = Rx_C.rx_buf[(Rx_C.rx_head + 6) & 0x00FF];
                        SIMU_U32_U8.BYTE.Byte_3 = Rx_C.rx_buf[(Rx_C.rx_head + 7) & 0x00FF];
                        Rx_SimToFlight_Rx.FrameCnt1 = SIMU_U32_U8.U32;
                        //加速度ax
                        SIMU_i32_U8.BYTE.Byte_0 = Rx_C.rx_buf[(Rx_C.rx_head + 8) & 0x00FF];
                        SIMU_i32_U8.BYTE.Byte_1 = Rx_C.rx_buf[(Rx_C.rx_head + 9) & 0x00FF];
                        SIMU_i32_U8.BYTE.Byte_2 = Rx_C.rx_buf[(Rx_C.rx_head + 10) & 0x00FF];
                        SIMU_i32_U8.BYTE.Byte_3 = Rx_C.rx_buf[(Rx_C.rx_head + 11) & 0x00FF];
                        Rx_SimToFlight_Rx.Rx_Acc_X_Sim = ((float32)SIMU_i32_U8.i32) / 1000000;
                        //加速度ay
                        SIMU_i32_U8.BYTE.Byte_0 = Rx_C.rx_buf[(Rx_C.rx_head + 12) & 0x00FF];
                        SIMU_i32_U8.BYTE.Byte_1 = Rx_C.rx_buf[(Rx_C.rx_head + 13) & 0x00FF];
                        SIMU_i32_U8.BYTE.Byte_2 = Rx_C.rx_buf[(Rx_C.rx_head + 14) & 0x00FF];
                        SIMU_i32_U8.BYTE.Byte_3 = Rx_C.rx_buf[(Rx_C.rx_head + 15) & 0x00FF];
                        Rx_SimToFlight_Rx.Rx_Acc_Y_Sim = ((float32)SIMU_i32_U8.i32) / 1000000;
                        //加速度az
                        SIMU_i32_U8.BYTE.Byte_0 = Rx_C.rx_buf[(Rx_C.rx_head + 16) & 0x00FF];
                        SIMU_i32_U8.BYTE.Byte_1 = Rx_C.rx_buf[(Rx_C.rx_head + 17) & 0x00FF];
                        SIMU_i32_U8.BYTE.Byte_2 = Rx_C.rx_buf[(Rx_C.rx_head + 18) & 0x00FF];
                        SIMU_i32_U8.BYTE.Byte_3 = Rx_C.rx_buf[(Rx_C.rx_head + 19) & 0x00FF];
                        Rx_SimToFlight_Rx.Rx_Acc_Z_Sim = ((float32)SIMU_i32_U8.i32) / 1000000;//调试用改动1e5
                        //俯仰角速度
                        SIMU_i32_U8.BYTE.Byte_0 = Rx_C.rx_buf[(Rx_C.rx_head + 20) & 0x00FF];
                        SIMU_i32_U8.BYTE.Byte_1 = Rx_C.rx_buf[(Rx_C.rx_head + 21) & 0x00FF];
                        SIMU_i32_U8.BYTE.Byte_2 = Rx_C.rx_buf[(Rx_C.rx_head + 22) & 0x00FF];
                        SIMU_i32_U8.BYTE.Byte_3 = Rx_C.rx_buf[(Rx_C.rx_head + 23) & 0x00FF];
                        Rx_SimToFlight_Rx.Rx_Pitch_VelAng_Sim = ((float32)SIMU_i32_U8.i32) / 1000000;
                        //偏航角速度
                        SIMU_i32_U8.BYTE.Byte_0 = Rx_C.rx_buf[(Rx_C.rx_head + 24) & 0x00FF];
                        SIMU_i32_U8.BYTE.Byte_1 = Rx_C.rx_buf[(Rx_C.rx_head + 25) & 0x00FF];
                        SIMU_i32_U8.BYTE.Byte_2 = Rx_C.rx_buf[(Rx_C.rx_head + 26) & 0x00FF];
                        SIMU_i32_U8.BYTE.Byte_3 = Rx_C.rx_buf[(Rx_C.rx_head + 27) & 0x00FF];
                        Rx_SimToFlight_Rx.Rx_Yaw_VelAng_Sim = ((float32)SIMU_i32_U8.i32) / 1000000;
                        //滚转角速度
                        SIMU_i32_U8.BYTE.Byte_0 = Rx_C.rx_buf[(Rx_C.rx_head + 28) & 0x00FF];
                        SIMU_i32_U8.BYTE.Byte_1 = Rx_C.rx_buf[(Rx_C.rx_head + 29) & 0x00FF];
                        SIMU_i32_U8.BYTE.Byte_2 = Rx_C.rx_buf[(Rx_C.rx_head + 30) & 0x00FF];
                        SIMU_i32_U8.BYTE.Byte_3 = Rx_C.rx_buf[(Rx_C.rx_head + 31) & 0x00FF];
                        Rx_SimToFlight_Rx.Rx_Roll_VelAng_Sim = ((float32)SIMU_i32_U8.i32) / 1000000;


                        //用于调试控制系统
                        SIMU_i32_U8.BYTE.Byte_0 = Rx_C.rx_buf[(Rx_C.rx_head + 32) & 0x00FF];
                        SIMU_i32_U8.BYTE.Byte_1 = Rx_C.rx_buf[(Rx_C.rx_head + 33) & 0x00FF];
                        SIMU_i32_U8.BYTE.Byte_2 = Rx_C.rx_buf[(Rx_C.rx_head + 34) & 0x00FF];
                        SIMU_i32_U8.BYTE.Byte_3 = Rx_C.rx_buf[(Rx_C.rx_head + 35) & 0x00FF];
                        Rx_SimToFlight_Rx.Rx_overload_YC_Sim = ((float32)SIMU_i32_U8.i32) / 1e7;

                        SIMU_i32_U8.BYTE.Byte_0 = Rx_C.rx_buf[(Rx_C.rx_head + 36) & 0x00FF];
                        SIMU_i32_U8.BYTE.Byte_1 = Rx_C.rx_buf[(Rx_C.rx_head + 37) & 0x00FF];
                        SIMU_i32_U8.BYTE.Byte_2 = Rx_C.rx_buf[(Rx_C.rx_head + 38) & 0x00FF];
                        SIMU_i32_U8.BYTE.Byte_3 = Rx_C.rx_buf[(Rx_C.rx_head + 39) & 0x00FF];
                        Rx_SimToFlight_Rx.Rx_overload_ZC_Sim = ((float32)SIMU_i32_U8.i32) / 1e7;

                        SIMU_i32_U8.BYTE.Byte_0 = Rx_C.rx_buf[(Rx_C.rx_head + 40) & 0x00FF];
                        SIMU_i32_U8.BYTE.Byte_1 = Rx_C.rx_buf[(Rx_C.rx_head + 41) & 0x00FF];
                        SIMU_i32_U8.BYTE.Byte_2 = Rx_C.rx_buf[(Rx_C.rx_head + 42) & 0x00FF];
                        SIMU_i32_U8.BYTE.Byte_3 = Rx_C.rx_buf[(Rx_C.rx_head + 43) & 0x00FF];
                        Rx_SimToFlight_Rx.TarApproachSpeed = ((float32)SIMU_i32_U8.i32) / 1e3;

                        receive_ocunter++;
//                        for(i = 0;i<(Rx_SimToFlight_Rx.FrameCnt1-last_frameCount);i++)
//                        {
//                            get_attitude_angle();
//                        }
//                        last_frameCount = Rx_SimToFlight_Rx.FrameCnt1;
//                        Cal_Para();
                        break;
                        //仿真机数据2
                    case 0x63:
                        //目标弹编号
                        Rx_SimToFlight_Rx.Target_Missile = Rx_C.rx_buf[(Rx_C.rx_head + 3) & 0x00FF];
                        //帧计数
                        SIMU_U32_U8.BYTE.Byte_0 = Rx_C.rx_buf[(Rx_C.rx_head + 4) & 0x00FF];
                        SIMU_U32_U8.BYTE.Byte_1 = Rx_C.rx_buf[(Rx_C.rx_head + 5) & 0x00FF];
                        SIMU_U32_U8.BYTE.Byte_2 = Rx_C.rx_buf[(Rx_C.rx_head + 6) & 0x00FF];
                        SIMU_U32_U8.BYTE.Byte_3 = Rx_C.rx_buf[(Rx_C.rx_head + 7) & 0x00FF];
                        Rx_SimToFlight_Rx.FrameCnt2 = SIMU_U32_U8.U32;
                        //控制状态字
                        Rx_SimToFlight_Rx.StateControlWord1 = Rx_C.rx_buf[(Rx_C.rx_head + 8) & 0x00FF];
                        //目标编号
                        Rx_SimToFlight_Rx.TarNunber = Rx_C.rx_buf[(Rx_C.rx_head + 9) & 0x00FF];
                        //目标状态（弹体坐标系）
                        //目标高低角
                        SIMU_i16_U8.Byte.Byte_0 = Rx_C.rx_buf[(Rx_C.rx_head + 10) & 0x00FF];
                        SIMU_i16_U8.Byte.Byte_1 = Rx_C.rx_buf[(Rx_C.rx_head + 11) & 0x00FF];
                        Rx_SimToFlight_Rx.TarHL_Angle = (float32)(SIMU_i16_U8.i16t) / 100;
                        //目标方位角
                        SIMU_i16_U8.Byte.Byte_0 = Rx_C.rx_buf[(Rx_C.rx_head + 12) & 0x00FF];
                        SIMU_i16_U8.Byte.Byte_1 = Rx_C.rx_buf[(Rx_C.rx_head + 13) & 0x00FF];
                        Rx_SimToFlight_Rx.TarAz_Angle = (float32)(SIMU_i16_U8.i16t) / 100;
                        //目标距离
                        SIMU_i16_U8.Byte.Byte_0 = Rx_C.rx_buf[(Rx_C.rx_head + 14) & 0x00FF];
                        SIMU_i16_U8.Byte.Byte_1 = Rx_C.rx_buf[(Rx_C.rx_head + 15) & 0x00FF];
                        Rx_SimToFlight_Rx.TarDistance = SIMU_i16_U8.i16t;
                        //目标接近速度
                        SIMU_i16_U8.Byte.Byte_0 = Rx_C.rx_buf[(Rx_C.rx_head + 16) & 0x00FF];
                        SIMU_i16_U8.Byte.Byte_1 = Rx_C.rx_buf[(Rx_C.rx_head + 17) & 0x00FF];
                        Rx_SimToFlight_Rx.TarApproachSpeed = (float32)(SIMU_i16_U8.i16t) / 10;
                        //目标视线角速度Wy
                        SIMU_i32_U8.BYTE.Byte_0 = Rx_C.rx_buf[(Rx_C.rx_head + 18) & 0x00FF];
                        SIMU_i32_U8.BYTE.Byte_1 = Rx_C.rx_buf[(Rx_C.rx_head + 19) & 0x00FF];
                        SIMU_i32_U8.BYTE.Byte_2 = Rx_C.rx_buf[(Rx_C.rx_head + 20) & 0x00FF];
                        SIMU_i32_U8.BYTE.Byte_3 = Rx_C.rx_buf[(Rx_C.rx_head + 21) & 0x00FF];
                        Rx_SimToFlight_Rx.Tar_SightAngleSpeed_Wy = (float32)(SIMU_i32_U8.i32) / 1000000 / 180 * pi;
                        //目标视线角速度Wz
                        SIMU_i32_U8.BYTE.Byte_0 = Rx_C.rx_buf[(Rx_C.rx_head + 22) & 0x00FF];
                        SIMU_i32_U8.BYTE.Byte_1 = Rx_C.rx_buf[(Rx_C.rx_head + 23) & 0x00FF];
                        SIMU_i32_U8.BYTE.Byte_2 = Rx_C.rx_buf[(Rx_C.rx_head + 24) & 0x00FF];
                        SIMU_i32_U8.BYTE.Byte_3 = Rx_C.rx_buf[(Rx_C.rx_head + 25) & 0x00FF];
                        Rx_SimToFlight_Rx.Tar_SightAngleSpeed_Wz = (float32)(SIMU_i32_U8.i32) / 1000000 / 180 * pi;

                        //预测点状态（弹体坐标系）
                        //预测点高低角
                        SIMU_i16_U8.Byte.Byte_0 = Rx_C.rx_buf[(Rx_C.rx_head + 26) & 0x00FF];
                        SIMU_i16_U8.Byte.Byte_1 = Rx_C.rx_buf[(Rx_C.rx_head + 27) & 0x00FF];
                        Rx_SimToFlight_Rx.ForPointHL_Angle = (float32)(SIMU_i16_U8.i16t) / 100;
                        //预测点方位角
                        SIMU_i16_U8.Byte.Byte_0 = Rx_C.rx_buf[(Rx_C.rx_head + 28) & 0x00FF];
                        SIMU_i16_U8.Byte.Byte_1 = Rx_C.rx_buf[(Rx_C.rx_head + 29) & 0x00FF];
                        Rx_SimToFlight_Rx.ForPointAz_Angle = (float32)(SIMU_i16_U8.i16t) / 100;
                        //预测点距离
                        SIMU_i16_U8.Byte.Byte_0 = Rx_C.rx_buf[(Rx_C.rx_head + 30) & 0x00FF];
                        SIMU_i16_U8.Byte.Byte_1 = Rx_C.rx_buf[(Rx_C.rx_head + 31) & 0x00FF];
                        Rx_SimToFlight_Rx.ForPointDistance = SIMU_i16_U8.i16t;
                        //预测点接近速度
                        SIMU_i16_U8.Byte.Byte_0 = Rx_C.rx_buf[(Rx_C.rx_head + 32) & 0x00FF];
                        SIMU_i16_U8.Byte.Byte_1 = Rx_C.rx_buf[(Rx_C.rx_head + 33) & 0x00FF];
                        Rx_SimToFlight_Rx.ForPointApproachSpeed = (float32)(SIMU_i16_U8.i16t) / 10;
                        //预测点视线角速度Wz
                        SIMU_i32_U8.BYTE.Byte_0 = Rx_C.rx_buf[(Rx_C.rx_head + 34) & 0x00FF];
                        SIMU_i32_U8.BYTE.Byte_1 = Rx_C.rx_buf[(Rx_C.rx_head + 35) & 0x00FF];
                        SIMU_i32_U8.BYTE.Byte_2 = Rx_C.rx_buf[(Rx_C.rx_head + 36) & 0x00FF];
                        SIMU_i32_U8.BYTE.Byte_3 = Rx_C.rx_buf[(Rx_C.rx_head + 37) & 0x00FF];
                        Rx_SimToFlight_Rx.ForPoint_SightAngleSpeed_Wy = (float32)(SIMU_i32_U8.i32) / 100;
                        //预测点视线角速度Xz
                        SIMU_i32_U8.BYTE.Byte_0 = Rx_C.rx_buf[(Rx_C.rx_head + 38) & 0x00FF];
                        SIMU_i32_U8.BYTE.Byte_1 = Rx_C.rx_buf[(Rx_C.rx_head + 39) & 0x00FF];
                        SIMU_i32_U8.BYTE.Byte_2 = Rx_C.rx_buf[(Rx_C.rx_head + 40) & 0x00FF];
                        SIMU_i32_U8.BYTE.Byte_3 = Rx_C.rx_buf[(Rx_C.rx_head + 41) & 0x00FF];
                        Rx_SimToFlight_Rx.ForPoint_SightAngleSpeed_Wz = (float32)(SIMU_i32_U8.i32) / 100;
                        //剩余拦截时间
                        SIMU_i16_U8.Byte.Byte_0 = Rx_C.rx_buf[(Rx_C.rx_head + 42) & 0x00FF];
                        SIMU_i16_U8.Byte.Byte_1 = Rx_C.rx_buf[(Rx_C.rx_head + 43) & 0x00FF];
                        Rx_SimToFlight_Rx.RemainingInterceptTime = (float32)(SIMU_i16_U8.i16t) / 100;

                        receive_ocunter++;


//                        for(i = 0;i<(Rx_SimToFlight_Rx.FrameCnt2-last_frameCount);i++)
//                        {
//                            get_attitude_angle();
//                        }
//                        last_frameCount = Rx_SimToFlight_Rx.FrameCnt2;
//                        Cal_Para();
                        break;
                    default:
                        break;
                    }
                    Rx_C.rx_head = Rx_C.rx_head + 48;
                    Rx_C.rx_head = Rx_C.rx_head & RXBUF_MASK;
                    rdataC_length = rdataC_length - 48;
                }
                else
                {
                    Rx_C.rx_head = Rx_C.rx_head + 1;
                    Rx_C.rx_head = Rx_C.rx_head & RXBUF_MASK;
                    rdataC_length = rdataC_length - 1;
                }
            }
            else
            {
                Rx_C.rx_head = Rx_C.rx_head + 1;
                Rx_C.rx_head = Rx_C.rx_head & RXBUF_MASK;
                rdataC_length = rdataC_length - 1;
            }
        }
    }
}


unsigned char SCIC_SEND_SIM[80];
float testdata1;
float testdata2;
void SCIC_TO_SIMULINK()
{
    //帧计数
    static uint32_t Frame_Counter = 0;
    //CRC
    static unsigned char buf16_crc[76];
    Uint16 send_crc = 0;
    U16_Byte C16T8_A_send;
    unsigned int CRC_count = 0;
    //结构体实例化
    U32_Byte U32_DATA;
    i16_Byte i16_DATA;
    U16_Byte U16_DATA;

    //帧头，帧识别码
    SCIC_SEND_SIM[0] = 0XEB;
    SCIC_SEND_SIM[1] = 0X90;
    SCIC_SEND_SIM[2] = 0X96;//96
    //飞行器编号
    SCIC_SEND_SIM[3] = 0X00;
    //帧计数
    U32_DATA.U32 = send_counter;
    SCIC_SEND_SIM[4] = U32_DATA.BYTE.Byte_0;
    SCIC_SEND_SIM[5] = U32_DATA.BYTE.Byte_1;
    SCIC_SEND_SIM[6] = U32_DATA.BYTE.Byte_2;
    SCIC_SEND_SIM[7] = U32_DATA.BYTE.Byte_3;
    //状态字：引信安保陀螺弹计电源信标
    SCIC_SEND_SIM[8] = 0XFF;
    //状态字：导引头
    SCIC_SEND_SIM[9] = 0x00;

    // 导引头伺服角度-俯仰
    i16_DATA.i16t = SeekerToFlight_Rx.Seeker_Pitch_feedback * 100;//分辨率0.01
    SCIC_SEND_SIM[10] = i16_DATA.Byte.Byte_0;
    SCIC_SEND_SIM[11] = i16_DATA.Byte.Byte_1;
    //导引头伺服角度-滚转
    U16_DATA.U16 = SeekerToFlight_Rx.Seeker_Pitch_feedback * 100;//分辨率0.01
    SCIC_SEND_SIM[12] = U16_DATA.Byte.Byte_0;
    SCIC_SEND_SIM[13] = U16_DATA.Byte.Byte_1;
    //导引头目标-X坐标
    i16_DATA.i16t = SeekerToFlight_Rx.Target_x;
    SCIC_SEND_SIM[14] = i16_DATA.Byte.Byte_0;
    SCIC_SEND_SIM[15] = i16_DATA.Byte.Byte_1;
    //导引头目标-Y坐标
    i16_DATA.i16t = SeekerToFlight_Rx.Target_y;
    SCIC_SEND_SIM[16] = i16_DATA.Byte.Byte_0;
    SCIC_SEND_SIM[17] = i16_DATA.Byte.Byte_1;
    //导引头目标-能量强度
    i16_DATA.i16t = SeekerToFlight_Rx.Energy;
    SCIC_SEND_SIM[18] = i16_DATA.Byte.Byte_0;
    SCIC_SEND_SIM[19] = i16_DATA.Byte.Byte_1;

    //导引头视线角速度未加入协议
    //导引头-视线角速度-方位
    i16_DATA.i16t = 12345;//分辨率0.01
    SCIC_SEND_SIM[20] = i16_DATA.Byte.Byte_0;
    SCIC_SEND_SIM[21] = i16_DATA.Byte.Byte_1;
    //导引头-视线角速度-高低
    i16_DATA.i16t = 22345;//分辨率0.01
    SCIC_SEND_SIM[22] = i16_DATA.Byte.Byte_0;
    SCIC_SEND_SIM[23] = i16_DATA.Byte.Byte_1;

    //导弹AVP通过加速度以及角速度解算出
    //拦截弹经度
    U32_DATA.U32 = (longitude + (longitude_Proportion-1) * longitude0) * rad2deg * 1e7;;//avp[6]*1000;//分辨率1e-7
    SCIC_SEND_SIM[24] = U32_DATA.BYTE.Byte_0;
    SCIC_SEND_SIM[25] = U32_DATA.BYTE.Byte_1;
    SCIC_SEND_SIM[26] = U32_DATA.BYTE.Byte_2;
    SCIC_SEND_SIM[27] = U32_DATA.BYTE.Byte_3;
    //拦截弹纬度
    U32_DATA.U32 = (latitude+(longitude_Proportion-1)*latitude0) * rad2deg * 1e7;;//avp[7]*1000;// //分辨率1e-7
    SCIC_SEND_SIM[28] = U32_DATA.BYTE.Byte_0;
    SCIC_SEND_SIM[29] = U32_DATA.BYTE.Byte_1;
    SCIC_SEND_SIM[30] = U32_DATA.BYTE.Byte_2;
    SCIC_SEND_SIM[31] = U32_DATA.BYTE.Byte_3;
    //拦截弹高度
    i16_DATA.i16t = altitude * 5;//分辨率0.2
    SCIC_SEND_SIM[32] = i16_DATA.Byte.Byte_0;
    SCIC_SEND_SIM[33] = i16_DATA.Byte.Byte_1;
    //导弹速度VN
//    i16_DATA.i16t = avp[3] * 20;//分辨率0.05
//    SCIC_SEND_SIM[34] = i16_DATA.Byte.Byte_0;
//    SCIC_SEND_SIM[35] = i16_DATA.Byte.Byte_1;
//    //导弹速度VE
//    i16_DATA.i16t = avp[4] * 20;//分辨率0.05
//    SCIC_SEND_SIM[36] = i16_DATA.Byte.Byte_0;
//    SCIC_SEND_SIM[37] = i16_DATA.Byte.Byte_1;
//    //导弹速度VD
//    i16_DATA.i16t = avp[5] * 20;//分辨率0.05
//    SCIC_SEND_SIM[38] = i16_DATA.Byte.Byte_0;
//    SCIC_SEND_SIM[39] = i16_DATA.Byte.Byte_1;

//    //弹指令过载aY
//    i16_DATA.i16t = (signed int)(InsOverload_aY * 50);//分辨率0.02
//    SCIC_SEND_SIM[40] = i16_DATA.Byte.Byte_0;
//    SCIC_SEND_SIM[41] = i16_DATA.Byte.Byte_1;
//    //弹指令过载aZ
//    i16_DATA.i16t = (signed int)(InsOverload_aZ * 50);//分辨率0.02
//    SCIC_SEND_SIM[42] = i16_DATA.Byte.Byte_0;
//    SCIC_SEND_SIM[43] = i16_DATA.Byte.Byte_1;


    //弹加速度aX
    i16_DATA.i16t = Rx_SimToFlight_Rx.Rx_Acc_X_Sim * 50;//分辨率0.02
    SCIC_SEND_SIM[44] = i16_DATA.Byte.Byte_0;
    SCIC_SEND_SIM[45] = i16_DATA.Byte.Byte_1;
    //弹加速度aY
    i16_DATA.i16t = Rx_SimToFlight_Rx.Rx_Acc_Y_Sim * 50;//分辨率0.02
    SCIC_SEND_SIM[46] = i16_DATA.Byte.Byte_0;
    SCIC_SEND_SIM[47] = i16_DATA.Byte.Byte_1;
    //弹加速度aZ
    i16_DATA.i16t = Rx_SimToFlight_Rx.Rx_Acc_Z_Sim * 50;//分辨率0.02
    SCIC_SEND_SIM[48] = i16_DATA.Byte.Byte_0;
    SCIC_SEND_SIM[49] = i16_DATA.Byte.Byte_1;
    //偏航角
    i16_DATA.i16t = yaw_f*100;//分辨率0.01
    SCIC_SEND_SIM[50] = i16_DATA.Byte.Byte_0;
    SCIC_SEND_SIM[51] = i16_DATA.Byte.Byte_1;
    //俯仰角
    i16_DATA.i16t = pitch_f*100;//分辨率0.01
    SCIC_SEND_SIM[52] = i16_DATA.Byte.Byte_0;
    SCIC_SEND_SIM[53] = i16_DATA.Byte.Byte_1;
    //滚转角
    i16_DATA.i16t = roll_f*100;//分辨率0.01
    SCIC_SEND_SIM[54] = i16_DATA.Byte.Byte_0;
    SCIC_SEND_SIM[55] = i16_DATA.Byte.Byte_1;
#define SIMULINK
#ifdef  SIMULINK
    //俯仰角速度
    i16_DATA.i16t =  Rx_SimToFlight_Rx.Rx_Pitch_VelAng_Sim * 100;//分辨率0.01
    SCIC_SEND_SIM[56] = i16_DATA.Byte.Byte_0;
    SCIC_SEND_SIM[57] = i16_DATA.Byte.Byte_1;
    //偏航角速度
    i16_DATA.i16t =  Rx_SimToFlight_Rx.Rx_Yaw_VelAng_Sim * 100;//分辨率0.01
    SCIC_SEND_SIM[58] = i16_DATA.Byte.Byte_0;
    SCIC_SEND_SIM[59] = i16_DATA.Byte.Byte_1;
    //滚转角速度
    i16_DATA.i16t =  Rx_SimToFlight_Rx.Rx_Roll_VelAng_Sim * 100;//分辨率0.01
    SCIC_SEND_SIM[60] = i16_DATA.Byte.Byte_0;
    SCIC_SEND_SIM[61] = i16_DATA.Byte.Byte_1;
#else
    //俯仰角速度
    i16_DATA.i16t =  IMUDATA.Y_gyro * 100;//分辨率0.01
    SCIC_SEND_SIM[52] = i16_DATA.Byte.Byte_0;
    SCIC_SEND_SIM[53] = i16_DATA.Byte.Byte_1;
    //偏航角速度
    i16_DATA.i16t =  IMUDATA.Z_gyro * 100;//分辨率0.01
    SCIC_SEND_SIM[54] = i16_DATA.Byte.Byte_0;
    SCIC_SEND_SIM[55] = i16_DATA.Byte.Byte_1;
    //滚转角速度
    i16_DATA.i16t =  IMUDATA.X_gyro * 100;//分辨率0.01
    SCIC_SEND_SIM[56] = i16_DATA.Byte.Byte_0;
    SCIC_SEND_SIM[57] = i16_DATA.Byte.Byte_1;
#endif
    //舵机舵量1
//    i16_DATA.i16t = (signed int)(Result_duoliang1_1 * 100);//分辨率0.01
//    SCIC_SEND_SIM[62] = i16_DATA.Byte.Byte_0;
//    SCIC_SEND_SIM[63] = i16_DATA.Byte.Byte_1;
//    //舵机舵量2
//    i16_DATA.i16t = (signed int)(Result_duoliang2_1 * 100);//分辨率0.01
//    SCIC_SEND_SIM[64] = i16_DATA.Byte.Byte_0;
//    SCIC_SEND_SIM[65] = i16_DATA.Byte.Byte_1;
//    //舵机舵量3
//    i16_DATA.i16t = (signed int)(Result_duoliang3_1 * 100);//分辨率0.01
//    SCIC_SEND_SIM[66] = i16_DATA.Byte.Byte_0;
//    SCIC_SEND_SIM[67] = i16_DATA.Byte.Byte_1;

    i16_DATA.i16t = (signed int)(PID_Result_GyroX_1 * 1000);//分辨率0.01
    SCIC_SEND_SIM[62] = i16_DATA.Byte.Byte_0;
    SCIC_SEND_SIM[63] = i16_DATA.Byte.Byte_1;

    i16_DATA.i16t = (signed int)(PID_Result_GyroY_1 * 1000);//分辨率0.01
    SCIC_SEND_SIM[64] = i16_DATA.Byte.Byte_0;
    SCIC_SEND_SIM[65] = i16_DATA.Byte.Byte_1;

    i16_DATA.i16t = (signed int)(PID_Result_GyroZ_1 * 1000);//分辨率0.01
    SCIC_SEND_SIM[66] = i16_DATA.Byte.Byte_0;
    SCIC_SEND_SIM[67] = i16_DATA.Byte.Byte_1;

    //舵机舵量4
    i16_DATA.i16t = (signed int)(Result_duoliang4_1 * 100);//分辨率0.01
    SCIC_SEND_SIM[68] = i16_DATA.Byte.Byte_0;
    SCIC_SEND_SIM[69] = i16_DATA.Byte.Byte_1;
    //保留，70 77位用于测试
    SCIC_SEND_SIM[70] = Flag_Start_Fzj;
    SCIC_SEND_SIM[71] = 0X00;
    SCIC_SEND_SIM[72] = 0X00;
    SCIC_SEND_SIM[73] = 0X00;
    SCIC_SEND_SIM[74] = 0X00;
    SCIC_SEND_SIM[75] = 0X00;
    SCIC_SEND_SIM[76] = 0X00;
    SCIC_SEND_SIM[77] = 0X00;
    //CRC校验
    for(CRC_count = 0; CRC_count < 76; CRC_count++)
    {
        buf16_crc[CRC_count] = SCIC_SEND_SIM[CRC_count +2];
    }
    send_crc = crc16_ibm(buf16_crc,76);
    C16T8_A_send.U16 = send_crc;
    SCIC_SEND_SIM[78] = C16T8_A_send.Byte.Byte_0;
    SCIC_SEND_SIM[79] = C16T8_A_send.Byte.Byte_1;
    Frame_Counter++;

    send_counter++;

    SCIC_Tx_State.State = Start_Tx;
    SCIC_Tx_State.Field_Number = One;
    ScicRegs.SCIFFTX.bit.TXFFINTCLR = 1;
}




//i16_Byte search_region;
//i16_Byte servo_contral;
//int32 gz_acm = 60,fy_acm = -60;
//unsigned char SCIC_SEND_MOTRO[48];
////TEST USE
//uint16_t search_x = 0;
//void SCIC_TO_MOTRO(void)
//{
//    static unsigned char buf16_crc[44];
//    unsigned int CRC_count = 0;
//    Uint16 send_crc = 0;
//    U16_Byte C16T8_A_send;
//
//    SCIC_SEND_MOTRO[0] = 0xEB;//帧头
//    SCIC_SEND_MOTRO[1] = 0x90;//帧头
//    SCIC_SEND_MOTRO[2] = 0xCA;//帧识别码
//    SCIC_SEND_MOTRO[3] = 0x33;//帧识别码，备份
//    //舵机角度1指令
//    //十六字节整形
//    servo_contral.i16t = 15 * 1000;
//    SCIC_SEND_MOTRO[4] = servo_contral.Byte.Byte_1;
//    SCIC_SEND_MOTRO[5] = servo_contral.Byte.Byte_0;
//    //舵机角度2指令
//    servo_contral.i16t = -15 * 1000;
//    SCIC_SEND_MOTRO[6] = servo_contral.Byte.Byte_1;
//    SCIC_SEND_MOTRO[7] = servo_contral.Byte.Byte_0;
//    //舵机角度3指令
//    servo_contral.i16t = 30 * 1000;
//    SCIC_SEND_MOTRO[8] = servo_contral.Byte.Byte_1;
//    SCIC_SEND_MOTRO[9] = servo_contral.Byte.Byte_0;
//    //舵机角度4指令
//    servo_contral.i16t = (-60) * 1000;
//    SCIC_SEND_MOTRO[10] = servo_contral.Byte.Byte_1;;
//    SCIC_SEND_MOTRO[11] = servo_contral.Byte.Byte_0;
//    //命令字
//    SCIC_SEND_MOTRO[12] = 0x00;
//    //导引头滚转指令
//    SCIC_SEND_MOTRO[13] = (gz_acm & 0x00ff0000) >> 16;
//    if(gz_acm < 0)
//    {
//        SCIC_SEND_MOTRO[13] = SCIC_SEND_MOTRO[13] | 0x80;
//    }
//    SCIC_SEND_MOTRO[14] = (gz_acm & 0x0000ff00) >> 8;
//    SCIC_SEND_MOTRO[15] = gz_acm & 0x000000ff;
//    //导引头俯仰指令
//    SCIC_SEND_MOTRO[16] = (fy_acm & 0x00ff0000) >> 16;
//    if(fy_acm < 0 )
//    {
//        SCIC_SEND_MOTRO[16] = SCIC_SEND_MOTRO[16] | 0x80;
//    }
//    SCIC_SEND_MOTRO[17] = (fy_acm & 0x0000ff00) >> 8;
//    SCIC_SEND_MOTRO[18] = fy_acm & 0x000000ff;
//
//    //自动搜索X
//    search_region.i16t = search_x;
//    SCIC_SEND_MOTRO[19] = search_region.Byte.Byte_1;
//    SCIC_SEND_MOTRO[20] = search_region.Byte.Byte_0;
//    search_x = (search_x + 1) & 0x00FF;
//    //自动搜索Y
//    search_region.i16t = 125;
//    SCIC_SEND_MOTRO[21] = search_region.Byte.Byte_1;
//    SCIC_SEND_MOTRO[22] = search_region.Byte.Byte_0;
//    //自动搜索宽度
//    search_region.i16t = 30;
//    SCIC_SEND_MOTRO[23] = search_region.Byte.Byte_1;
//    SCIC_SEND_MOTRO[24] = search_region.Byte.Byte_0;
//    //自动搜索高度
//    search_region.i16t = -60;
//    SCIC_SEND_MOTRO[25] = search_region.Byte.Byte_1;
//    SCIC_SEND_MOTRO[26] = search_region.Byte.Byte_0;
//    //CRC校验
//    for(CRC_count = 0; CRC_count < 44; CRC_count++)
//    {
//        buf16_crc[CRC_count] = SCIC_SEND_MOTRO[CRC_count +2];
//    }
//    send_crc = crc16_ibm(buf16_crc,44);
//    C16T8_A_send.U16 = send_crc;
//    SCIC_SEND_MOTRO[46] = C16T8_A_send.Byte.Byte_0;
//    SCIC_SEND_MOTRO[47] = C16T8_A_send.Byte.Byte_1;
//
//    /* 清错误标志，串口发送前进行软复位，防止复位影响发送缓存数据    */
//    if( ScicRegs.SCIRXST.bit.RXERROR )
//    {
//        ScicRegs.SCICTL1.bit.SWRESET = 0;
//        ScicRegs.SCICTL1.bit.SWRESET = 1;
//    }
//    int i;
//    i = 0;
//    for(i = 0 ; i < 48 ; i++)
//    {
//        scic_xmit(SCIC_SEND_MOTRO[i]);
//    }
//
//}








//发送任意数据程序
void scic_xmit(int a)
{
    while (ScicRegs.SCIFFTX.bit.TXFFST != 0) {}
    ScicRegs.SCITXBUF.all =a;
}
//发送任意数据程序
void scic_msg(char * msg)
{
    int i;
    i = 0;
    while(msg[i] != '\0')
    {
        scic_xmit(msg[i]);
        i++;
    }
}
#endif

#ifdef DATALINK

void SCIC_RX(void)
{
    //用于CRC循环存数组
    unsigned int rdataC_n = 0;
    //判断接收新数据的数组长度
    static int rdataC_length = 0;
    //暂存CRC结果，用于比对是否正确
    static Uint16 CRC16C_R = 0;
    //数组长度等于接受数据的长度减4，用于CRC校验
    static unsigned char buf16C[44];
    //帧识别码
    unsigned char FrameID_BUS;

    //拆分CRC的结构体
    U16_Byte C16T8_C_receive;
    U32_Byte DataLink_U32_U8;
    i32_Byte DataLink_i32_U8;
    i16_Byte DataLink_i16_U8;
    U16_Byte DataLink_U16_U8;
    if( Rx_C.rx_tail >= Rx_C.rx_head )    rdataC_length = Rx_C.rx_tail - Rx_C.rx_head;
    else                                  rdataC_length = Rx_C.rx_tail + 256 - Rx_C.rx_head;
    // 如果接收缓存数据数有48个或更多，则可能有一帧完整数据
    if( rdataC_length >= 48 )
    {
        // 如果接收缓存数据有超过96 - 1字节,正常情况下应有一帧完整数据，直接从倒数第96字节开始搜寻最新帧
        if( rdataC_length > 95 )
        {
            Rx_C.rx_head = Rx_C.rx_head + rdataC_length - 95;
            Rx_C.rx_head = Rx_C.rx_head & RXBUF_MASK;
            rdataC_length = 95;
        }

        while( rdataC_length >= 48 )
        {
            // 如果当前数据和同步帧数据吻合，则判断帧信息是否完整
            if( Rx_C.rx_buf[Rx_C.rx_head] == 0xEB && Rx_C.rx_buf[( Rx_C.rx_head + 1 ) & RXBUF_MASK] == 0x90 )
            {
                // CRC校验
                for(rdataC_n = 0; rdataC_n < 44; rdataC_n++)//校验第2到第45字节的数据
                {
                    buf16C[rdataC_n] = Rx_C.rx_buf[(Rx_C.rx_head + rdataC_n + 2) & RXBUF_MASK];
                }
                CRC16C_R =  crc16_ibm( buf16C, 44 );
                C16T8_C_receive.U16 = CRC16C_R;
                // 接收处理
                if(C16T8_C_receive.Byte.Byte_1 == Rx_C.rx_buf[(Rx_C.rx_head + 47) & RXBUF_MASK] && C16T8_C_receive.Byte.Byte_0 == Rx_C.rx_buf[(Rx_C.rx_head + 46) & RXBUF_MASK])//C16T8_C_receive.Byte.Byte_1 == Rx_C.rx_buf[(Rx_C.rx_head + 47) & RXBUF_MASK] && C16T8_C_receive.Byte.Byte_0 == Rx_C.rx_buf[(Rx_C.rx_head + 46) & RXBUF_MASK]
                {
                    //帧功能识别码
                    RX_Datalink.FrameID = Rx_C.rx_buf[(Rx_C.rx_head + 2) & RXBUF_MASK];
                    switch(RX_Datalink.FrameID)
                    {
                    case 0x66:
                        //飞行器编号
                        RX_Datalink.FlightNum = Rx_C.rx_buf[(Rx_C.rx_head + 3) & RXBUF_MASK];

                        //帧计数
                        DataLink_U32_U8.BYTE.Byte_0 = Rx_C.rx_buf[(Rx_C.rx_head + 4) & RXBUF_MASK];
                        DataLink_U32_U8.BYTE.Byte_1 = Rx_C.rx_buf[(Rx_C.rx_head + 5) & RXBUF_MASK];
                        DataLink_U32_U8.BYTE.Byte_2 = Rx_C.rx_buf[(Rx_C.rx_head + 6) & RXBUF_MASK];
                        DataLink_U32_U8.BYTE.Byte_3 = Rx_C.rx_buf[(Rx_C.rx_head + 7) & RXBUF_MASK];
                        RX_Datalink.FrameCnt = DataLink_U32_U8.U32;

                        //导引头、安保解锁、引信解锁、数据模式
                        RX_Datalink.StateWord = Rx_C.rx_buf[(Rx_C.rx_head + 8) & RXBUF_MASK];

                        //目标编号
                        RX_Datalink.TargetNum = Rx_C.rx_buf[(Rx_C.rx_head + 9) & RXBUF_MASK];

                        //目标状态，弹坐标系
                        //高低角 -90 ~ 90   0.01deg
                        DataLink_i16_U8.Byte.Byte_0 = Rx_C.rx_buf[(Rx_C.rx_head + 10) & RXBUF_MASK];
                        DataLink_i16_U8.Byte.Byte_1 = Rx_C.rx_buf[(Rx_C.rx_head + 11) & RXBUF_MASK];
                        RX_Datalink.TarHL_Angle = (float32)(DataLink_i16_U8.i16t) / 100;

                        //目标方位角
                        DataLink_i16_U8.Byte.Byte_0 = Rx_C.rx_buf[(Rx_C.rx_head + 12) & RXBUF_MASK];
                        DataLink_i16_U8.Byte.Byte_1 = Rx_C.rx_buf[(Rx_C.rx_head + 13) & RXBUF_MASK];
                        RX_Datalink.TarAz_Angle = (float32)(DataLink_i16_U8.i16t) / 100;

                        //目标距离
                        DataLink_i16_U8.Byte.Byte_0 = Rx_C.rx_buf[(Rx_C.rx_head + 14) & RXBUF_MASK];
                        DataLink_i16_U8.Byte.Byte_1 = Rx_C.rx_buf[(Rx_C.rx_head + 15) & RXBUF_MASK];
                        RX_Datalink.TarDistance = DataLink_i16_U8.i16t;

                        //接近速度（导弹与目标）
                        DataLink_i16_U8.Byte.Byte_0 = Rx_C.rx_buf[(Rx_C.rx_head + 16) & RXBUF_MASK];
                        DataLink_i16_U8.Byte.Byte_1 = Rx_C.rx_buf[(Rx_C.rx_head + 17) & RXBUF_MASK];
                        RX_Datalink.TarApproachSpeed = (float32)(DataLink_i16_U8.i16t) / 10;

                        //目标-视线角速度Wy
                        DataLink_i16_U8.Byte.Byte_0 = Rx_C.rx_buf[(Rx_C.rx_head + 18) & RXBUF_MASK];
                        DataLink_i16_U8.Byte.Byte_1 = Rx_C.rx_buf[(Rx_C.rx_head + 19) & RXBUF_MASK];
                        RX_Datalink.Tar_SightAngleSpeed_Wy = (float32)(DataLink_i16_U8.i16t) / 100;

                        //目标-视线角速度Wz
                        DataLink_i16_U8.Byte.Byte_0 = Rx_C.rx_buf[(Rx_C.rx_head + 20) & RXBUF_MASK];
                        DataLink_i16_U8.Byte.Byte_1 = Rx_C.rx_buf[(Rx_C.rx_head + 21) & RXBUF_MASK];
                        RX_Datalink.Tar_SightAngleSpeed_Wz = (float32)(DataLink_i16_U8.i16t) / 100;

                        //预测点，弹坐标系
                        //预测点高低角
                        DataLink_i16_U8.Byte.Byte_0 = Rx_C.rx_buf[(Rx_C.rx_head + 22) & RXBUF_MASK];
                        DataLink_i16_U8.Byte.Byte_1 = Rx_C.rx_buf[(Rx_C.rx_head + 23) & RXBUF_MASK];
                        RX_Datalink.ForPointHL_Angle = (float32)(DataLink_i16_U8.i16t) / 100;

                        //预测点方位角
                        DataLink_i16_U8.Byte.Byte_0 = Rx_C.rx_buf[(Rx_C.rx_head + 24) & RXBUF_MASK];
                        DataLink_i16_U8.Byte.Byte_1 = Rx_C.rx_buf[(Rx_C.rx_head + 25) & RXBUF_MASK];
                        RX_Datalink.ForPointAz_Angle = (float32)(DataLink_i16_U8.i16t) / 100;

                        //预测点距离
                        DataLink_i16_U8.Byte.Byte_0 = Rx_C.rx_buf[(Rx_C.rx_head + 26) & RXBUF_MASK];
                        DataLink_i16_U8.Byte.Byte_1 = Rx_C.rx_buf[(Rx_C.rx_head + 27) & RXBUF_MASK];
                        RX_Datalink.ForPointDistance = DataLink_i16_U8.i16t;

                        //预测点接近速度（导弹与目标）
                        DataLink_i16_U8.Byte.Byte_0 = Rx_C.rx_buf[(Rx_C.rx_head + 28) & RXBUF_MASK];
                        DataLink_i16_U8.Byte.Byte_1 = Rx_C.rx_buf[(Rx_C.rx_head + 29) & RXBUF_MASK];
                        RX_Datalink.ForPointApproachSpeed = (float32)(DataLink_i16_U8.i16t) / 10;

                        //预测点-视线角速度Wy
                        DataLink_i16_U8.Byte.Byte_0 = Rx_C.rx_buf[(Rx_C.rx_head + 30) & RXBUF_MASK];
                        DataLink_i16_U8.Byte.Byte_1 = Rx_C.rx_buf[(Rx_C.rx_head + 31) & RXBUF_MASK];
                        RX_Datalink.ForPoint_SightAngleSpeed_Wy = (float32)(DataLink_i16_U8.i16t) / 100;

                        //预测点-视线角速度Wz
                        DataLink_i16_U8.Byte.Byte_0 = Rx_C.rx_buf[(Rx_C.rx_head + 32) & RXBUF_MASK];
                        DataLink_i16_U8.Byte.Byte_1 = Rx_C.rx_buf[(Rx_C.rx_head + 33) & RXBUF_MASK];
                        RX_Datalink.ForPoint_SightAngleSpeed_Wz = (float32)(DataLink_i16_U8.i16t) / 100;

                        //剩余拦截时间
                        DataLink_U16_U8.Byte.Byte_0 = Rx_C.rx_buf[(Rx_C.rx_head + 34) & RXBUF_MASK];
                        DataLink_U16_U8.Byte.Byte_1 = Rx_C.rx_buf[(Rx_C.rx_head + 35) & RXBUF_MASK];
                        RX_Datalink.RemainingInterceptTime = (float32)(DataLink_U16_U8.U16) / 100;

                        //目标经度
                        DataLink_i32_U8.BYTE.Byte_0 = Rx_C.rx_buf[(Rx_C.rx_head + 36) & RXBUF_MASK];
                        DataLink_i32_U8.BYTE.Byte_1 = Rx_C.rx_buf[(Rx_C.rx_head + 37) & RXBUF_MASK];
                        DataLink_i32_U8.BYTE.Byte_2 = Rx_C.rx_buf[(Rx_C.rx_head + 38) & RXBUF_MASK];
                        DataLink_i32_U8.BYTE.Byte_3 = Rx_C.rx_buf[(Rx_C.rx_head + 39) & RXBUF_MASK];
                        RX_Datalink.Tar_Longitude = (float32)(DataLink_i32_U8.i32) / 1e-7;

                        //目标纬度
                        DataLink_i32_U8.BYTE.Byte_0 = Rx_C.rx_buf[(Rx_C.rx_head + 40) & RXBUF_MASK];
                        DataLink_i32_U8.BYTE.Byte_1 = Rx_C.rx_buf[(Rx_C.rx_head + 41) & RXBUF_MASK];
                        DataLink_i32_U8.BYTE.Byte_2 = Rx_C.rx_buf[(Rx_C.rx_head + 42) & RXBUF_MASK];
                        DataLink_i32_U8.BYTE.Byte_3 = Rx_C.rx_buf[(Rx_C.rx_head + 43) & RXBUF_MASK];
                        RX_Datalink.Tar_Latitude = (float32)(DataLink_i32_U8.i32) / 1e-7;

                        //目标高度
                        //预测点-视线角速度Wz
                        DataLink_i16_U8.Byte.Byte_0 = Rx_C.rx_buf[(Rx_C.rx_head + 44) & RXBUF_MASK];
                        DataLink_i16_U8.Byte.Byte_1 = Rx_C.rx_buf[(Rx_C.rx_head + 45) & RXBUF_MASK];
                        RX_Datalink.Tar_Altitude = (float32)(DataLink_i16_U8.i16t) * 0.2;
                        break;
                    default:
                        break;
                    }

                    Rx_C.rx_head = Rx_C.rx_head + 48;
                    Rx_C.rx_head = Rx_C.rx_head & RXBUF_MASK;
                    rdataC_length = rdataC_length - 48;
                }
                else
                {
                    Rx_C.rx_head = Rx_C.rx_head + 1;
                    Rx_C.rx_head = Rx_C.rx_head & RXBUF_MASK;
                    rdataC_length = rdataC_length - 1;
                }
            }
            else
            {
                Rx_C.rx_head = Rx_C.rx_head + 1;
                Rx_C.rx_head = Rx_C.rx_head & RXBUF_MASK;
                rdataC_length = rdataC_length - 1;
            }
        }
    }
}




unsigned char SCIC_SEND_DATALINK[80];

#endif


Uint32 SCIC_RX_Cnt = 0;
void DataLink_Rx(void)
{
    //用于CRC循环存数组
    unsigned int rdataC_n = 0;
    //判断接收新数据的数组长度
    static int rdataC_length = 0;
    //暂存CRC结果，用于比对是否正确
    static Uint16 CRC16C_R = 0;
    //数组长度等于接受数据的长度减4，用于CRC校验
    static unsigned char buf16C[30];
    //帧识别码
    unsigned char FrameID_BUS;

    //拆分CRC的结构体
    U16_Byte C16T8_C_receive;
    U32_Byte DataLink_U32_U8;
    i32_Byte DataLink_i32_U8;
    i16_Byte DataLink_i16_U8;
    U16_Byte DataLink_U16_U8;


    if( ScicRegs.SCIRXST.bit.RXERROR )
    {
        ScicRegs.SCICTL1.bit.SWRESET = 0;
        ScicRegs.SCICTL1.bit.SWRESET = 1;
    }
    if( Rx_C.rx_tail >= Rx_C.rx_head )    rdataC_length = Rx_C.rx_tail - Rx_C.rx_head;
    else                                  rdataC_length = Rx_C.rx_tail + 256 - Rx_C.rx_head;
    // 如果接收缓存数据数有48个或更多，则可能有一帧完整数据
    if( rdataC_length >= 48 )
    {
        // 如果接收缓存数据有超过96 - 1字节,正常情况下应有一帧完整数据，直接从倒数第96字节开始搜寻最新帧
        if( rdataC_length > 95 )
        {
            Rx_C.rx_head = Rx_C.rx_head + rdataC_length - 95;
            Rx_C.rx_head = Rx_C.rx_head & RXBUF_MASK;
            rdataC_length = 95;
        }

        while( rdataC_length >= 48 )
        {
            // 如果当前数据和同步帧数据吻合，则判断帧信息是否完整
            if( Rx_C.rx_buf[Rx_C.rx_head] == 0xEB && Rx_C.rx_buf[( Rx_C.rx_head + 1 ) & RXBUF_MASK] == 0x90 )
            {
                // CRC校验
                  for(rdataC_n = 0; rdataC_n < 44; rdataC_n++)//校验第2到第45字节的数据
                  {
                      buf16C[rdataC_n] = Rx_C.rx_buf[(Rx_C.rx_head + rdataC_n + 2) & RXBUF_MASK];
                  }
                  CRC16C_R =  crc16_ibm( buf16C, 44 );
                  C16T8_C_receive.U16 = CRC16C_R;
                  // 接收处理
                  if(C16T8_C_receive.Byte.Byte_1 == Rx_C.rx_buf[(Rx_C.rx_head + 47) & RXBUF_MASK] && C16T8_C_receive.Byte.Byte_0 == Rx_C.rx_buf[(Rx_C.rx_head + 46) & RXBUF_MASK])//C16T8_C_receive.Byte.Byte_1 == Rx_C.rx_buf[(Rx_C.rx_head + 47) & RXBUF_MASK] && C16T8_C_receive.Byte.Byte_0 == Rx_C.rx_buf[(Rx_C.rx_head + 46) & RXBUF_MASK]
                  {
                      //帧功能识别码
                      RX_Datalink.FrameID = Rx_C.rx_buf[(Rx_C.rx_head + 2) & RXBUF_MASK];
                      switch(RX_Datalink.FrameID)
                      {
                      case 0x66:
                          //飞行器编号
                          RX_Datalink.FlightNum = Rx_C.rx_buf[(Rx_C.rx_head + 3) & RXBUF_MASK];

                          //帧计数
                          DataLink_U32_U8.BYTE.Byte_0 = Rx_C.rx_buf[(Rx_C.rx_head + 4) & RXBUF_MASK];
                          DataLink_U32_U8.BYTE.Byte_1 = Rx_C.rx_buf[(Rx_C.rx_head + 5) & RXBUF_MASK];
                          DataLink_U32_U8.BYTE.Byte_2 = Rx_C.rx_buf[(Rx_C.rx_head + 6) & RXBUF_MASK];
                          DataLink_U32_U8.BYTE.Byte_3 = Rx_C.rx_buf[(Rx_C.rx_head + 7) & RXBUF_MASK];
                          RX_Datalink.FrameCnt = DataLink_U32_U8.U32;

                          //导引头、安保解锁、引信解锁、数据模式
                          RX_Datalink.StateWord = Rx_C.rx_buf[(Rx_C.rx_head + 8) & RXBUF_MASK];

                          //目标编号
                          RX_Datalink.TargetNum = Rx_C.rx_buf[(Rx_C.rx_head + 9) & RXBUF_MASK];

                          //目标状态，弹坐标系
                          //高低角 -90 ~ 90   0.01deg
                          DataLink_i16_U8.Byte.Byte_0 = Rx_C.rx_buf[(Rx_C.rx_head + 10) & RXBUF_MASK];
                          DataLink_i16_U8.Byte.Byte_1 = Rx_C.rx_buf[(Rx_C.rx_head + 11) & RXBUF_MASK];
                          RX_Datalink.TarHL_Angle = (float32)(DataLink_i16_U8.i16t) / 100;

                          //目标方位角
                          DataLink_i16_U8.Byte.Byte_0 = Rx_C.rx_buf[(Rx_C.rx_head + 12) & RXBUF_MASK];
                          DataLink_i16_U8.Byte.Byte_1 = Rx_C.rx_buf[(Rx_C.rx_head + 13) & RXBUF_MASK];
                          RX_Datalink.TarAz_Angle = (float32)(DataLink_i16_U8.i16t) / 100;

                          //目标距离
                          DataLink_i16_U8.Byte.Byte_0 = Rx_C.rx_buf[(Rx_C.rx_head + 14) & RXBUF_MASK];
                          DataLink_i16_U8.Byte.Byte_1 = Rx_C.rx_buf[(Rx_C.rx_head + 15) & RXBUF_MASK];
                          RX_Datalink.TarDistance = DataLink_i16_U8.i16t;

                          //接近速度（导弹与目标）
                          DataLink_i16_U8.Byte.Byte_0 = Rx_C.rx_buf[(Rx_C.rx_head + 16) & RXBUF_MASK];
                          DataLink_i16_U8.Byte.Byte_1 = Rx_C.rx_buf[(Rx_C.rx_head + 17) & RXBUF_MASK];
                          RX_Datalink.TarApproachSpeed = (float32)(DataLink_i16_U8.i16t) / 10;

                          //目标-视线角速度Wy
                          DataLink_i16_U8.Byte.Byte_0 = Rx_C.rx_buf[(Rx_C.rx_head + 18) & RXBUF_MASK];
                          DataLink_i16_U8.Byte.Byte_1 = Rx_C.rx_buf[(Rx_C.rx_head + 19) & RXBUF_MASK];
                          RX_Datalink.Tar_SightAngleSpeed_Wy = (float32)(DataLink_i16_U8.i16t) / 100;

                          //目标-视线角速度Wz
                          DataLink_i16_U8.Byte.Byte_0 = Rx_C.rx_buf[(Rx_C.rx_head + 20) & RXBUF_MASK];
                          DataLink_i16_U8.Byte.Byte_1 = Rx_C.rx_buf[(Rx_C.rx_head + 21) & RXBUF_MASK];
                          RX_Datalink.Tar_SightAngleSpeed_Wz = (float32)(DataLink_i16_U8.i16t) / 100;

                          //预测点，弹坐标系
                          //预测点高低角
                          DataLink_i16_U8.Byte.Byte_0 = Rx_C.rx_buf[(Rx_C.rx_head + 22) & RXBUF_MASK];
                          DataLink_i16_U8.Byte.Byte_1 = Rx_C.rx_buf[(Rx_C.rx_head + 23) & RXBUF_MASK];
                          RX_Datalink.ForPointHL_Angle = (float32)(DataLink_i16_U8.i16t) / 100;

                          //预测点方位角
                          DataLink_i16_U8.Byte.Byte_0 = Rx_C.rx_buf[(Rx_C.rx_head + 24) & RXBUF_MASK];
                          DataLink_i16_U8.Byte.Byte_1 = Rx_C.rx_buf[(Rx_C.rx_head + 25) & RXBUF_MASK];
                          RX_Datalink.ForPointAz_Angle = (float32)(DataLink_i16_U8.i16t) / 100;

                          //预测点距离
                          DataLink_i16_U8.Byte.Byte_0 = Rx_C.rx_buf[(Rx_C.rx_head + 26) & RXBUF_MASK];
                          DataLink_i16_U8.Byte.Byte_1 = Rx_C.rx_buf[(Rx_C.rx_head + 27) & RXBUF_MASK];
                          RX_Datalink.ForPointDistance = DataLink_i16_U8.i16t;

                          //预测点接近速度（导弹与目标）
                          DataLink_i16_U8.Byte.Byte_0 = Rx_C.rx_buf[(Rx_C.rx_head + 28) & RXBUF_MASK];
                          DataLink_i16_U8.Byte.Byte_1 = Rx_C.rx_buf[(Rx_C.rx_head + 29) & RXBUF_MASK];
                          RX_Datalink.ForPointApproachSpeed = (float32)(DataLink_i16_U8.i16t) / 10;

                          //预测点-视线角速度Wy
                          DataLink_i16_U8.Byte.Byte_0 = Rx_C.rx_buf[(Rx_C.rx_head + 30) & RXBUF_MASK];
                          DataLink_i16_U8.Byte.Byte_1 = Rx_C.rx_buf[(Rx_C.rx_head + 31) & RXBUF_MASK];
                          RX_Datalink.ForPoint_SightAngleSpeed_Wy = (float32)(DataLink_i16_U8.i16t) / 100;

                          //预测点-视线角速度Wz
                          DataLink_i16_U8.Byte.Byte_0 = Rx_C.rx_buf[(Rx_C.rx_head + 32) & RXBUF_MASK];
                          DataLink_i16_U8.Byte.Byte_1 = Rx_C.rx_buf[(Rx_C.rx_head + 33) & RXBUF_MASK];
                          RX_Datalink.ForPoint_SightAngleSpeed_Wz = (float32)(DataLink_i16_U8.i16t) / 100;

                          //剩余拦截时间
                          DataLink_U16_U8.Byte.Byte_0 = Rx_C.rx_buf[(Rx_C.rx_head + 34) & RXBUF_MASK];
                          DataLink_U16_U8.Byte.Byte_1 = Rx_C.rx_buf[(Rx_C.rx_head + 35) & RXBUF_MASK];
                          RX_Datalink.RemainingInterceptTime = (float32)(DataLink_U16_U8.U16) / 100;

                          //目标经度
                          DataLink_i32_U8.BYTE.Byte_0 = Rx_C.rx_buf[(Rx_C.rx_head + 36) & RXBUF_MASK];
                          DataLink_i32_U8.BYTE.Byte_1 = Rx_C.rx_buf[(Rx_C.rx_head + 37) & RXBUF_MASK];
                          DataLink_i32_U8.BYTE.Byte_2 = Rx_C.rx_buf[(Rx_C.rx_head + 38) & RXBUF_MASK];
                          DataLink_i32_U8.BYTE.Byte_3 = Rx_C.rx_buf[(Rx_C.rx_head + 39) & RXBUF_MASK];
                          RX_Datalink.Tar_Longitude = (float32)(DataLink_i32_U8.i32) / 1e-7;

                          //目标纬度
                          DataLink_i32_U8.BYTE.Byte_0 = Rx_C.rx_buf[(Rx_C.rx_head + 40) & RXBUF_MASK];
                          DataLink_i32_U8.BYTE.Byte_1 = Rx_C.rx_buf[(Rx_C.rx_head + 41) & RXBUF_MASK];
                          DataLink_i32_U8.BYTE.Byte_2 = Rx_C.rx_buf[(Rx_C.rx_head + 42) & RXBUF_MASK];
                          DataLink_i32_U8.BYTE.Byte_3 = Rx_C.rx_buf[(Rx_C.rx_head + 43) & RXBUF_MASK];
                          RX_Datalink.Tar_Latitude = (float32)(DataLink_i32_U8.i32) / 1e-7;

                          //目标高度
                          //预测点-视线角速度Wz
                          DataLink_i16_U8.Byte.Byte_0 = Rx_C.rx_buf[(Rx_C.rx_head + 44) & RXBUF_MASK];
                          DataLink_i16_U8.Byte.Byte_1 = Rx_C.rx_buf[(Rx_C.rx_head + 45) & RXBUF_MASK];
                          RX_Datalink.Tar_Altitude = (float32)(DataLink_i16_U8.i16t) * 0.2;
                          SCIC_RX_Cnt++;
                          break;
                    default:
                        break;
                    }

                    Rx_C.rx_head = Rx_C.rx_head + 32;
                    Rx_C.rx_head = Rx_C.rx_head & RXBUF_MASK;
                    rdataC_length = rdataC_length - 32;
                }
                else
                {
                    Rx_C.rx_head = Rx_C.rx_head + 1;
                    Rx_C.rx_head = Rx_C.rx_head & RXBUF_MASK;
                    rdataC_length = rdataC_length - 1;
                }
            }
            else
            {
                Rx_C.rx_head = Rx_C.rx_head + 1;
                Rx_C.rx_head = Rx_C.rx_head & RXBUF_MASK;
                rdataC_length = rdataC_length - 1;
            }
        }
    }
}




unsigned char SCIC_SEND_DATALINK[80];
extern volatile struct can_rx CanDaTa_rx;
extern float real_Pitch_angle;
extern float real_Roll_angle;
extern float Yaw_VelAng_Sim;
extern float Pitch_VelAng_Sim;
extern float testdata3,testdata4;
//帧计数
uint32_t SCIC_Counter = 0;
void SCIC_TO_DATALINK(void)
{
    static unsigned char buf16_crc[76];//CRC缓存数组
     Uint16 send_crc = 0;//CEC最终结果
     U16_Byte C16T8_C_send;//CRC结构体转换
     unsigned int CRC_count = 0;//CRC缓存遍历使用
     int i;

     //结构体实例化
     U32_Byte U32_DATA;
     i16_Byte i16_DATA;
     U16_Byte U16_DATA;
     i32_Byte i32_DATA;
     Float32_Byte Float32_DATA;

     SCIC_SEND_DATALINK[0] = 0xEB;
     SCIC_SEND_DATALINK[1] = 0x90;
     SCIC_SEND_DATALINK[2] = 0x93;

     //飞行器编号
     SCIC_SEND_DATALINK[3] = 0x02;

     //帧计数
     U32_DATA.U32 = SCIC_Counter;
     SCIC_SEND_DATALINK[4] = U32_DATA.BYTE.Byte_0;
     SCIC_SEND_DATALINK[5] = U32_DATA.BYTE.Byte_1;
     SCIC_SEND_DATALINK[6] = U32_DATA.BYTE.Byte_2;
     SCIC_SEND_DATALINK[7] = U32_DATA.BYTE.Byte_3;

     //状态字：引信安保陀螺弹计电源信标
     SCIC_SEND_DATALINK[8] = 0XFF;

     //状态字：导引头
     SCIC_SEND_DATALINK[9] = 0;//SeekerToFlight_Rx.Pitch_work_mode;

     // 导引头伺服角度-俯仰
 //    real_Pitch_angle = -1.5;
     i16_DATA.i16t = (int16)(real_Pitch_angle * 100);//分辨率0.01
     SCIC_SEND_DATALINK[10] = i16_DATA.Byte.Byte_0;
     SCIC_SEND_DATALINK[11] = i16_DATA.Byte.Byte_1;
     //导引头伺服角度-滚转
 //    real_Roll_angle = 3.52;
     U16_DATA.U16 = (int16)(real_Roll_angle * 100);//分辨率0.01
     SCIC_SEND_DATALINK[12] = U16_DATA.Byte.Byte_0;
     SCIC_SEND_DATALINK[13] = U16_DATA.Byte.Byte_1;
     //导引头目标-X坐标
 //    CanDaTa_rx.Target_x = -50;
     i16_DATA.i16t = (int16)(CanDaTa_rx.Target_x);
     SCIC_SEND_DATALINK[14] = i16_DATA.Byte.Byte_0;
     SCIC_SEND_DATALINK[15] = i16_DATA.Byte.Byte_1;
     //导引头目标-Y坐标
 //    CanDaTa_rx.Target_y = 100;
     i16_DATA.i16t = (int16)(CanDaTa_rx.Target_y);
     SCIC_SEND_DATALINK[16] = i16_DATA.Byte.Byte_0;
     SCIC_SEND_DATALINK[17] = i16_DATA.Byte.Byte_1;
     //导引头目标-能量强度
 //    i16_DATA.i16t = RX_HUOKONG.Launch_cmd;
     SCIC_SEND_DATALINK[18] = i16_DATA.Byte.Byte_0;
     SCIC_SEND_DATALINK[19] = i16_DATA.Byte.Byte_1;
     //导引头-视线角速度-方位
 //    Yaw_VelAng_Sim = 23.5;
     i16_DATA.i16t = (int16)(Yaw_VelAng_Sim * 100);//12345;//分辨率0.01
     SCIC_SEND_DATALINK[20] = i16_DATA.Byte.Byte_0;
     SCIC_SEND_DATALINK[21] = i16_DATA.Byte.Byte_1;
     //导引头-视线角速度-高低
 //    Pitch_VelAng_Sim = -0.5;
     i16_DATA.i16t = (int16)(Pitch_VelAng_Sim * 100);//22345;//分辨率0.01
     SCIC_SEND_DATALINK[22] = i16_DATA.Byte.Byte_0;
     SCIC_SEND_DATALINK[23] = i16_DATA.Byte.Byte_1;
     //拦截导弹经度
//     testdata3 = (IPC_Data_Cpu2.AttitudeSolution_Longitude + (longitude_Proportion-1) * longitude0) * rad2deg * 1e7;
     testdata3 = IPC_Data_Cpu2.AttitudeSolution_Longitude * 1e7;
 //    testdata3 = 23.564;
     Float32_DATA.Float32 = testdata3;//分辨率1E-7
     SCIC_SEND_DATALINK[24] = Float32_DATA.BYTE.Byte_0;//0x12;
     SCIC_SEND_DATALINK[25] = Float32_DATA.BYTE.Byte_1;//0x34;
     SCIC_SEND_DATALINK[26] = Float32_DATA.BYTE.Byte_2;//0x56;
     SCIC_SEND_DATALINK[27] = Float32_DATA.BYTE.Byte_3;//0x78;
     //拦截导弹纬度
//     testdata4 = (IPC_Data_Cpu2.AttitudeSolution_Latitude+(longitude_Proportion-1)*latitude0) * rad2deg * 1e7;
     testdata4 = IPC_Data_Cpu2.AttitudeSolution_Latitude * 1e7;
 //    testdata4 = -16.356;
     Float32_DATA.Float32 = testdata4;//分辨率1E-7
     SCIC_SEND_DATALINK[28] = Float32_DATA.BYTE.Byte_0;
     SCIC_SEND_DATALINK[29] = Float32_DATA.BYTE.Byte_1;
     SCIC_SEND_DATALINK[30] = Float32_DATA.BYTE.Byte_2;
     SCIC_SEND_DATALINK[31] = Float32_DATA.BYTE.Byte_3;
     //拦截导弹高度
 //    altitude = 12.5;
     i16_DATA.i16t = (int16)(IPC_Data_Cpu2.AttitudeSolution_Height * 5);//分辨率0.2
     SCIC_SEND_DATALINK[32] = i16_DATA.Byte.Byte_0;
     SCIC_SEND_DATALINK[33] = i16_DATA.Byte.Byte_1;
     //导弹速度VN
 //    VN = 1.5;
     i16_DATA.i16t = (int16)(IPC_Data_Cpu2.AttitudeSolution_VN * 20);//;//分辨率0.05
     SCIC_SEND_DATALINK[34] = i16_DATA.Byte.Byte_0;
     SCIC_SEND_DATALINK[35] = i16_DATA.Byte.Byte_1;
     //导弹速度VE
 //    VE = 20.3;
     i16_DATA.i16t = (int16)(IPC_Data_Cpu2.AttitudeSolution_VE * 20);//VE * 20;//分辨率0.05
     SCIC_SEND_DATALINK[36] = i16_DATA.Byte.Byte_0;
     SCIC_SEND_DATALINK[37] = i16_DATA.Byte.Byte_1;
     //导弹速度VD
 //    VD = -3.8;
     i16_DATA.i16t = (int16)(IPC_Data_Cpu2.AttitudeSolution_VD * 20);//VD * 20;//分辨率0.05
     SCIC_SEND_DATALINK[38] = i16_DATA.Byte.Byte_0;
     SCIC_SEND_DATALINK[39] = i16_DATA.Byte.Byte_1;
     //弹指令过载aY//分辨率0.02
 //    InsOverload_aY = 20.6;
     i16_DATA.i16t = (int16)(InsOverload_aY * 50);
     SCIC_SEND_DATALINK[40] = i16_DATA.Byte.Byte_0;
     SCIC_SEND_DATALINK[41] = i16_DATA.Byte.Byte_1;
     //弹指令过载aZ//分辨率0.02
 //    InsOverload_aZ = -10.25;
     i16_DATA.i16t = (int16)(InsOverload_aZ * 50);
     SCIC_SEND_DATALINK[42] = i16_DATA.Byte.Byte_0;
     SCIC_SEND_DATALINK[43] = i16_DATA.Byte.Byte_1;
     //弹加速度aX
 //    IMUDATA.X_acc = 3.56;
     i16_DATA.i16t = (int16)(IPC_Data_Cpu2.IMU_X_acc * 50);//分辨率0.02
     SCIC_SEND_DATALINK[44] = i16_DATA.Byte.Byte_0;
     SCIC_SEND_DATALINK[45] = i16_DATA.Byte.Byte_1;
     //弹加速度aY
 //    IMUDATA.Y_acc = 2.56;
     i16_DATA.i16t = (int16)(IPC_Data_Cpu2.IMU_Y_acc * 50);//分辨率0.02
     SCIC_SEND_DATALINK[46] = i16_DATA.Byte.Byte_0;
     SCIC_SEND_DATALINK[47] = i16_DATA.Byte.Byte_1;
     //弹加速度aZ
 //    IMUDATA.Z_acc = -0.52;
     i16_DATA.i16t = (int16)(IPC_Data_Cpu2.IMU_Z_acc * 50);//分辨率0.02
     SCIC_SEND_DATALINK[48] = i16_DATA.Byte.Byte_0;
     SCIC_SEND_DATALINK[49] = i16_DATA.Byte.Byte_1;
     //偏航角
 //    yaw_f = 3.26;
     i16_DATA.i16t = IPC_Data_Cpu2.AttitudeSolution_Yaw*100;//分辨率0.01
     SCIC_SEND_DATALINK[50] = i16_DATA.Byte.Byte_0;
     SCIC_SEND_DATALINK[51] = i16_DATA.Byte.Byte_1;
     //俯仰角
 //    pitch_f = -2.65;
     i16_DATA.i16t = IPC_Data_Cpu2.AttitudeSolution_Pitch*100;//分辨率0.01
     SCIC_SEND_DATALINK[52] = i16_DATA.Byte.Byte_0;
     SCIC_SEND_DATALINK[53] = i16_DATA.Byte.Byte_1;
     //滚转角
 //    roll_f = -1.36;
     i16_DATA.i16t = IPC_Data_Cpu2.AttitudeSolution_Roll*100;//分辨率0.01
     SCIC_SEND_DATALINK[54] = i16_DATA.Byte.Byte_0;
     SCIC_SEND_DATALINK[55] = i16_DATA.Byte.Byte_1;
     //偏航角速度
 //    IMUDATA.Z_gyro = -3.44;
     i16_DATA.i16t =  (int16)(IPC_Data_Cpu2.IMU_Z_gyro*100);//分辨率0.01
     SCIC_SEND_DATALINK[56] = i16_DATA.Byte.Byte_0;
     SCIC_SEND_DATALINK[57] = i16_DATA.Byte.Byte_1;
     //俯仰角速度
 //    IMUDATA.Y_gyro = 5.63;
     i16_DATA.i16t =  (int16)(IPC_Data_Cpu2.IMU_Y_gyro*100);//分辨率0.01
     SCIC_SEND_DATALINK[58] = i16_DATA.Byte.Byte_0;
     SCIC_SEND_DATALINK[59] = i16_DATA.Byte.Byte_1;
     //滚转角速度
 //    IMUDATA.X_gyro = -2.5;
     i16_DATA.i16t =  (int16)(IPC_Data_Cpu2.IMU_X_gyro*100);//分辨率0.01
     SCIC_SEND_DATALINK[60] = i16_DATA.Byte.Byte_0;
     SCIC_SEND_DATALINK[61] = i16_DATA.Byte.Byte_1;
     //舵机舵量1
 //    Servo_Deflection1 = 2.36;
     i16_DATA.i16t = (int16)(Servo_Deflection1 * 100);//分辨率0.01
     SCIC_SEND_DATALINK[62] = i16_DATA.Byte.Byte_0;
     SCIC_SEND_DATALINK[63] = i16_DATA.Byte.Byte_1;
     //舵机舵量2
 //    Servo_Deflection2 = 6.56;
     i16_DATA.i16t = (int16)(Servo_Deflection2 * 100);//分辨率0.01
     SCIC_SEND_DATALINK[64] = i16_DATA.Byte.Byte_0;
     SCIC_SEND_DATALINK[65] = i16_DATA.Byte.Byte_1;
     //舵机舵量3
 //    Servo_Deflection3 = -1.53;
     i16_DATA.i16t = (int16)(Servo_Deflection3 * 100);//分辨率0.01
     SCIC_SEND_DATALINK[66] = i16_DATA.Byte.Byte_0;
     SCIC_SEND_DATALINK[67] = i16_DATA.Byte.Byte_1;
     //舵机舵量4
 //    Servo_Deflection4 = -6.21;
     i16_DATA.i16t = (int16)(Servo_Deflection4 * 100);//分辨率0.01
     SCIC_SEND_DATALINK[68] = i16_DATA.Byte.Byte_0;
     SCIC_SEND_DATALINK[69] = i16_DATA.Byte.Byte_1;

     //保留
     SCIC_SEND_DATALINK[70] = 0xFF;
     SCIC_SEND_DATALINK[71] = 0xFF;
     SCIC_SEND_DATALINK[72] = 0xFF;
     SCIC_SEND_DATALINK[73] = 0xFF;
     SCIC_SEND_DATALINK[74] = 0xFF;
     SCIC_SEND_DATALINK[75] = 0xFF;
     SCIC_SEND_DATALINK[76] = 0xFF;
     SCIC_SEND_DATALINK[77] = 0xFF;

     //CRC校验
     for(CRC_count = 0; CRC_count < 76; CRC_count++)
     {
         buf16_crc[CRC_count] = SCIC_SEND_DATALINK[CRC_count +2];
     }
     send_crc = crc16_ibm(buf16_crc,76);
     C16T8_C_send.U16 = send_crc;
     SCIC_SEND_DATALINK[78] = C16T8_C_send.Byte.Byte_0;
     SCIC_SEND_DATALINK[79] = C16T8_C_send.Byte.Byte_1;
     SCIC_Counter++;

    SCIC_Tx_State.State = Start_Tx;
    SCIC_Tx_State.Field_Number = One;
    ScicRegs.SCIFFTX.bit.TXFFINTCLR = 1;
}

