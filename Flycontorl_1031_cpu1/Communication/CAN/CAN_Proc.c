/*
 * CAN.c
 *
 *  Created on: 2022年9月6日
 *      Author: 110
 */

#include "CAN_Proc.h"
#include"COMMON.h"
tCANMsgObject sTXCANMessage;
tCANMsgObject sRXCANMessage1;
tCANMsgObject sRXCANMessage2;
tCANMsgObject sRXCANMessage3;
tCANMsgObject sRXCANMessage4;
volatile struct can_tx CanDaTa;
volatile struct can_rx CanDaTa_rx;


unsigned char ucTXMsgData[8];// ={ 0x55, 0x55, 0x55, 0x55, 0x55, 0x55, 0x55,
//                                0x55 };

unsigned char ucRXMsgData1[8];
unsigned char ucRXMsgData2[8];
unsigned char ucRXMsgData3[8];
unsigned char ucRXMsgData4[8];
Uint16 wheel_num = 1;
Uint16 feedback_flat;
Uint16 wait_time;
volatile unsigned long g_ulTxMsgCount = 0; // A counter that keeps track of the
                                           // number of times the TX interrupt
                                           // has occurred, which should match
                                           // the number of TX messages that
                                           // were sent.
volatile unsigned long g_ulRxMsgCount = 0;
unsigned long u32CanAErrorStatus;
volatile unsigned long g_bErrFlag = 0;     // A flag to indicate that some
                                           // transmission error occurred

extern int DYT_work_mode,Ld_State;

void InitCAN()
{
    //初始化CAN控制器
    CANInit(CANA_BASE);
    //选择CAN通信的时钟源
    CANClkSourceSelect(CANA_BASE, 0);//Selected CPU SYSCLKOUT
    //设置CAN总线的波特率
    CANBitRateSet(CANA_BASE, 200000000, 1000000);
    //启用CAN中断
    CANIntEnable(CANA_BASE, CAN_INT_MASTER | CAN_INT_ERROR | CAN_INT_STATUS);
    //设置CAN通信的模式，分别为配置，测试，自循环。与外部通信时使用测试模式
    HWREG(CANA_BASE + CAN_O_CTL) |= CAN_CTL_TEST;

    //CAN使能
    CANEnable(CANA_BASE);

    //启用CAN全局中断线路0
    CANGlobalIntEnable(CANA_BASE, CAN_GLB_INT_CANINT0);

    //初始化CAN发送
    sTXCANMessage.ui32MsgID = 0x1000;              // CAN发送节点的ID号，详见https://www.cnblogs.com/arron-zx/p/11399488.html
    sTXCANMessage.ui32MsgIDMask = 0;                  // 过滤器是IDmask，相应的位置1，就是使用id中相应的位过滤，这是硬件过滤。
    sTXCANMessage.ui32Flags = MSG_OBJ_TX_INT_ENABLE;  // 使能CAN发送中断
    sTXCANMessage.ui32MsgLen = sizeof(ucTXMsgData);   // 初始化CAN发送的字节数
    sTXCANMessage.pucMsgData = ucTXMsgData;           // 初始化CAN通信的数组指针



    //初始化CAN接收
    *(unsigned long *)ucRXMsgData1 = 0;                 //清零数组
    sRXCANMessage1.ui32MsgID = 0x7020;               // CAN接收节点的ID号，
    sRXCANMessage1.ui32MsgIDMask = 0;                   //过滤器是IDmask，相应的位置1，就是使用id中相应的位过滤，这是硬件过滤。
    sRXCANMessage1.ui32Flags = MSG_OBJ_RX_INT_ENABLE;   // 使能CAN接收中断
    sRXCANMessage1.ui32MsgLen = sizeof(ucRXMsgData1);   // 初始化CAN发送的字节数
    sRXCANMessage1.pucMsgData = ucRXMsgData1;           //初始化CAN通信的数组指针

    *(unsigned long *)ucRXMsgData2 = 0;                 //清零数组
    sRXCANMessage2.ui32MsgID = 0x7030;               // CAN接收节点的ID号，
    sRXCANMessage2.ui32MsgIDMask = 0;                   //过滤器是IDmask，相应的位置1，就是使用id中相应的位过滤，这是硬件过滤。
    sRXCANMessage2.ui32Flags = MSG_OBJ_RX_INT_ENABLE;   // 使能CAN接收中断
    sRXCANMessage2.ui32MsgLen = sizeof(ucRXMsgData2);   // 初始化CAN发送的字节数
    sRXCANMessage2.pucMsgData = ucRXMsgData2;           //初始化CAN通信的数组指针
//
    *(unsigned long *)ucRXMsgData3 = 0;                 //清零数组
    sRXCANMessage3.ui32MsgID = 0x7040;               // CAN接收节点的ID号，
    sRXCANMessage3.ui32MsgIDMask = 0;                   //过滤器是IDmask，相应的位置1，就是使用id中相应的位过滤，这是硬件过滤。
    sRXCANMessage3.ui32Flags = MSG_OBJ_RX_INT_ENABLE;   // 使能CAN接收中断
    sRXCANMessage3.ui32MsgLen = sizeof(ucRXMsgData3);   // 初始化CAN发送的字节数
    sRXCANMessage3.pucMsgData = ucRXMsgData3;           //初始化CAN通信的数组指针

    *(unsigned long *)ucRXMsgData4 = 0;                 //清零数组
    sRXCANMessage4.ui32MsgID = 0x7050;               // CAN接收节点的ID号，
    sRXCANMessage4.ui32MsgIDMask = 0;                   //过滤器是IDmask，相应的位置1，就是使用id中相应的位过滤，这是硬件过滤。
    sRXCANMessage4.ui32Flags = MSG_OBJ_RX_INT_ENABLE;   // 使能CAN接收中断
    sRXCANMessage4.ui32MsgLen = sizeof(ucRXMsgData4);   // 初始化CAN发送的字节数
    sRXCANMessage4.pucMsgData = ucRXMsgData4;           //初始化CAN通信的数组指针

    //设置用于接收消息的消息对象
    CANMessageSet(CANA_BASE, 2, &sRXCANMessage1, MSG_OBJ_TYPE_RX);
    CANMessageSet(CANA_BASE, 3, &sRXCANMessage2, MSG_OBJ_TYPE_RX);
    CANMessageSet(CANA_BASE, 4, &sRXCANMessage3, MSG_OBJ_TYPE_RX);
    CANMessageSet(CANA_BASE, 5, &sRXCANMessage4, MSG_OBJ_TYPE_RX);

}


unsigned char can_count1 = 0;
unsigned char can_count2 = 0;
unsigned char can_count3 = 0;

interrupt void  CANIntHandler(void)
{
    unsigned long ulStatus;
    can_count1++;
    //
    // Read the CAN interrupt status to find the cause of the interrupt
    //
    ulStatus = CANIntStatus(CANA_BASE, CAN_INT_STS_CAUSE);

    //
    // If the cause is a controller status interrupt, then get the status
    //
    if(ulStatus == CAN_INT_INT0ID_STATUS)
    {
        ulStatus = CANStatusGet(CANA_BASE, CAN_STS_CONTROL);
        if(((ulStatus  & ~(CAN_ES_TXOK | CAN_ES_RXOK)) != 7) &&
           ((ulStatus  & ~(CAN_ES_TXOK | CAN_ES_RXOK)) != 0))
        {
            g_bErrFlag = 1;
        }
    }
    else if(ulStatus == 1)//发送状态判断
    {
        CANIntClear(CANA_BASE, 1);
        g_ulTxMsgCount++;
        g_bErrFlag = 0;
//        can_count2++;
    }


    else if(ulStatus == 2)//接收状态判断
    {
        CANMessageGet(CANA_BASE, 2, &sRXCANMessage1, true);
        CANIntClear(CANA_BASE, 2);
        g_ulRxMsgCount++;
        g_bErrFlag = 0;
        flag_angle1 = 1;
//        can_count3++;
    }
    else if(ulStatus == 3)//接收状态判断
    {

        CANMessageGet(CANA_BASE, 3, &sRXCANMessage2, true);
        CANIntClear(CANA_BASE, 3);
        g_ulRxMsgCount++;
        g_bErrFlag = 0;
        flag_angle = 1;
//        can_count3++;
    }
    else if(ulStatus == 4)//接收状态判断
    {
        CANMessageGet(CANA_BASE, 4, &sRXCANMessage3, true);
        CANIntClear(CANA_BASE, 4);
        g_ulRxMsgCount++;
        g_bErrFlag = 0;
        flag_angle2 = 1;
//        can_count3++;
    }

    else if(ulStatus == 5)//接收状态判断
    {
        CANMessageGet(CANA_BASE, 5, &sRXCANMessage4, true);
        CANIntClear(CANA_BASE, 5);
        g_ulRxMsgCount++;
        g_bErrFlag = 0;
        flag_angle3 = 1;
//        can_count3++;
    }
    else
    {


    }
    CANGlobalIntClear(CANA_BASE, CAN_GLB_INT_CANINT0);
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP9;
}
float Pit = 0;
float Roll = 0;
float Pit_send;
float Roll_send;
int Pit_Sign;
int Roll_Sign;

i24_Byte i24_DATA;
i24_Byte i24_DATA1;

void CAN_Tx(void)
{
    //第一个参数CAN类型，第二个参数，CAN ID ,第三个参数发送数据，第四个参数数据类型
    CANMessageSet(CANA_BASE, 1, &sTXCANMessage, MSG_OBJ_TYPE_TX);


    CanDaTa.Frame_Count = Leida_state;
//    CanDaTa.Pitch_angle = 20;
//    CanDaTa.Roll_angle = 30;
    CanDaTa.Work_mode = DYT_Work_Mode;

    ucTXMsgData[0] = (CanDaTa.Work_mode & 0x00FF);
//

    if(Pit >= 0)
    {
        Pit_Sign = 0;
        Pit_send = Pit;
    }
    else
    {
        Pit_Sign = 1;
        Pit_send = -Pit;
    }

    if(Roll >= 0)
    {
        Roll_Sign = 0;
        Roll_send = Roll;
    }
    else
    {
        Roll_Sign = 1;
        Roll_send = -Roll;
    }

    i24_DATA.i24t = (Uint32)(Pit_send * 40000); //分辨率 0.000025
    ucTXMsgData[1] = i24_DATA.Byte.Byte_0;
    ucTXMsgData[2] = i24_DATA.Byte.Byte_1;
    i24_DATA.Byte.Byte_2 |=  (Pit_Sign << 7);
    ucTXMsgData[3] = i24_DATA.Byte.Byte_2;

//    ucTXMsgData[4] = ((int32_t)(Roll_send *40000)) & 0x0000ff;
//    ucTXMsgData[5] = (((int32_t)(Roll_send*40000)) & 0x00ff00) >> 8;
//    ucTXMsgData[6] = (((int32_t)(Roll_send*40000)) & 0xff0000) >> 16;

    i24_DATA1.i24t = (Uint32)(Roll_send * 40000); //分辨率 0.000025
    ucTXMsgData[4] = i24_DATA1.Byte.Byte_0;
    ucTXMsgData[5] = i24_DATA1.Byte.Byte_1;
    i24_DATA1.Byte.Byte_2 |=  (Roll_Sign << 7);
    ucTXMsgData[6] = i24_DATA1.Byte.Byte_2;

    ucTXMsgData[7] = (CanDaTa.Frame_Count & 0x00FF);
//
    Count++;
}

int Rx_Yaw_sign;//符号位 +-
int  Rx_Pitch_sign;
int Pitch_angle_sign;//俯仰角符号标志位
int Roll_angle_sign;//滚转角符号标志位
int Yaw_VelAng_Sim_sign;
int Pitch_VelAng_Sim_sign;
int Pitch_speed_sign;//俯仰电机速度符号标志位
int Roll_speed_sign;//滚转电机速度符号标志位

float real_Pitch_angle = 0.0;
float real_Roll_angle = 0.0;
float Yaw_VelAng_Sim = 0.0;
float Pitch_VelAng_Sim = 0.0;
float Pitch_speed = 0.0;
float Roll_speed = 0.0;

void CAN_Rx(void)
{
    //0x7020
    CanDaTa_rx.ID = ucRXMsgData1[0] & 0x00ff;

    Yaw_VelAng_Sim_sign = (ucRXMsgData1[3] & 0x80)>>7;
//    ucRXMsgData1[3] = ucRXMsgData1[3] & 0x7F;

    Pitch_VelAng_Sim_sign = (ucRXMsgData1[6] & 0x80)>>7;
//    ucRXMsgData1[6] = ucRXMsgData1[6] & 0x7F;

    Yaw_VelAng_Sim = (uint32_t)(ucRXMsgData1[1] + ((Uint32)ucRXMsgData1[2] << 8) + ((Uint32)(ucRXMsgData1[3] & 0x7f) << 16)) * 6.25e-5;//
    Pitch_VelAng_Sim = (uint32_t)(ucRXMsgData1[4] + ((Uint32)ucRXMsgData1[5] << 8) + ((Uint32)(ucRXMsgData1[6] & 0x7f) << 16)) * 6.25e-5;

    if(Yaw_VelAng_Sim_sign)
    {
        Yaw_VelAng_Sim = -Yaw_VelAng_Sim;
    }
    if(Pitch_VelAng_Sim_sign)
    {
        Pitch_VelAng_Sim = -Pitch_VelAng_Sim;
    }

    CanDaTa_rx.frame_count1 = ucRXMsgData1[7] & 0x00ff;

    //0x7030
    CanDaTa_rx.Picture_state = ucRXMsgData2[0] & 0x00ff;

    CanDaTa_rx.Target_x = (uint32_t)(ucRXMsgData2[1] + ((Uint32)ucRXMsgData2[2] << 8));
    CanDaTa_rx.Target_y = (uint32_t)(ucRXMsgData2[3] + ((Uint32)ucRXMsgData2[4] << 8));

    CanDaTa_rx.frame_count4 = ucRXMsgData2[5] & 0x00ff;

    //0x7040
    CanDaTa_rx.Pitch_work_mode = ucRXMsgData3[0] & 0x00ff;

    Pitch_angle_sign = (ucRXMsgData3[3] & 0x80)>>7;
//    ucRXMsgData3[3] = ucRXMsgData3[3] & 0x7F;

    Pitch_speed_sign = (ucRXMsgData3[6] & 0x80)>>7;
//    ucRXMsgData3[6] = ucRXMsgData3[6] & 0x7F;

    real_Pitch_angle = (uint32_t)(ucRXMsgData3[1] + ((Uint32)ucRXMsgData3[2] << 8) + ((Uint32)(ucRXMsgData3[3] &0x7f) << 16)) * 2.5e-5;//
    Pitch_speed = (uint32_t)(ucRXMsgData3[4] + ((Uint32)ucRXMsgData3[5] << 8) + ((Uint32)(ucRXMsgData3[6] & 0x7f)<< 16)) * 6.25e-5;

    if(Pitch_angle_sign)
    {
        real_Pitch_angle = -real_Pitch_angle;
    }

    if(Pitch_speed_sign)
    {
        Pitch_speed = -Pitch_speed;
    }

    CanDaTa_rx.frame_count2 = ucRXMsgData3[7] & 0x00ff;

    //0x7050
    CanDaTa_rx.Roll_work_mode = ucRXMsgData4[0] & 0x00ff;

    Roll_angle_sign = (ucRXMsgData4[3] & 0x80)>>7;
//    ucRXMsgData4[3] = ucRXMsgData4[3] & 0x7F;

    Roll_speed_sign = (ucRXMsgData4[6] & 0x80)>>7;
//    ucRXMsgData4[6] = ucRXMsgData4[6] & 0x7F;

    real_Roll_angle = (uint32_t)(ucRXMsgData4[1] + ((Uint32)ucRXMsgData4[2] << 8) + ((Uint32)(ucRXMsgData4[3] & 0x7f) << 16)) * 2.5e-5;
    Roll_speed = (uint32_t)(ucRXMsgData4[4] + ((Uint32)ucRXMsgData4[5] << 8) + ((Uint32)(ucRXMsgData4[6] & 0x7f) << 16)) * 6.25e-5;

    if(Roll_angle_sign)
    {
        real_Roll_angle = -real_Roll_angle;
    }

    if(Roll_speed_sign)
    {
        Roll_speed = -Roll_speed;
    }

    CanDaTa_rx.frame_count3 = ucRXMsgData4[7] & 0x00ff;


}

void InitCANGpio(void)
{
//    GPIO_SetupPinMux(7, GPIO_MUX_CPU1, 6); // GPIO7 - CANRXB
//    GPIO_SetupPinMux(6, GPIO_MUX_CPU1, 6); // GPIO6 - CANTXB
//    GPIO_SetupPinOptions(7, GPIO_INPUT, GPIO_ASYNC);
//    GPIO_SetupPinOptions(6, GPIO_OUTPUT, GPIO_PUSHPULL);
    GPIO_SetupPinMux(70, GPIO_MUX_CPU1, 5); // GPIO70 - CANRXA
    GPIO_SetupPinMux(4, GPIO_MUX_CPU1, 6);  // GPIO4  - CANTXA
    GPIO_SetupPinOptions(70, GPIO_INPUT, GPIO_ASYNC);
    GPIO_SetupPinOptions(4, GPIO_OUTPUT, GPIO_PUSHPULL);
}



