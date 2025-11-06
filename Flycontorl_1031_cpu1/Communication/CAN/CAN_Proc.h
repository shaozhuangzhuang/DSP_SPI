/*
 * CAN_Porc.h
 *
 *  Created on: 2022年9月6日
 *      Author: 110
 */

#ifndef COMMUNICATION_CAN_CAN_PORC_H_
#define COMMUNICATION_CAN_CAN_PORC_H_

#include "inc/hw_types.h"
#include "inc/hw_memmap.h"
#include "inc/hw_can.h"
#include "can.h"
#include"COMMON.h"

extern tCANMsgObject sTXCANMessage;
extern tCANMsgObject sRXCANMessage1;
extern unsigned char ucTXMsgData[8];
extern unsigned char ucRXMsgData1[8];
extern unsigned char ucRXMsgData2[8];
extern unsigned char ucRXMsgData3[8];
extern unsigned char ucRXMsgData4[8];
extern tCANMsgObject sRXCANMessage2;
extern tCANMsgObject sRXCANMessage3;
extern tCANMsgObject sRXCANMessage4;
extern unsigned int Count;
extern int Pitch_angle;
extern int Roll_angle;
extern int flag_angle;
extern int flag_angle1;
extern int flag_angle2;
extern int flag_angle3;
struct can_tx
{
    unsigned int Frame_Count;
    signed int Pitch_angle; //俯仰角
    signed int Roll_angle;//滚转角
    unsigned int Work_mode;
};

struct can_rx
{
    unsigned int ID;//陀螺仪ID
    unsigned int frame_count1;
    unsigned int frame_count2;
    unsigned int frame_count3;
    unsigned int frame_count4;
    signed int Rx_Yaw_VelAng_Sim; //导引头偏航角速度
    signed int Rx_Pitch_VelAng_Sim;//导引头俯仰角速度

    signed int real_Pitch_angle;//当前俯仰角
    signed int real_Roll_angle;//当前滚转角
    unsigned int Pitch_work_mode;//俯仰当前工作模式
    unsigned int Roll_work_mode;//滚转当前工作模式

    unsigned int Target_x;//当前目标x坐标
    unsigned int Target_y;//当前目标y坐标
    unsigned int Picture_state;//图像工作状态
};

typedef union i24_Byte
{
    Uint32 i24t;
    struct
    {
        unsigned char Byte_0:8;
        unsigned char Byte_1:8;
        unsigned char Byte_2:8;
        unsigned char rsv:8;
    }Byte;
}i24_Byte;

extern void InitCAN(void);
extern void CAN_Tx(void);
extern void CAN_Rx(void);
void InitCANGpio(void);
interrupt void  CANIntHandler(void);




#endif /* COMMUNICATION_CAN_CAN_PORC_H_ */
