/*
 * COMMON.c
 *
 *  Created on: 2022年9月5日
 *      Author: 110
 */
#include"COMMON.h"

float pitch_f,roll_f,yaw_f;
int Flag_Start_Fzj;
float InsOverload_aY;    //弹指令过载aY
float InsOverload_aZ;    //弹指令过载aZ
float PID_Result_GyroX_1;
float PID_Result_GyroY_1;
float PID_Result_GyroZ_1;
double latitude;
double longitude;
int longitude_Proportion;
double longitude0;
double latitude0;
float altitude;
float Result_duoliang1_1;
float Result_duoliang2_1;
float Result_duoliang3_1;
float Result_duoliang4_1;

//仿真机结构体实例化
struct Rx_SimToFlight             Rx_SimToFlight_Rx;      //飞控接收到仿真的数据帧
//struct Rx_ServoToFlight           Rx_ServoToFlight_Rx;    //飞控接收到舵机的数据帧
struct Rx_SeekerToFlight          SeekerToFlight_Rx;      //飞控接收到导引头的数据帧
struct RX_Datalink RX_Datalink;

void Init_Fire_YinZhan_Gpio(void)
{
    // 点火I/O初始化
    GPIO_SetupPinMux(15,GPIO_MUX_CPU1,0);
    GPIO_SetupPinOptions(15, GPIO_OUTPUT,GPIO_PUSHPULL);
    GPIO_SetupPinMux(16,GPIO_MUX_CPU1,0);
    GPIO_SetupPinOptions(16, GPIO_OUTPUT,GPIO_PUSHPULL);
    GPIO_SetupPinMux(17,GPIO_MUX_CPU1,0);
    GPIO_SetupPinOptions(17, GPIO_OUTPUT,GPIO_PUSHPULL);
    GPIO_SetupPinMux(18,GPIO_MUX_CPU1,0);
    GPIO_SetupPinOptions(18, GPIO_OUTPUT,GPIO_PUSHPULL);

    GPIO_SetupPinMux(19,GPIO_MUX_CPU1,0);
    GPIO_SetupPinOptions(19, GPIO_OUTPUT,GPIO_PUSHPULL);
    GPIO_SetupPinMux(20,GPIO_MUX_CPU1,0);
    GPIO_SetupPinOptions(20, GPIO_OUTPUT,GPIO_PUSHPULL);
    GPIO_SetupPinMux(21,GPIO_MUX_CPU1,0);
    GPIO_SetupPinOptions(21, GPIO_OUTPUT,GPIO_PUSHPULL);
    GPIO_SetupPinMux(99,GPIO_MUX_CPU1,0);
    GPIO_SetupPinOptions(99, GPIO_OUTPUT,GPIO_PUSHPULL);

    //引战IO初始化
    GPIO_SetupPinMux(11,GPIO_MUX_CPU1,0);
    GPIO_SetupPinOptions(11, GPIO_INPUT,0);
    GPIO_SetupPinMux(13,GPIO_MUX_CPU1,0);
    GPIO_SetupPinOptions(13, GPIO_INPUT,0);
    GPIO_SetupPinMux(14,GPIO_MUX_CPU1,0);
    GPIO_SetupPinOptions(14, GPIO_INPUT,0);

    //IO拉高
    GPIO_WritePin(15 , 1);
    GPIO_WritePin(16 , 1);
    GPIO_WritePin(17 , 1);
    GPIO_WritePin(18 , 1);
    GPIO_WritePin(19 , 1);
    GPIO_WritePin(20 , 1);
    GPIO_WritePin(21 , 1);
    GPIO_WritePin(99 , 1);
}
