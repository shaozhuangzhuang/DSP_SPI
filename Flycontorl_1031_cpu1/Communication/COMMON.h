/*
 * COMMON.h
 *
 *  Created on: 2022年9月2日
 *      Author: 110
 */

#ifndef COMMUNICATION_COMMON_H_
#define COMMUNICATION_COMMON_H_
#include "F28x_Project.h"
#include "Sci_A.h"
#include "Sci_C.h"
#include "CAN_Proc.h"
#include "MCBSP.h"
#include "math.h"

//#define HUOKONG
//#define FZJ_TEST
#define DATALINK
#define Servo_TEST

void Init_Fire_YinZhan_Gpio(void);

#define pi 3.1415926f
#define Gravity0 9.7803
#define REarth 6378000
#define TS_IMU 0.002f
#define G0 9.8015f
//基本变量
#define deg2rad 0.017453292519943f
#define rad2deg 57.295779513082323f

extern bool fireCtrlFlag;//用于判断串口连接状态
extern Uint16 fireState;
extern float Angle_Back[12];

struct RX_B_FRAME_REG
{
    unsigned char SYNC[2];
//    unsigned char FRAME_ID;//功能码
//    unsigned char MOTOR_NUM;//地址码
    unsigned char DATA[44];     // Data of frame
    unsigned char CRC[2];           // Checksum of frame
};

struct HexToBit
{
    Uint16 bit0:1;
    Uint16 bit1:1;
    Uint16 bit2:1;
    Uint16 bit3:1;
    Uint16 bit4:1;
    Uint16 bit5:1;
    Uint16 bit6:1;
    Uint16 bit7:1;
    Uint16 rsv:8;
};
union HexData
{
    Uint16 Hexall;
    struct HexToBit bit;
};

struct RX_McbspData
{
    Uint16  FrameID;                      //帧功能识别码



    float32 Angle_1;
    float32 Angle_2;
    float32 Angle_3;
    float32 Angle_4;

    float32 Angle_speed_1;
    float32 Angle_speed_2;
    float32 Angle_speed_3;
    float32 Angle_speed_4;

    float32 Angle_cmd_1;
    float32 Angle_cmd_2;
    float32 Angle_cmd_3;
    float32 Angle_cmd_4;

};

typedef union F32_Byte
{
    int32 I32;
    struct
    {
        unsigned char Byte_0:8;
        unsigned char Byte_1:8;
        unsigned char Byte_2:8;
        unsigned char Byte_3:8;
    }BYTE;
}F32_Byte;

struct RX_Sim
{
    Uint16  FrameID;                      //帧功能识别码

    Uint32 FrameCnt;

    float32 FZJ_Acc_X;
    float32 FZJ_Acc_Y;
    float32 FZJ_Acc_Z;

    float32 FZJ_Pitch_VelAng;
    float32 FZJ_Yaw_VelAng;
    float32 FZJ_Roll_VelAng;

    float32 Yaw_VelAng_Sim;//视线角速度-方位
    float32 Pitch_VelAng_Sim;//视线角速度-高低
};

enum SCI_Tx_State
{
    Start_Tx,
    Stop_Tx,
};
enum SCI_Tx_Number
{
    Zero,
    One,
    Two,
    Three,
    Four,
    Five
};

struct State_SCI_Tx
{
    enum SCI_Tx_State State;//发送状态
    enum SCI_Tx_Number Field_Number;//字段顺序
};

#define RXBUF_MASK 0xFF
struct RX_BUF
{
    unsigned char rx_buf[RXBUF_MASK + 1];
    unsigned int rx_head;
    unsigned int rx_tail;     // Data of frame
};




typedef union i16_Byte
{
    signed int i16t;
    struct
    {
        unsigned char Byte_0:8;
        unsigned char Byte_1:8;
    }Byte;
}i16_Byte;

typedef union Ui16_Byte
{
    Uint16 ui16t;
    struct
    {
        unsigned char Byte_0:8;
        unsigned char Byte_1:8;
    }Byte;
}Ui16_Byte;


typedef union U16_Byte
{
    Uint16 U16;
    struct
    {
        unsigned char Byte_0:8;
        unsigned char Byte_1:8;
    }Byte;
}U16_Byte;

typedef union U32_Byte
{
    Uint32 U32;
    struct
    {
        unsigned char Byte_0:8;
        unsigned char Byte_1:8;
        unsigned char Byte_2:8;
        unsigned char Byte_3:8;
    }BYTE;
}U32_Byte;



typedef union Float32_Byte
{
    float Float32;
    struct
    {
        unsigned char Byte_0:8;
        unsigned char Byte_1:8;
        unsigned char Byte_2:8;
        unsigned char Byte_3:8;
    }BYTE;
}Float32_Byte;

typedef union i32_Byte
{
    int32 i32;
    struct
    {
        unsigned char Byte_0:8;
        unsigned char Byte_1:8;
        unsigned char Byte_2:8;
        unsigned char Byte_3:8;
    }BYTE;
}i32_Byte;

typedef union
{
    uint32_t h;
    float f;
}F2HVALUE;


typedef union uint8_i32
{
    struct
    {
        unsigned char first :8;
        unsigned char second :8;
        unsigned char third :8;
        unsigned char forth :8;
    }uint32;

    int32 int32tdata;
}uint8_i32;

struct IPC_Comm_Data
{
    //IMU原始数据
    float IMU_X_gyro;
    float IMU_Y_gyro;
    float IMU_Z_gyro;
    float IMU_X_acc;
    float IMU_Y_acc;
    float IMU_Z_acc;
    //航姿解算
    float AttitudeSolution_Latitude;
    float AttitudeSolution_Longitude;
    float AttitudeSolution_Height;
    float AttitudeSolution_X;
    float AttitudeSolution_Y;
    float AttitudeSolution_Z;
    float AttitudeSolution_VN;
    float AttitudeSolution_VE;
    float AttitudeSolution_VD;
    float AttitudeSolution_Yaw;
    float AttitudeSolution_Pitch;
    float AttitudeSolution_Roll;
};


struct RX_Datalink
{
    Uint16  FrameID:8;                      //帧功能识别码
    Uint16  FlightNum:8;                    //飞行器编号
    Uint32  FrameCnt;                      //帧计数
    int16  StateWord:8;                    //导引头、安保解锁、引信解锁、数据模式
    Uint16  TargetNum:8;                    //发射筒编号
    Uint16  Ready_cmd;                      //查询准备状态指令
    Uint16  Battery_cmd;                    //电池上电指令
    Uint16  Engine_fire;                    //发动机点火
    Uint16  Emulator_cmd;                    //仿真程序指令
    Uint16  Launch_cmd;                    //发射指令
    Uint16  Alignment_cmd;                    //对准指令
    Uint16  DYT_RESTE;                    //导引头重置—北理工联调用

    //目标状态，弹坐标系
    float32 TarHL_Angle;                     //高低角 -90 ~ 90   0.01deg
    float32 TarAz_Angle;                     //目标方位角
    Uint32 TarDistance;                      //目标距离
    float32 TarApproachSpeed;                //接近速度（导弹与目标）
    float32 Tar_SightAngleSpeed_Wy;            //目标-视线角速度Wy
    float32 Tar_SightAngleSpeed_Wz;            //目标-视线角速度Wz

    //预测点，弹坐标系
    float32 ForPointHL_Angle;                    //预测点高低角
    float32 ForPointAz_Angle;                    //预测点方位角
    Uint16 ForPointDistance;                    //预测点距离
    float32 ForPointApproachSpeed;               //预测点接近速度（导弹与目标）
    float32 ForPoint_SightAngleSpeed_Wy;         //预测点-视线角速度Wy
    float32 ForPoint_SightAngleSpeed_Wz;         //预测点-视线角速度Wz

    float32 RemainingInterceptTime;              //剩余拦截时间
    float32  Tar_Longitude;                      //目标经度
    float32  Tar_Latitude;                       //目标纬度
    float32  Tar_Altitude;                       //拦截弹高度

    float32  Carrire_Longitude;                      //拦截弹经度
    float32  Carrire_Latitude;                       //拦截弹纬度
    float32  Carrire_Altitude;                       //拦截弹高度
    float32  VN;
    float32  VE;
    float32  VD;
    float32  Yaw;
    float32  Pitch;
    float32  Roll;
    float32  HL_Angle;                           //高低角
    float32  Az_Angle;                          //方位角

    Uint16 hour;
    Uint16 min;
    Uint16 Second;
    Uint16 M_second;

    double OmegaY;
    double OmegaZ;
    float Velocity;
};

//飞控接收到导引头的数据帧
struct Rx_SeekerToFlight
{
    float32  Seeker_Pitch_feedback;         //导引头俯仰角
    float32  Seeker_Roll_feedback;          //导引头滚转角
    float32  Seeker_Pitch_AngVel;           //导引头滚转角速度
    float32  Seeker_Roll_AngVel;            //导引头俯仰角速度
    unsigned char    CMD:8;                 //导引头指令
    int16    Target_x;                      //目标值的X坐标
    int16    Target_y;                      //目标值Y坐标
    int16    Energy;                        //目标能量强度
    float32  Gyro_yaw_AngVel;               //陀螺仪偏航角速度
    float32  Gyro_pitch_AngVel;             //陀螺仪俯仰角速度
    int16    work_mode;                     //导引头状态字
    int16    Pitch_work_mode;               //俯仰工作状态
    int16    Roll_work_mode;                //滚转工作状态
    Uint16    Picture_work_mode;                //图像板工作状态
    Uint16   Power_on_state;                //上电状态反馈
};

//飞控接收到仿真的数据帧
struct Rx_SimToFlight
{
    /* data1  0X36*/
    Uint16 FlightNum:8;                         //飞行器编号
    Uint32  FrameCnt1;                           //帧计数
    float32  Rx_Acc_X_Sim;                       //加速度ax
    float32  Rx_Acc_Y_Sim;                       //加速度ay
    float32  Rx_Acc_Z_Sim;                       //加速度az
    float32  Rx_Pitch_VelAng_Sim;                //俯仰角速度
    float32  Rx_Yaw_VelAng_Sim;                  //偏航角速度
    float32  Rx_Roll_VelAng_Sim;                 //滚转角速度

    float32 Rx_overload_YC_Sim;
    float32 Rx_overload_ZC_Sim;

    /* data2  0X69*/
    Uint16 Target_Missile:8;                    //目标弹编号
    Uint32  FrameCnt2;                          //帧计数
    Uint16 StateControlWord1:8;                 //状态控制字
    Uint16 TarNunber:8;                         //目标编号
    Uint16 Battery_voltage;                     //弹上电池电压
    //目标状态（弹体坐标系）
    float32 TarHL_Angle;                         //目标高低角
    float32 TarAz_Angle;                         //目标方位角
    Uint32 TarDistance;                         //目标距离
    float32 TarApproachSpeed;                    //接近速度（导弹与目标）
    float32 Tar_SightAngleSpeed_Wy;              //目标-视线角速度Wy
    float32 Tar_SightAngleSpeed_Wz;              //目标-视线角速度Wz
    //预测点（弹体坐标系）
    float32 ForPointHL_Angle;                    //预测点高低角
    float32 ForPointAz_Angle;                    //预测点方位角
    Uint16  ForPointDistance;                    //预测点距离
    float32 ForPointApproachSpeed;               //预测点接近速度（导弹与目标）
    float32 ForPoint_SightAngleSpeed_Wy;         //预测点-视线角速度Wy
    float32 ForPoint_SightAngleSpeed_Wz;         //预测点-视线角速度Wz
    float32 RemainingInterceptTime;              //剩余拦截时间
    float32 Angle_Roll;                          //导引头的滚转角速度，由于导引头本身的出的数值不准确，因此先暂时使用仿真机模拟给出
};

extern struct Rx_SeekerToFlight          SeekerToFlight_Rx;      //飞控接收到导引头的数据帧
extern struct Rx_SimToFlight             Rx_SimToFlight_Rx;      //飞控接收到仿真的数据帧
//////struct Rx_ServoToFlight           Rx_ServoToFlight_Rx;    //飞控接收到舵机的数据帧
//extern struct Rx_SeekerToFlight          SeekerToFlight_Rx;      //飞控接收到导引头的数据帧
extern struct RX_Datalink RX_Datalink;
extern struct RX_Datalink RX_HUOKONG;

extern unsigned char SCIA_SEND_HUOKONG[80];
extern unsigned char SCIC_SEND_SIM[80];
extern unsigned char SCIC_SEND_DATALINK[80];

extern float pitch_f,roll_f,yaw_f;
extern int Flag_Start_Fzj;
extern float InsOverload_aY;    //弹指令过载aY
extern float InsOverload_aZ;    //弹指令过载aZ
extern float PID_Result_GyroX_1;
extern float PID_Result_GyroY_1;
extern float PID_Result_GyroZ_1;
extern double latitude;
extern double longitude;
extern int longitude_Proportion;
extern double longitude0;
extern double latitude0;
extern float altitude;
extern float Result_duoliang1_1;
extern float Result_duoliang2_1;
extern float Result_duoliang3_1;
extern float Result_duoliang4_1;
//MCBSP
extern int McBsp_Rec_Flag;
extern int TransFinish_Flag;

//导引头
extern int DYT_Work_Mode;
extern int Leida_state;

extern struct IPC_Comm_Data IPC_Data_Cpu2;
extern struct RX_McbspData RX_SteeringEngine;
extern struct RX_Sim RX_SimData;

#endif /* COMMUNICATION_COMMON_H_ */
