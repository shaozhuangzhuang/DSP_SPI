/*
 * GlobalData.h
 *
 *  Created on: 2022年2月16日
 *      Author: SZZ
 *  Modified:   2025-10-20 - 精简版本，仅保留类型定义和常量
 */

#ifndef GLOBALDATA_H_
#define GLOBALDATA_H_

#include <math.h>
#include <string.h>

/******************************************************************************
 * 常量定义
 ******************************************************************************/
#define EarthSemimajorAxis  6378000.0       // 地球长半轴（米）
#define EarthCuvature       (1/298.257)     // 地球曲率
#define PI                  3.14159265359   // 圆周率
#define ToRad               (PI/180)        // 角度转弧度
#define ToDeg               (180/PI)        // 弧度转角度

/******************************************************************************
 * 结构体类型定义
 ******************************************************************************/
// 地理位置元素结构体
struct SGeographyElements
{
    float64 Altitude;       // 高度（米）
    float64 Longitude;      // 经度（度）
    float64 Latitude;       // 纬度（度）
};

// 欧拉角结构体
struct SEularAngle
{
    float64 Roll;           // 滚转角（度）
    float64 Pitch;          // 俯仰角（度）
    float64 Yaw;            // 航向角（度）
};

/******************************************************************************
 * Union类型定义
 ******************************************************************************/
// 32位整数到16位整数转换
union Int32_To_Int16
{
    int32 data_i32;
    struct
    {
        Uint16 Byte0;
        Uint16 Byte1;
    }Byte;
};

// 浮点数到32位字节转换
typedef union Float_Toi32_Byte
{
    float FloatData;
    struct
    {
        Uint16 Byte0;
        Uint16 Byte1;
    }Byte;
}Float_Toi32_Byte;

// 控制指令联合体（位操作）
typedef union
{
    Uint16 HexByte1;
    struct
    {
        Uint16 bit15_14:2;
        Uint16 bit13_12:2;
        Uint16 bit11_10:2;
        Uint16 bit9_8:2;
        Uint16 bit7_6:2;
        Uint16 bit5_4:2;
        Uint16 bit3_2:2;
        Uint16 bit1_0:2;
    }bitData;
    struct
    {
        Uint16 Byte1:8;
        Uint16 Byte2:8;
    }ByteData;
}ControlInstruction;

/******************************************************************************
 * Typedef类型定义
 ******************************************************************************/
// 时间结构体
typedef struct FlightTime
{
    Uint16 PPS_Year;        // UTC时间，年，0~9999
    Uint16 PPS_Month;       // UTC时间，月，1~12
    Uint16 PPS_Day;         // UTC时间，日，1~31
    Uint16 PPS_Hour;        // UTC时间，时，00~23
    Uint16 PPS_Min;         // UTC时间，分，00~59
    Uint16 PPS_Sec;         // UTC时间，秒，00~59
    Uint16 PPS_Ms;          // UTC时间，毫秒，0~999
    Uint16 PPS_us;          // UTC时间，微秒，0~999
    Uint64 PPS_UTC_us;      // UTC时间，总微秒数
}Para_FlightTime;

#endif /* GLOBALDATA_H_ */
