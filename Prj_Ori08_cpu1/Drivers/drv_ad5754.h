/*******************************************************************************
 * 文件名：drv_ad5754.h
 * 描述：  AD5754 4通道16位DAC驱动头文件
 * 作者：  Auto-generated
 * 日期：  2025-10-20
 ******************************************************************************/

#ifndef DRV_AD5754_H_
#define DRV_AD5754_H_

#include "F28x_Project.h"
#include <stdbool.h>

/******************************************************************************
 * 芯片规格常量
 ******************************************************************************/
#define AD5754_CHANNELS         4       // 4通道独立DAC
#define AD5754_RESOLUTION       16      // 16位分辨率
#define AD5754_CMD_WIDTH        24      // 命令字长度24位

/******************************************************************************
 * 寄存器选择位定义 - REG[2:0] (根据AD5754R数据手册表18)
 ******************************************************************************/

/**
 * @brief  寄存器选择（REG[2:0]，3位）
 */
typedef enum {
    AD5754_REG_DAC = 0x00,          // 000: DAC寄存器
    AD5754_REG_RANGE = 0x01,        // 001: 输出范围选择寄存器
    AD5754_REG_POWER = 0x02,        // 010: 电源控制寄存器
    AD5754_REG_CONTROL = 0x03       // 011: 控制寄存器
} AD5754_Register_t;

/**
 * @brief  通道地址（A[2:0]，3位）
 */
typedef enum {
    AD5754_ADDR_DAC_A = 0x00,       // 000: DAC A
    AD5754_ADDR_DAC_B = 0x01,       // 001: DAC B
    AD5754_ADDR_DAC_C = 0x02,       // 010: DAC C
    AD5754_ADDR_DAC_D = 0x03,       // 011: DAC D
    AD5754_ADDR_DAC_ALL = 0x04      // 100: 所有四个DAC
} AD5754_Channel_t;

/**
 * @brief  控制寄存器的特殊地址（A[2:0]，用于特殊命令）
 */
typedef enum {
    AD5754_CTRL_ADDR_NOP = 0x00,    // 000: 无操作
    AD5754_CTRL_ADDR_CONFIG = 0x01, // 001: 配置位
    AD5754_CTRL_ADDR_CLEAR = 0x04,  // 100: 软件清零
    AD5754_CTRL_ADDR_LOAD = 0x05    // 101: 软件加载
} AD5754_CtrlAddr_t;

/******************************************************************************
 * 命令字构建宏（根据AD5754R数据手册表17标准格式）
 ******************************************************************************/

/**
 * @brief  24位命令字结构（MSB先行）
 * @note   格式：[R/W:1][Zero:1][REG:3][A:3][Data:16]
 *         Bit[23]:    R/W    - 读(1)或写(0)
 *         Bit[22]:    Zero   - 必须为0（省略，默认为0）
 *         Bit[21:19]: REG    - 寄存器选择(3位)
 *         Bit[18:16]: A      - 通道地址(3位)
 *         Bit[15:0]:  Data   - 数据(16位)
 * @note   使用两步移位避免C28x平台超过16位移位的警告
 *         例如：<< 23 = << 16 << 7
 */
#define AD5754_MAKE_CMD(rw, reg, addr, data) \
    (((((Uint32)(rw) & 0x01UL) << 16) << 7) | \
     ((((Uint32)(reg) & 0x07UL) << 16) << 3) | \
     (((Uint32)(addr) & 0x07UL) << 16) | \
     ((Uint32)(data) & 0xFFFFUL))

// ============================================================================
// 快捷命令宏（基于标准格式）
// ============================================================================

// 写DAC寄存器（REG=000）
#define AD5754_WR_DAC(channel, data) \
    AD5754_MAKE_CMD(0, AD5754_REG_DAC, channel, data)

// 写输出范围寄存器（REG=001）
#define AD5754_WR_RANGE(channel, range) \
    AD5754_MAKE_CMD(0, AD5754_REG_RANGE, channel, range)

// 写电源控制寄存器（REG=010, A=000）
#define AD5754_WR_POWER(data) \
    AD5754_MAKE_CMD(0, AD5754_REG_POWER, 0, data)

// 写控制寄存器配置（REG=011, A=001）
#define AD5754_WR_CONTROL(data) \
    AD5754_MAKE_CMD(0, AD5754_REG_CONTROL, AD5754_CTRL_ADDR_CONFIG, data)

// 软件清零命令（REG=011, A=100）
#define AD5754_CMD_CLEAR \
    AD5754_MAKE_CMD(0, AD5754_REG_CONTROL, AD5754_CTRL_ADDR_CLEAR, 0)

// 软件加载命令（REG=011, A=101）
#define AD5754_CMD_LOAD \
    AD5754_MAKE_CMD(0, AD5754_REG_CONTROL, AD5754_CTRL_ADDR_LOAD, 0)

// NOP命令（REG=011, A=000）
#define AD5754_CMD_NOP \
    AD5754_MAKE_CMD(0, AD5754_REG_CONTROL, AD5754_CTRL_ADDR_NOP, 0)

// 读DAC寄存器（REG=000）
#define AD5754_RD_DAC(channel) \
    AD5754_MAKE_CMD(1, AD5754_REG_DAC, channel, 0)

// 读控制寄存器（REG=011, A=001）
#define AD5754_RD_CONTROL \
    AD5754_MAKE_CMD(1, AD5754_REG_CONTROL, AD5754_CTRL_ADDR_CONFIG, 0)

/******************************************************************************
 * 控制寄存器位定义（REG=011, A=001）- 根据数据手册表24-25
 ******************************************************************************/

/**
 * @brief  控制寄存器位字段（Data[3:0]）
 */
#define AD5754_CTRL_SDO_DIS         (0x0001)    // bit[0]: 1=禁用SDO, 0=使能SDO
#define AD5754_CTRL_CLR_SEL         (0x0002)    // bit[1]: CLR选择(0=零电平, 1=中间电平)
#define AD5754_CTRL_CLAMP_EN        (0x0004)    // bit[2]: 1=使能电流钳位, 0=过流关断
#define AD5754_CTRL_TSD_EN          (0x0008)    // bit[3]: 1=使能热关断, 0=禁用

// 推荐默认值：SDO使能、CLR到零电平、钳位使能、TSD禁用
#define AD5754_CTRL_DEFAULT  \
    (AD5754_CTRL_CLAMP_EN | 0x0000)  // 0x0004

/******************************************************************************
 * 输出范围定义（写入RANGE寄存器）
 ******************************************************************************/

/**
 * @brief  输出电压范围枚举
 */
typedef enum {
    AD5754_RANGE_0_5V = 0x0000,     // 单极性：0V ~ +5V
    AD5754_RANGE_0_10V = 0x0001,    // 单极性：0V ~ +10V
    AD5754_RANGE_0_10V_8 = 0x0002,  // 单极性：0V ~ +10.8V
    AD5754_RANGE_NEG_5_5V = 0x0003, // 双极性：-5V ~ +5V
    AD5754_RANGE_NEG_10_10V = 0x0004, // 双极性：-10V ~ +10V
    AD5754_RANGE_NEG_10_8_10_8V = 0x0005 // 双极性：-10.8V ~ +10.8V
} AD5754_Range_t;

/******************************************************************************
 * 电源控制寄存器位定义（REG=010, A=000）- 根据数据手册表27-28
 ******************************************************************************/

/**
 * @brief  电源控制位（Data[15:0]中的位定义）
 * 
 * Bit[10]: OC_D  - DAC D过流报警（只读）
 * Bit[9]:  OC_C  - DAC C过流报警（只读）
 * Bit[8]:  OC_B  - DAC B过流报警（只读）
 * Bit[7]:  OC_A  - DAC A过流报警（只读）
 * Bit[6]:  TSD   - 热关断报警（只读）
 * Bit[5]:  PU_REF - 内部基准上电（可写，1=上电）
 * Bit[4]:  PU_D  - DAC D上电（可写，1=上电）
 * Bit[3]:  PU_C  - DAC C上电（可写，1=上电）
 * Bit[2]:  PU_B  - DAC B上电（可写，1=上电）
 * Bit[1]:  PU_A  - DAC A上电（可写，1=上电）
 * Bit[0]:  保留
 */
#define AD5754_PWR_PU_A             (0x0002)    // bit[1]: DAC A上电
#define AD5754_PWR_PU_B             (0x0004)    // bit[2]: DAC B上电
#define AD5754_PWR_PU_C             (0x0008)    // bit[3]: DAC C上电
#define AD5754_PWR_PU_D             (0x0010)    // bit[4]: DAC D上电
#define AD5754_PWR_PU_REF           (0x0020)    // bit[5]: 内部基准上电
#define AD5754_PWR_ALL_CHANNELS     (0x001E)    // 所有通道上电(bit[4:1])

// 上电所有通道+内部基准
#define AD5754_PWR_ALL_WITH_REF     (AD5754_PWR_ALL_CHANNELS | AD5754_PWR_PU_REF)  // 0x003E

/******************************************************************************
 * GPIO引脚定义
 ******************************************************************************/
#define AD5754_CS_GPIO              27      // SPIB CS (GPIO27, #SYNC信号)
#define AD5754_LDAC_GPIO            55      // #LDAC控制 (GPIO55)

// 注意：硬件设计中，AD5754的控制引脚连接：
// - #LDAC: 连接GPIO55，可通过软件控制DAC更新时序
// - #CLR:  通过10kΩ上拉至DVCC，禁用硬件清零功能
// - #SYNC: 使用SPI CS (GPIO27)

/******************************************************************************
 * 数据结构定义
 ******************************************************************************/

/**
 * @brief  AD5754配置结构
 */
typedef struct {
    AD5754_Range_t range[AD5754_CHANNELS];  // 各通道输出范围
    Uint16 ctrl_reg;                        // 控制寄存器值
    Uint16 pwr_reg;                         // 电源控制寄存器值
    // 注意：use_ldac_gpio已删除，硬件上#LDAC直接接地
} AD5754_Config_t;

/**
 * @brief  AD5754回调函数类型
 */
typedef void (*AD5754_WriteCompleteCallback_t)(void);

/******************************************************************************
 * 外部变量声明
 ******************************************************************************/

// DMA传输状态标志
extern volatile bool g_ad5754_write_busy;

/******************************************************************************
 * 时序参数定义（参考AD5754数据手册）
 ******************************************************************************/
#define AD5754_TCSS_NS              5       // CS建立时间（min 5ns）
#define AD5754_TCSH_NS              5       // CS保持时间（min 5ns）
#define AD5754_TVREF_SETTLING_MS    10      // VREF稳定时间（典型10ms）

// 注意：#LDAC和#CLR时序由硬件固定，无需软件控制

/******************************************************************************
 * 函数声明
 ******************************************************************************/

/**
 * @brief  配置AD5754控制引脚GPIO
 * @note   配置GPIO55作为#LDAC控制引脚
 */
void Drv_AD5754_GPIO_Config(void);

/**
 * @brief  控制AD5754的#LDAC引脚电平
 * @param  level - 0=低电平（触发DAC更新），1=高电平（禁用更新）
 * @note   #LDAC为低电平有效，下降沿触发所有DAC同步更新
 */
void Drv_AD5754_SetLDAC(Uint16 level);

/**
 * @brief  触发所有DAC同步更新（通过LDAC脉冲）
 * @note   先拉低LDAC，等待t11（最小20ns），再拉高，触发所有通道同步更新
 */
void Drv_AD5754_TriggerLDAC(void);

/**
 * @brief  软件复位AD5754（通过控制寄存器）
 * @note   硬件上没有连接#CLR引脚，使用软件复位命令
 */
void Drv_AD5754_SoftReset(void);

/**
 * @brief  初始化AD5754
 * @param  config - 配置结构体指针
 * @return bool - true=成功，false=失败
 */
bool Drv_AD5754_Init(AD5754_Config_t *config);

/**
 * @brief  使用默认配置初始化AD5754
 * @return bool - true=成功，false=失败
 * @note   默认：±10V范围、SDO使能、所有通道上电
 */
bool Drv_AD5754_InitDefault(void);

/**
 * @brief  写单通道DAC值（阻塞模式）
 * @param  channel - 通道号（0-3对应A-D）
 * @param  value - 16位DAC值
 * @param  timeout_ms - 超时时间（毫秒）
 * @return bool - true=成功，false=超时或失败
 */
bool Drv_AD5754_WriteChannelBlocking(Uint16 channel, Uint16 value, Uint32 timeout_ms);

/**
 * @brief  写单通道DAC值（非阻塞模式）
 * @param  channel - 通道号（0-3对应A-D）
 * @param  value - 16位DAC值
 * @return bool - true=成功触发，false=忙碌中
 */
bool Drv_AD5754_WriteChannel(Uint16 channel, Uint16 value);

/**
 * @brief  批量写入所有通道（阻塞模式）
 * @param  values - 4个通道的DAC值数组
 * @param  timeout_ms - 超时时间（毫秒）
 * @return bool - true=成功，false=超时或失败
 */
bool Drv_AD5754_WriteAllChannelsBlocking(Uint16 *values, Uint32 timeout_ms);

/**
 * @brief  批量写入所有通道（非阻塞模式）
 * @param  values - 4个通道的DAC值数组
 * @return bool - true=成功触发，false=忙碌中
 */
bool Drv_AD5754_WriteAllChannels(Uint16 *values);

/**
 * @brief  读取DAC寄存器值（阻塞模式）
 * @param  channel - 通道号（0-3对应A-D）
 * @param  value - 输出读取值的指针
 * @param  timeout_ms - 超时时间（毫秒）
 * @return bool - true=成功，false=超时或失败
 */
bool Drv_AD5754_ReadChannelBlocking(Uint16 channel, Uint16 *value, Uint32 timeout_ms);

/**
 * @brief  读取控制寄存器（阻塞模式）
 * @param  value - 输出读取值的指针
 * @param  timeout_ms - 超时时间（毫秒）
 * @return bool - true=成功，false=超时或失败
 */
bool Drv_AD5754_ReadCtrlRegBlocking(Uint16 *value, Uint32 timeout_ms);

/**
 * @brief  写控制寄存器
 * @param  value - 控制寄存器值
 * @param  timeout_ms - 超时时间（毫秒）
 * @return bool - true=成功，false=超时或失败
 */
bool Drv_AD5754_WriteCtrlReg(Uint16 value, Uint32 timeout_ms);

/**
 * @brief  写输出范围寄存器
 * @param  channel - 通道号（0-3对应A-D）
 * @param  range - 输出范围枚举
 * @param  timeout_ms - 超时时间（毫秒）
 * @return bool - true=成功，false=超时或失败
 */
bool Drv_AD5754_WriteRangeReg(Uint16 channel, AD5754_Range_t range, Uint32 timeout_ms);

/**
 * @brief  写电源控制寄存器
 * @param  value - 电源控制值
 * @param  timeout_ms - 超时时间（毫秒）
 * @return bool - true=成功，false=超时或失败
 */
bool Drv_AD5754_WritePwrReg(Uint16 value, Uint32 timeout_ms);

/**
 * @brief  设置单通道输出电压（物理电压值）
 * @param  channel - 通道号（0-3对应A-D）
 * @param  voltage - 目标电压（V）
 * @param  timeout_ms - 超时时间（毫秒）
 * @return bool - true=成功，false=超时或失败
 * @note   根据通道的range配置自动转换为DAC码值
 */
bool Drv_AD5754_SetVoltage(Uint16 channel, float voltage, Uint32 timeout_ms);

/**
 * @brief  注册写完成回调函数
 * @param  callback - 回调函数指针
 */
void Drv_AD5754_RegisterCallback(AD5754_WriteCompleteCallback_t callback);

/**
 * @brief  检查AD5754是否忙碌
 * @return bool - true=忙碌，false=空闲
 */
bool Drv_AD5754_IsBusy(void);

/**
 * @brief  电压到DAC码值转换
 * @param  voltage - 电压值（V）
 * @param  range - 输出范围
 * @return Uint16 - DAC码值（0-65535）
 */
Uint16 Drv_AD5754_VoltageToDacCode(float voltage, AD5754_Range_t range);

/**
 * @brief  DAC码值到电压转换
 * @param  dac_code - DAC码值（0-65535）
 * @param  range - 输出范围
 * @return float - 电压值（V）
 */
float Drv_AD5754_DacCodeToVoltage(Uint16 dac_code, AD5754_Range_t range);

/******************************************************************************
 * 内部辅助函数声明
 ******************************************************************************/

/**
 * @brief  发送24位命令字
 * @param  cmd - 24位命令字
 * @param  timeout_ms - 超时时间（毫秒）
 * @return bool - true=成功，false=超时或失败
 * @note   使用DMA发送，阻塞等待完成
 */
bool Drv_AD5754_SendCommand(Uint32 cmd, Uint32 timeout_ms);

/**
 * @brief  发送24位命令并接收响应
 * @param  cmd - 24位命令字
 * @param  response - 接收缓冲区指针（24位）
 * @param  timeout_ms - 超时时间（毫秒）
 * @return bool - true=成功，false=超时或失败
 */
bool Drv_AD5754_SendReceiveCommand(Uint32 cmd, Uint32 *response, Uint32 timeout_ms);

/******************************************************************************
 * 通信测试函数声明（用于验证SPIB与AD5754通信）
 ******************************************************************************/

// 测试结果全局变量（通过CCS Watch查看）
extern volatile Uint16 g_ad5754_test_status;      // 0=未测试, 1=进行中, 2=通过, 3=失败
extern volatile Uint16 g_ad5754_test_step;        // 当前测试步骤
extern volatile Uint16 g_ad5754_test_write_val;   // 写入值
extern volatile Uint16 g_ad5754_test_read_val;    // 读回值
extern volatile Uint16 g_ad5754_test_error_code;  // 错误码

/**
 * @brief  快速通信测试（推荐用于初次验证）
 * @return bool - true=通过, false=失败
 * @note   测试流程：控制寄存器 + 通道A的3个数据模式
 *         在main函数初始化后调用一次即可
 *         通过CCS Watch查看g_ad5754_test_xxx变量来诊断问题
 */
bool Drv_AD5754_TestCommunication(void);

/**
 * @brief  完整通信测试（测试所有通道和多个数据模式）
 * @return bool - true=通过, false=失败
 * @note   测试流程：控制寄存器 + 所有通道 + 5种数据模式
 *         测试时间约5秒，适用于全面验证
 */
bool Drv_AD5754_TestCommunicationFull(void);

/******************************************************************************
 * 使用说明
 ******************************************************************************/

/*
 * 1. 初始化流程：
 *    a. 调用Drv_AD5754_GPIO_Config()配置GPIO
 *    b. 调用Drv_AD5754_InitDefault()或Drv_AD5754_Init()
 *    c. 可选：调用Drv_AD5754_WriteRangeReg()设置各通道输出范围
 *
 * 2. 单通道写入：
 *    Drv_AD5754_WriteChannelBlocking(0, 0x8000, 10);  // 写通道A=中点
 *
 * 3. 批量写入：
 *    Uint16 values[4] = {0x0000, 0x4000, 0x8000, 0xC000};
 *
 * 4. 通信测试（推荐在初始化后执行一次）：
 *    if(Drv_AD5754_TestCommunication()) {
 *        // 通信正常
 *    } else {
 *        // 通信失败，查看g_ad5754_test_error_code诊断
 *    Drv_AD5754_WriteAllChannelsBlocking(values, 10);
 *
 * 4. 物理电压设置：
 *    Drv_AD5754_SetVoltage(0, 5.0, 10);  // 通道A输出5.0V
 *
 * 5. 读回验证：
 *    Uint16 readback;
 *    Drv_AD5754_ReadChannelBlocking(0, &readback, 10);
 *
 * 6. 输出范围配置：
 *    - 单极性0-10V: AD5754_RANGE_0_10V
 *    - 双极性±10V: AD5754_RANGE_NEG_10_10V
 *
 * 7. 命令格式：
 *    - 24位：[RW/CMD:4][ADDR:4][DATA:16]
 *    - 使用16位SPI字长传输：2个16位字
 *
 * 8. 硬件固定配置：
 *    - #LDAC直接接地 → 写入后立即自动更新，无需软件控制
 *    - #CLR通过10kΩ上拉 → 禁用硬件清零，可使用软件复位
 */

#endif /* DRV_AD5754_H_ */

