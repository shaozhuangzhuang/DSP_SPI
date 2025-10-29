/*******************************************************************************
 * 文件名：drv_ads1278.h
 * 描述：  ADS1278 8通道24位ADC驱动头文件
 * 作者：  Auto-generated
 * 日期：  2025-10-20
 * 
 * 硬件连接说明：
 * ==============================================================================
 * ADS1278的配置引脚已在PCB硬件上固定连接，无需软件配置：
 * 
 * 1. 工作模式配置（MODE引脚）：
 *    - MODE[1:0] = 01 (硬件固定)
 *      → MODE0 接 IOVDD(3.3V)
 *      → MODE1 接 GND
 *      → 配置为：High Resolution模式，52.7kSPS
 * 
 * 2. 数据格式配置（FORMAT引脚）：
 *    - FORMAT[2:0] = 001 (硬件固定)
 *      → FORMAT0 接 IOVDD
 *      → FORMAT1 接 GND
 *      → FORMAT2 接 GND
 *      → 配置为：SPI协议，TDM模式，固定位置
 * 
 * 3. 同步控制（SYNC引脚）：
 *    - SYNC 接 IOVDD 或硬件上拉至IOVDD
 *    → 连续采样模式，芯片上电后自动开始采样
 * 
 * 4. 掉电控制（PWDN引脚）：
 *    - PWDN 接 IOVDD 或硬件上拉至IOVDD
 *    → 正常工作模式（非掉电）
 * 
 * 5. 时钟分频（CLKDIV引脚）：
 *    - CLKDIV 根据CLK频率连接
 *    → 27MHz时接IOVDD（不分频）
 * 
 * 6. 菊花链输入（DIN引脚）：
 *    - DIN 接 GND
 *    → 单片应用，不使用菊花链
 * 
 * 7. 片选控制（CS引脚）：
 *    - CS 硬件固定拉低
 *    → 单片应用，芯片始终选中
 * 
 * 8. 数据就绪中断（DRDY引脚）：
 *    - DRDY 连接到 F28377D GPIO57
 *    → 配置为XINT2外部中断（下降沿触发）
 * 
 * 关键参数：
 * - ADC主时钟：27MHz (外部晶振)
 * - 采样率：52.734kSPS (27MHz ÷ 512)
 * - SPI时钟：10MHz (LSPCLK 50MHz ÷ 5)
 * - SPI模式：Mode 0 (CPOL=0, CPHA=0, 上升沿采样)
 * - 数据格式：24位二进制补码，MSB先行
 * ==============================================================================
 ******************************************************************************/

#ifndef DRV_ADS1278_H_
#define DRV_ADS1278_H_

#include "F28x_Project.h"
#include <stdbool.h>

/******************************************************************************
 * 芯片规格常量
 ******************************************************************************/
#define ADS1278_CHANNELS        8       // 8通道同步采样
#define ADS1278_RESOLUTION      24      // 24位分辨率
#define ADS1278_MAX_SAMPLE_RATE 144000  // 最大采样率144kSPS

/******************************************************************************
 * 工作模式枚举
 ******************************************************************************/

/**
 * @brief  ADS1278工作模式
 * @note   通过MODE[1:0]引脚配置
 */
typedef enum {
    ADS1278_MODE_HR = 0,    // High Resolution (高分辨率): 52.7kSPS, MODE[1:0]=01
    ADS1278_MODE_LP = 1,    // Low Power (低功耗): 105kSPS, MODE[1:0]=10
    ADS1278_MODE_LS = 2     // Low Speed (低速): 26.4kSPS, MODE[1:0]=00
} ADS1278_Mode_t;

/**
 * @brief  ADS1278数据格式
 * @note   通过FORMAT[2:0]引脚配置
 */
typedef enum {
    ADS1278_FORMAT_SPI = 1,     // SPI模式: FORMAT[2:0]=001
    ADS1278_FORMAT_TDM = 2,     // TDM模式: FORMAT[2:0]=010
    ADS1278_FORMAT_FRAME = 4    // Frame模式: FORMAT[2:0]=100
} ADS1278_Format_t;

/**
 * @brief  时钟分频
 */
typedef enum {
    ADS1278_CLKDIV_1 = 0,       // 不分频: CLKDIV=0
    ADS1278_CLKDIV_2 = 1        // 2分频: CLKDIV=1
} ADS1278_ClkDiv_t;

/******************************************************************************
 * 数据结构定义
 ******************************************************************************/

/**
 * @brief  ADS1278采样数据结构
 */
typedef struct {
    int32  ch[ADS1278_CHANNELS];    // 8通道24位数据（符号扩展到32位）
    Uint32 timestamp;                // 时间戳（可选，基于系统时钟）
    Uint16 sample_id;                // 样本序号
    bool   valid;                    // 数据有效标志
    bool   overflow;                 // 溢出标志（FIFO或DMA）
} ADS1278_Data_t;

/**
 * @brief  ADS1278配置结构
 */
typedef struct {
    ADS1278_Mode_t   mode;          // 工作模式
    ADS1278_Format_t format;        // 数据格式
    ADS1278_ClkDiv_t clkdiv;        // 时钟分频
    bool             use_interrupt; // 是否使用DRDY中断（true=XINT1，false=轮询）
} ADS1278_Config_t;

/**
 * @brief  ADS1278回调函数类型
 */
typedef void (*ADS1278_DataReadyCallback_t)(void);

/******************************************************************************
 * GPIO引脚定义
 ******************************************************************************/
#define ADS1278_CS_GPIO         58      // SPIA CS (GPIO58或硬件固定拉低)
#define ADS1278_DRDY_GPIO       57      // Data Ready (GPIO57, XINT2)
#define ADS1278_SYNC_GPIO       10      // 同步控制 (GPIO10)
#define ADS1278_PWDN_GPIO       11      // 掉电控制 (GPIO11)
#define ADS1278_MODE0_GPIO      12      // MODE[0] (GPIO12)
#define ADS1278_MODE1_GPIO      13      // MODE[1] (GPIO13)
#define ADS1278_FORMAT0_GPIO    14      // FORMAT[0] (GPIO14)
#define ADS1278_FORMAT1_GPIO    15      // FORMAT[1] (GPIO15)
#define ADS1278_FORMAT2_GPIO    17      // FORMAT[2] (GPIO17)
#define ADS1278_CLKDIV_GPIO     18      // CLKDIV (GPIO18)

/******************************************************************************
 * 外部变量声明
 ******************************************************************************/

// Ping-Pong缓冲区状态
extern volatile bool g_ads1278_ping_active;    // true=使用Ping缓冲，false=使用Pong缓冲
extern volatile Uint16 g_ads1278_sample_count; // 采样计数

/******************************************************************************
 * 函数声明
 ******************************************************************************/

/**
 * @brief  配置ADS1278控制引脚GPIO
 * @param  无
 * @return 无
 * @note   配置SYNC/PWDN/MODE/FORMAT/CLKDIV等GPIO引脚
 */
void Drv_ADS1278_GPIO_Config(void);

/**
 * @brief  初始化ADS1278
 * @param  config - 配置结构体指针
 * @return bool - true=成功，false=失败
 * @note   包含上电时序、模式配置、DRDY中断设置
 */
bool Drv_ADS1278_Init(ADS1278_Config_t *config);

/**
 * @brief  使用默认配置初始化ADS1278
 * @return bool - true=成功，false=失败
 * @note   默认：HR模式、SPI格式、CLKDIV=0、使用DRDY中断
 * @note   v2.5更新：仅配置GPIO和中断，不注册DMA回调（由Drv_DMA_Init负责）
 */
bool Drv_ADS1278_InitDefault(void);

/**
 * @brief  获取ADS1278的DMA回调函数指针
 * @return DMA_Callback_t - DMA RX完成回调函数
 * @note   v2.5新增：供Drv_DMA_Init()注册DMA回调使用
 */
void* Drv_ADS1278_GetDMACallback(void);

/* 
 * 注意：以下函数已删除，因为SYNC引脚硬件固定连接：
 * - void Drv_ADS1278_Start(void);  - SYNC硬件上拉，芯片自动连续采样
 * - void Drv_ADS1278_Stop(void);   - SYNC硬件上拉，芯片自动连续采样
 */

/**
 * @brief  执行单次ADC读取（阻塞模式）
 * @param  data - 输出数据结构指针
 * @param  timeout_ms - 超时时间（毫秒）
 * @return bool - true=成功，false=超时
 * @note   等待DRDY→SPI读取→数据转换
 */
bool Drv_ADS1278_ReadBlocking(ADS1278_Data_t *data, Uint32 timeout_ms);

/**
 * @brief  触发单次ADC读取（非阻塞模式）
 * @param  无
 * @return bool - true=成功触发，false=忙碌中
 * @note   启动DMA传输，数据通过回调或GetData获取
 */
bool Drv_ADS1278_TriggerRead(void);

/**
 * @brief  获取最新的ADC数据（非阻塞模式）
 * @param  data - 输出数据结构指针
 * @return bool - true=有新数据，false=无新数据
 * @note   从Ping-Pong缓冲区复制数据到用户缓冲区
 */
bool Drv_ADS1278_GetData(ADS1278_Data_t *data);

/**
 * @brief  检查是否有新数据可用
 * @return bool - true=有新数据，false=无新数据
 */
bool Drv_ADS1278_IsDataReady(void);

/**
 * @brief  清除数据有效标志
 * @param  无
 * @return 无
 */
void Drv_ADS1278_ClearDataReady(void);

/**
 * @brief  注册DRDY回调函数
 * @param  callback - 回调函数指针
 * @return 无
 * @note   在DRDY中断或DMA完成时调用
 */
void Drv_ADS1278_RegisterCallback(ADS1278_DataReadyCallback_t callback);

/* 
 * 注意：以下硬件控制函数已删除，因为相应引脚已在PCB硬件上固定连接：
 * 
 * - void Drv_ADS1278_SoftReset(void);        - PWDN引脚硬件上拉至IOVDD
 * - void Drv_ADS1278_SetMode(mode);          - MODE[1:0]=01硬件固定（High Resolution）
 * - void Drv_ADS1278_SetFormat(format);      - FORMAT[2:0]=001硬件固定（SPI, TDM, Fixed）
 * - void Drv_ADS1278_SetClkDiv(clkdiv);      - CLKDIV硬件固定（27MHz不分频）
 */

/**
 * @brief  从16位缓冲区转换24位ADC数据
 * @param  src_buffer - 源缓冲区（16位字数组，12个字=24字节）
 * @param  dest_data - 目标数据结构
 * @return 无
 * @note   重组24位数据并进行符号扩展
 */
void Drv_ADS1278_ConvertData(Uint16 *src_buffer, ADS1278_Data_t *dest_data);

/**
 * @brief  获取当前采样率
 * @param  mode - 工作模式
 * @return Uint32 - 采样率（Hz）
 */
Uint32 Drv_ADS1278_GetSampleRate(ADS1278_Mode_t mode);

/**
 * @brief  获取ADS1278状态
 * @param  is_running - 输出：是否正在采样
 * @param  sample_count - 输出：当前采样计数
 * @return 无
 */
void Drv_ADS1278_GetStatus(bool *is_running, Uint32 *sample_count);

/******************************************************************************
 * 中断服务函数声明（在interrupt.c中实现）
 ******************************************************************************/
__interrupt void ADS1278_DRDY_ISR(void);  // DRDY中断（XINT1）

/******************************************************************************
 * 时序参数定义（参考ADS1278数据手册）
 ******************************************************************************/
#define ADS1278_TCSS_NS         10      // CS建立时间（min 10ns）
#define ADS1278_TCSH_NS         10      // CS保持时间（min 10ns）
#define ADS1278_TSCLK_NS        37      // SCLK周期（min 37ns @ 27MHz）
#define ADS1278_TDOPD_NS        10      // DOUT延迟时间（max 10ns）
#define ADS1278_TPWDN_US        10      // PWDN脉冲宽度（min 10us）
#define ADS1278_TSYNC_US        1       // SYNC建立时间（min 1us）

/******************************************************************************
 * 数据转换宏
 ******************************************************************************/

/**
 * @brief  24位有符号数转32位有符号数（符号扩展）
 * @param  x - 24位数据（存储在32位变量中）
 * @return 符号扩展后的32位数据
 */
#define ADS1278_SIGN_EXTEND_24_TO_32(x)  \
    (((x) & 0x800000) ? ((x) | 0xFF000000) : (x))

/**
 * @brief  将3个16位字重组为1个24位数据
 * @param  h - 高16位字
 * @param  m - 中8位字（取低8位）
 * @param  l - 低8位字（实际未使用，ADS1278每通道3字节）
 * @return 24位数据
 * @note   ADS1278 SPI格式：每通道3字节，MSB先行
 */
#define ADS1278_COMBINE_24BIT(h, m, l)  \
    (((Uint32)(h) << 8) | ((Uint32)(m) & 0xFF))

/******************************************************************************
 * 使用说明
 ******************************************************************************/

/*
 * 1. 初始化流程：
 *    a. 调用Drv_ADS1278_GPIO_Config()配置GPIO
 *    b. 调用Drv_ADS1278_InitDefault()或Drv_ADS1278_Init()
 *    c. 调用Drv_ADS1278_Start()启动采样
 *
 * 2. 数据读取（中断模式）：
 *    a. 在DRDY ISR中自动触发DMA读取
 *    b. 在DMA完成ISR中调用Drv_ADS1278_ConvertData()
 *    c. 应用层调用Drv_ADS1278_GetData()获取数据
 *
 * 3. 数据读取（轮询模式）：
 *    a. 调用Drv_ADS1278_ReadBlocking()等待并读取数据
 *
 * 4. Ping-Pong机制：
 *    - Ping缓冲：DMA正在写入
 *    - Pong缓冲：CPU可以读取
 *    - 每次DMA完成后自动切换
 *
 * 5. 数据格式：
 *    - SPI模式：24位×8通道=24字节/帧
 *    - 传输格式：MSB先行，2's补码
 *    - 电压范围：±VREF（通常±2.5V）
 *
 * 6. 中断配置：
 *    - DRDY: GPIO16 → XINT1 (PIE1.4)
 *    - DMA CH1/CH2: PIE7.1, PIE7.2
 *
 * 7. 性能参数：
 *    - HR模式: 52.7kSPS, 19us/样本
 *    - LP模式: 105kSPS, 9.5us/样本
 *    - LS模式: 26.4kSPS, 38us/样本
 */

#endif /* DRV_ADS1278_H_ */

