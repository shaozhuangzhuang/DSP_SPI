/*******************************************************************************
 * 文件名：drv_ads1278.c
 * 描述：  ADS1278 8通道24位ADC驱动实现
 * 作者：  Auto-generated
 * 日期：  2025-10-20
 ******************************************************************************/

#include "drv_ads1278.h"
#include "drv_spi.h"
#include "drv_dma.h"
#include "drv_dma_buffers.h"
#include "F28x_Project.h"

/******************************************************************************
 * 内部函数声明
 ******************************************************************************/
static void Drv_ADS1278_DMA_RxCallback(void);

/******************************************************************************
 * 全局变量定义
 ******************************************************************************/

// Ping-Pong缓冲区状态
volatile bool g_ads1278_ping_active = true;      // true=使用Ping，false=使用Pong
volatile Uint16 g_ads1278_sample_count = 0;      // 采样计数

// 内部状态变量
static bool g_ads1278_initialized = false;        // 初始化标志
static bool g_ads1278_running = false;            // 采样运行标志
// static ADS1278_Config_t g_ads1278_config;      // 已注释：变量设置但未使用
static ADS1278_DataReadyCallback_t g_ads1278_callback = NULL; // DRDY回调

// 最新数据缓存
static volatile ADS1278_Data_t g_ads1278_latest_data;

/******************************************************************************
 * GPIO控制函数
 ******************************************************************************/

/**
 * @brief  配置ADS1278控制引脚GPIO
 */
void Drv_ADS1278_GPIO_Config(void)
{
    EALLOW;
    
    //==========================================================================
    // 配置DRDY输入引脚（GPIO57 - XINT2）
    //==========================================================================
    
    // DRDY (GPIO55) - 配置为GPIO输入
    GPIO_SetupPinMux(ADS1278_DRDY_GPIO, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(ADS1278_DRDY_GPIO, GPIO_INPUT, GPIO_PULLUP | GPIO_SYNC);
    GpioCtrlRegs.GPBQSEL2.bit.GPIO55 = 0;  // 同步到SYSCLKOUT（GPIO57在GPBQSEL2中）
    
    // 配置为XINT2（外部中断2）
    GPIO_SetupXINT2Gpio(ADS1278_DRDY_GPIO);
    
    EDIS;
    
    /* 
     * 注意：ADS1278的控制引脚已在PCB硬件上固定连接，无需软件配置：
     * - MODE[1:0] = 01 (硬件): MODE0接IOVDD, MODE1接GND → High Resolution模式
     * - FORMAT[2:0] = 001 (硬件): FORMAT0接IOVDD, FORMAT1/2接GND → SPI, TDM, Fixed
     * - SYNC: 硬件上拉至IOVDD → 连续采样模式
     * - PWDN: 硬件上拉至IOVDD → 正常工作
     * - CLKDIV: 硬件固定（27MHz不分频时接IOVDD）
     * - DIN: 硬件接GND（单片应用）
     * - CS: 硬件固定拉低（单片应用）
     */
}

/* 
 * 注意：以下硬件控制函数已删除，因为相应引脚已在PCB硬件上固定连接：
 * - Drv_ADS1278_SoftReset()  - PWDN引脚硬件上拉
 * - Drv_ADS1278_SetMode()    - MODE[1:0]引脚硬件固定为01
 * - Drv_ADS1278_SetFormat()  - FORMAT[2:0]引脚硬件固定为001
 * - Drv_ADS1278_SetClkDiv()  - CLKDIV引脚硬件固定
 * - Drv_ADS1278_Start()      - SYNC引脚硬件上拉，芯片自动连续采样
 * - Drv_ADS1278_Stop()       - SYNC引脚硬件上拉，芯片自动连续采样
 */

/******************************************************************************
 * 数据转换函数
 ******************************************************************************/

/**
 * @brief  从16位缓冲区转换24位ADC数据
 * @note   ADS1278 SPI格式：每通道3字节（MSB先行），8通道共24字节
 *         16位字长传输：12个16位字
 *         数据组织：[CH1_H16][CH1_L8+CH2_H8][CH2_L16]...[CH8_L8+padding]
 */
void Drv_ADS1278_ConvertData(Uint16 *src_buffer, ADS1278_Data_t *dest_data)
{
    Uint16 i, word_idx;
    Uint32 raw_data;
    
    if(src_buffer == NULL || dest_data == NULL) {
        return;
    }
    
    // ADS1278每通道24位，分为3个8位字节传输
    // 使用16位SPI字长时，每通道占用1.5个16位字
    // 实际上，ADS1278在SPI模式下，每次传输24字节（8通道×3字节）
    // 用16位字长传输需要12个16位字
    
    // 重组数据：每个通道由连续的3个字节组成
    for(i = 0; i < ADS1278_CHANNELS; i++) {
        word_idx = (i * 3) / 2;  // 计算起始字索引
        
        if((i * 3) % 2 == 0) {
            // 偶数通道：从偶数字节开始
            // [Word0_H8 Word0_L8] [Word1_H8 Word1_L8]
            // CH0: Word0_H8, Word0_L8, Word1_H8
            raw_data = ((Uint32)src_buffer[word_idx] << 8) | 
                       ((Uint32)(src_buffer[word_idx + 1] >> 8) & 0xFF);
        } else {
            // 奇数通道：从奇数字节开始
            // CH1: Word1_L8, Word2_H8, Word2_L8
            raw_data = ((Uint32)(src_buffer[word_idx] & 0xFF) << 16) | 
                       ((Uint32)src_buffer[word_idx + 1]);
        }
        
        // 符号扩展24位到32位
        dest_data->ch[i] = ADS1278_SIGN_EXTEND_24_TO_32(raw_data);
    }
    
    // 更新时间戳和标志
    dest_data->timestamp = g_ads1278_sample_count;
    dest_data->sample_id = g_ads1278_sample_count;
    dest_data->valid = true;
    dest_data->overflow = false;
    
    g_ads1278_sample_count++;
}

/**
 * @brief  获取当前采样率
 */
Uint32 Drv_ADS1278_GetSampleRate(ADS1278_Mode_t mode)
{
    switch(mode) {
        case ADS1278_MODE_LS:  return 26400;   // 26.4kSPS
        case ADS1278_MODE_HR:  return 52700;   // 52.7kSPS (默认)
        case ADS1278_MODE_LP:  return 105000;  // 105kSPS
        default:               return 52700;
    }
}

/******************************************************************************
 * 初始化函数
 ******************************************************************************/

/**
 * @brief  初始化ADS1278
 */
bool Drv_ADS1278_Init(ADS1278_Config_t *config)
{
    if(config == NULL) {
        return false;
    }
    
    // 保存配置（仅用于记录）
    // g_ads1278_config = *config;  // 已注释：变量未使用
    
    //========================================================================
    // 1. 配置DSP端GPIO（仅DRDY引脚，GPIO55）
    //========================================================================
    Drv_ADS1278_GPIO_Config();
    
    /* 
     * 注意：ADS1278芯片无寄存器需要配置！
     * 根据芯片手册："All operations are controlled directly by pins;
     * there are no registers to program."
     * 
     * 以下配置已通过PCB硬件固定：
     * - MODE[1:0] = 01 (硬件): High Resolution模式（52.7kSPS）
     * - FORMAT[2:0] = 001 (硬件): SPI模式
     * - SYNC: 硬件上拉 → 连续采样模式
     * - PWDN: 硬件上拉 → 正常工作
     * - CLKDIV: 硬件固定（27MHz不分频）
     * - CS: 硬件固定拉低 → 芯片始终选中
     */
    
    //========================================================================
    // v2.5变更：移除DMA回调注册（已移到Drv_DMA_Init()）
    //========================================================================
    // 旧代码（v2.4及之前）：
    // Drv_DMA_RegisterCallback(DMA_CH1, &Drv_ADS1278_DMA_RxCallback);
    //
    // 新设计（v2.5）：
    // DMA回调在Drv_DMA_Init()中统一注册，芯片驱动通过
    // Drv_ADS1278_GetDMACallback()提供回调函数指针
    
    //========================================================================
    // 2. 配置DRDY中断（XINT2）
    //========================================================================
    if(config->use_interrupt) {
        EALLOW;
        
        // 配置XINT2为下降沿触发
        XintRegs.XINT2CR.bit.POLARITY = 0;  // 下降沿
        XintRegs.XINT2CR.bit.ENABLE = 1;    // 使能XINT2
        
        // 使能PIE中断（PIE1.5）
        PieCtrlRegs.PIEIER1.bit.INTx5 = 1;  // 使能PIE组1的INT5
        IER |= M_INT1;                       // 使能CPU INT1
        
        EDIS;
    }
    
    //========================================================================
    // 3. 初始化内部状态
    //========================================================================
    g_ads1278_latest_data.valid = false;
    g_ads1278_ping_active = true;
    g_ads1278_sample_count = 0;
    g_ads1278_initialized = true;
    
    return true;
}

/**
 * @brief  使用默认配置初始化ADS1278
 */
bool Drv_ADS1278_InitDefault(void)
{
    ADS1278_Config_t config;
    
    config.mode = ADS1278_MODE_HR;      // High Resolution模式
    config.format = ADS1278_FORMAT_SPI; // SPI格式
    config.clkdiv = ADS1278_CLKDIV_1;   // 不分频
    config.use_interrupt = true;         // 使用DRDY中断
    
    return Drv_ADS1278_Init(&config);
}

/******************************************************************************
 * 数据读取函数
 ******************************************************************************/

/**
 * @brief  触发单次ADC读取（非阻塞模式）
 */
bool Drv_ADS1278_TriggerRead(void)
{
    Uint16 *target_buffer;
    DMA_Error_t dma_err;
    
    if(!g_ads1278_initialized) {
        return false;
    }
    
    // 检查DMA是否忙碌
    if(!g_dma_ch1_done || !g_dma_ch2_done) {
        return false;  // 上一次传输还未完成
    }
    
    // 选择Ping或Pong缓冲区
    target_buffer = g_ads1278_ping_active ? 
                    Drv_DMA_GetAdcPingBuffer() : 
                    Drv_DMA_GetAdcPongBuffer();
    
    // 清空SPI FIFO
    Drv_SPI_ClearFIFO(SPI_MODULE_A);
    
    //========================================================================
    // v2.5优化：仅更新目标地址（Ping-Pong切换），不重新配置整个通道
    //========================================================================
    // CH1：更新RX目标地址（切换Ping/Pong缓冲）
    dma_err = Drv_DMA_UpdateDestAddr(DMA_CH1, target_buffer);
    if(dma_err != DMA_ERROR_NONE) {
        return false;
    }
    
    // CH2：TX源地址和目标地址固定，无需更新
    // （dummy_tx → SPITXBUF，地址在ConfigChannel中已配置）
    
    // 清除DMA完成标志
    g_dma_ch1_done = false;
    g_dma_ch2_done = false;
    
    /* CS片选硬件固定拉低，无需软件控制 */
    // Drv_SPI_SetCS(SPI_MODULE_A, true);
    
    // 启动DMA传输（RX+TX配对）
    Drv_DMA_StartPair(DMA_CH1, DMA_CH2);
    
    return true;
}

/**
 * @brief  执行单次ADC读取（阻塞模式）
 */
bool Drv_ADS1278_ReadBlocking(ADS1278_Data_t *data, Uint32 timeout_ms)
{
    Uint32 timeout_us = timeout_ms * 1000;
    
    if(data == NULL) {
        return false;
    }
    
    // 触发读取
    if(!Drv_ADS1278_TriggerRead()) {
        return false;
    }
    
    // 等待DMA完成
    if(!Drv_DMA_WaitComplete(DMA_CH1, timeout_us)) {
        // CS片选硬件固定拉低，无需软件控制
        return false;  // 超时
    }
    
    if(!Drv_DMA_WaitComplete(DMA_CH2, timeout_us)) {
        // CS片选硬件固定拉低，无需软件控制
        return false;  // 超时
    }
    
    /* CS片选硬件固定拉低，无需软件控制 */
    // Drv_SPI_SetCS(SPI_MODULE_A, false);
    
    // 转换数据
    Uint16 *src_buffer = g_ads1278_ping_active ? 
                         Drv_DMA_GetAdcPingBuffer() : 
                         Drv_DMA_GetAdcPongBuffer();
    
    Drv_ADS1278_ConvertData(src_buffer, data);
    
    // 切换Ping-Pong缓冲区
    g_ads1278_ping_active = !g_ads1278_ping_active;
    
    return true;
}

/**
 * @brief  获取最新的ADC数据（非阻塞模式）
 */
bool Drv_ADS1278_GetData(ADS1278_Data_t *data)
{
    Uint16 gie_state;
    
    if(data == NULL) {
        return false;
    }
    
    // 临界区保护
    gie_state = __disable_interrupts();
    
    // 检查数据有效性
    if(!g_ads1278_latest_data.valid) {
        __restore_interrupts(gie_state);
        return false;
    }
    
    // 拷贝数据
    *data = g_ads1278_latest_data;
    
    // 清除有效标志
    g_ads1278_latest_data.valid = false;
    
    // 退出临界区
    __restore_interrupts(gie_state);
    
    return true;
}

/**
 * @brief  检查是否有新数据可用
 */
bool Drv_ADS1278_IsDataReady(void)
{
    return g_ads1278_latest_data.valid;
}

/**
 * @brief  清除数据有效标志
 */
void Drv_ADS1278_ClearDataReady(void)
{
    g_ads1278_latest_data.valid = false;
}

/**
 * @brief  注册DRDY回调函数
 */
void Drv_ADS1278_RegisterCallback(ADS1278_DataReadyCallback_t callback)
{
    g_ads1278_callback = callback;
}

/**
 * @brief  获取ADS1278的DMA回调函数指针
 * @return void* - DMA RX完成回调函数指针
 * @note   v2.5新增：供Drv_DMA_Init()注册DMA回调使用
 */
void* Drv_ADS1278_GetDMACallback(void)
{
    return (void*)&Drv_ADS1278_DMA_RxCallback;
}

/**
 * @brief  获取ADS1278状态
 */
void Drv_ADS1278_GetStatus(bool *is_running, Uint32 *sample_count)
{
    if(is_running != NULL) {
        *is_running = g_ads1278_running;
    }
    if(sample_count != NULL) {
        *sample_count = g_ads1278_sample_count;
    }
}

/******************************************************************************
 * 中断服务函数（DRDY中断）
 ******************************************************************************/

/**
 * @brief  DRDY中断服务函数（XINT2）
 * @note   在DRDY下降沿触发，启动SPI+DMA读取
 */
uint16_t ADS1278_Rec_Cnt = 0;
__interrupt void ADS1278_DRDY_ISR(void)
{
    ADS1278_Rec_Cnt++;
    // 触发ADC读取
    Drv_ADS1278_TriggerRead();

    // 调用用户回调（如果已注册）
    if(g_ads1278_callback != NULL) {
        g_ads1278_callback();
    }
    
    // 清除中断标志
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;
}

/******************************************************************************
 * DMA完成回调函数（在DMA ISR中调用）
 ******************************************************************************/

/**
 * @brief  DMA CH1完成回调（SPIA RX完成）
 * @note   在drv_dma.c的DMA_CH1_ISR中注册并调用
 */
static void Drv_ADS1278_DMA_RxCallback(void)
{
    Uint16 *src_buffer;
    
    /* CS片选硬件固定拉低，无需软件控制 */
    // Drv_SPI_SetCS(SPI_MODULE_A, false);
    
    // 选择刚刚接收数据的缓冲区
    src_buffer = g_ads1278_ping_active ? 
                 Drv_DMA_GetAdcPingBuffer() : 
                 Drv_DMA_GetAdcPongBuffer();
    
    // 转换数据到最新数据缓存
    Drv_ADS1278_ConvertData(src_buffer, (ADS1278_Data_t *)&g_ads1278_latest_data);
    
    // 切换Ping-Pong缓冲区
    g_ads1278_ping_active = !g_ads1278_ping_active;
}

/******************************************************************************
 * End of File
 ******************************************************************************/

