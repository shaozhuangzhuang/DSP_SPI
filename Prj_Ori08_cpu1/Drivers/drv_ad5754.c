/*******************************************************************************
 * 文件名：drv_ad5754.c
 * 描述：  AD5754 4通道16位DAC驱动实现
 * 作者：  Auto-generated
 * 日期：  2025-10-20
 ******************************************************************************/

#include "drv_ad5754.h"
#include "drv_spi.h"
#include "drv_dma.h"
#include "drv_dma_buffers.h"
#include "F28x_Project.h"
#include <math.h>

/******************************************************************************
 * 全局变量定义
 ******************************************************************************/

volatile bool g_ad5754_write_busy = false;      // 写入忙碌标志

// 内部状态变量
// static bool g_ad5754_initialized = false;  // 已注释：变量设置但未使用
static AD5754_Config_t g_ad5754_config;
static AD5754_WriteCompleteCallback_t g_ad5754_callback = NULL;

// 重传错误统计
static volatile Uint32 g_ad5754_retry_count = 0;
static volatile Uint32 g_ad5754_timeout_count = 0;

/******************************************************************************
 * GPIO控制函数
 ******************************************************************************/

/**
 * @brief  配置AD5754控制引脚GPIO
 * @note   硬件设计说明：
 *         - #LDAC直接接地（AGND）→ 写入后自动更新，无需GPIO控制
 *         - #CLR通过10kΩ上拉至DVCC → 禁用硬件清零，无需GPIO控制
 *         - #SYNC使用SPI CS (GPIO27) → 在drv_spi.c中配置
 */
void Drv_AD5754_GPIO_Config(void)
{
    // 硬件上AD5754的控制引脚配置：
    // - #LDAC使用GPIO55控制（本函数配置）
    // - #CLR通过10kΩ上拉至DVCC → 禁用硬件清零，无需GPIO控制
    // - #SYNC使用SPI CS (GPIO27) → 在drv_spi.c中配置
    
    EALLOW;
    
    // 配置GPIO55作为AD5754的#LDAC控制引脚
    // 1. 设置为GPIO功能（非外设复用）
    GPIO_SetupPinMux(55, GPIO_MUX_CPU1, 0);  // 0=GPIO模式
    
    // 2. 设置为输出方向，推挽输出模式
    GPIO_SetupPinOptions(55, GPIO_OUTPUT, GPIO_PUSHPULL);
    
    // 3. 初始化为高电平（#LDAC为低电平有效，高电平为禁用状态）
    //    后续可通过软件控制GPIO55来触发DAC更新
    GPIO_WritePin(55, 1);  // 设置为高电平
    
    EDIS;
    
    // SPI引脚（CLK/SIMO/SOMI/CS）在drv_spi.c的Drv_SPI_GPIO_Config()中统一配置
}

/**
 * @brief  控制AD5754的#LDAC引脚电平
 * @param  level - 0=低电平（触发DAC更新），1=高电平（禁用更新）
 * @note   #LDAC为低电平有效，下降沿触发所有DAC同步更新
 */
void Drv_AD5754_SetLDAC(Uint16 level)
{
    GPIO_WritePin(AD5754_LDAC_GPIO, level);
}

/**
 * @brief  触发所有DAC同步更新（通过LDAC脉冲）
 * @note   先拉低LDAC，等待t11（最小20ns），再拉高
 *         根据AD5754数据手册表4：t11=20ns（最小值）
 *         在200MHz CPU下，1条指令约5ns，故使用4条NOP即可满足20ns
 */
void Drv_AD5754_TriggerLDAC(void)
{
    // 拉低LDAC（产生下降沿）
    GPIO_WritePin(AD5754_LDAC_GPIO, 0);
    
    // 等待t11时间（最小20ns）
    asm(" NOP");
    asm(" NOP");
    asm(" NOP");
    asm(" NOP");
    
    // 拉高LDAC（结束脉冲，触发DAC更新）
    GPIO_WritePin(AD5754_LDAC_GPIO, 1);
}

/**
 * @brief  软件复位AD5754（通过控制寄存器）
 * @note   硬件上未连接#CLR引脚，使用软件复位命令代替
 */
void Drv_AD5754_SoftReset(void)
{
    // 方法1：写入特定的控制寄存器值执行软件复位
    // 根据AD5754数据手册，可通过写入控制寄存器实现软件清零
    
    // 清零所有DAC到中点值（双极性模式下为0V）
    Uint32 cmd = AD5754_WR_DAC(AD5754_ADDR_DAC_ALL, 0x8000);
    Drv_AD5754_SendCommand(cmd, 100);
    
    // 等待输出稳定
    DELAY_US(100);
}

/******************************************************************************
 * 命令打包与发送函数
 ******************************************************************************/

/**
 * @brief  将24位命令打包到16位缓冲区
 * @param  cmd - 24位命令字
 * @param  buffer - 输出缓冲区（至少2个16位字）
 * @return 打包后的字数（通常为2）
 * @note   24位命令拆分为：[高16位][低8位+padding]
 */
static Uint16 Drv_AD5754_PackCommand(Uint32 cmd, Uint16 *buffer)
{
    // 24位命令拆分为2个16位字
    // Word0: [CMD/RW:4][ADDR:4][DATA_H:8]
    // Word1: [DATA_L:8][Padding:8]
    
    buffer[0] = (Uint16)((cmd >> 8) & 0xFFFF);   // 高16位
    buffer[1] = (Uint16)((cmd << 8) & 0xFF00);   // 低8位左移
    
    return 2;  // 返回字数
}

/**
 * @brief  从16位缓冲区解包24位响应
 * @param  buffer - 输入缓冲区（至少2个16位字）
 * @return 解包后的24位数据
 */
static Uint32 Drv_AD5754_UnpackResponse(Uint16 *buffer)
{
    Uint32 response;
    
    // 重组24位响应
    response = ((Uint32)buffer[0] << 8) | ((Uint32)(buffer[1] >> 8) & 0xFF);
    
    return response;
}

/**
 * @brief  发送24位命令字（阻塞模式）
 * @param  cmd - 24位命令字
 * @param  timeout_ms - 超时时间（毫秒）
 * @return bool - true=成功，false=超时或失败
 */
bool Drv_AD5754_SendCommand(Uint32 cmd, Uint32 timeout_ms)
{
    Uint16 *tx_buffer = Drv_DMA_GetDacTxBuffer();
    Uint16 word_count;
    DMA_Error_t dma_err;
    Uint32 timeout_us = timeout_ms * 1000;
    Uint16 retry = 0;
    const Uint16 MAX_RETRY = 3;
    
    // 打包命令
    word_count = Drv_AD5754_PackCommand(cmd, tx_buffer);
    
    while(retry < MAX_RETRY) {
        // 清空SPI FIFO
        Drv_SPI_ClearFIFO(SPI_MODULE_B);
        
        // 重新配置DMA地址
        dma_err = Drv_DMA_ReConfigAddr(DMA_CH5, 
                                        tx_buffer, 
                                        (void *)&SpibRegs.SPITXBUF, 
                                        word_count - 1);  // BURST_SIZE = word_count - 1
        if(dma_err != DMA_ERROR_NONE) {
            g_ad5754_timeout_count++;
            retry++;
            continue;
        }
        
        // 清除DMA完成标志
        g_dma_ch5_done = false;
        g_ad5754_write_busy = true;
        
        // 拉低CS
        Drv_SPI_SetCS(SPI_MODULE_B, true);
        DELAY_US(1);  // CS建立时间
        
        // 启动DMA传输
        Drv_DMA_Start(DMA_CH5);
        
        // 等待DMA完成
        if(Drv_DMA_WaitComplete(DMA_CH5, timeout_us)) {
            // 成功
            DELAY_US(1);  // CS保持时间
            Drv_SPI_SetCS(SPI_MODULE_B, false);
            g_ad5754_write_busy = false;
            return true;
        } else {
            // 超时，重试
            Drv_SPI_SetCS(SPI_MODULE_B, false);
            g_ad5754_timeout_count++;
            g_ad5754_retry_count++;
            retry++;
            DELAY_US(10);  // 重试前延时
        }
    }
    
    // 超过最大重试次数
    g_ad5754_write_busy = false;
    return false;
}

/**
 * @brief  发送24位命令并接收响应（阻塞模式）
 * @param  cmd - 24位命令字
 * @param  response - 接收缓冲区指针（24位）
 * @param  timeout_ms - 超时时间（毫秒）
 * @return bool - true=成功，false=超时或失败
 */
bool Drv_AD5754_SendReceiveCommand(Uint32 cmd, Uint32 *response, Uint32 timeout_ms)
{
    Uint16 *tx_buffer = Drv_DMA_GetDacTxBuffer();
    Uint16 *rx_buffer = Drv_DMA_GetDacRxBuffer();
    Uint16 word_count;
    DMA_Error_t dma_err;
    Uint32 timeout_us = timeout_ms * 1000;
    
    if(response == NULL) {
        return false;
    }
    
    // 打包命令
    word_count = Drv_AD5754_PackCommand(cmd, tx_buffer);
    
    // 清空SPI FIFO
    Drv_SPI_ClearFIFO(SPI_MODULE_B);
    
    // 配置DMA TX
    dma_err = Drv_DMA_ReConfigAddr(DMA_CH5, 
                                    tx_buffer, 
                                    (void *)&SpibRegs.SPITXBUF, 
                                    word_count - 1);
    if(dma_err != DMA_ERROR_NONE) {
        return false;
    }
    
    // 配置DMA RX
    dma_err = Drv_DMA_ReConfigAddr(DMA_CH6, 
                                    (void *)&SpibRegs.SPIRXBUF, 
                                    rx_buffer, 
                                    word_count - 1);
    if(dma_err != DMA_ERROR_NONE) {
        return false;
    }
    
    // 清除DMA完成标志
    g_dma_ch5_done = false;
    g_dma_ch6_done = false;
    g_ad5754_write_busy = true;
    
    // 拉低CS
    Drv_SPI_SetCS(SPI_MODULE_B, true);
    DELAY_US(1);
    
    // 启动DMA传输（RX + TX）
    Drv_DMA_StartPair(DMA_CH6, DMA_CH5);
    
    // 等待DMA完成
    bool tx_ok = Drv_DMA_WaitComplete(DMA_CH5, timeout_us);
    bool rx_ok = Drv_DMA_WaitComplete(DMA_CH6, timeout_us);
    
    // 拉高CS
    DELAY_US(1);
    Drv_SPI_SetCS(SPI_MODULE_B, false);
    g_ad5754_write_busy = false;
    
    if(tx_ok && rx_ok) {
        // 解包响应
        *response = Drv_AD5754_UnpackResponse(rx_buffer);
        return true;
    } else {
        g_ad5754_timeout_count++;
        return false;
    }
}

/******************************************************************************
 * 初始化函数
 ******************************************************************************/

/**
 * @brief  初始化AD5754
 */
bool Drv_AD5754_Init(AD5754_Config_t *config)
{
    Uint16 i;
    
    if(config == NULL) {
        return false;
    }
    
    // 保存配置
    g_ad5754_config = *config;
    
    // 1. 配置GPIO（当前硬件版本无需额外配置，SPI引脚已在drv_spi中配置）
    Drv_AD5754_GPIO_Config();
    
    // 2. 软件复位（清零所有DAC）
    Drv_AD5754_SoftReset();
    
    // 3. 等待内部基准稳定
    DELAY_US(AD5754_TVREF_SETTLING_MS * 1000);
    
    // 4. 写控制寄存器
    if(!Drv_AD5754_WriteCtrlReg(config->ctrl_reg, 100)) {
        return false;
    }
    
    // 4. 配置各通道输出范围
    for(i = 0; i < AD5754_CHANNELS; i++) {
        if(!Drv_AD5754_WriteRangeReg(i, config->range[i], 100)) {
            return false;
        }
    }
    
    // 5. 写入中点值到所有DAC
    if(!Drv_AD5754_WriteChannelBlocking(AD5754_ADDR_DAC_ALL, 0x8000, 100)) {
        return false;
    }
    
    // 6. 使能所有通道输出
    if(!Drv_AD5754_WritePwrReg(config->pwr_reg, 100)) {
        return false;
    }
    
    // 7. 由于#LDAC直接接地，DAC输出会在每次写入后自动更新
    //    无需额外的LDAC触发操作
    
    // g_ad5754_initialized = true;  // 已注释：变量未使用
    
    return true;
}

/**
 * @brief  使用默认配置初始化AD5754
 */
bool Drv_AD5754_InitDefault(void)
{
    AD5754_Config_t config;
    Uint16 i;
    
    // 默认配置：±10V范围、SDO使能、所有通道上电
    for(i = 0; i < AD5754_CHANNELS; i++) {
        config.range[i] = AD5754_RANGE_NEG_10_10V;  // ±10V
    }
    
    config.ctrl_reg = AD5754_CTRL_DEFAULT;
    config.pwr_reg = AD5754_PWR_ALL_WITH_REF;  // 上电所有通道+内部基准
    // 注意：硬件上#LDAC已接地，写入后自动更新
    
    return Drv_AD5754_Init(&config);
}

/******************************************************************************
 * 写入函数
 ******************************************************************************/

/**
 * @brief  写单通道DAC值（阻塞模式）
 */
bool Drv_AD5754_WriteChannelBlocking(Uint16 channel, Uint16 value, Uint32 timeout_ms)
{
    Uint32 cmd;
    
    if(channel > AD5754_ADDR_DAC_ALL) {
        return false;
    }
    
    // 构建写并更新命令
    cmd = AD5754_WR_DAC(channel, value);
    
    // 发送命令
    return Drv_AD5754_SendCommand(cmd, timeout_ms);
}

/**
 * @brief  写单通道DAC值（非阻塞模式）
 */
bool Drv_AD5754_WriteChannel(Uint16 channel, Uint16 value)
{
    if(g_ad5754_write_busy) {
        return false;  // 忙碌中
    }
    
    return Drv_AD5754_WriteChannelBlocking(channel, value, 10);
}

/**
 * @brief  批量写入所有通道（阻塞模式）
 */
bool Drv_AD5754_WriteAllChannelsBlocking(Uint16 *values, Uint32 timeout_ms)
{
    Uint16 i;
    
    if(values == NULL) {
        return false;
    }
    
    // 逐通道写入（使用WRITE_UPDATE命令，写入后立即更新）
    for(i = 0; i < AD5754_CHANNELS; i++) {
        if(!Drv_AD5754_WriteChannelBlocking(i, values[i], timeout_ms)) {
            return false;
        }
    }
    
    return true;
}

/**
 * @brief  批量写入所有通道（非阻塞模式）
 */
bool Drv_AD5754_WriteAllChannels(Uint16 *values)
{
    if(g_ad5754_write_busy) {
        return false;
    }
    
    return Drv_AD5754_WriteAllChannelsBlocking(values, 50);
}

/******************************************************************************
 * 读取函数
 ******************************************************************************/

/**
 * @brief  读取DAC寄存器值（阻塞模式）
 * @note   AD5754读取需要两次SPI传输：
 *         1. 第一次：发送读命令
 *         2. 第二次：发送NOP，在SDO上读回数据
 */
bool Drv_AD5754_ReadChannelBlocking(Uint16 channel, Uint16 *value, Uint32 timeout_ms)
{
    Uint32 read_cmd, nop_cmd;
    Uint32 response;
    
    if(channel > AD5754_ADDR_DAC_D || value == NULL) {
        return false;
    }
    
    // 第一次传输：发送读命令
    read_cmd = AD5754_RD_DAC(channel);
    if(!Drv_AD5754_SendCommand(read_cmd, timeout_ms)) {
        return false;
    }
    
    DELAY_US(10);  // 等待芯片准备数据
    
    // 第二次传输：发送NOP，同时读回SDO数据
    nop_cmd = AD5754_CMD_NOP;
    if(Drv_AD5754_SendReceiveCommand(nop_cmd, &response, timeout_ms)) {
        *value = (Uint16)(response & 0xFFFF);
        return true;
    }
    
    return false;
}

/**
 * @brief  读取控制寄存器（阻塞模式）
 * @note   两次传输：读命令 + NOP
 */
bool Drv_AD5754_ReadCtrlRegBlocking(Uint16 *value, Uint32 timeout_ms)
{
    Uint32 read_cmd, nop_cmd;
    Uint32 response;
    
    if(value == NULL) {
        return false;
    }
    
    // 第一次：发送读命令
    read_cmd = AD5754_RD_CONTROL;
    if(!Drv_AD5754_SendCommand(read_cmd, timeout_ms)) {
        return false;
    }
    
    DELAY_US(10);
    
    // 第二次：发送NOP并读回数据
    nop_cmd = AD5754_CMD_NOP;
    if(Drv_AD5754_SendReceiveCommand(nop_cmd, &response, timeout_ms)) {
        *value = (Uint16)(response & 0xFFFF);
        return true;
    }
    
    return false;
}

/******************************************************************************
 * 配置寄存器写入函数
 ******************************************************************************/

/**
 * @brief  写控制寄存器
 */
bool Drv_AD5754_WriteCtrlReg(Uint16 value, Uint32 timeout_ms)
{
    Uint32 cmd = AD5754_WR_CONTROL(value);
    return Drv_AD5754_SendCommand(cmd, timeout_ms);
}

/**
 * @brief  写输出范围寄存器
 */
bool Drv_AD5754_WriteRangeReg(Uint16 channel, AD5754_Range_t range, Uint32 timeout_ms)
{
    Uint32 cmd;
    
    if(channel >= AD5754_CHANNELS) {
        return false;
    }
    
    // 构建范围寄存器命令（REG=001, A=通道号, Data=范围码）
    cmd = AD5754_WR_RANGE(channel, range);
    return Drv_AD5754_SendCommand(cmd, timeout_ms);
}

/**
 * @brief  写电源控制寄存器
 */
bool Drv_AD5754_WritePwrReg(Uint16 value, Uint32 timeout_ms)
{
    Uint32 cmd = AD5754_WR_POWER(value);
    return Drv_AD5754_SendCommand(cmd, timeout_ms);
}

/******************************************************************************
 * 电压转换函数
 ******************************************************************************/

/**
 * @brief  电压到DAC码值转换
 */
Uint16 Drv_AD5754_VoltageToDacCode(float voltage, AD5754_Range_t range)
{
    float v_min, v_max, v_range;
    int32 dac_code;
    
    // 根据范围确定电压范围
    switch(range) {
        case AD5754_RANGE_0_5V:
            v_min = 0.0f;
            v_max = 5.0f;
            break;
        case AD5754_RANGE_0_10V:
            v_min = 0.0f;
            v_max = 10.0f;
            break;
        case AD5754_RANGE_0_10V_8:
            v_min = 0.0f;
            v_max = 10.8f;
            break;
        case AD5754_RANGE_NEG_5_5V:
            v_min = -5.0f;
            v_max = 5.0f;
            break;
        case AD5754_RANGE_NEG_10_10V:
            v_min = -10.0f;
            v_max = 10.0f;
            break;
        case AD5754_RANGE_NEG_10_8_10_8V:
            v_min = -10.8f;
            v_max = 10.8f;
            break;
        default:
            v_min = 0.0f;
            v_max = 10.0f;
            break;
    }
    
    v_range = v_max - v_min;
    
    // 限幅
    if(voltage < v_min) voltage = v_min;
    if(voltage > v_max) voltage = v_max;
    
    // 转换为DAC码值
    dac_code = (int32)(((voltage - v_min) / v_range) * 65535.0f + 0.5f);
    
    // 限制在0-65535范围内
    if(dac_code < 0) dac_code = 0;
    if(dac_code > 65535) dac_code = 65535;
    
    return (Uint16)dac_code;
}

/**
 * @brief  DAC码值到电压转换
 */
float Drv_AD5754_DacCodeToVoltage(Uint16 dac_code, AD5754_Range_t range)
{
    float v_min, v_max, v_range;
    float voltage;
    
    // 根据范围确定电压范围
    switch(range) {
        case AD5754_RANGE_0_5V:
            v_min = 0.0f;
            v_max = 5.0f;
            break;
        case AD5754_RANGE_0_10V:
            v_min = 0.0f;
            v_max = 10.0f;
            break;
        case AD5754_RANGE_0_10V_8:
            v_min = 0.0f;
            v_max = 10.8f;
            break;
        case AD5754_RANGE_NEG_5_5V:
            v_min = -5.0f;
            v_max = 5.0f;
            break;
        case AD5754_RANGE_NEG_10_10V:
            v_min = -10.0f;
            v_max = 10.0f;
            break;
        case AD5754_RANGE_NEG_10_8_10_8V:
            v_min = -10.8f;
            v_max = 10.8f;
            break;
        default:
            v_min = 0.0f;
            v_max = 10.0f;
            break;
    }
    
    v_range = v_max - v_min;
    
    // 转换为电压值
    voltage = v_min + (((float)dac_code / 65535.0f) * v_range);
    
    return voltage;
}

/**
 * @brief  设置单通道输出电压（物理电压值）
 */
bool Drv_AD5754_SetVoltage(Uint16 channel, float voltage, Uint32 timeout_ms)
{
    Uint16 dac_code;
    
    if(channel >= AD5754_CHANNELS) {
        return false;
    }
    
    // 转换电压到DAC码值
    dac_code = Drv_AD5754_VoltageToDacCode(voltage, g_ad5754_config.range[channel]);
    
    // 写入DAC
    return Drv_AD5754_WriteChannelBlocking(channel, dac_code, timeout_ms);
}

/******************************************************************************
 * 回调与状态函数
 ******************************************************************************/

/**
 * @brief  注册写完成回调函数
 */
void Drv_AD5754_RegisterCallback(AD5754_WriteCompleteCallback_t callback)
{
    g_ad5754_callback = callback;
}

/**
 * @brief  检查AD5754是否忙碌
 */
bool Drv_AD5754_IsBusy(void)
{
    return g_ad5754_write_busy;
}

/******************************************************************************
 * DMA完成回调函数（在DMA ISR中调用）
 ******************************************************************************/

/**
 * @brief  DMA CH5完成回调（SPIB TX完成）
 * @note   在drv_dma.c的DMA_CH5_ISR中注册并调用
 */
void Drv_AD5754_DMA_TxCallback(void)
{
    // 拉高CS（如果需要在ISR中处理）
    // 注意：当前实现在阻塞函数中处理CS
    
    // 调用用户回调
    if(g_ad5754_callback != NULL) {
        g_ad5754_callback();
    }
}

/******************************************************************************
 * 通信测试函数（用于验证SPIB与AD5754通信是否正常）
 ******************************************************************************/

// 测试结果全局变量（方便通过CCS Watch查看）
volatile Uint16 g_ad5754_test_status = 0;      // 0=未测试, 1=进行中, 2=通过, 3=失败
volatile Uint16 g_ad5754_test_step = 0;        // 当前测试步骤
volatile Uint16 g_ad5754_test_write_val = 0;   // 写入值
volatile Uint16 g_ad5754_test_read_val = 0;    // 读回值
volatile Uint16 g_ad5754_test_error_code = 0;  // 错误码: 0=无错误, 1=写失败, 2=读失败, 3=数据不匙配

/**
 * @brief  SPI轮询发送单个16位字
 */
static Uint16 SPI_TransferWord(Uint16 data)
{
    Uint16 rx_data;
    
    // 等待TX FIFO有空间
    while(SpibRegs.SPIFFTX.bit.TXFFST >= 16);
    
    // 写入数据
    SpibRegs.SPITXBUF = data;
    
    // 等待RX FIFO有数据
    while(SpibRegs.SPIFFRX.bit.RXFFST == 0);
    
    // 读取数据
    rx_data = SpibRegs.SPIRXBUF;
    
    return rx_data;
}

/**
 * @brief  SPI轮询发送24位命令（不使用DMA）
 */
static bool SPI_SendCommand_Polling(Uint32 cmd)
{
    Uint16 word_hi, word_lo;
    
    // 打包命令：24位拆成2个16位字
    word_hi = (Uint16)((cmd >> 8) & 0xFFFF);   // 高16位
    word_lo = (Uint16)((cmd << 8) & 0xFF00);   // 低8位左移
    
    // 拉低CS
    GpioDataRegs.GPACLEAR.bit.GPIO27 = 1;
    asm(" NOP"); asm(" NOP"); asm(" NOP"); asm(" NOP");
    
    // 发送两个字
    SPI_TransferWord(word_hi);
    SPI_TransferWord(word_lo);
    
    // 拉高CS
    asm(" NOP"); asm(" NOP"); asm(" NOP"); asm(" NOP");
    GpioDataRegs.GPASET.bit.GPIO27 = 1;
    
    return true;
}

/**
 * @brief  SPI轮询发送并接收（不使用DMA）
 */
static bool SPI_SendReceive_Polling(Uint32 cmd, Uint32 *response)
{
    Uint16 word_hi, word_lo;
    Uint16 rx_hi, rx_lo;
    
    if(response == NULL) {
        return false;
    }
    
    // 打包命令
    word_hi = (Uint16)((cmd >> 8) & 0xFFFF);
    word_lo = (Uint16)((cmd << 8) & 0xFF00);
    
    // 拉低CS
    GpioDataRegs.GPACLEAR.bit.GPIO27 = 1;
    asm(" NOP"); asm(" NOP"); asm(" NOP"); asm(" NOP");
    
    // 发送并接收
    rx_hi = SPI_TransferWord(word_hi);
    rx_lo = SPI_TransferWord(word_lo);
    
    // 拉高CS
    asm(" NOP"); asm(" NOP"); asm(" NOP"); asm(" NOP");
    GpioDataRegs.GPASET.bit.GPIO27 = 1;
    
    // 解包响应
    *response = ((Uint32)rx_hi << 8) | ((Uint32)(rx_lo >> 8) & 0xFF);
    
    return true;
}

/**
 * @brief  测试控制寄存器读写（使用SPI轮询方式）
 * @return bool - true=通过, false=失败
 */
bool Drv_AD5754_TestCtrlReg(void)
{
    Uint32 write_cmd, read_cmd, nop_cmd;
    Uint32 response;
    Uint16 write_val = AD5754_CTRL_DEFAULT;  // 0x0004
    Uint16 read_val = 0;
    
    g_ad5754_test_step = 1;
    g_ad5754_test_write_val = write_val;
    
    // 第一步：写入控制寄存器
    write_cmd = AD5754_WR_CONTROL(write_val);
    if(!SPI_SendCommand_Polling(write_cmd)) {
        g_ad5754_test_error_code = 1;
        return false;
    }
    
    // 第二步：发送读命令
    read_cmd = AD5754_RD_CONTROL;
    if(!SPI_SendCommand_Polling(read_cmd)) {
        g_ad5754_test_error_code = 1;
        return false;
    }
    
    // 第三步：发送NOP并读回数据
    nop_cmd = AD5754_CMD_NOP;
    if(!SPI_SendReceive_Polling(nop_cmd, &response)) {
        g_ad5754_test_error_code = 2;
        return false;
    }
    
    read_val = (Uint16)(response & 0xFFFF);
    g_ad5754_test_read_val = read_val;
    
    // 比较结果
    if((read_val & 0x0F) != (write_val & 0x0F)) {
        g_ad5754_test_error_code = 3;
        return false;
    }
    
    g_ad5754_test_error_code = 0;
    return true;
}

/**
 * @brief  测试单个DAC通道寄存器读写（使用SPI轮询方式）
 * @param  channel - 通道号(0-3)
 * @param  test_val - 测试值
 * @return bool - true=通过, false=失败
 */
bool Drv_AD5754_TestDacChannel(Uint16 channel, Uint16 test_val)
{
    Uint32 write_cmd, read_cmd, nop_cmd;
    Uint32 response;
    Uint16 read_val = 0;
    
    if(channel > 3) {
        return false;
    }
    
    g_ad5754_test_step = 10 + channel;
    g_ad5754_test_write_val = test_val;
    
    // 第一步：写入DAC寄存器
    write_cmd = AD5754_WR_DAC(channel, test_val);
    if(!SPI_SendCommand_Polling(write_cmd)) {
        g_ad5754_test_error_code = 1;
        return false;
    }
    
    // 第二步：发送读命令
    read_cmd = AD5754_RD_DAC(channel);
    if(!SPI_SendCommand_Polling(read_cmd)) {
        g_ad5754_test_error_code = 1;
        return false;
    }
    
    // 第三步：发送NOP并读回数据
    nop_cmd = AD5754_CMD_NOP;
    if(!SPI_SendReceive_Polling(nop_cmd, &response)) {
        g_ad5754_test_error_code = 2;
        return false;
    }
    
    read_val = (Uint16)(response & 0xFFFF);
    g_ad5754_test_read_val = read_val;
    
    // 比较结果
    if(read_val != test_val) {
        g_ad5754_test_error_code = 3;
        return false;
    }
    
    g_ad5754_test_error_code = 0;
    return true;
}

/**
 * @brief  快速通信测试（推荐用于初次验证）
 * @return bool - true=通过, false=失败
 * @note   测试流程：
 *         1. 测试控制寄存器读写
 *         2. 测试通道A写入中点值(0x8000)
 *         3. 测试通道A写入满量程(0xFFFF)
 *         4. 测试通道A写入零值(0x0000)
 */
bool Drv_AD5754_TestCommunication(void)
{
    g_ad5754_test_status = 1;  // 测试进行中
    g_ad5754_test_error_code = 0;
    
    // 测试1: 控制寄存器
    if(!Drv_AD5754_TestCtrlReg()) {
        g_ad5754_test_status = 3;
        return false;
    }
    
    // 测试2: 通道A - 中点值
    if(!Drv_AD5754_TestDacChannel(0, 0x8000)) {
        g_ad5754_test_status = 3;
        return false;
    }
    
    // 测试3: 通道A - 满量程
    if(!Drv_AD5754_TestDacChannel(0, 0xFFFF)) {
        g_ad5754_test_status = 3;
        return false;
    }
    
    // 测试4: 通道A - 零值
    if(!Drv_AD5754_TestDacChannel(0, 0x0000)) {
        g_ad5754_test_status = 3;
        return false;
    }
    
    g_ad5754_test_status = 2;  // 测试通过
    return true;
}

/**
 * @brief  完整通信测试（测试所有通道和多个数据模式）
 * @return bool - true=通过, false=失败
 * @note   测试流程：
 *         1. 测试控制寄存器
 *         2. 测试所有通道(A/B/C/D)
 *         3. 每个通道测试5种数据模式：0x0000, 0x4000, 0x8000, 0xC000, 0xFFFF
 */
bool Drv_AD5754_TestCommunicationFull(void)
{
    Uint16 ch;
    Uint16 test_patterns[5] = {0x0000, 0x4000, 0x8000, 0xC000, 0xFFFF};
    Uint16 i;
    
    g_ad5754_test_status = 1;  // 测试进行中
    g_ad5754_test_error_code = 0;
    
    // 测试控制寄存器
    if(!Drv_AD5754_TestCtrlReg()) {
        g_ad5754_test_status = 3;
        return false;
    }
    
    // 测试所有通道
    for(ch = 0; ch < 4; ch++) {
        // 测试多种数据模式
        for(i = 0; i < 5; i++) {
            if(!Drv_AD5754_TestDacChannel(ch, test_patterns[i])) {
                g_ad5754_test_status = 3;
                return false;
            }
        }
    }
    
    g_ad5754_test_status = 2;  // 测试通过
    return true;
}

/******************************************************************************
 * End of File
 ******************************************************************************/

