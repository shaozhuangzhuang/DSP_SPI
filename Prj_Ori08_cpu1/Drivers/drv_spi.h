#ifndef DRV_SPI_H_
#define DRV_SPI_H_

#include "F28x_Project.h"
#include "drv_dma.h" // 新增：包含DMA驱动头文件

//****************************************************************************
//
// 文件名: drv_spi.h
// 功能: SPIA与SPIB驱动头文件
// 说明:
//   - SPIA: 主模式，GPIO54-57，16位，2MHz，Mode 0 (CPOL=0, CPHA=0)
//   - SPIB: 主模式，GPIO24-27，8位，500kHz，Mode 1 (CPOL=0, CPHA=1)
//
//****************************************************************************

//==========================================================
// 宏定义
//==========================================================
#define SPI_FRAME_SIZE          4       // 每帧发送字数
#define SPI_RX_BUFFER_SIZE      16      // 接收缓冲区大小

// SPI波特率配置（LSPCLK=50MHz，LSPCLKDIV=2，SYSCLK/4分频）
#define SPIA_BAUDRATE           24      // 50MHz/(24+1) = 2MHz
#define SPIB_BAUDRATE           49      // 50MHz/(49+1) = 1MHz

// SPI数据位宽配置
#define SPIA_CHAR_LENGTH        15      // SPIA: 16位 (15表示16位)
#define SPIB_CHAR_LENGTH        7       // SPIB: 8位 (7表示8位)

//==========================================================
// SPI模式定义
//==========================================================
#define SPI_MODE_MASTER         1       // 主模式
#define SPI_MODE_SLAVE          0       // 从模式

#define SPI_CLK_POLARITY_LOW    0       // 时钟空闲为低
#define SPI_CLK_POLARITY_HIGH   1       // 时钟空闲为高

#define SPI_CLK_PHASE_NORMAL    0       // 无延迟
#define SPI_CLK_PHASE_DELAY     1       // 半周期延迟

// SPI时钟模式配置
#define SPIA_CLK_POLARITY       SPI_CLK_POLARITY_LOW    // SPIA: Mode 0 (CPOL=0)
#define SPIA_CLK_PHASE          SPI_CLK_PHASE_NORMAL    // SPIA: Mode 0 (CPHA=0)

// ✅ AD5754R正确配置：SPI Mode 0 (CPOL=0, CPHA=0)
// 根据实际测试：SCLK空闲为低，数据在上升沿（第1个边沿）采样
#define SPIB_CLK_POLARITY       SPI_CLK_POLARITY_LOW    // SPIB: Mode 0 (CPOL=0) - SCLK空闲为低
#define SPIB_CLK_PHASE          SPI_CLK_PHASE_NORMAL    // SPIB: Mode 0 (CPHA=0) - 数据在上升沿采样

//==========================================================
// 函数原型
//==========================================================

// 初始化函数（保持原有函数名兼容性）
void InitSpiGpios(void);          // 初始化SPIA和SPIB的GPIO引脚
void InitSpiInterrupts(void);     // 初始化SPI中断
void InitSpiModules(void);        // 初始化SPIA和SPIB模块

// SPIA独立函数（使用Drv_前缀避免与原有代码冲突）
void Drv_InitSpiaGpio(void);      // 初始化SPIA的GPIO
void Drv_InitSpiaModule(void);    // 初始化SPIA模块

// SPIB独立函数（使用Drv_前缀避免与原有代码冲突）
void Drv_InitSpibGpio(void);      // 初始化SPIB的GPIO
void Drv_InitSpibModule(void);    // 初始化SPIB模块

// 发送函数（主循环或定时器中调用）
void SpiaSendData(Uint16 *data, Uint16 length);
void SpibSendData(Uint16 *data, Uint16 length);

// 接收数据获取函数
Uint16 SpiaGetRxCount(void);
Uint16 SpibGetRxCount(void);

//==========================================================
// SPIB简化发送函数（参考spi_loopback）
//==========================================================
void spib_xmit(Uint16 a);

//==========================================================
// 新增SPIB发送函数声明
//==========================================================
void SpibSendByte(Uint16 byte);                           // 基础单字节发送函数
void SpibSendByte_Fast(Uint16 byte);                     // 高效单字节发送函数
void SpibSendByte_Continuous(Uint16 byte);                // 连续发送函数
void AD5754_Send24BitCommand_Simple(uint32_t command);   // 简化的24位命令发送函数
void SpibSendDataBlock(Uint16 *data, Uint16 length);      // 批量数据发送函数

//==========================================================
// AD5754R专用宏定义
//==========================================================
#define AD5754_SYNC_LOW()     GpioDataRegs.GPACLEAR.bit.GPIO27 = 1
#define AD5754_SYNC_HIGH()    GpioDataRegs.GPASET.bit.GPIO27 = 1
#define AD5754_SYNC_IS_LOW()  (GpioDataRegs.GPADAT.bit.GPIO27 == 0)

// AD5754R指令格式宏定义
#define AD5754_WRITE_CMD      0x000000    // 写命令
#define AD5754_READ_CMD       0x800000    // 读命令

// 寄存器选择
#define AD5754_REG_DAC        0x00        // DAC寄存器
#define AD5754_REG_RANGE      0x01        // 输出范围选择寄存器
#define AD5754_REG_POWER      0x02        // 电源控制寄存器
#define AD5754_REG_CONTROL    0x03        // 控制寄存器

// DAC通道地址
#define AD5754_CH_A           0x00        // 通道A
#define AD5754_CH_B           0x01        // 通道B
#define AD5754_CH_C           0x02        // 通道C
#define AD5754_CH_D           0x03        // 通道D
#define AD5754_CH_ALL         0x04        // 所有通道

//==========================================================
// AD5754R通信测试函数原型
//==========================================================
void AD5754_Send24BitCommand(uint32_t command);
uint32_t AD5754_Send24BitCommand_WithRead(uint32_t command);

// FIFO批量传输函数（优化版本）
void AD5754_Send24BitCommand_FIFO(uint32_t command);
uint32_t AD5754_Send24BitCommand_WithRead_FIFO(uint32_t command);

void AD5754R_Init(void);                    // AD5754R初始化（只执行一次）
void Test_AD5754R_Communication(void);      // 周期性通信测试（只读取）
void Test_AD5754R_PowerRegister(void);      // 电源寄存器写入和读回验证（只执行一次）
void Test_AD5754R_DACRegister(void);        // DAC寄存器循环测试

//==========================================================
// AD5754R DMA专用函数 (新增)
//==========================================================
bool AD5754_SendCommand_DMA(uint32_t command, uint32_t timeout_ms);
bool AD5754_ReadCommand_DMA(uint32_t command, uint32_t* response, uint32_t timeout_ms);
void Test_AD5754R_DACRegister_DMA(void);

// AD5754R通信测试变量声明
extern volatile uint32_t ad5754_test_write_value;
extern volatile uint32_t ad5754_test_read_value;
extern volatile uint16_t ad5754_comm_test_pass;
extern volatile uint16_t ad5754_initialized;

// 电源寄存器测试变量声明
extern volatile uint16_t ad5754_power_test_done;      // 电源寄存器测试完成标志
extern volatile uint32_t ad5754_power_readback;       // 电源寄存器读回值
extern volatile uint16_t ad5754_power_test_pass;      // 电源寄存器测试通过标志

// DAC寄存器测试变量声明
extern volatile uint32_t ad5754_dac_write_value;      // DAC写入值
extern volatile uint32_t ad5754_dac_readback_value;   // DAC读回值
extern volatile uint16_t ad5754_dac_test_pass;        // DAC测试通过标志
extern volatile uint16_t ad5754_dac_test_count;       // DAC测试计数器

// AD5754R初始化专用调试变量声明（不会被周期性通信覆盖）
extern volatile uint32_t ad5754_init_ctrl_cmd;      // 初始化时写入控制寄存器的命令
extern volatile uint32_t ad5754_init_power_cmd;     // 初始化时写入电源寄存器的命令
extern volatile uint32_t ad5754_init_power_readback; // 初始化时读回的电源寄存器值
extern volatile uint16_t ad5754_init_success;        // 初始化成功标志（1=成功，0=失败）

//==========================================================
// DMA模式调试变量声明（与FIFO模式分离）
//==========================================================
extern volatile uint16_t debug_dma_rx_word1;  // DMA接收的第1个字
extern volatile uint16_t debug_dma_rx_word2;  // DMA接收的第2个字
extern volatile uint16_t debug_dma_rx_word3;  // DMA接收的第3个字
extern volatile uint32_t debug_dma_result;    // DMA解析后的结果
extern volatile uint32_t debug_dma_isr_count; // DMA中断执行次数计数器

//==========================================================
// SPIB DMA 传输层函数 (新增)
//==========================================================
// void Drv_SPIB_DMA_Init(void);  // ⚠️ 已废弃：空函数，无实际作用
DMA_ErrorCode_t Drv_SPIB_TransmitReceive_DMA(Uint16* tx_data, Uint16* rx_data, Uint16 word_count, uint32_t timeout_ms);


#endif /* DRV_SPI_H_ */
