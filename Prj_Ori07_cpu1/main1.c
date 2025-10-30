//****************************************************************************
//
// 文件名: main1.c (CPU1主程序)
//
// 功能说明: 使用RAM_management方式实现双核共享RAM通信
//
// 改造说明:
//   - 移除了复杂的IPC消息队列机制
//   - 采用简单的IPC标志位同步 + 直接共享RAM访问
//   - 静态内存权限分配，无需动态请求
//   - 主循环轮询模式，无IPC中断
//
//****************************************************************************

#include "F28x_Project.h"
#include "F2837xD_Ipc_drivers.h"

// SPI驱动头文件
#include "Drivers/drv_spi.h"
#include "Drivers/drv_dma.h"
#include "Drivers/drv_ad5754.h"
#include "Drivers/drv_ads1278.h"
#include "Drivers/drv_watchdog.h"
#include "Framework/system_config.h"

// ===== 全局变量定义 =====
// 共享RAM数组定义（减少到24个元素）
uint16_t c1_r_array[24];       // CPU1只读，CPU2写入，映射到GS0（CPU2拥有）
uint16_t c1_r_w_array[24];     // CPU1读写，映射到GS1（CPU1拥有）
#pragma DATA_SECTION(c1_r_array,"SHARERAMGS0");
#pragma DATA_SECTION(c1_r_w_array,"SHARERAMGS1");

// 通信控制变量
uint16_t error = 0;            // 错误标志：数据验证失败时置1
uint16_t multiplier = 0;       // 乘数因子：0-255循环递增

// 20ms周期控制标志
volatile uint16_t flag_20ms_write = 0;  // 20ms写入标志

// SPI任务控制标志（20kHz处理）
volatile uint16_t flag_50us = 0;        // 50us任务标志（20kHz ADC数据处理）
volatile uint16_t flag_100ms = 0;       // 100ms任务标志（DAC更新）

// ADC数据缓存（方案B：仅保留最新数据）
static volatile ADS1278_Data_t g_latestAdc;        // 最新ADC数据
static volatile bool g_latestAdcValid = false;     // 数据有效标志

// 定时器控制变量（保留原有业务逻辑）
unsigned int IPC_Timer_1MS = 0;
// extern uint32_t SCIA_Counter;  // 未使用，已删除

// ===== 函数原型声明 =====
void System_Init(void);
interrupt void cpu_timer0_isr(void);

// RAM_management方式的数据交换函数
void Shared_Ram_dataWrite_c1(void);  // 向GS1写入数据
void Shared_Ram_dataRead_c1(void);   // 从GS0读取并验证数据

// SPI任务函数
void SPI_Task_50us(void);   // 50us任务：读取ADC数据（20kHz）
void SPI_Task_100ms(void);  // 100ms任务：更新DAC输出

//****************************************************************************
// 函数名: main
// 功能: CPU1主函数 - 初始化系统并执行双核共享RAM通信 IPC_Timer_1MS
//****************************************************************************
void main(void)
{
    // 系统初始化
    System_Init();

    // 初始化通信变量
    error = 0;
    multiplier = 0;
    flag_20ms_write = 0;

    // ===== 主循环 =====
    while(1)
    {
        // ===== 50us任务（20kHz ADC数据处理） =====
        if(flag_50us)
        {
            flag_50us = 0;
            SPI_Task_50us();  // 读取最新ADC数据
        }
        
        // ===== 20ms任务（IPC通信 + 喂狗） =====
        if(flag_20ms_write)
        {
            flag_20ms_write = 0;  // 清除标志
            
            // 更新乘数因子（循环0-255）
            if(multiplier++ > 255)
            {
                multiplier = 0;
            }
            
            // 写入GS1共享RAM
            Shared_Ram_dataWrite_c1();
            
            // 设置FLAG10，通知CPU2数据已准备好
            IPCLtoRFlagSet(IPC_FLAG10);
            
            // 喂狗（新增）
             Drv_Watchdog_Kick(); // <-- 【测试】在此处注释掉喂狗，以验证看门狗复位功能
        }
        
        // ===== 100ms任务（DAC更新） =====
        if(flag_100ms)
        {
            flag_100ms = 0;
            SPI_Task_100ms();  // 更新DAC输出
        }
        
        // ===== 原有IPC读取任务（保持不变） =====
        if(IPCRtoLFlagBusy(IPC_FLAG11) == 1)
        {
            // 读取CPU2写入的GS0数据并验证
            Shared_Ram_dataRead_c1();
            
            // ACK清除FLAG11
            IPCRtoLFlagAcknowledge(IPC_FLAG11);
        }
    }
}

//****************************************************************************
// 函数名: Shared_Ram_dataWrite_c1
// 功能: 向共享RAM写入数据
// 说明: 将数据写入c1_r_w_array（映射到GS1，由CPU1拥有）
//       数组第一个元素存储乘数，其余元素存储索引值
//****************************************************************************
void Shared_Ram_dataWrite_c1(void)
{
    uint16_t index;

    // 第一个元素存储乘数因子
    c1_r_w_array[0] = multiplier;

    // 填充数组：索引1-23存储对应的索引值
    for(index = 1; index < 24; index++)
    {
        c1_r_w_array[index] = index;
    }
}

//****************************************************************************
// 函数名: Shared_Ram_dataRead_c1
// 功能: 从共享RAM读取数据并验证
// 说明: 从c1_r_array读取CPU2写入的数据（映射到GS0，由CPU2拥有）
//       读取CPU2发送的数据（可根据需求添加验证逻辑）
//****************************************************************************
void Shared_Ram_dataRead_c1(void)
{
    uint16_t index;
    uint16_t cpu2_counter;

    // 读取CPU2的计数器值
    cpu2_counter = c1_r_array[0];

    // 读取其余数据元素
    for(index = 1; index < 24; index++)
    {
        // 读取数据（这里可以添加验证或处理逻辑）
        // 例如：验证数据是否为 cpu2_counter + index
        if(c1_r_array[index] != cpu2_counter + index)
        {
            error = 1;  // 数据不匹配，设置错误标志
        }
    }
}

//****************************************************************************
// 函数名: cpu_timer0_isr
// 功能: CPU定时器0中断服务程序
// 说明: 50us定时中断（修改为20kHz），实现多级任务周期标志
//****************************************************************************
interrupt void cpu_timer0_isr(void)
{
    static uint16_t counter_50us = 0;    // 50us计数器（用于20ms和100ms标志）
    static uint16_t counter_250us = 0;   // 250us计数器（用于IPC_Timer_1MS兼容）
    
    counter_50us++;
    counter_250us++;
    
    // 保持IPC_Timer_1MS的原有时基（每250us增加1）
    if(counter_250us >= 5) {  // 5 * 50us = 250us
        IPC_Timer_1MS++;
        counter_250us = 0;
    }
    
    // ===== 50us任务标志（20kHz ADC数据处理） =====
    flag_50us = 1;
    
    // ===== 20ms任务标志（IPC通信 + 喂狗） =====
    if(counter_50us % 400 == 0) {  // 每400次触发（20ms）
        flag_20ms_write = 1;
    }
    
    // ===== 100ms任务标志（DAC更新） =====
    if(counter_50us >= 2000) {  // 2000 * 50us = 100ms
        flag_100ms = 1;
        counter_50us = 0;  // 重置计数器
    }

    // 应答PIE组1中断
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;
}

//****************************************************************************
// 函数名: System_Init
// 功能: 系统初始化
// 说明: 初始化系统控制、GPIO、中断、定时器等
//****************************************************************************
void System_Init(void)
{
    // ===== 步骤1: 初始化系统控制 =====
    InitSysCtrl();

    // ===== 步骤2: 初始化GPIO =====
    InitGpio();

    EALLOW;
    GPIO_SetupPinMux(135, GPIO_MUX_CPU2, 0);
    GPIO_SetupPinMux(136, GPIO_MUX_CPU2, 0);
    EDIS;

    // ===== 步骤3: 配置中断系统 =====
    DINT;  // 禁用CPU中断

    InitPieCtrl();  // 初始化PIE控制寄存器

    // 清除所有CPU中断标志
    IER = 0x0000;
    IFR = 0x0000;

    InitPieVectTable();  // 初始化PIE向量表

    // 注册中断向量
    EALLOW;
    PieVectTable.TIMER0_INT = &cpu_timer0_isr;
    PieVectTable.XINT2_INT = &ADS1278_DRDY_ISR;      // XINT2：ADS1278 DRDY信号
    PieVectTable.DMA_CH1_INT = &DMA_CH1_ISR;         // DMA CH1：SPIA RX
    PieVectTable.DMA_CH2_INT = &DMA_CH2_ISR;         // DMA CH2：SPIA TX
    PieVectTable.DMA_CH5_INT = &DMA_CH5_ISR;         // DMA CH5：SPIB TX

    EDIS;

    // ===== 步骤4: 配置共享RAM和外设所有权 =====
    // 静态分配GS0给CPU2，分配SCIA所有权给CPU2
    EALLOW;
    MemCfgRegs.GSxMSEL.bit.MSEL_GS0 = 1;  // GS0分配给CPU2
    DevCfgRegs.CPUSEL5.bit.SCI_A = 1;//SCIA分配给CPU2
    EDIS;
    InitSciGpio();//为在CPU2运行的SCIA的GPIO初始化
    // ===== 步骤5: 通知CPU2配置完成并启动CPU2 =====
    // 通知CPU2：SCIA配置就绪（GPIO和所有权分配已完成）
    IPCLtoRFlagSet(IPC_FLAG12);

    // 启动CPU2（所有配置已完成，CPU2可以安全初始化）
    #ifdef _FLASH
//        IPCBootCPU2(C1C2_BROM_BOOTMODE_BOOT_FROM_FLASH);
    #else
        IPCBootCPU2(C1C2_BROM_BOOTMODE_BOOT_FROM_RAM);
    #endif

    // ===== 步骤6: 配置外设 =====
    EALLOW;
    CpuSysRegs.SECMSEL.bit.PF2SEL = 1;  // 将第二外设帧连接到DMA
    EDIS;

    // ===== 步骤6.5: SPI/DMA驱动初始化（新增） =====
    Drv_SPI_GPIO_Config();          // SPI GPIO配置（SPIA/SPIB引脚）
    Drv_SPIA_Init();                // SPIA初始化（ADS1278专用）
    Drv_SPIB_Init();                // SPIB初始化（AD5754专用）
    Drv_DMA_Init();                 // DMA初始化（CH1/2:SPIA, CH5/6:SPIB）
    Drv_ADS1278_InitDefault();      // ADS1278 ADC初始化
    Drv_AD5754_InitDefault();       // AD5754 DAC初始化
    
    // 注册DMA回调
    Drv_DMA_RegisterCallback(DMA_CH1, (DMA_Callback_t)Drv_ADS1278_GetDMACallback());

    // ===== 步骤7: 初始化并启动定时器 =====
    InitCpuTimers();
    ConfigCpuTimer(&CpuTimer0, 200, 50);  // 200MHz, 50us周期（修改为20kHz）
    CpuTimer0Regs.TCR.all = 0x4000;       // 启动定时器

    // ===== 步骤8: 使能中断（新增DMA和XINT2中断） =====
    IER |= M_INT1;  // 使能INT1组（Timer0 + XINT2）
    IER |= M_INT7;  // 使能INT7组（DMA中断）
    
    // 使能PIE中断
    PieCtrlRegs.PIEIER1.bit.INTx7 = 1;  // Timer0
    PieCtrlRegs.PIEIER1.bit.INTx5 = 1;  // XINT2
    PieCtrlRegs.PIEIER7.bit.INTx1 = 1;  // DMA CH1
    PieCtrlRegs.PIEIER7.bit.INTx2 = 1;  // DMA CH2
    PieCtrlRegs.PIEIER7.bit.INTx5 = 1;  // DMA CH5

    // ===== 步骤7: 初始化SCIA =====
    EnableInterrupts();

    // ===== 步骤9: 等待CPU2就绪 =====
    // 等待CPU2发送就绪标志
    while(!IPCRtoLFlagBusy(IPC_FLAG17));
    IPCRtoLFlagAcknowledge(IPC_FLAG17);

    // ===== 步骤10: 看门狗初始化（新增，最后启动） =====
    Drv_Watchdog_Init();  // 检测复位原因并配置看门狗
}

//****************************************************************************
// 函数名: SPI_Task_50us
// 功能: 50us任务 - 读取ADC数据（20kHz处理速率）
// 说明: 从Ping-Pong缓冲区读取最新ADC数据并更新到全局变量
//****************************************************************************
void SPI_Task_50us(void)
{
    ADS1278_Data_t adc_data;
    
    // 从Ping-Pong缓冲区读取最新ADC数据（非阻塞）
    if(Drv_ADS1278_GetData(&adc_data))
    {
        // 更新最新数据到全局变量
        g_latestAdc = adc_data;
        g_latestAdcValid = true;
        
        // 可选：简单的阈值检测（保持任务简短）
        // if(adc_data.ch[0] > 0x7FFFFF) {
        //     // 处理超限情况
        // }
    }
}

//****************************************************************************
// 函数名: SPI_Task_100ms
// 功能: 100ms任务 - 更新DAC输出
// 说明: 读取最新ADC数据并根据需要更新DAC输出
//****************************************************************************
void SPI_Task_100ms(void)
{
    if(g_latestAdcValid)
    {
        // 快速拷贝最新数据
        ADS1278_Data_t adc_data = g_latestAdc;
        g_latestAdcValid = false;
        
        // 生成DAC输出数据（示例：固定值）
        uint16_t dac_values[4];
        dac_values[0] = 0x8000;  // 通道A: 0V（中点）
        dac_values[1] = 0xC000;  // 通道B: +5V
        dac_values[2] = 0x4000;  // 通道C: -5V
        dac_values[3] = 0x8000;  // 通道D: 0V（中点）
        
        // TODO: 可根据adc_data进行计算，生成实际需要的DAC值
        // 例如：闭环控制、波形生成等
        
        // 写入DAC（非阻塞）
        Drv_AD5754_WriteAllChannels(dac_values);
    }
}

//
// End of file
//
