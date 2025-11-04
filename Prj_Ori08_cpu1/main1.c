#define CPU1 1
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



#include "Drivers/drv_watchdog.h"
#include "Framework/system_config.h"
#include "Framework/global_data.h"
#include "Drivers/drv_spi.h"

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

// 10ms SPI发送标志（用于AD5754R通信测试）
volatile uint16_t flag_10ms_spi = 0;

// SPI任务控制标志（20kHz处理）

// 定时器控制变量（保留原有业务逻辑）
unsigned int IPC_Timer_1MS = 0;
// extern uint32_t SCIA_Counter;  // 未使用，已删除

// ===== 函数原型声明 =====
void System_Init(void);
interrupt void cpu_timer0_isr(void);

// RAM_management方式的数据交换函数
void Shared_Ram_dataWrite_c1(void);  // 向GS1写入数据
void Shared_Ram_dataRead_c1(void);   // 从GS0读取并验证数据

//
// SPI通信
//
volatile Uint16 spiARxBuffer[4];
interrupt void spiARxISR(void);



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
        // ===== 10ms任务（SPIB发送，用于AD5754R通信测试） =====
        if(flag_10ms_spi )
        {
            flag_10ms_spi = 0;   // 清除标志

            // 周期性AD5754R通信测试（写入-读取验证）
            Test_AD5754R_Communication();
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
//             Drv_Watchdog_Kick(); //
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
// 说明: 50us定时中断（20kHz），实现多级任务周期标志
//****************************************************************************
interrupt void cpu_timer0_isr(void)
{
    static uint16_t counter_20ms = 0;
    static uint16_t counter_250us = 0;   // 250us计数器（用于IPC_Timer_1MS兼容）
    static uint16_t counter_10ms = 0;    // 10ms计数器（用于SPI发送）

    counter_20ms++;
    counter_250us++;
    counter_10ms++;

    // 保持IPC_Timer_1MS的原有时基（每250us增加1）
    if(counter_250us >= 5) {  // 5 * 50us = 250us
        IPC_Timer_1MS++;
        counter_250us = 0;
    }

    // ===== 20ms任务标志（IPC通信 + 喂狗） =====
    if(counter_20ms >= 400) {  // 每400次触发（400 * 50us = 20ms）
        flag_20ms_write = 1;
        counter_20ms = 0;
    }

    // ===== 10ms任务标志（SPIB发送，用于AD5754R通信测试） =====
    if(counter_10ms >= 200) { // 每200次触发（200 * 50us = 10ms）
        flag_10ms_spi = 1;
        counter_10ms = 0;
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
    InitSpiInterrupts(); // 添加SPI中断初始化(包含ISR注册和PIE使能)

    EALLOW;
    PieVectTable.TIMER0_INT = &cpu_timer0_isr;
    EDIS;
    
    // ===== 步骤3.5: 确保SPIB分配给CPU1（在初始化SPI之前）=====
    EALLOW;
    DevCfgRegs.CPUSEL6.bit.SPI_B = 0;  // SPIB分配给CPU1 (0=CPU1, 1=CPU2)
    EDIS;
    
    // ===== 步骤3.6: 初始化SPI（确保所有权正确后再初始化）=====
    InitSpiGpios();   // SPI GPIO初始化
    InitSpiModules(); // SPI模块初始化（此时中断已配置好）

    // ===== 步骤4: 配置共享RAM和外设所有权 =====
    // 静态分配GS0给CPU2，分配SCIA所有权给CPU2
    EALLOW;
    MemCfgRegs.GSxMSEL.bit.MSEL_GS0 = 1;  // GS0分配给CPU2
    DevCfgRegs.CPUSEL5.bit.SCI_A = 1;     // SCIA分配给CPU2
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

    // ===== 步骤7: 初始化并启动定时器 =====
    InitCpuTimers();
    ConfigCpuTimer(&CpuTimer0, 200, 50);  // 200MHz, 50us周期（修改为20kHz）
    CpuTimer0Regs.TCR.all = 0x4000;       // 启动定时器

    // ===== 步骤8: 使能中断 =====
    IER |= M_INT1 | M_INT6; // 使能CPU中断组1(Timer0)和6(SPI)
    EnableInterrupts();

    // ===== 步骤9: 等待CPU2就绪 =====
    // 等待CPU2发送就绪标志
//    while(!IPCRtoLFlagBusy(IPC_FLAG17));
//    IPCRtoLFlagAcknowledge(IPC_FLAG17);

    // ===== 步骤10: 全局数据结构初始化（新增） =====
    GlobalData_Init();  // 初始化全局数据结构（ADC/DAC数据）

    // ===== 步骤11: 看门狗初始化（新增，最后启动） =====
//    Drv_Watchdog_Init();  // 检测复位原因并配置看门狗


}



//****************************************************************************
// 函数名: spiARxISR
// 功能: SPIA接收FIFO中断服务程序
// 说明: 当RX FIFO中的字数 >= RXFFIL(4)时触发
//****************************************************************************
interrupt void spiARxISR(void)
{
    Uint16 i;
    Uint16 fifoLevel;
    
    // 读取FIFO中的所有数据
    fifoLevel = SpiaRegs.SPIFFRX.bit.RXFFST;
    
    for(i = 0; i < fifoLevel && i < 4; i++)
    {
        spiARxBuffer[i] = SpiaRegs.SPIRXBUF;
    }
    
    // 检查并清除溢出标志
    if(SpiaRegs.SPIFFRX.bit.RXFFOVF == 1)
    {
        SpiaRegs.SPIFFRX.bit.RXFFOVFCLR = 1;
    }
    
    // 清除中断标志
    SpiaRegs.SPIFFRX.bit.RXFFINTCLR = 1;
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP6;
}

//
// End of file
//
