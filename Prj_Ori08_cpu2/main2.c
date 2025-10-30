//****************************************************************************
//
// 文件名: main2.c (CPU2主程序)
//
// 功能说明: 使用RAM_management方式实现双核共享RAM通信
//
// 改造说明:
//   - 移除了复杂的IPC消息队列机制
//   - 采用简单的IPC标志位同步 + 直接共享RAM访问
//   - 等待CPU1分配内存权限，无需动态请求
//   - 主循环轮询模式，无IPC中断
//
//****************************************************************************

#include "F28x_Project.h"
#include "F2837xD_Ipc_drivers.h"
#include "Sci.h"
#include "Drivers/drv_watchdog.h"

// ===== 全局变量定义 =====
// 共享RAM数组定义（减少到24个元素）
uint16_t c2_r_w_array[24];     // CPU2读写，映射到GS0（CPU2拥有）
uint16_t c2_r_array[24];       // CPU2只读，CPU1写入，映射到GS1（CPU1拥有）
#pragma DATA_SECTION(c2_r_w_array,"SHARERAMGS0");
#pragma DATA_SECTION(c2_r_array,"SHARERAMGS1");

// 20ms周期控制标志
volatile uint16_t flag_20ms_write = 0;  // 20ms写入标志
uint16_t SCI_Send_Timer = 0;


// ===== 函数原型声明 =====
void System_Init(void);
interrupt void TIMER0_ISR(void);  // CPU定时器0中断（定义在drive/DefaultlISR.c）

// RAM_management方式的数据处理函数
void Shared_Ram_dataWrite_c2(void);  // 向GS0写入数据
void Shared_Ram_dataRead_c2(void);   // 从GS1读取CPU1的数据并处理

//****************************************************************************
// 函数名: main
// 功能: CPU2主函数 - 等待CPU1通知并处理共享RAM数据
//****************************************************************************
void main(void)
{
    // 系统初始化
    System_Init();
    
    // 初始化标志
    flag_20ms_write = 0;

    // ===== 主循环 =====
    while(1)
    {
        SCI_RX();
        if(SCI_Send_Timer >= 200)
        {
            SCI_Send_Timer = 0;
            SCI_TO_Sim();//向上位机发送数据

            //============================Trigger ISR=============================================
            /*    清错误标志，串口发送前进行软复位，防止复位影响发送缓存数据         */
            if( SciaRegs.SCIRXST.bit.RXERROR )
            {
                SciaRegs.SCICTL1.bit.SWRESET = 0;
                SciaRegs.SCICTL1.bit.SWRESET = 1;
            }
            // 清串口发送中断标志
            SciaRegs.SCIFFTX.bit.TXFIFORESET = 1;
            SciaRegs.SCIFFTX.bit.TXFFINTCLR = 1;                    // Clear SCI Interrupt Flag 清串口发送中断标志，发送
        }

        // 处理20ms周期写入任务
        if(flag_20ms_write)
        {
            flag_20ms_write = 0;  // 清除标志
            
            // 写入GS0共享RAM
            Shared_Ram_dataWrite_c2();
            
            // 设置FLAG11，通知CPU1数据已准备好
            IPCLtoRFlagSet(IPC_FLAG11);
            
            // 喂狗（新增）
            Drv_Watchdog_Kick();
        }
        
        // 检查是否收到CPU1的IPC标志10
        if(IPCRtoLFlagBusy(IPC_FLAG10) == 1)
        {
            // 读取CPU1写入的GS1数据
            Shared_Ram_dataRead_c2();
            
            // ACK清除FLAG10
            IPCRtoLFlagAcknowledge(IPC_FLAG10);
        }

        // 原有业务逻辑可以在这里处理
        // 例如：其他任务、状态机等
    }
}

//****************************************************************************
// 函数名: Shared_Ram_dataWrite_c2
// 功能: 向GS0写入数据
// 说明: 将测试数据写入c2_r_w_array（映射到GS0，由CPU2拥有）
//       这个函数在主循环中调用，周期性地更新GS0的内容
//****************************************************************************
void Shared_Ram_dataWrite_c2(void)
{
    static uint16_t cpu2_counter = 0;  // CPU2的计数器
    uint16_t index;

    // 第一个元素存储计数器值
    c2_r_w_array[0] = cpu2_counter;

    // 填充测试数据：计数器值 + 索引
    for(index = 1; index < 24; index++)
    {
        c2_r_w_array[index] = cpu2_counter + index;
    }
    
    // 计数器递增
    cpu2_counter++;
}

//****************************************************************************
// 函数名: Shared_Ram_dataRead_c2
// 功能: 从GS1读取CPU1写入的数据并处理
// 说明: 从c2_r_array读取CPU1写入的数据（映射到GS1，由CPU1拥有）
//       这个函数在主循环中调用，当检测到FLAG10时读取数据
//       可以在这里添加数据处理逻辑
//****************************************************************************
void Shared_Ram_dataRead_c2(void)
{
    uint16_t index;
    uint16_t multiplier;

    // 从数组第一个元素读取乘数因子
    multiplier = c2_r_array[0];
    (void)multiplier;  // 消除未使用警告（预留给将来的处理逻辑）

    // 这里可以添加数据处理逻辑
    // 例如：验证数据、执行计算、更新状态等
    // 当前实现：简单读取数据（可根据需求扩展）
    for(index = 1; index < 24; index++)
    {
        // 读取数据（这里可以添加处理逻辑）
        // 例如：保存到本地缓存、进行运算等
        // 期望值应该是索引值本身
    }
}

//****************************************************************************
// 函数名: System_Init
// 功能: 系统初始化
// 说明: 初始化中断、定时器等（系统控制由CPU1完成）
//****************************************************************************
void System_Init(void)
{
    // ===== 步骤1: 系统控制 =====
     InitSysCtrl();

    // ===== 步骤2: 配置中断系统 =====
    DINT;  // 禁用CPU中断

    InitPieCtrl();  // 初始化PIE控制寄存器

    // 清除所有CPU中断标志
    IER = 0x0000;
    IFR = 0x0000;

    InitPieVectTable();  // 初始化PIE向量表



    // ===== 步骤3: 等待共享RAM可用 =====
    // 等待CPU1分配GS0内存给CPU2
    while(!(MemCfgRegs.GSxMSEL.bit.MSEL_GS0));

    // ===== 步骤4: 等待CPU1完成SCIA配置 =====
    // 等待CPU1完成SCIA的GPIO配置和所有权分配
    while(!IPCRtoLFlagBusy(IPC_FLAG12));
    IPCRtoLFlagAcknowledge(IPC_FLAG12);

    // 注册中断向量
      EALLOW;
      PieVectTable.TIMER0_INT = &TIMER0_ISR;
      PieVectTable.SCIA_TX_INT = &SCIA_TX_ISR;
      PieVectTable.SCIA_RX_INT = &SCIA_RX_ISR;

      EDIS;

    // ===== 步骤5: 初始化并启动定时器 =====
    InitCpuTimers();
    ConfigCpuTimer(&CpuTimer0, 200, 50);  // 200MHz, 50us周期
    CpuTimer0Regs.TCR.all = 0x4000;       // 启动定时器

    // ===== 步骤7: 初始化SCIA =====
    // CPU1配置完成，现在可以安全配置SCIA寄存器
    InitSci();

    // ===== 步骤6: 使能中断 =====
    // TIMER0中断使能已在 EnableInterrupts() 中配置
    EnableInterrupts();



    // ===== 步骤8: 看门狗初始化（新增，最后启动） =====
//    Drv_Watchdog_Init();  // 检测复位原因并配置看门狗
    
    // ===== 步骤9: 通知CPU1已就绪 =====
    IPCLtoRFlagSet(IPC_FLAG17);
}

//
// End of file
//
