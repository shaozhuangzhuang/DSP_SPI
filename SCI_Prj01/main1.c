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
#include "Sci.h"

unsigned int IPC_Timer_1MS = 0;
uint16_t SCI_Send_Timer = 0;

// ===== 函数原型声明 =====
void System_Init(void);
interrupt void cpu_timer0_isr(void);
double Cal_u = 0;
double Rec_Vel1 = 0;
double Rec_Vel2 = 0;
double Rec_Vel3 = 0;
uint16_t SCI_Connect = 0;

//****************************************************************************
// 函数名: main
// 功能: CPU1主函数 - 初始化系统并执行双核共享RAM通信 IPC_Timer_1MS
//****************************************************************************
void main(void)
{
    // 系统初始化
    System_Init();

    // ===== 主循环 =====
    while(1)
    {
        SCI_RX();
        if(SCI_Send_Timer >= 20)
        {
            SCI_Send_Timer = 0;
            SCI_TO_Sim(Cal_u);//向仿真机发送数据

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
    }
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

    // ===== 步骤2: 启动CPU2 =====
    #ifdef _FLASH
//        IPCBootCPU2(C1C2_BROM_BOOTMODE_BOOT_FROM_FLASH);
    #else
        IPCBootCPU2(C1C2_BROM_BOOTMODE_BOOT_FROM_RAM);
    #endif

    // ===== 步骤3: 初始化GPIO =====
    InitGpio();

    InitSciGpio();

    // ===== 步骤4: 配置中断系统 =====
    DINT;  // 禁用CPU中断

    InitPieCtrl();  // 初始化PIE控制寄存器

    // 清除所有CPU中断标志
    IER = 0x0000;
    IFR = 0x0000;

    InitPieVectTable();  // 初始化PIE向量表

    // 注册中断向量
    EALLOW;
    PieVectTable.TIMER0_INT = &cpu_timer0_isr;
    PieVectTable.SCIA_TX_INT = &SCIA_TX_ISR;
    PieVectTable.SCIA_RX_INT = &SCIA_RX_ISR;
    // 注意：不再注册IPC中断
    EDIS;

    // ===== 步骤7: 初始化并启动定时器 =====
    InitCpuTimers();
    ConfigCpuTimer(&CpuTimer0, 200, 50);  // 200MHz, 50us周期
    CpuTimer0Regs.TCR.all = 0x4000;        // 启动定时器

    EnableInterrupts();

    InitSci();

}


//****************************************************************************
// 函数名: cpu_timer0_isr
// 功能: CPU定时器0中断服务程序
// 说明: 1ms定时中断，用于定时任务
//****************************************************************************
interrupt void cpu_timer0_isr(void)
{
    IPC_Timer_1MS++;
    SCI_Send_Timer++;
    // 应答PIE组1中断
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;
}


//
// End of file
//
