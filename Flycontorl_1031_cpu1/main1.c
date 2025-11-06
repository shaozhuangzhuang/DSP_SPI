/*
 * MAIN.c
 *
 *  Created on: 2022年8月11日
 *      Author: 110
 */

// Included Files
//
#include "F28x_Project.h"
#include "F2837xD_Ipc_drivers.h"
#include"COMMON.h"

#define CPU01TOCPU02_PASSMSG  0x0003FD00         // CPU01 to CPU02 MSG RAM offsets for passing address
#define CPU02TOCPU01_PASSMSG  0x0003F900
#define GS10SARAM_START        0x16000             // Start of GS10 SARAM

#define PI 3.14159265359
#define ToRad PI/180
#define ToDeg 180/PI
double Phi_S,Theta_S;
float get_Code_Cnt = 0;
//IPC数据
volatile tIpcController g_sIpcController1;       // IPC控制
volatile tIpcController g_sIpcController2;       // IPC控制
float usCPU01Buffer[32];                         // 接收缓存
float *pusCPU01BufferPt;
Uint32 *pulMsgRam;
Uint32 *pulMsgRam2;                        //CPU1发送的结构体指针

//定时器
Uint32 Timer = 0;
unsigned int timer_20us = 0;
unsigned int DataLink_Timer_20ms = 0;
unsigned int TIMER0_1MS = 0;
Uint16 TimeCnt = 0;

bool Rec_HUOKONG_Launch_Flag;//接收到发射指令的标志位
Uint16 fireState = 0x00;
bool fireCtrlSync = 0;
bool fireCtrlFlag;
Uint16 timerSciSync = 0;


//can
int Pitch_angle = 0;
int Roll_angle  = 0;
int flag_angle  = 0;
int flag_angle1 = 0;
int flag_angle2 = 0;
int flag_angle3 = 0;

//int DYT_work_mode = 0;
//int Ld_State = 0;
unsigned int Count;
//引战、点火反馈
float YJLJB_flag = 0.0;
float DLG_flag = 0.0;
float JGTC_flag = 0.0;
float Battery_active_signal = 0.0;
//ramfuncs
// Main
//
void System_Init(void);
interrupt void cpu_timer0_isr(void);
__interrupt void CPU02toCPU01IPC0IntHandler(void);
__interrupt void CPU02toCPU01IPC1IntHandler(void);
void IPC11_TX_CPU1(void);
void IPC13_RX_CPU1(void);
void InitIPC(void);

interrupt void YJLJB_Interrupt(void);
interrupt void DLG_Interrupt(void);
interrupt void JGTC_Interrupt(void);
interrupt void Battery_Interrupt(void);
void YinZhan_Set(void);
void Cal_SeekerAngle(void);
void Ignite_Operation(void);
void McBSP_Deal(void);
void Seeker_Deal(void);

//点火
Uint16 Battery_fire_FLAG = 0;
Uint16 flag_fire = 0;
Uint16 Engine_fire_FLAG = 0;
Uint16 Battery_finsh_flag = 0;
Uint16 Battery_state = 0;
unsigned int TIMER0_Battery_fire = 0;
unsigned int TIMER0_Engine_fire = 0;
unsigned int Timer_1s = 0;
unsigned int TIMER_20MS = 0;
struct IPC_Comm_Data IPC_Data_Cpu2;
Uint16 FlagCnt = 0;

extern float Angle_cmd1;
extern float Angle_cmd2;
extern float Angle_cmd3;
extern float Angle_cmd4;
extern float Angle_cmd[4];
extern float Tra_acc[3];
extern float Tra_a_p[3];

//导引头
int DYT_Work_Mode = 0;
int Leida_state = 0x55;
extern unsigned int TIMER0_1MS_1;

int Servo_TEST_Flag = 0;
float Servo_TEST_Cmd = 0;
int Start_Servo_Move_Flag = 0;

void main(void)
{
    //初始化
    Rec_HUOKONG_Launch_Flag = 0;
    Battery_active_signal = 0;
    Engine_fire_FLAG = 0;
    Battery_fire_FLAG = 0;
    fireState = 0x00;
    fireCtrlSync = 0;
    fireCtrlFlag = 0;
    timerSciSync = 0;
    System_Init();

//#ifdef _STANDALONE
//#ifdef _FLASH
//    //
//    // Send boot command to allow the CPU2 application to begin execution
//    //
//    IPCBootCPU2(C1C2_BROM_BOOTMODE_BOOT_FROM_FLASH);
//#else·
//    //
//    // Send boot command to allow the CPU2 application to begin execution
//    //
//    IPCBootCPU2(C1C2_BROM_BOOTMODE_BOOT_FROM_RAM);
//#endif
//#endif

    while(1)
    {
        if(Servo_TEST_Flag == 1)
        {
            Angle_cmd1 = Servo_TEST_Cmd;
            Angle_cmd2 = Servo_TEST_Cmd;
            Angle_cmd3 = Servo_TEST_Cmd;
            Angle_cmd4 = Servo_TEST_Cmd;
        }
#ifdef Servo_TEST
        SCIA_RX();//接收舵机的反馈数据
#endif
#ifdef FZJ_TEST
        SCIA_RX_FZJ();//接收仿真机数据
#endif
        DataLink_Rx();//接收数据链的反馈数据
        Ignite_Operation();//点火操作，包含热电池点火和发动机点火



        if (timerSciSync >= 4 * 20 * 3) //串口通信的间隔时间
        {
            timerSciSync = 0;
            if (fireCtrlFlag == 1)
            {
                fireCtrlFlag = 0;
                fireCtrlSync = 1;
            }
            else if (fireCtrlFlag == 0)
            {
                fireCtrlSync = 0;
            }
        }

        if(timer_20us >= 80)
        {
            timer_20us = 0;
#ifdef Servo_TEST
            SCIA_TO_Servo();//向舵机发送数据
#endif
#ifdef FZJ_TEST
            SCIA_TO_FZJ();//向仿真机发送数据
#endif
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
        if(DataLink_Timer_20ms >= 80)
        {
            DataLink_Timer_20ms = 0;

            SCIC_TO_DATALINK();
            //============================Trigger ISR=============================================
            /*    清错误标志，串口发送前进行软复位，防止复位影响发送缓存数据         */
            if( ScicRegs.SCIRXST.bit.RXERROR )
            {
                ScicRegs.SCICTL1.bit.SWRESET = 0;
                ScicRegs.SCICTL1.bit.SWRESET = 1;
            }
            // 清串口发送中断标志
            ScicRegs.SCIFFTX.bit.TXFIFORESET = 1;
            ScicRegs.SCIFFTX.bit.TXFFINTCLR = 1;                    // Clear SCI Interrupt Flag 清串口发送中断标志，发送
        }
        McBSP_Deal();//McBSP处理，与火控完成通信
        Seeker_Deal();//导引头操作，CAN通信显示数据传输

        IPC13_RX_CPU1();
        if(TIMER0_1MS_1 >= 16 * 4)
        {
            TIMER0_1MS_1 = 0;
            IPC11_TX_CPU1();
        }

    }
}

//点火操作，包含热电池点火和发动机点火
void Ignite_Operation(void)
{
    if (RX_HUOKONG.Launch_cmd == 0xF0 && Rec_HUOKONG_Launch_Flag == 0) //火控发射指令 开始点火
     {
         Rec_HUOKONG_Launch_Flag = 1;
         if (Battery_active_signal == 0) //热电池未激活
         {
             Battery_fire_FLAG = 1; //热电池开始点火
         }
         else if (Battery_active_signal == 1) //热电池已激活
         {
             fireState = 0xB9; //热电池点火成功
             TIMER0_Battery_fire = 0;
             Battery_fire_FLAG = 0;
             Engine_fire_FLAG = 1; //开启发动点火标志位
         }
     }
     //热电池点火
     if ((Battery_fire_FLAG == 1) && (Rec_HUOKONG_Launch_Flag == 1))
     {
         GPIO_WritePin(15, 0); //热电池点火充电i/o
         GPIO_WritePin(16, 0);

         fireState = 0xB5; //热电池点火中

         if (TIMER0_Battery_fire >= 200) // 200 50ms
         {

             GPIO_WritePin(17, 0); //热电池点火放电i/o
             GPIO_WritePin(18, 0);

             if (TIMER0_Battery_fire >= 600) ////&& (Battery_active_signal == 1)
             {
                 TIMER0_Battery_fire = 0;
                 Battery_fire_FLAG = 0;//热电池激活标志置零

                 if (Battery_active_signal == 1) //热电池状态检测IO
                 {
//                     Rec_HUOKONG_Launch_Flag = 0;
                     fireState = 0xB9; //热电池点火成功
                     TIMER0_Battery_fire = 0;
                     Engine_fire_FLAG = 1; //开启发动点火标志位
                 }
                 else
                 {
                     Rec_HUOKONG_Launch_Flag = 0;
                     fireState = 0xB7; //热电池点火失败
                 }
             }
         }
     }
     else if (Battery_fire_FLAG == 0)
     {
         GPIO_WritePin(15, 1);
         GPIO_WritePin(16, 1);
         GPIO_WritePin(17, 1);
         GPIO_WritePin(18, 1);
     }
     //发动机点火
     if (Engine_fire_FLAG && TIMER_20MS >= 80)
     {
         TIMER_20MS = 0;
         GPIO_WritePin(21, 0); //发动机点火充电i/o
         GPIO_WritePin(99, 0);
         fireState = 0xE5; //发动机点火中

         if (TIMER0_Engine_fire >= 200)
         {

             GPIO_WritePin(19, 0); //发动机点火放电i/o
             GPIO_WritePin(20, 0);
             if (TIMER0_Engine_fire >= 600)
             {
                 if (fireCtrlSync == 1)
                 {
                     fireState = 0xE7; //发动机点火失败
                 }
                 else if(fireCtrlSync == 0)
                 {
                     fireState = 0xE9; //发动机点火成功
                     Rec_HUOKONG_Launch_Flag = 0;
                 }
                 TIMER0_Engine_fire = 0;
                 Engine_fire_FLAG = 0;
                 //                Battery_finsh_flag = 1;
                 //                    Battery_state = 0xf0;
                 //                    DYT_work_mode = 4;//导引头点火实验指令
                 //                    DJ_fire_test = 0xaa;//舵机点火实验指令
             }
         }
     }
     else if (Engine_fire_FLAG == 0)
     {

         GPIO_WritePin(19, 1);
         GPIO_WritePin(20, 1);
         GPIO_WritePin(21, 1);
         GPIO_WritePin(99, 1);
     }
     if(fireState == 0xE7)
     {
         Start_Servo_Move_Flag = 1;//开始舵机运动
     }
}

//McBSP接收处理操作
void McBSP_Deal(void)
{
    /****************火控接收发送---START**********************/
     if(DmaRegs.CH1.CONTROL.bit.TRANSFERSTS == 0)
     {
         FlagCnt++;
     }
     if(McBsp_Rec_Flag == 1)
     {
         McBSP_Decode(McBSP_Rx_SCI_Interim, &McBSP_Rdata_use, 48);
         McBSP_RX();
         McBsp_Rec_Flag = 0;
     }

     if(TransFinish_Flag == 2)
     {
         if(McbspbRegs.SPCR2.bit.XRDY == 1)
         {
             McbspbRegs.DXR1.all = 0xFFFF;
         }
     }
     if(TimeCnt >= 80)
     {
        McBSP_TX();
        McBSP_Encode(Mcbsp_Txdate,42);
        SCI_TX_Cnt++;

        if(DmaRegs.CH1.TRANSFER_COUNT == 0)
        {
            TransFinish_Flag = 1;
            EALLOW;
            DmaRegs.CH1.CONTROL.bit.RUN = 1;      // Start DMA Transmit from McBSP-A
            EDIS;
        }
        TimeCnt = 0;
     }
     /****************火控接收发送---END**********************/
}

//导引头操作，CAN通信显示数据传输
void Seeker_Deal(void)
{
    //导引头
    if(RX_HUOKONG.TargetNum == 1)//&& SeekerToFlight_Rx.Roll_work_mode == 1
    {
        Leida_state = 0xaa;
    }
    else
    {

        Leida_state = 0x55;
    }

    if(RX_HUOKONG.DYT_RESTE == 0x0F)
    {
        DYT_Work_Mode = 3;
    }
    else
    {
        DYT_Work_Mode = 0;
    }
    Cal_SeekerAngle();

    //CAN
    if(flag_angle || flag_angle1 || flag_angle2 || flag_angle3 )
    {
        CAN_Rx();
        flag_angle = 0;
        flag_angle1 = 0;
        flag_angle2 = 0;
        flag_angle3 = 0;
    }

    if(TIMER0_1MS >= 20)
    {
        TIMER0_1MS = 0;

        CAN_Tx();

    }
}

void InitIPC(void)
{
    Uint16 counter = 0;

    pulMsgRam = (void *)CPU01TOCPU02_PASSMSG;
    pulMsgRam[0] = (Uint32)&usCPU01Buffer[0];

    pulMsgRam2 = (void *)CPU02TOCPU01_PASSMSG;
    pusCPU01BufferPt = (void *)GS10SARAM_START;


    for(counter = 0; counter <32; counter++)
    {
        usCPU01Buffer[counter] = 0.0;
    }

}
//*****************************************************************************
// CPU01 to CPU02 IPC INT1 Interrupt Handler -
// Should never reach this ISR. This is an optional placeholder for
// g_sIpcController2.
//*****************************************************************************
//CPU1发送中断程序
__interrupt void CPU02toCPU01IPC1IntHandler (void)
{
    tIpcMessage sMessage;

    while(IpcGet(&g_sIpcController2, &sMessage, DISABLE_BLOCKING)!= STATUS_FAIL)
    {
        switch (sMessage.ulcommand)
        {
        case IPC_SET_BITS_PROTECTED:
            IPCRtoLSetBits_Protected(&sMessage);       // Processes IPCReqMemAccess() function

            break;
        case IPC_CLEAR_BITS_PROTECTED:
            IPCRtoLClearBits_Protected(&sMessage);     // Processes IPCReqMemAccess() function

            break;
        case IPC_BLOCK_WRITE:
            IPCRtoLBlockWrite(&sMessage);

            break;
        case IPC_BLOCK_READ:
            IPCRtoLBlockRead(&sMessage);
            break;
        default:
            //                  ErrorFlag = 1;
            break;
        }
    }

    // Acknowledge IPC INT1 Flag and PIE to receive more interrupts
    IpcRegs.IPCACK.bit.IPC1 = 1;
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;
}

__interrupt void CPU02toCPU01IPC0IntHandler (void)
{
    tIpcMessage sMessage;

    // Continue processing messages as long as CPU01toCPU02 GetBuffer2 is full
    while(IpcGet(&g_sIpcController1, &sMessage, DISABLE_BLOCKING)!= STATUS_FAIL)
    {
        switch (sMessage.ulcommand)
        {
        case IPC_SET_BITS_PROTECTED:
            IPCRtoLSetBits_Protected(&sMessage);       // Processes IPCReqMemAccess() function

            break;
        case IPC_CLEAR_BITS_PROTECTED:
            IPCRtoLClearBits_Protected(&sMessage);     // Processes IPCReqMemAccess() function

            break;
        case IPC_BLOCK_WRITE:
            IPCRtoLBlockWrite(&sMessage);
            break;
        case IPC_BLOCK_READ:
            IPCRtoLBlockRead(&sMessage);
            break;
        default:
            //                  ErrorFlag = 1;
            break;
        }
    }

    // Acknowledge IPC INT0 Flag and PIE to receive more interrupts
    IpcRegs.IPCACK.bit.IPC0 = 1;
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;
}

/*
CPU1获得全局共享内存写权限，进行赋值、写块，产生IPC1中断，CPU2响应IPC1中断，完成块写命令。
 * 使用IPC11标志位，通知CPU2我已发送完成，CPU2就可以读啦。
 * CPU1中发送函数：
 */
extern uint32_t SCIA_Counter;
void IPC11_TX_CPU1(void)
{
    if(IPCRtoLFlagBusy(IPC_FLAG11) == 1)
    {
        if((MemCfgRegs.GSxMSEL.bit.MSEL_GS10) == 1)
        {
            EALLOW;
            MemCfgRegs.GSxMSEL.bit.MSEL_GS10 = 0;
            EDIS;
        }
        pusCPU01BufferPt[0] = RX_SimData.FZJ_Acc_X;
        pusCPU01BufferPt[1] = RX_SimData.FZJ_Acc_Y;
        pusCPU01BufferPt[2] = RX_SimData.FZJ_Acc_Z;
        pusCPU01BufferPt[3] = RX_SimData.FZJ_Pitch_VelAng;
        pusCPU01BufferPt[4] = RX_SimData.FZJ_Yaw_VelAng;
        pusCPU01BufferPt[5] = RX_SimData.FZJ_Roll_VelAng;
        pusCPU01BufferPt[6] = RX_SimData.Yaw_VelAng_Sim;
        pusCPU01BufferPt[7] = RX_SimData.Pitch_VelAng_Sim;
        pusCPU01BufferPt[8] = SCIA_Counter;
        pusCPU01BufferPt[9] = Start_Servo_Move_Flag;
//        pusCPU01BufferPt[9] = real_Pitch_angle;
//        pusCPU01BufferPt[10] = real_Roll_angle;
//        pusCPU01BufferPt[11] = Yaw_VelAng_Sim;
//        pusCPU01BufferPt[12] = Pitch_VelAng_Sim;
//        pusCPU01BufferPt[13] = DYT_work_mode;
//        pusCPU01BufferPt[14] = CanDaTa_rx.Pitch_work_mode;
//        pusCPU01BufferPt[15] = CanDaTa_rx.Roll_work_mode;
//        pusCPU01BufferPt[16] = CanDaTa_rx.Picture_state;

        //(实例的地址,指定要写入的远程 CPU 内存块起始地,指定要写入的数据所在的本地 CPU 共享内存地址,以 16 位或 32 位字指定块大小,指定字长,指定是否允许函数阻塞);
        IPCLtoRBlockWrite(&g_sIpcController2, pulMsgRam2[1],(uint32_t)pusCPU01BufferPt,128,IPC_LENGTH_16_BITS,ENABLE_BLOCKING);

        IPCRtoLFlagAcknowledge(IPC_FLAG11);
        IPCLtoRFlagClear(IPC_FLAG11);
    }
}


/*   数据接收
 *
 * （usCPU01Buffer已在CPU1的IPC1中断中处理好）
 *
 */
extern uint32_t McBSP_TX_Counter;
void IPC13_RX_CPU1(void)
{
    /********** Data Block Reads************/
    if(IPCRtoLFlagBusy(IPC_FLAG13) == 1)
    {
        IPCRtoLFlagAcknowledge(IPC_FLAG13);
        IPCLtoRFlagClear(IPC_FLAG13);
    }

    IPC_Data_Cpu2.IMU_X_gyro = usCPU01Buffer[0];
    IPC_Data_Cpu2.IMU_Y_gyro = usCPU01Buffer[1];
    IPC_Data_Cpu2.IMU_Z_gyro = usCPU01Buffer[2];
    IPC_Data_Cpu2.IMU_X_acc  = usCPU01Buffer[3];
    IPC_Data_Cpu2.IMU_Y_acc  = usCPU01Buffer[4];
    IPC_Data_Cpu2.IMU_Z_acc  = usCPU01Buffer[5];

    IPC_Data_Cpu2.AttitudeSolution_Longitude = usCPU01Buffer[6];
    IPC_Data_Cpu2.AttitudeSolution_Latitude  = usCPU01Buffer[7];
    IPC_Data_Cpu2.AttitudeSolution_Height    = usCPU01Buffer[8];
    IPC_Data_Cpu2.AttitudeSolution_X         = usCPU01Buffer[9];
    IPC_Data_Cpu2.AttitudeSolution_Y         = usCPU01Buffer[10];
    IPC_Data_Cpu2.AttitudeSolution_Z         = usCPU01Buffer[11];
    IPC_Data_Cpu2.AttitudeSolution_VN        = usCPU01Buffer[12];
    IPC_Data_Cpu2.AttitudeSolution_VE        = usCPU01Buffer[13];
    IPC_Data_Cpu2.AttitudeSolution_VD        = usCPU01Buffer[14];
    IPC_Data_Cpu2.AttitudeSolution_Yaw       = usCPU01Buffer[15];
    IPC_Data_Cpu2.AttitudeSolution_Pitch     = usCPU01Buffer[16];
    IPC_Data_Cpu2.AttitudeSolution_Roll      = usCPU01Buffer[17];


    Angle_cmd[0]  = usCPU01Buffer[18];               //[指令]4号舵打舵指令
    Angle_cmd[1]  = usCPU01Buffer[19];               //[指令]1号舵打舵指令
    Angle_cmd[2]  = usCPU01Buffer[20];               //[指令]2号舵打舵指令
    Angle_cmd[3]  = usCPU01Buffer[21];               //[指令]4号舵打舵指令

    Tra_a_p[1]  = usCPU01Buffer[22];               //[指令]1号舵打舵指令
    Tra_a_p[2]  = usCPU01Buffer[23];               //[指令]2号舵打舵指令

    get_Code_Cnt = usCPU01Buffer[24];
}

interrupt void cpu_timer0_isr(void)
{
    TimeCnt++;
    Timer++;
    timer_20us++;
    TIMER0_1MS++;
    DataLink_Timer_20ms++;
    TIMER0_1MS_1++;
    timerSciSync++;
    if (Battery_fire_FLAG)
    {
        TIMER0_Battery_fire++;
    }

    if(Engine_fire_FLAG)
    {
        TIMER_20MS++;
        TIMER0_Engine_fire++;
    }

    if(Battery_finsh_flag)
    {
        Timer_1s++;
    }

    // Acknowledge this interrupt to receive more interrupts from group 1
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;
}

void System_Init(void)
{
    InitSysCtrl();

//    IPCBootCPU2(C1C2_BROM_BOOTMODE_BOOT_FROM_FLASH);

    InitGpio();
    InitSciGpio();
    InitCANGpio();
    Gpioselect();

    GPIO_SetupPinMux(69,GPIO_MUX_CPU1,0);     // DE_A2
    GPIO_SetupPinOptions(69, GPIO_OUTPUT,1);
    GpioDataRegs.GPCSET.bit.GPIO69 = 1;
    GPIO_SetupPinMux(71,GPIO_MUX_CPU1,0);     // /RE_A2
    GPIO_SetupPinOptions(71, GPIO_OUTPUT,0);
    GpioDataRegs.GPCCLEAR.bit.GPIO71 = 1;

    //点火I/O初始化,引战IO初始化
    Init_Fire_YinZhan_Gpio();

    McBsp_GPIO_Init();

    DINT;

    InitPieCtrl();

    IER = 0x0000;
    IFR = 0x0000;

    InitPieVectTable();

    // Interrupts that are used in this example are re-mapped to
    // ISR functions found within this file.
    EALLOW;  // This is needed to write to EALLOW protected registers
    PieVectTable.TIMER0_INT = &cpu_timer0_isr;
    PieVectTable.IPC0_INT = &CPU02toCPU01IPC0IntHandler;
    PieVectTable.IPC1_INT = &CPU02toCPU01IPC1IntHandler;
    PieVectTable.SCIA_TX_INT = &SCIA_TX_ISR;
    PieVectTable.SCIA_RX_INT = &SCIA_RX_ISR;
    PieVectTable.SCIC_TX_INT = &SCIC_TX_ISR;
    PieVectTable.SCIC_RX_INT = &SCIC_RX_ISR;
    PieVectTable.CANA0_INT = &CANIntHandler;
    PieVectTable.DMA_CH1_INT = &McBsp_DMA_Tx;//McBsp的发送DMA中断函数
    PieVectTable.DMA_CH2_INT = &McBsp_DMA_Rx;//McBsp的接收DMA中断函数/
    EDIS;    // This is needed to disable write to EALLOW protected registers

    EALLOW;
    CpuSysRegs.SECMSEL.bit.PF2SEL = 1;  //与dma链接
    EDIS;



    //选择串口CPU
    EALLOW;
//    DevCfgRegs.CPUSEL5.bit.SCI_A = 0;
    DevCfgRegs.CPUSEL5.bit.SCI_B = 1;
//    DevCfgRegs.CPUSEL5.bit.SCI_C = 0;
//    DevCfgRegs.CPUSEL8.bit.CAN_A = 0;
    EDIS;

    InitCpuTimers();
    ConfigCpuTimer(&CpuTimer0, 200, 250);
    CpuTimer0Regs.TCR.all = 0x4000; // Use write-only instruction to set TSS bit = 0

    //注册IPC的中断号（实例的地址，CPU2 IPC 中断号， CPU1 IPC 中断号）
    IPCInitialize(&g_sIpcController1, IPC_INT0, IPC_INT0);
    IPCInitialize(&g_sIpcController2, IPC_INT1, IPC_INT1);

    EnableInterrupts();



    InitIPC();
    InitScia();
    SCIC_Reg_Init();
    InitCAN();

    McBSP_A_DMA_Init();
    start_dma();
    McBsp_USART_Init();

    YinZhan_Set();
    //
    // Spin here until CPU02 is ready
    //
    while(!IPCRtoLFlagBusy(IPC_FLAG17));

    IPCRtoLFlagAcknowledge(IPC_FLAG17);
}

void YinZhan_Set(void)
{

    //配置GPIO11的下降沿中断
    EALLOW;
    XintRegs.XINT1CR.bit.POLARITY = 0;
    EDIS;
    GPIO_SetupXINT1Gpio(11);
    EALLOW;
    XintRegs.XINT1CR.bit.ENABLE = 1;
    PieCtrlRegs.PIEIER1.bit.INTx4 = 1;
    PieVectTable.XINT1_INT = &YJLJB_Interrupt;
    EDIS;

    //配置GPIO13的下降沿中断
    EALLOW;
    XintRegs.XINT2CR.bit.POLARITY = 0;
    EDIS;
    GPIO_SetupXINT2Gpio(13);
    EALLOW;
    XintRegs.XINT2CR.bit.ENABLE = 1;
    PieCtrlRegs.PIEIER1.bit.INTx5 = 1;
    PieVectTable.XINT2_INT = &DLG_Interrupt;
    EDIS;

    //配置GPIO14的下降沿中断
    EALLOW;
    XintRegs.XINT3CR.bit.POLARITY = 0;
    EDIS;
    GPIO_SetupXINT3Gpio(14);
    EALLOW;
    XintRegs.XINT3CR.bit.ENABLE = 1;
    PieCtrlRegs.PIEIER12.bit.INTx1 = 1;
    PieVectTable.XINT3_INT = &JGTC_Interrupt;
    EDIS;

    //配置GPIO41的下降沿中断
    EALLOW;
    XintRegs.XINT4CR.bit.POLARITY = 0;
    EDIS;
    GPIO_SetupXINT4Gpio(41);
    EALLOW;
    XintRegs.XINT4CR.bit.ENABLE = 1;
    PieCtrlRegs.PIEIER12.bit.INTx2 = 1;
    PieVectTable.XINT4_INT = &Battery_Interrupt;
    EDIS;
}

interrupt void YJLJB_Interrupt(void)
{
    YJLJB_flag = 1;

    PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;
}

interrupt void DLG_Interrupt(void)
{
    DLG_flag = 1;

    PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;
}

interrupt void JGTC_Interrupt(void)
{
    JGTC_flag = 1;

    PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;
}

interrupt void Battery_Interrupt(void)
{
    Battery_active_signal = 1;

    PieCtrlRegs.PIEACK.all = PIEACK_GROUP12;
}

//雷达探测目标，将方位角和高低角转为导引头的滚转和俯仰角度
void Cal_SeekerAngle(void)
{
    double X_DA,Y_DA,Z_DA;
    double Psi_BD,Theta_BD;
//    double Phi_S,Theta_S;
    X_DA = 1;
    Psi_BD = Rx_SimToFlight_Rx.TarAz_Angle * ToRad;
    Theta_BD = Rx_SimToFlight_Rx.TarHL_Angle * ToRad;
    Y_DA = tan(Psi_BD);
    Z_DA = -tan(Theta_BD)*(sqrt(pow(X_DA,2) + pow(Y_DA,2)));

    Phi_S = atan(-Y_DA / Z_DA) * ToDeg;//导引头滚转角
    Theta_S = atan(-(sqrt(pow(Y_DA,2) + pow(Z_DA,2))) / X_DA) * ToDeg;//导引头俯仰角

}
//
// End of file
//



