# TMS320F28377D SPIA-SPIB通信实施指南

本文档根据`Prj_Ori08_cpu1`工程的实际情况,提供一份详尽的、分步的SPI双向通信功能实施方案。

---

### 阶段 0: 准备与验证

在开始编码前,请完成以下硬件和环境的确认工作。

-   **任务 0.1: 确认硬件物理连接**
    -   **操作**: 仔细核对电路板,确保SPIA和SPIB的引脚一一对应正确连接。
    -   **连接表**:
        -   `GPIO55 (SPISIMOA)` <--> `GPIO24 (SPISIMOB)` (MOSI)
        -   `GPIO56 (SPISOMIA)` <--> `GPIO25 (SPISOMIB)` (MISO)
        -   `GPIO54 (SPICLKA)` <--> `GPIO26 (SPICLKB)` (CLK)
        -   `GPIO57 (SPISTEA)` <--> `GPIO27 (SPISTEB)` (STE)

-   **任务 0.2: 确认LSPCLK频率**
    -   **操作**: 检查`InitSysCtrl()`函数确保低速外设时钟(LSPCLK)已配置为50MHz。所有后续的时序计算(如波特率、定时器周期)都基于此频率。

---

### 阶段 1: 初始化代码编写

此阶段将完成所有硬件模块的初始化配置代码。

-   **任务 1.1: 在项目中创建SPI驱动文件**
    -   **操作**: 在`Prj_Ori08_cpu1/Drivers`目录下,创建`drv_spi.c`和`drv_spi.h`两个新文件。

-   **任务 1.2: 编写GPIO初始化代码**
    -   **操作**: 在`drv_spi.h`中声明`void InitSpiGpios(void);`。
    -   在`drv_spi.c`中实现该函数,使用`GPIO_SetupPinMux`为SPIA和SPIB共8个引脚配置正确的复用模式。

-   **任务 1.3: 编写中断系统初始化代码**
    -   **操作**: 在`drv_spi.h`中声明`void InitSpiInterrupts(void);`。
    -   在`drv_spi.c`中实现该函数,完成PIE初始化,将ISR函数地址注册到`PieVectTable`,并使能PIE中断组1和6中对应的中断位。

-   **任务 1.4: 编写SPI外设初始化代码**
    -   **操作**: 在`drv_spi.h`中声明`void InitSpiModules(void);`。
    -   在`drv_spi.c`中实现该函数,严格按照“先从机(SPIB)后主机(SPIA)”的顺序,通过直接操作寄存器完成配置。**关键参数**: SPIA的`SPIBRR`寄存器应设置为`24`以获得2MHz波特率。

-   **任务 1.5: 编写CPU定时器初始化代码**
    -   **操作**: 在`drv_spi.h`中声明`void InitTimer0ForSpi(void);`。
    -   在`drv_spi.c`中实现该函数,通过直接操作`CpuTimer0Regs`寄存器,将其配置为10ms周期的中断源。

---

### 阶段 2: 中断服务程序(ISR)实现

此阶段是程序运行时的核心,由中断驱动。

-   **任务 2.1: 定义ISR所需的全局/静态变量**
    -   **操作**: 在`main1.c`或`drv_spi.c`中,定义用于任务调度的静态计数器`taskCounter`,以及用于接收SPI数据的全局缓冲区数组(例如`spiARxBuffer[4]`和`spiBRxBuffer[4]`)。

-   **任务 2.2: 实现CPU定时器ISR (`cpuTimer0ISR`)**
    -   **操作**: 编写该函数,实现以下逻辑:
        1.  `taskCounter`自增。
        2.  准备SPIA的8字节数据,分4次写入`SpiaRegs.SPITXBUF` (10ms任务)。
        3.  判断`taskCounter`是否达到2,如果达到则准备SPIB的8字节数据,分4次写入`SpibRegs.SPITXBUF`,然后将`taskCounter`清零 (20ms任务)。
        4.  向PIE应答中断: `PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;`。

-   **任务 2.3: 实现SPIA接收ISR (`spiARxISR`)**
    -   **操作**: 编写该函数,实现以下逻辑:
        1.  循环4次,从`SpiaRegs.SPIRXBUF`读取数据并存入`spiARxBuffer`。
        2.  (可选)设置一个标志位,通知主循环数据已更新。
        3.  清除中断标志并向PIE应答: `SpiaRegs.SPIFFRX.bit.RXFFINTCLR=1; PieCtrlRegs.PIEACK.all = PIEACK_GROUP6;`。

-   **任务 2.4: 实现SPIB接收ISR (`spiBRxISR`)**
    -   **操作**: 逻辑与`spiARxISR`完全相同,但操作对象是SPIB的寄存器和缓冲区。

---

### 阶段 3: 最终整合与测试

此阶段将所有代码模块整合到主函数中并进行测试。

-   **任务 3.1: 在`main`函数中调用初始化函数**
    -   **操作**: 在`main1.c`顶部`#include "Drivers/drv_spi.h"`。
    -   在`main`函数中,依次调用`InitSpiGpios()`, `InitSpiInterrupts()`, `InitSpiModules()`, `InitTimer0ForSpi()`。

-   **任务 3.2: 使能总中断并进入主循环**
    -   **操作**: 在调用完所有初始化函数后,通过`IER |= M_INT1 | M_INT6;`使能CPU中断组,然后调用`EINT;`全局使能中断。最后进入`for(;;){}`无限循环。

-   **任务 3.3: 编译、下载和调试**
    -   **操作**: 编译工程并下载到目标板。在3个ISR的入口处和`for(;;)`循环处设置断点。

-   **任务 3.4: 功能验证**
    -   **操作**: 全速运行代码,观察`cpuTimer0ISR`断点是否稳定进入。通过`SpiaRegs.SPITXBUF`写入数据后,观察`spiBRxISR`断点是否被触发。
    -   在接收ISR的断点处,使用调试器的Watch窗口观察接收缓冲区的内容,确认数据是否与发送方的数据帧一致。
