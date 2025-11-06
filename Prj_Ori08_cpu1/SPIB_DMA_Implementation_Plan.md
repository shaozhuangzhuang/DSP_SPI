# 计划文档：Prj_Ori08_cpu1 SPIB DMA实现方案 (v3.0 - 强化版)

**目标：** 构建一个独立的、健壮的、可维护的SPIB-DMA传输层，为AD5754R通信提供高效、可靠的数据通路，并包含完整的错误处理和状态管理机制。

**核心策略：**

1.  **分层架构：** 保持底层DMA、中间传输层、上层应用驱动的三层结构。
2.  **状态机驱动：** 引入明确的DMA状态机（空闲、忙碌、错误），使驱动状态可追溯。
3.  **显式接口：** 提供清晰的缓冲区访问和回调注册接口，杜绝实现细节的隐晦不明。
4.  **全面的错误处理：** 不仅处理超时，还处理DMA硬件本身可能产生的错误（如总线错误）。
5.  **原子操作：** 在关键代码路径使用中断保护，确保操作的原子性。

---

## 实施步骤 (Phase Breakdown)

### 阶段一：创建底层DMA驱动 (硬件抽象层)

**任务1：创建 `drv_dma.c/h` 文件**

*   **职责：** 封装所有对DMA寄存器的直接操作，作为DMA的“硬件引擎”。
*   **`drv_dma.h` 接口定义：**
    *   **数据结构：**
        *   `DMA_State_t`: `DMA_STATE_IDLE`, `DMA_STATE_BUSY`, `DMA_STATE_ERROR`
        *   `DMA_ErrorCode_t`: `DMA_ERR_NONE`, `DMA_ERR_BUSY`, `DMA_ERR_TIMEOUT`, `DMA_ERR_CONFIG`, `DMA_ERR_HARDWARE`
        *   `DMA_Callback_t`: `typedef void (*DMA_Callback_t)(void);`
    *   **接口函数：**
        *   `void Drv_DMA_Init(void);`
        *   `void Drv_DMA_Config_SPIB(void);`
        *   `DMA_ErrorCode_t Drv_DMA_Start_SPIB_Transfer(Uint16 word_count);`
        *   `void Drv_DMA_Register_SyncCallback(DMA_Callback_t callback);`
        *   `Uint16* Drv_DMA_Get_TX_Buffer(void);`
        *   `Uint16* Drv_DMA_Get_RX_Buffer(void);`
        *   `DMA_State_t Drv_DMA_Get_State(void);`
    *   **中断原型：**
        *   `__interrupt void DMA_CH6_ISR(void);` (RX完成)
        *   `__interrupt void DMA_Error_ISR(void);` (DMA错误)
*   **`drv_dma.c` 实现细节：**
    *   **内部变量：** 定义`static`的DMA缓冲区 (`s_tx_buffer[3]`, `s_rx_buffer[3]`)、状态变量 (`s_dma_state`) 和回调函数指针 (`s_sync_callback`)。
    *   **`Drv_DMA_Config_SPIB()`:**
        *   **触发源配置：** 设置`DMACHSRCSEL2`寄存器，将CH5触发源设为`111` (SPITXDMAB)，CH6触发源设为`112` (SPIRXDMAB)。
        *   **通道配置 (CH5/TX):** `DATASIZE=16bit`, `BURST_SIZE=2`, `SRC_BURST_STEP=+1`, `DST_BURST_STEP=0`, 硬件触发 (`PERINT_ENABLE=1`)，**禁用通道中断**。
        *   **通道配置 (CH6/RX):** `DATASIZE=16bit`, `BURST_SIZE=2`, `SRC_BURST_STEP=0`, `DST_BURST_STEP=+1`, 硬件触发，**使能通道中断**。
    *   **`Drv_DMA_Start_SPIB_Transfer()`:** 使用临界区保护，检查并设置状态机，然后按RX->TX顺序启动DMA。
    *   **`DMA_CH6_ISR()`:** 调用回调函数（如果已注册），然后将状态机设为`IDLE`。
    *   **`DMA_Error_ISR()`:** 将状态机设为`ERROR`，并可选择记录错误详情。

### 阶段二：创建中间SPI-DMA传输层 (通用服务层)

**任务2：在 `drv_spi.c/h` 中新增通用DMA传输函数**

*   **职责：** 封装一次完整的、带#SYNC控制和超时处理的SPIB-DMA事务。
*   **`drv_spi.h` 接口定义：**
    *   `void Drv_SPIB_DMA_Init(void);`
    *   `DMA_ErrorCode_t Drv_SPIB_TransmitReceive_DMA(Uint16* tx_data, Uint16* rx_data, Uint16 word_count, uint32_t timeout_ms);`
*   **`drv_spi.c` 实现细节：**
    *   **`Drv_SPIB_DMA_Init()`:** 调用`Drv_DMA_Register_SyncCallback(&AD5754_SYNC_HIGH);`来注册#SYNC拉高函数。
    *   **`Drv_SPIB_TransmitReceive_DMA()`:**
        1.  获取DMA的TX/RX缓冲区指针。
        2.  将`tx_data`复制到DMA的TX缓冲区。
        3.  **拉低#SYNC信号**。
        4.  调用`Drv_DMA_Start_SPIB_Transfer()`启动硬件传输。
        5.  **进入带超时的等待循环**，通过`Drv_DMA_Get_State()`检查DMA状态（IDLE, BUSY, ERROR）。
        6.  传输成功后，将DMA的RX缓冲区内容复制到`rx_data`。
        7.  返回最终的状态码。

### 阶段三：创建上层应用驱动 (设备协议层)

**任务3：在 `drv_spi.c/h` 中创建AD5754R专用的DMA函数**

*   **职责：** 封装AD5754R的24位命令协议，处理8位/16位对齐问题。
*   **`drv_spi.c` 实现细节：**
    *   **`AD5754_SendCommand_DMA()`:**
        1.  准备一个临时的3字节`tx_buffer`。
        2.  **数据打包：** 将24位`command`拆分为3个字节，**并分别左移8位**，存入`tx_buffer`。
        3.  调用`Drv_SPIB_TransmitReceive_DMA()`执行传输。
    *   **`AD5754_ReadCommand_DMA()`:**
        1.  执行与发送函数相同的步骤1-2。
        2.  调用`Drv_SPIB_TransmitReceive_DMA()`，并传入一个有效的`rx_buffer`。
        3.  **数据解析：** 传输成功后，从`rx_buffer`中读取3个16位字，**分别右移8位**，再重新组合为24位或32位的`response`。

### 阶段四：系统集成与测试 (强化版)

**任务4：修改 `main1.c` 并扩展测试用例**

*   **中断配置：** 在`System_Init()`中，配置PIE Group 7，将`DMA_CH6_INT` (INT7.6) 和 `DMASYS_INT` (INT7.7, DMA系统错误) 连接到对应的ISR，并使能中断。
*   **性能指标：** 设定量化目标，如“DMA模式下，CPU在SPI通信任务上的阻塞时间应小于10微秒”。
*   **扩展测试用例：**
    *   **边界测试：** 传输`0x000000`, `0xFFFFFF`等边界值。
    *   **异常测试：** 连续快速调用DMA传输函数，验证是否返回`DMA_ERR_BUSY`。
    *   **压力测试：** 在主循环中连续执行100万次DMA读写，监控错误计数器。
    *   **时序验收：** 使用示波器捕获#SYNC, SCLK, MOSI波形，确认#SYNC在整个DMA传输期间（约3个字节时间）保持低电平。

---

### 附录：关键技术决策

*   **#SYNC控制：** 通过回调函数实现，由底层ISR触发，中间层定义具体操作，实现了解耦。
*   **DMA触发方式：** 采用硬件触发（SPI FIFO），`PERINT_ENABLE=1`，效率最高。
*   **错误处理：** 增加专用的DMA错误中断ISR，处理硬件异常，提升驱动健壮性。
*   **兼容性：** 新增的所有DMA函数与现有FIFO函数完全独立，通过`#define USE_DMA_MODE`宏在应用层选择调用，不影响原有功能。
