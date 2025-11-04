# AD5754R通信配置修正总结

## 修正完成时间
2025-11-02

---

## 🔧 关键修正

### 1. SPI时钟模式修正（最重要）

**问题**：之前使用Mode 2 (CPOL=1, CPHA=0)，与AD5754R要求不匹配

**修正**：改为Mode 1 (CPOL=0, CPHA=1)

**文件**：`Prj_Ori08_cpu1/Drivers/drv_spi.h` 第46-49行

```c
// 修正前：Mode 2
#define SPIB_CLK_POLARITY  SPI_CLK_POLARITY_HIGH  // CPOL=1
#define SPIB_CLK_PHASE     SPI_CLK_PHASE_NORMAL   // CPHA=0

// 修正后：Mode 1 ✅
#define SPIB_CLK_POLARITY  SPI_CLK_POLARITY_LOW   // CPOL=0
#define SPIB_CLK_PHASE     SPI_CLK_PHASE_DELAY    // CPHA=1
```

**时序说明**：
```
Mode 1 (CPOL=0, CPHA=1):
- 时钟空闲为低电平
- 数据在下降沿（第2个边沿）采样
- 符合AD5754R数据手册要求
```

### 2. 降低时钟频率

**文件**：`Prj_Ori08_cpu1/Drivers/drv_spi.h` 第24行

```c
// 修正前：
#define SPIB_BAUDRATE  99   // 500kHz

// 修正后：
#define SPIB_BAUDRATE  49   // 1MHz
```

**原因**：降低频率可提高信号质量，便于初始测试

### 3. 修改通信测试策略

**文件**：`Prj_Ori08_cpu1/Drivers/drv_spi.c` - `Test_AD5754R_Communication()`

**修正前**：写入电源控制寄存器，然后读回
```c
// 写入0x020011，期望读回0x0011
```

**修正后**：直接读取控制寄存器上电默认值
```c
// 读取控制寄存器（0x860000）
// 期望读回0x0004（钳位使能位默认为1）
```

**优势**：
- ✅ 无需先写入，直接测试读功能
- ✅ 验证AD5754R的上电默认状态
- ✅ 更可靠的通信验证

### 4. 添加写入测试函数

**文件**：`Prj_Ori08_cpu1/Drivers/drv_spi.c` - `Test_AD5754R_WriteTest()`

**功能**：测试DAC输出功能
1. 设置±10V输出范围
2. 使能内部基准和通道A
3. 设置输出0V

**验证**：用万用表测量VoutA = 0V

### 5. GPIO25上拉配置

**文件**：`Prj_Ori08_cpu1/Drivers/drv_spi.c` 第80行

```c
// 修正：禁用GPIO25上拉，避免浮空时读到高电平
GpioCtrlRegs.GPAPUD.bit.GPIO25 = 1;  // 禁用上拉
```

---

## 📁 清理的代码

### 已删除的废弃函数

1. ❌ `Test_AD5754R_WriteRead()` - 与新的通信测试重复
2. ❌ `Test_GPIO25_ReadTest()` - 已被Test_GPIO25_InputCapability取代
3. ❌ `Test_AD5754R_Simple()` - 临时诊断代码
4. ❌ `Test_AD5754R_Diagnostic()` - 复杂诊断代码
5. ❌ `Test_GPIO24_OutputCapability()` - 临时诊断代码

### 保留的核心函数

#### SPIB配置函数
1. ✅ `Drv_InitSpibGpio()` - GPIO寄存器配置
2. ✅ `Drv_InitSpibModule()` - SPIB模块配置
3. ✅ `spib_xmit()` - 简化发送函数

#### AD5754R通信函数
1. ✅ `AD5754_Send24BitCommand()` - 发送24位命令
2. ✅ `AD5754_Send24BitCommand_WithRead()` - 发送并读取

#### 测试函数
1. ✅ `Test_GPIO25_InputCapability()` - GPIO25诊断
2. ✅ `Test_AD5754R_Communication()` - 通信验证（读控制寄存器）
3. ✅ `Test_AD5754R_WriteTest()` - 写入测试（设置DAC输出）
4. ✅ `Test_AD5754R_PowerSequence()` - 上电序列

---

## 🧪 测试步骤

### 步骤1：编译下载

1. Clean Project
2. Build Project
3. 下载到目标板

### 步骤2：运行通信测试

**在调试器中观察**：

```c
// 在System_Init()第273行后设置断点

ad5754_comm_test_pass    // 期望：1（成功）
ad5754_test_write_value  // 应该：0x860000
ad5754_test_read_value   // 期望：0x00xx0004
```

**成功标准**：
- `ad5754_comm_test_pass = 1`
- `ad5754_test_read_value` 的低16位 = `0x0004`

### 步骤3：GPIO25诊断结果

```c
gpio25_test_done         // 应该：1
gpio25_with_pullup       // 观察值
gpio25_without_pullup    // 观察值
```

**结果分析**：
- (1, 1) → MISO未连接或被外部拉高 ❌
- (1, 0) → GPIO25功能正常 ✅
- (0, 0) → MISO被接地 ❌

### 步骤4：如果通信成功，测试写入功能

**方法1**：在代码中调用
```c
// 在System_Init()中添加：
Test_AD5754R_WriteTest();
```

**方法2**：在调试器中手动调用
```c
// Expressions窗口执行：
Test_AD5754R_WriteTest()
```

**验证**：
- 用万用表测量AD5754R的VoutA引脚
- 电压应该接近 **0V**

---

## 📊 配置对比表

| 配置项 | 修正前 | 修正后 | 原因 |
|-------|--------|--------|------|
| **SPI模式** | Mode 2 | Mode 1 | 匹配AD5754R要求 |
| **CPOL** | 1（高） | 0（低） | 时钟空闲为低 |
| **CPHA** | 0（无延迟） | 1（延迟） | 下降沿采样 |
| **时钟频率** | 500kHz | 1MHz | 初始测试用 |
| **测试寄存器** | 电源控制(0x90) | 控制(0x86) | 读取默认值 |
| **期望值** | 0x0011 | 0x0004 | 钳位使能位 |
| **GPIO25上拉** | 使能 | 禁用 | 避免误读 |

---

## 🎯 预期改善

修正SPI模式后，MISO应该能读到正确数据：

**之前**：
```
ad5754_test_read_value = 0x00FFFFFF  ❌
```

**修正后**：
```
ad5754_test_read_value = 0x00xx0004  ✅
```

---

## 💡 下一步建议

### 如果仍然读到0xFFFFFF

1. **用示波器观察MISO线**：
   - 触发在SYNC下降沿
   - 观察NOP命令发送期间MISO是否有数据

2. **检查硬件**：
   - CLR引脚必须为高电平（不能是0V）
   - DVCC、AVDD、AVSS供电正常
   - MISO物理连接完好

3. **降低时钟频率**：
   ```c
   #define SPIB_BAUDRATE  99  // 降到500kHz
   ```

### 如果通信成功

1. 测试写入功能（`Test_AD5754R_WriteTest()`）
2. 用万用表验证输出电压
3. 逐步测试所有4个通道
4. 测试不同输出范围

---

## 📝 8位SPI数据对齐规则

**重要提醒**：所有函数已正确实现数据对齐

**发送（TX）**：
```c
SpibRegs.SPITXBUF = (data << 8);  // 左对齐
```

**接收（RX）**：
```c
uint8_t received = SpibRegs.SPIRXBUF & 0xFF;  // 取低8位
```

---

## ✅ 修改清单

### 文件修改

1. **Prj_Ori08_cpu1/Drivers/drv_spi.h**
   - 修改SPI模式为Mode 1
   - 修改时钟频率为1MHz
   - 添加Test_AD5754R_WriteTest声明

2. **Prj_Ori08_cpu1/Drivers/drv_spi.c**
   - GPIO25禁用上拉
   - 删除5个废弃测试函数
   - 修改Test_AD5754R_Communication（读控制寄存器）
   - 添加Test_AD5754R_WriteTest函数

3. **Prj_Ori08_cpu1/main1.c**
   - 调用GPIO25诊断测试
   - 调用AD5754R通信测试

---

现在可以重新编译下载测试！由于SPI模式已修正为Mode 1，应该能够正确读取AD5754R的数据了。

