# AD5754R通信测试使用指南

## 概述

本文档说明如何在Prj_Ori08_cpu1工程中测试SPIB与AD5754R的通信功能，包括自动化的写入-读取验证测试。

---

## 已实现的功能

### 1. SPIB GPIO配置（寄存器方式）

**文件**：`Prj_Ori08_cpu1/Drivers/drv_spi.c` - `Drv_InitSpibGpio()`

**GPIO引脚分配**：
- GPIO24：SPISIMOB（MOSI）- SPI数据输出
- GPIO25：SPISOMIB（MISO）- SPI数据输入
- GPIO26：SPICLKB（SCLK）- SPI时钟
- GPIO27：**普通GPIO**（手动SYNC控制）

**配置特点**：
- ✅ 寄存器直接配置（参考spi_loopback_cpu01）
- ✅ 推挽输出模式（确保信号质量）
- ✅ 异步输入限定
- ✅ 手动SYNC控制（适合AD5754R的24位命令）

### 2. 24位命令发送函数

#### AD5754_Send24BitCommand()

**功能**：发送24位命令到AD5754R（只发送，不接收）

**关键实现**：
```c
void AD5754_Send24BitCommand(uint32_t command)
{
    // 拆分为3个字节
    byte1 = (command >> 16) & 0xFF;
    byte2 = (command >> 8) & 0xFF;
    byte3 = command & 0xFF;

    // 手动控制SYNC
    AD5754_SYNC_LOW();
    
    // ⚠️ 关键：8位SPI必须左对齐
    SpibRegs.SPITXBUF = (byte1 << 8);  // 左移8位
    // 等待发送完成...
    
    AD5754_SYNC_HIGH();
}
```

#### AD5754_Send24BitCommand_WithRead()

**功能**：发送24位命令并接收AD5754R返回的数据

**关键实现**：
```c
uint32_t AD5754_Send24BitCommand_WithRead(uint32_t command)
{
    // 发送：左对齐
    SpibRegs.SPITXBUF = (byte1 << 8);
    
    // 接收：右对齐，取低8位
    rx_byte1 = SpibRegs.SPIRXBUF & 0xFF;
    
    // 组合返回24位数据
    return (rx_byte1 << 16) | (rx_byte2 << 8) | rx_byte3;
}
```

### 3. 自动化通信测试函数

#### Test_AD5754R_Communication()

**功能**：自动测试SPIB与AD5754R的通信是否正常

**测试流程**：
1. 写入电源控制寄存器（0x020011）
2. 发送读命令（0x900000）
3. 发送NOP命令获取读回数据
4. 验证读回数据是否与写入一致

**测试结果变量**：
- `ad5754_comm_test_pass`：通信测试结果（1=成功，0=失败）
- `ad5754_test_write_value`：写入的值（0x020011）
- `ad5754_test_read_value`：读回的值（应该是0x00xx0011）

---

## 使用方法

### 编译和下载

1. **编译工程**：Build Configuration → Debug
2. **下载程序**：使用仿真器下载到目标板
3. **运行程序**：F8启动运行

### 调试验证

#### 方法1：使用调试器查看变量

**步骤**：
1. 在`System_Init()`函数的第264行后设置断点
2. 运行程序，等待断点触发
3. 查看Watch窗口中的变量：

```c
ad5754_comm_test_pass    // 1 = 成功 ✅, 0 = 失败 ❌
ad5754_test_write_value  // 应该显示：0x020011
ad5754_test_read_value   // 低16位应该是：0x0011
```

**结果判断**：

| ad5754_comm_test_pass | ad5754_test_read_value | 说明 |
|----------------------|------------------------|------|
| **1** | 0x00xx0011 | 通信成功 ✅ |
| **0** | 0x00FFFF | MISO可能未连接或芯片无响应 |
| **0** | 0x000000 | MISO一直为低，检查硬件 |
| **0** | 其他值 | 通信有问题，检查时序 |

#### 方法2：使用示波器观察信号

**观察点**：

1. **SYNC (GPIO27)**：
   - 应该看到3次24位传输（写命令、读命令、NOP命令）
   - 每次传输SYNC拉低，24个SCLK周期后拉高

2. **SCLK (GPIO26)**：
   - 空闲为高电平（Mode 2）
   - 每帧24个时钟脉冲

3. **MOSI (GPIO24)**：
   - 第1帧：0x02, 0x00, 0x11（写命令）
   - 第2帧：0x90, 0x00, 0x00（读命令）
   - 第3帧：0x18, 0x00, 0x00（NOP命令）

4. **MISO (GPIO25)**：
   - 第3帧应该返回：0x00, 0x00, 0x11（之前写入的数据）

---

## 8位SPI数据对齐规则

### ⚠️ 核心原则

**发送（TX）**：数据必须左对齐
```c
// 发送0xAA
SpibRegs.SPITXBUF = (0xAA << 8);  // 写入0xAA00
```

**接收（RX）**：数据自动右对齐
```c
// 读取数据
uint8_t data = SpibRegs.SPIRXBUF & 0xFF;  // 取低8位
```

### 数据流示意图

```
发送0xAA到MOSI线：
┌──────────────┬──────────────┐
│ 写SPITXBUF   │   0xAA00     │ ← 必须左移8位
│              │ ┌─────┬────┐ │
│              │ │0xAA │0x00│ │
│              │ └─────┴────┘ │
│ 硬件发送→    │   ↑          │ ← 发送高8位
└──────────────┴──────────────┘

从MISO线接收0xBB：
┌──────────────┬──────────────┐
│ 读SPIRXBUF   │   0x00BB     │ ← 硬件自动右对齐
│              │ ┌─────┬────┐ │
│              │ │0x00 │0xBB│ │
│              │ └─────┴────┘ │
│ 取低8位→     │        ↑     │ ← & 0xFF
└──────────────┴──────────────┘
```

---

## 故障排除

### 问题1：ad5754_comm_test_pass = 0

**可能原因**：
1. 硬件连接问题（检查MISO线）
2. AD5754R未上电或损坏
3. SPI时序不匹配
4. 数据对齐错误

**解决步骤**：
1. 用万用表检查AD5754R供电（DVCC=3.3V, AVDD/AVSS=±12V或±15V）
2. 检查MISO线连接
3. 用示波器观察MISO是否有响应
4. 降低SPI时钟频率重试

### 问题2：ad5754_test_read_value = 0xFFFFFF

**原因**：MISO线一直为高电平

**检查**：
1. MISO线是否连接到AD5754R的SDO引脚
2. AD5754R是否正常供电
3. GPIO25是否正确配置为输入

### 问题3：ad5754_test_read_value = 0x000000

**原因**：MISO线一直为低电平

**检查**：
1. MISO线是否短路到地
2. AD5754R的SDO引脚是否正常
3. 芯片是否处于复位状态

### 问题4：示波器看不到MOSI数据

**原因**：数据未左对齐

**解决**：确认所有发送代码都使用了`<< 8`：
```c
SpibRegs.SPITXBUF = (data << 8);  // ✅ 正确
SpibRegs.SPITXBUF = data;         // ❌ 错误
```

---

## 下一步：DAC功能测试

当`ad5754_comm_test_pass = 1`后，可以继续进行DAC功能测试：

1. 设置输出范围
2. 上电DAC通道
3. 设置DAC输出值
4. 用万用表测量输出电压

详细步骤参见：`AD5754R_Integration_Guide.md`

---

## 技术参数

| 参数 | 值 | 说明 |
|------|----|----|
| SPI模式 | Mode 2 | CPOL=1, CPHA=0 |
| 数据宽度 | 8位 | SPICHAR=7 |
| 波特率 | 500kHz | SPIBRR=99 |
| 命令格式 | 24位 | 3个连续的8位字节 |
| SYNC控制 | 手动 | GPIO27软件控制 |

---

## 重要提醒

1. **数据对齐是关键**：所有8位SPI发送必须左移8位
2. **SYNC时序必须正确**：24位完整传输期间SYNC保持低电平
3. **超时保护**：所有发送函数都有10000次超时检测
4. **延时要求**：严格按照AD5754R时序要求添加延时

---

## 修改文件清单

### ✅ 已修改文件

1. **Prj_Ori08_cpu1/Drivers/drv_spi.c**
   - `Drv_InitSpibGpio()`：GPIO27改为手动GPIO控制
   - `AD5754_Send24BitCommand()`：添加左对齐（<< 8）
   - `AD5754_Send24BitCommand_WithRead()`：添加左/右对齐处理
   - `Test_AD5754R_Communication()`：新增自动化通信测试

2. **Prj_Ori08_cpu1/Drivers/drv_spi.h**
   - 恢复`AD5754_SYNC_*`宏定义
   - 添加函数声明
   - 添加测试变量声明

3. **Prj_Ori08_cpu1/main1.c**
   - `System_Init()`：添加通信测试调用
   - 主循环：更新为使用AD5754命令发送

---

**文档创建时间**：2025-11-02
**适用工程**：Prj_Ori08_cpu1
**芯片型号**：TMS320F28377D + AD5754R

