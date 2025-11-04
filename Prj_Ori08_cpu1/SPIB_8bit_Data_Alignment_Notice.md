# SPIB 8位模式数据对齐重要说明

## ⚠️ 关键注意事项

**当使用SPIB的8位数据模式时，写入SPITXBUF的数据必须左对齐（Left-Justified）！**

## 问题描述

在8位SPI模式下，如果直接写入数据而不进行左对齐，会导致MOSI线上**无法输出正确的数据**，示波器观察时可能只看到低电平或错误的波形。

## 技术原理

根据**TI官方文档**（TMS320F28377D Technical Reference Manual, SPI章节）：

> **For characters with fewer than 16 bits:**
> - Data must be **left-justified** when written to SPIDAT and SPITXBUF.
> - Data read back from SPIRXBUF is **right-justified**.

### 为什么需要左对齐？

SPI数据寄存器（SPITXBUF/SPIDAT）是16位寄存器，但发送时是**从最高位（MSB）开始逐位移出**：

```
16位寄存器结构：
┌─────────────────┬─────────────────┐
│   Bit 15-8      │    Bit 7-0      │
│   (高字节)       │   (低字节)       │
└─────────────────┴─────────────────┘
      ↑
    MSB先发送
```

- **8位模式**：只发送高8位（Bit 15-8）
- **16位模式**：发送全部16位

## 错误示例 vs 正确示例

### ❌ 错误方式（未左对齐）

```c
// 错误：直接写入8位数据
void spib_xmit_wrong(Uint16 a)
{
    SpibRegs.SPITXBUF = a;  // 例如 a = 0xAA
}

// 实际写入：0x00AA
// ┌─────────────────┬─────────────────┐
// │     0x00        │     0xAA        │
// └─────────────────┴─────────────────┘
//        ↑
//    高8位 = 0x00（全是0！）
// 结果：MOSI线输出 00000000，示波器看不到数据
```

### ✅ 正确方式（左对齐）

```c
// 正确：数据左对齐（左移8位）
void spib_xmit(Uint16 a)
{
    SpibRegs.SPITXBUF = (a << 8);  // 例如 a = 0xAA
}

// 实际写入：0xAA00
// ┌─────────────────┬─────────────────┐
// │     0xAA        │     0x00        │
// └─────────────────┴─────────────────┘
//        ↑
//    高8位 = 0xAA
// 结果：MOSI线输出 10101010，示波器看到正确波形
```

## 数据对齐规则总结

| SPI模式 | 发送数据对齐 | 接收数据对齐 | 示例 |
|---------|-------------|-------------|------|
| **8位** | 左对齐（高字节） | 右对齐（低字节） | 发送0xAA：写入0xAA00 |
| **16位** | 自然对齐 | 自然对齐 | 发送0xAABB：写入0xAABB |

## 本项目的实现

### 配置信息

```c
// drv_spi.h
#define SPIB_CHAR_LENGTH  7  // 8位模式（SPICHAR=7表示8位）
```

### 发送函数实现

```c
// drv_spi.c
void spib_xmit(Uint16 a)
{
    // 8位SPI模式：数据必须左对齐（左移8位）
    SpibRegs.SPITXBUF = (a << 8);
}
```

### 使用示例

```c
// main1.c - 主循环发送测试数据
if(flag_500ms_spi)
{
    flag_500ms_spi = 0;
    
    // 发送0xAA（10101010）
    spib_xmit(0xAA);  // 内部会自动左对齐为0xAA00
    DELAY_US(100);
    
    // 读取接收数据（接收数据是右对齐的）
    test_result = SpibRegs.SPIRXBUF;  // 数据在低8位
}
```

## 接收数据处理

接收到的数据存储在SPIRXBUF中是**右对齐**的：

```c
// 接收到的8位数据在低字节
Uint16 received = SpibRegs.SPIRXBUF;  // 例如：0x00BB

// 提取实际的8位数据
Uint16 data = received & 0xFF;  // data = 0xBB
```

## 对比：spi_loopback_cpu01项目

**spi_loopback_cpu01使用16位SPI模式，不需要左对齐：**

```c
// F2837xD_Spi.c
SpiaRegs.SPICCR.bit.SPICHAR = (16-1);  // 16位模式

// spi_xmit - 16位模式直接发送
void spi_xmit(Uint16 a)
{
    SpiaRegs.SPITXBUF = a;  // 不需要左对齐
}
```

## 验证方法

### 示波器观察

发送0xAA（10101010）时，MOSI线应该看到：

```
时间 →
MOSI: ──┐  ┌──┐  ┌──┐  ┌──┐  ┌──
        └──┘  └──┘  └──┘  └──┘
        1  0  1  0  1  0  1  0
```

如果看到的是全低或全高，说明数据对齐有问题。

### 调试检查

在调试器中查看：

```c
// 断点设置在发送函数
spib_xmit(0xAA);

// 查看寄存器值
// SPITXBUF应该 = 0xAA00 ✅（左对齐）
// 如果是0x00AA ❌（错误，未左对齐）
```

## 常见问题

### Q1: 为什么示波器上MOSI没有高电平？
**A:** 最常见原因是数据未左对齐。检查发送函数是否有`<< 8`操作。

### Q2: 16位模式需要左对齐吗？
**A:** 不需要。16位模式下数据自然对齐，直接写入即可。

### Q3: 接收数据也需要处理对齐吗？
**A:** 接收数据是自动右对齐的，直接读取低8位即可：`data = SPIRXBUF & 0xFF`

### Q4: 如何判断当前是几位模式？
**A:** 查看`SPICCR.SPICHAR`：
- `SPICHAR = 7` → 8位模式（需要左对齐）
- `SPICHAR = 15` → 16位模式（不需要左对齐）

## 参考资料

1. **TMS320F28377D Technical Reference Manual**
   - Chapter 18: Serial Peripheral Interface (SPI)
   - Section 18.3.4: Data Format

2. **本项目相关文件**
   - `Prj_Ori08_cpu1/Drivers/drv_spi.c` - SPIB驱动实现
   - `Prj_Ori08_cpu1/Drivers/drv_spi.h` - 配置定义

3. **参考项目**
   - `spi_loopback_cpu01/` - TI官方16位SPI示例

## 修订历史

| 日期 | 版本 | 说明 |
|------|------|------|
| 2025-11-02 | 1.0 | 初始版本 - 添加8位SPI数据左对齐说明 |

---

**重要提醒：** 任何使用8位SPI模式的代码，都必须遵循此数据对齐规则！

