/*******************************************************************************
 * 文件名：drv_emif.c
 * 描述：  EMIF驱动实现 - 外部存储器接口驱动（迁移自EMIF_Proc.c）
 * 作者：  Auto-generated
 * 日期：  2025-10-20
 ******************************************************************************/

#include "drv_emif.h"

/******************************************************************************
 * 私有变量
 ******************************************************************************/
static Uint16 ErrCount = 0;  // 错误计数

/******************************************************************************
 * 私有函数声明
 ******************************************************************************/
extern void setup_emif1_pinmux_async_16bit(Uint16 cpu_sel);

/******************************************************************************
 * 函数实现
 ******************************************************************************/

/**
 * @brief  EMIF初始化（迁移自InitEMIF）
 */
void Drv_EMIF_Init(void)
{
    EALLOW;
    
    // 配置EMIF时钟分频
    ClkCfgRegs.PERCLKDIVSEL.bit.EMIF1CLKDIV = 0x0;
    
    EDIS;

    EALLOW;
    
    // 配置访问保护
    Emif1ConfigRegs.EMIF1ACCPROT0.all = 0x0;
    if(Emif1ConfigRegs.EMIF1ACCPROT0.all != 0x0)
    {
        ErrCount++;
    }

    // 提交保护配置（此位设置后EMIF1ACCPROT0寄存器内容不可更改）
    Emif1ConfigRegs.EMIF1COMMIT.all = 0x1;
    if(Emif1ConfigRegs.EMIF1COMMIT.all != 0x1)
    {
        ErrCount++;
    }

    // 锁定配置（EMIF1COMMIT寄存器不可更改）
    Emif1ConfigRegs.EMIF1LOCK.all = 0x1;
    if(Emif1ConfigRegs.EMIF1LOCK.all != 1)
    {
        ErrCount++;
    }

    EDIS;

    // 配置EMIF1引脚复用（16位异步）
    setup_emif1_pinmux_async_16bit(0);

    // 配置CS2空间时序
    Emif1Regs.ASYNC_CS2_CR.all = (EMIF_ASYNC_ASIZE_32    | // 16位存储器接口
                                  EMIF_ASYNC_TA_1        | // Turn Around时间
                                  EMIF_ASYNC_RHOLD_8     | // 读保持时间
                                  EMIF_ASYNC_RSTROBE_64  | // 读选通时间
                                  EMIF_ASYNC_RSETUP_16   | // 读建立时间
                                  EMIF_ASYNC_WHOLD_8     | // 写保持时间
                                  EMIF_ASYNC_WSTROBE_64  | // 写选通时间
                                  EMIF_ASYNC_WSETUP_16   | // 写建立时间
                                  EMIF_ASYNC_EW_DISABLE  | // 扩展等待禁用
                                  EMIF_ASYNC_SS_DISABLE);  // Strobe Select模式禁用
}

/**
 * @brief  EMIF读取数据（迁移自mem_read）
 */
int Drv_EMIF_Read(Uint32 start_addr, volatile unsigned int *data, Uint32 mem_size)
{
    long *XMEM_ps;
    int i;
    unsigned int d;

    XMEM_ps = (long *)start_addr;
    
    for(i = 0; i < mem_size; i++)
    {
        d = (unsigned int)(*XMEM_ps & 0x0000FFFF);
        *(data + i) = d;
        XMEM_ps++;
    }
    
    return i;
}

/**
 * @brief  EMIF写入数据（迁移自mem_write）
 */
int Drv_EMIF_Write(Uint32 start_addr, volatile unsigned int *data, Uint32 mem_size)
{
    long *XMEM_ps;
    int i;
    unsigned int d;

    XMEM_ps = (long *)start_addr;

    for(i = 0; i < mem_size; i++)
    {
        d = *(data + i);
        *XMEM_ps = d;
        XMEM_ps++;
    }

    return i;
}

/**
 * @brief  EMIF写入单个字（迁移自word_write）
 */
int Drv_EMIF_WriteWord(Uint32 start_addr, unsigned int data)
{
    long *XMEM_ps;
    
    XMEM_ps = (long *)start_addr;
    *XMEM_ps = data;

    return 0;
}

