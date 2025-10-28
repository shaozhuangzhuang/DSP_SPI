/*
 * EMIF_Proc.c
 *
 *  Created on: 2020年7月5日
 *      Author: A
 */

#include "F28x_Project.h"
#include "EMIF_Proc.h"


Uint16  ErrCount = 0;

void InitEMIF(void)
{
    EALLOW;
    ClkCfgRegs.PERCLKDIVSEL.bit.EMIF1CLKDIV = 0x0;
    EDIS;

    EALLOW;
    // 禁用访问保护
    Emif1ConfigRegs.EMIF1ACCPROT0.all = 0x0;
    if(Emif1ConfigRegs.EMIF1ACCPROT0.all != 0x0)
    {
        ErrCount++;
    }

    //
    // Commit the configuration related to protection. Till this bit remains set
    // content of EMIF1ACCPROT0 register can't be changed.
    //
    Emif1ConfigRegs.EMIF1COMMIT.all = 0x1;
    if(Emif1ConfigRegs.EMIF1COMMIT.all != 0x1)
    {
        ErrCount++;
    }

    //
    // Lock the configuration so that EMIF1COMMIT register can't be
    // changed any more.
    //
    Emif1ConfigRegs.EMIF1LOCK.all = 0x1;
    if(Emif1ConfigRegs.EMIF1LOCK.all != 1)
    {
        ErrCount++;
    }

    EDIS;

    // 配置 EMIF1 引脚
    setup_emif1_pinmux_async_16bit(0);

    // 配置 CS2 空间时序
    Emif1Regs.ASYNC_CS2_CR.all = (EMIF_ASYNC_ASIZE_32    | // 16 位内存接口
                                  EMIF_ASYNC_TA_1        | // Turn Around 时间
                                  EMIF_ASYNC_RHOLD_8     | // 读保持时间
                                  EMIF_ASYNC_RSTROBE_64  | // 读选通时间
                                  EMIF_ASYNC_RSETUP_16   | // 读建立时间
                                  EMIF_ASYNC_WHOLD_8     | // 写保持时间
                                  EMIF_ASYNC_WSTROBE_64  | // 写选通时间
                                  EMIF_ASYNC_WSETUP_16   | // 写建立时间
                                  EMIF_ASYNC_EW_DISABLE  | // 扩展等待禁用
                                  EMIF_ASYNC_SS_DISABLE);  // Strobe Select 模式禁用
}


int mem_write(Uint32 start_addr, volatile unsigned int *add, Uint32 mem_size)
{
    long *XMEM_ps;
    int i;
    unsigned int d;

    XMEM_ps = (long *)start_addr;

    for(i = 0; i < mem_size; i++)
    {

        d = *(add + i);
        *XMEM_ps = d;//(unsigned long int)(*(add + i));
        XMEM_ps++;
    }

    return i;
}

int word_write(Uint32 start_addr, unsigned int data)
{
    long *XMEM_ps;
    XMEM_ps = (long *)start_addr;

    *XMEM_ps = data;

    return 0;
}

int mem_read(Uint32 start_addr, volatile unsigned int *add,  Uint32 mem_size)
{
    long *XMEM_ps;
    int i;
    unsigned int d;

    XMEM_ps = (long *)start_addr;
    for(i = 0; i < mem_size; i++)
    {
        d = (unsigned int)(*XMEM_ps & 0x0000FFFF);
        *(add + i) = d;

        XMEM_ps++;
    }
    return i;
}

//============ End of File EMIF_Proc.c =============================
