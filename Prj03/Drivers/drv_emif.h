/*******************************************************************************
 * 文件名：drv_emif.h
 * 描述：  EMIF驱动头文件 - 外部存储器接口驱动（迁移自EMIF_Proc）
 * 作者：  Auto-generated
 * 日期：  2025-10-20
 ******************************************************************************/

#ifndef DRV_EMIF_H_
#define DRV_EMIF_H_

#include "F28x_Project.h"

/******************************************************************************
 * EMIF地址定义（保留原EMIF_Proc.h的定义）
 ******************************************************************************/
#define ASRAM_CS2_START_ADDR 0x100000            // EMIF起始地址
#define ASRAM_CS2_SIZE       0x8000

#define FPGA_SCIA_TX_ADDR 0x100000            // SCIA_TX EMIF地址（RS485/TTP）
#define FPGA_SCIB_TX_ADDR 0x100200            // SCIB_TX EMIF地址（RS485/TTP）
#define FPGA_SCIC_TX_ADDR 0x100400            // SCIC_TX EMIF地址（RS485/TTP）
#define FPGA_SCID_TX_ADDR 0x100600            // SCID_TX EMIF地址（RS485/TTP）
#define FPGA_SCIE_TX_ADDR 0x100800            // SCIE_TX EMIF地址（RS422/RS485）
#define FPGA_SCIF_TX_ADDR 0x100A00            // SCIF_TX EMIF地址（RS422/RS485）
#define FPGA_SCIG_TX_ADDR 0x100C00            // SCIG_TX EMIF地址（RS422/RS485）
#define FPGA_SCIH_TX_ADDR 0x100E00            // SCIH_TX EMIF地址（RS422/RS485）

#define FPGA_SCIA_RX_ADDR 0x100100            // SCIA_RX EMIF地址（RS485/TTP）
#define FPGA_SCIB_RX_ADDR 0x100300            // SCIB_RX EMIF地址（RS485/TTP）
#define FPGA_SCIC_RX_ADDR 0x100500            // SCIC_RX EMIF地址（RS485/TTP）
#define FPGA_SCID_RX_ADDR 0x100700            // SCID_RX EMIF地址（RS485/TTP）
#define FPGA_SCIE_RX_ADDR 0x100900            // SCIE_RX EMIF地址（RS422/RS485）
#define FPGA_SCIF_RX_ADDR 0x100B00            // SCIF_RX EMIF地址（RS422/RS485）
#define FPGA_SCIG_RX_ADDR 0x100D00            // SCIG_RX EMIF地址（RS422/RS485）
#define FPGA_SCIH_RX_ADDR 0x100F00            // SCIH_RX EMIF地址（RS422/RS485）
#define FPGA_TIME_RX_ADDR 0x100200            // FPGA时间戳地址（每微秒更新）

#define DATA_OFFSET   0x10                    // EMIF数据地址偏移
#define STATE_OFFSET  0x0C                    // EMIF状态地址偏移
#define CNT_OFFSET    0x0E                    // EMIF计数地址偏移

/******************************************************************************
 * 函数声明
 ******************************************************************************/

/**
 * @brief  EMIF初始化
 * @param  无
 * @return 无
 */
void Drv_EMIF_Init(void);

/**
 * @brief  EMIF读取数据
 * @param  start_addr: 起始地址
 * @param  data: 数据缓冲区指针
 * @param  mem_size: 读取数据大小（字数）
 * @return 实际读取的字数
 */
int Drv_EMIF_Read(Uint32 start_addr, volatile unsigned int *data, Uint32 mem_size);

/**
 * @brief  EMIF写入数据
 * @param  start_addr: 起始地址
 * @param  data: 数据缓冲区指针
 * @param  mem_size: 写入数据大小（字数）
 * @return 实际写入的字数
 */
int Drv_EMIF_Write(Uint32 start_addr, volatile unsigned int *data, Uint32 mem_size);

/**
 * @brief  EMIF写入单个字
 * @param  start_addr: 地址
 * @param  data: 数据
 * @return 0表示成功
 */
int Drv_EMIF_WriteWord(Uint32 start_addr, unsigned int data);

#endif /* DRV_EMIF_H_ */

