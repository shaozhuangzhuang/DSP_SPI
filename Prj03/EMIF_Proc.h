/*
 * EMIF_Proc.h
 *
 *  Created on: 2020年7月5日
 *      Author: A
 */

#ifndef EMIF_PROC_H_
#define EMIF_PROC_H_

#define ASRAM_CS2_START_ADDR 0x100000            //EMIF start address
#define ASRAM_CS2_SIZE       0x8000

#define FPGA_SCIA_TX_ADDR 0x100000            //SCIA_TX EMIF start address （RS485/TTP）
#define FPGA_SCIB_TX_ADDR 0x100200            //SCIB_TX EMIF start address （RS485/TTP）
#define FPGA_SCIC_TX_ADDR 0x100400            //SCIC_TX EMIF start address （RS485/TTP）
#define FPGA_SCID_TX_ADDR 0x100600            //SCID_TX EMIF start address （RS485/TTP）
#define FPGA_SCIE_TX_ADDR 0x100800            //SCIE_TX EMIF start address （RS422/RS485）
#define FPGA_SCIF_TX_ADDR 0x100A00            //SCIF_TX EMIF start address （RS422/RS485）
#define FPGA_SCIG_TX_ADDR 0x100C00            //SCIG_TX EMIF start address （RS422/RS485）
#define FPGA_SCIH_TX_ADDR 0x100E00            //SCIH_TX EMIF start address （RS422/RS485）

#define FPGA_SCIA_RX_ADDR 0x100100            //SCIA_RX EMIF start address （RS485/TTP）
#define FPGA_SCIB_RX_ADDR 0x100300            //SCIB_RX EMIF start address （RS485/TTP）
#define FPGA_SCIC_RX_ADDR 0x100500            //SCIC_RX EMIF start address （RS485/TTP）
#define FPGA_SCID_RX_ADDR 0x100700            //SCID_RX EMIF start address （RS485/TTP）
#define FPGA_SCIE_RX_ADDR 0x100900            //SCIE_RX EMIF start address （RS422/RS485）
#define FPGA_SCIF_RX_ADDR 0x100B00            //SCIF_RX EMIF start address （RS422/RS485）
#define FPGA_SCIG_RX_ADDR 0x100D00            //SCIG_RX EMIF start address （RS422/RS485）
#define FPGA_SCIH_RX_ADDR 0x100F00            //SCIG_RX EMIF start address （RS422/RS485）
#define FPGA_TIME_RX_ADDR 0x100200            //FPGA_TIME_RX start address (update per us)

#define DATA_OFFSET 0x10                      //EMIF Data address offset
#define STATE_OFFSET 0x0C                      //EMIF Data address offset
#define CNT_OFFSET 0x0E                      //EMIF Data address offset

extern void InitEMIF(void);
extern int mem_write(Uint32 start_addr, volatile unsigned int *data, Uint32 mem_size);
extern int mem_read(Uint32 start_addr, volatile unsigned int *data, Uint32 mem_size);
extern int word_write(Uint32 start_addr, unsigned int data);

#endif /* EMIF_PROC_H_ */
