/*
 * MCBSP.h
 *
 *  Created on: 2022年9月14日
 *      Author: 110
 */

#ifndef COMMUNICATION_MCBSP_MCBSP_H_
#define COMMUNICATION_MCBSP_MCBSP_H_

#include "F28x_Project.h"

/* MCLKXA-MAP(GPIO)
 *  22\86
 * */

#define MCLKX_A 22

/* MCLKRA-MAP(GPIO)
 *  7\58
 * */
#define MCLKR_A 7


/* MFSXA-MAP(GPIO)
 *  23\87
 * */
#define MFSX_A 23


/* MFSRA-MAP(GPIO)
 *  5\59
 * */
#define MFSR_A 5



/* MDXA-MAP(GPIO)
 *  20\84
 * */
#define MDX_A 20


/* MDRA-MAP(GPIO)
 *  21\85
 * */
#define MDR_A 21


// McBsp_A的DMA发送中断函数
interrupt void McBsp_DMA_Tx(void);
// McBsp_A的DMA接收中断函数
interrupt void McBsp_DMA_Rx(void);

extern void McBsp_USART_Init(void);
void McBsp_GPIO_Init(void);
void McBsp_REG_Init(void);
// McBsp_A的DMA初始化
extern void McBSP_A_DMA_Init(void);
extern void start_dma(void);
//McBSP接收函数
extern void McBSP_RX(void);
// McBSP发送函数
extern void McBSP_xmit(int a, int b);
extern void McBSP_TX(void);
extern void McBSP_Encode(Uint16 *Dat,Uint16 len);
extern void McBSP_Decode(union HexData* Src, Uint16* tar,int len);
//extern void McBSP_Decode1(void);
//extern int McBsp_Rec_Flag;
extern union HexData McBSP_Rx_SCI_Interim[128];
extern Uint16 McBSP_Sdata[128];                    // Sent Data
extern Uint16 McBSP_Rdata[128];                    // Received Data
extern Uint16 McBSP_Rdata_use[128];                    // Received Data
extern Uint16 McBSP_Sdata_NoEncode[128];                    // Sent Data
extern unsigned int Mcbsp_Txdate[42];
extern Uint32 SCI_TX_Cnt;
extern Uint16 crc16_Ibm(unsigned char *data, Uint32 length);

extern float Servo_Deflection1,Servo_Deflection2,Servo_Deflection3,Servo_Deflection4;


#endif /* COMMUNICATION_MCBSP_MCBSP_H_ */
