/*
 * Sci_D.h
 *
 *  Created on: 2022年8月2日
 *      Author: 110
 */

#ifndef COMMUNICATION_SCI_D_SCI_D_H_
#define COMMUNICATION_SCI_D_SCI_D_H_

void InitScid(void);
void scid_xmit(int a);
void scia_msg(char * msg);
void SCID_ReadFIFO(void);
void SCID_RX(void);
void SCID_TX(void);
Uint16 crc16_IBM(unsigned char *data, Uint32 length);

#define RXBUF_MASK 0xFF
struct RX_BUF
{
    unsigned char rx_buf[RXBUF_MASK + 1];
    unsigned int rx_head;
    unsigned int rx_tail;     // Data of frame
};

typedef union U16_Byte
{
    Uint16 U16;
    struct
    {
        unsigned char Byte_0:8;
        unsigned char Byte_1:8;
    }Byte;
}U16_Byte;

typedef union U32_Byte
{
    Uint32 U32;
    struct
    {
        unsigned char Byte_0:8;
        unsigned char Byte_1:8;
        unsigned char Byte_2:8;
        unsigned char Byte_3:8;
    }BYTE;
}U32_Byte;

//SCI Communication frame structure
struct TX_FRAME_Integrated_Control  // #1号控制器SCIA向综控机发送综控帧
{
    Uint16 SYNC[2];
    Uint16 LONGTH;
    Uint16 TXCNT[4];
    Uint16 RXCNT[4];
    Uint16 STATE[2];
    Uint16 FUNC[2];
    Uint16 DATA[20];
    Uint16 CSUM;

    // #1号控制器SCIA发送综控帧36字节
};

struct RX_FRAME_Integrated_Control_12SCIA           // #1号控制器和#2号控制器同时接收综控帧并各自进行解算
{
    Uint16 SYNC[2];
    Uint16 LONGTH;
    Uint16 FRAMECNT[4];
    Uint16 DATA[12];        // Data of frame
    Uint16 CSUM;            // Checksum of frame

    // #1、2号控制器SCIA接收综控帧20字节
};

struct COM_VAR
{
    unsigned int flightzero_cmd[2];
    unsigned int position_cmd1[2];
    unsigned int position_cmd2[2];
    unsigned int position_cmd3[2];
    unsigned int position_cmd4[2];
    unsigned int tx_cnt[4];
    unsigned int rx_cnt[4];
    unsigned int frame_cnt;
    unsigned int speed_cmd;
    unsigned int speed_fbk;
    unsigned int sys_voltage;
    unsigned int sys_current;
    unsigned int PhA_current;
    unsigned int PhB_current;
    unsigned int PhC_current;
    unsigned int PhD_current;
    unsigned int PhAC_current;
    unsigned int PhBD_current;
    int IA_d;
    int IA_q;
    int IB_d;
    int IB_q;
    int IC_d;
    int IC_q;
    int ID_d;
    int ID_q;
    int I_PhA[10];
    int I_PhB[10];
    int I_PhC[10];
    int I_PhD[10];
    unsigned int sys_status;
};
#endif /* COMMUNICATION_SCI_D_SCI_D_H_ */
