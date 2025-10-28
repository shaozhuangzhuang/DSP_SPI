/*
 * Sci.h
 *
 *  Created on: 2025年10月28日
 *      Author: szz
 */

#ifndef HARDWARE_SCI_H_
#define HARDWARE_SCI_H_




#define RXBUF_MASK 0xFF
struct RX_BUF
{
    unsigned char rx_buf[RXBUF_MASK + 1];
    unsigned int rx_head;
    unsigned int rx_tail;     // Data of frame
};


typedef union i32_Byte
{
    int32 i32;
    struct
    {
        unsigned char Byte_0:8;
        unsigned char Byte_1:8;
        unsigned char Byte_2:8;
        unsigned char Byte_3:8;
    }BYTE;
}i32_Byte;

extern void InitSci(void);
extern void InitSciGpio(void);
extern void SCI_ReadFIFO(void);
extern void SCI_TO_Sim(float u_res);
extern void SCI_RX(void);

#endif /* HARDWARE_SCI_H_ */
