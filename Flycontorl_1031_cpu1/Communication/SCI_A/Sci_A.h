/*
 * Sci_A.h
 *
 *  Created on: 2022Äê8ÔÂ1ÈÕ
 *      Author: 110
 */

#ifndef COMMUNICATION_SCI_A_SCI_A_H_
#define COMMUNICATION_SCI_A_SCI_A_H_

void InitScia(void);
void scia_xmit(int a);
void scia_msg(char * msg);
void SCIA_ReadFIFO(void);
void SCIA_RX(void);
void SCIA_TO_Servo(void);
void SCIA_RX_FZJ(void);
void SCIA_TO_FZJ(void);
Uint16 crc16_IBM(unsigned char *data, Uint32 length);

extern struct State_SCI_Tx SCIA_Tx_State;

#endif /* COMMUNICATION_SCI_A_SCI_A_H_ */
