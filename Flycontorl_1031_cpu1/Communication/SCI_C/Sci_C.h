/*
 * Sci_C.h
 *
 *  Created on: 2022年9月5日
 *      Author: 110
 */

#ifndef COMMUNICATION_SCI_C_SCI_C_H_
#define COMMUNICATION_SCI_C_SCI_C_H_


void SCIC_Reg_Init(void);
void SCIC_ReadFIFO(void);
void DataLink_Rx(void);
void SCIC_TO_SIMULINK();
void scic_msg(char * msg);
void scic_xmit(int a);
void SCIC_TO_MOTRO(void);
void SCIC_TO_DATALINK(void);

extern struct State_SCI_Tx SCIC_Tx_State;

extern struct Rx_SimToFlight             Rx_SimToFlight_Rx;      //飞控接收到仿真的数据帧
#endif /* COMMUNICATION_SCI_C_SCI_C_H_ */
