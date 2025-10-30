/* AD1258.h */

#ifndef AD1258_H_
#define AD1258_H_

extern void InitADS1258(void);
extern void ADS1258_GPIO(void);
extern void ADS1258(void);

#define  LOW_CS              GpioDataRegs.GPBCLEAR.bit.GPIO57 = 1;      // GPIO57 is LOW cspin
#define  HING_CS             GpioDataRegs.GPBSET.bit.GPIO57 = 1;        // GPIO57 is high CS pin  拉高
#define  STP_TRANSMIT        GpioDataRegs.GPACLEAR.bit.GPIO18 = 1;      // Lower GPIO18   startpin  停止转换
#define  START_TRANSMIT      GpioDataRegs.GPASET.bit.GPIO18 = 1;        //GPIO18 is high startpin  开始转换   通道0-7有数据输出
#define  RESET               GpioDataRegs.GPACLEAR.bit.GPIO9 = 1;       // Lower GPIO9  RESTpin  重设，至少两个周期，寄存器恢复初始值，滤波器清除
#define  NO_RESET            GpioDataRegs.GPASET.bit.GPIO9 = 1;         // HIGH GPIO9  RESTpin 不重设

#endif /* AD1258_H_ */
