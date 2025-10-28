/*
 * ON_OFF.h
 *
 *  Created on: 2019Äê8ÔÂ8ÈÕ
 *      Author: a
 */

#ifndef ON_OFF_H_
#define ON_OFF_H_

extern void ON_OFF_GPIO(void);

#define ON_CTRL1()    GpioDataRegs.GPACLEAR.bit.GPIO2 = 1
#define ON_CTRL2()    GpioDataRegs.GPACLEAR.bit.GPIO3 = 1
#define ON_CTRL3()    GpioDataRegs.GPACLEAR.bit.GPIO4 = 1
#define ON_CTRL4()    GpioDataRegs.GPACLEAR.bit.GPIO5 = 1
#define ON_CTRL5()    GpioDataRegs.GPACLEAR.bit.GPIO6 = 1
#define ON_CTRL6()    GpioDataRegs.GPACLEAR.bit.GPIO7 = 1
#define OFF_CTRL1()   GpioDataRegs.GPASET.bit.GPIO2 = 1
#define OFF_CTRL2()   GpioDataRegs.GPASET.bit.GPIO3 = 1
#define OFF_CTRL3()   GpioDataRegs.GPASET.bit.GPIO4 = 1
#define OFF_CTRL4()   GpioDataRegs.GPASET.bit.GPIO5 = 1
#define OFF_CTRL5()   GpioDataRegs.GPASET.bit.GPIO6 = 1
#define OFF_CTRL6()   GpioDataRegs.GPASET.bit.GPIO7 = 1

//extern void ON_CTRL1(void);
//extern void ON_CTRL2(void);
//extern void ON_CTRL3(void);
//extern void ON_CTRL4(void);
//extern void ON_CTRL5(void);
//extern void ON_CTRL6(void);
//extern void OFF_CTRL1(void);
//extern void OFF_CTRL2(void);
//extern void OFF_CTRL3(void);
//extern void OFF_CTRL4(void);
//extern void OFF_CTRL5(void);
//extern void OFF_CTRL6(void);

#endif /* ON_OFF_H_ */
