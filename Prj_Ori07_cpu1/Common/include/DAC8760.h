/* DAC8760.h */

#ifndef DAC8760_H_
#define DAC8760_H_

#define DAC8760_NOP                         0x00
#define DAC8760_DATA                        0x01
#define DAC8760_READ                        0x02
#define DAC8760_CONTROL                     0x55
#define DAC8760_RST                         0x56
#define DAC8760_CONFIG                      0x57
#define DAC8760_Gain_Calibration            0x58
#define DAC8760_Zero_Calibration            0x59
#define DAC8760_READ_status                 0x0000
#define DAC8760_READ_DAC_data               0x0001
#define DAC8760_READ_control                0x0002
#define DAC8760_READ_configuration          0x000B
#define DAC8760_READ_DAC_gain_calibration   0x0013
#define DAC8760_READ_DAC_zero_calibration   0x0017

extern void DAC8760();
extern void	InitDAC8760(void);
extern void InitDAC8760_A(void);
extern void InitDAC8760_B(void);
extern void WriteToDAC8760_A(unsigned int address, unsigned int data);
extern void WriteToDAC8760_B(unsigned int address, unsigned int data);
extern void mcbspa_init_SPI(void);
extern void mcbspb_init_SPI(void);
extern void mcbspa_xmit(unsigned int a, unsigned int b);
extern void mcbspb_xmit(unsigned int a, unsigned int b);

#endif /* DAC8760_H_ */
