/*
 * F2837xD_ADC.c
 *
 *  Created on: 2022年8月3日
 *      Author: 110
 */


//###########################################################################
// FILE:   F2837xD_Adc.c
// TITLE:  F2837xD Adc Support Functions.
//###########################################################################
// $TI Release: F2837xD Support Library v190 $
// $Release Date: Mon Feb  1 16:51:57 CST 2016 $
// $Copyright: Copyright (C) 2013-2016 Texas Instruments Incorporated -
//             http://www.ti.com/ ALL RIGHTS RESERVED $
//###########################################################################

#include "F2837xD_device.h"     // F2837xD Headerfile Include File
#include "F2837xD_Examples.h"   // F2837xD Examples Include File
//#include "Common_Definitions.h"     // Common Definitions Headerfile Include File
#include "math.h"

//Write ADC configurations and power up the ADC for both ADC A and ADC B
void InitAdc(void)
{
    EALLOW;

    //write configurations
    AdcaRegs.ADCCTL2.bit.PRESCALE = 6; //set ADCCLK divider to /4
    AdcbRegs.ADCCTL2.bit.PRESCALE = 6; //set ADCCLK divider to /4
    AdccRegs.ADCCTL2.bit.PRESCALE = 6; //set ADCCLK divider to /4
    AdcdRegs.ADCCTL2.bit.PRESCALE = 6; //set ADCCLK divider to /4

    AdcSetMode(ADC_ADCA, ADC_RESOLUTION_12BIT, ADC_SIGNALMODE_SINGLE);
    AdcSetMode(ADC_ADCB, ADC_RESOLUTION_12BIT, ADC_SIGNALMODE_SINGLE);
    AdcSetMode(ADC_ADCC, ADC_RESOLUTION_12BIT, ADC_SIGNALMODE_SINGLE);
    AdcSetMode(ADC_ADCD, ADC_RESOLUTION_12BIT, ADC_SIGNALMODE_SINGLE);

    //Set pulse positions to late
    AdcaRegs.ADCCTL1.bit.INTPULSEPOS = 1;
    AdcbRegs.ADCCTL1.bit.INTPULSEPOS = 1;
    AdccRegs.ADCCTL1.bit.INTPULSEPOS = 1;
    AdcdRegs.ADCCTL1.bit.INTPULSEPOS = 1;

    //power up the ADC
    AdcaRegs.ADCCTL1.bit.ADCPWDNZ = 1;
    AdcbRegs.ADCCTL1.bit.ADCPWDNZ = 1;
    AdccRegs.ADCCTL1.bit.ADCPWDNZ = 1;
    AdcdRegs.ADCCTL1.bit.ADCPWDNZ = 1;

    //delay for 1ms to allow ADC time to power up
    DELAY_US(1000);

    EDIS;

    SetupADC();
}

/*
* Set the resolution and signalmode for a given ADC. This will ensure that
* the correct trim is loaded.
*/
void AdcSetMode(Uint16 adc, Uint16 resolution, Uint16 signalmode)
{
    Uint16 adcOffsetTrimOTPIndex; //index into OTP table of ADC offset trims
    Uint16 adcOffsetTrim; //temporary ADC offset trim

    //re-populate INL trim
    CalAdcINL(adc);

    if(0xFFFF != *((Uint16*)GetAdcOffsetTrimOTP)){
        //offset trim function is programmed into OTP, so call it

        //calculate the index into OTP table of offset trims and call
        //function to return the correct offset trim
        adcOffsetTrimOTPIndex = 4*adc + 2*resolution + 1*signalmode;
        adcOffsetTrim = (*GetAdcOffsetTrimOTP)(adcOffsetTrimOTPIndex);
    }
    else {
        //offset trim function is not populated, so set offset trim to 0
        adcOffsetTrim = 0;
    }

    //Apply the resolution and signalmode to the specified ADC.
    //Also apply the offset trim and, if needed, linearity trim correction.
    switch(adc){
        case ADC_ADCA:
            AdcaRegs.ADCCTL2.bit.RESOLUTION = resolution;
            AdcaRegs.ADCCTL2.bit.SIGNALMODE = signalmode;
            AdcaRegs.ADCOFFTRIM.all = adcOffsetTrim;
            if(ADC_RESOLUTION_12BIT == resolution){

                //12-bit linearity trim workaround
                AdcaRegs.ADCINLTRIM1 &= 0xFFFF0000;
                AdcaRegs.ADCINLTRIM2 &= 0xFFFF0000;
                AdcaRegs.ADCINLTRIM4 &= 0xFFFF0000;
                AdcaRegs.ADCINLTRIM5 &= 0xFFFF0000;
            }
        break;
        case ADC_ADCB:
            AdcbRegs.ADCCTL2.bit.RESOLUTION = resolution;
            AdcbRegs.ADCCTL2.bit.SIGNALMODE = signalmode;
            AdcbRegs.ADCOFFTRIM.all = adcOffsetTrim;
            if(ADC_RESOLUTION_12BIT == resolution){

                //12-bit linearity trim workaround
                AdcbRegs.ADCINLTRIM1 &= 0xFFFF0000;
                AdcbRegs.ADCINLTRIM2 &= 0xFFFF0000;
                AdcbRegs.ADCINLTRIM4 &= 0xFFFF0000;
                AdcbRegs.ADCINLTRIM5 &= 0xFFFF0000;
            }
        break;
        case ADC_ADCC:
            AdccRegs.ADCCTL2.bit.RESOLUTION = resolution;
            AdccRegs.ADCCTL2.bit.SIGNALMODE = signalmode;
            AdccRegs.ADCOFFTRIM.all = adcOffsetTrim;
            if(ADC_RESOLUTION_12BIT == resolution){

                //12-bit linearity trim workaround
                AdccRegs.ADCINLTRIM1 &= 0xFFFF0000;
                AdccRegs.ADCINLTRIM2 &= 0xFFFF0000;
                AdccRegs.ADCINLTRIM4 &= 0xFFFF0000;
                AdccRegs.ADCINLTRIM5 &= 0xFFFF0000;
            }
        break;
        case ADC_ADCD:
            AdcdRegs.ADCCTL2.bit.RESOLUTION = resolution;
            AdcdRegs.ADCCTL2.bit.SIGNALMODE = signalmode;
            AdcdRegs.ADCOFFTRIM.all = adcOffsetTrim;
            if(ADC_RESOLUTION_12BIT == resolution){

                //12-bit linearity trim workaround
                AdcdRegs.ADCINLTRIM1 &= 0xFFFF0000;
                AdcdRegs.ADCINLTRIM2 &= 0xFFFF0000;
                AdcdRegs.ADCINLTRIM4 &= 0xFFFF0000;
                AdcdRegs.ADCINLTRIM5 &= 0xFFFF0000;
            }
        break;
    }
}

//setup the ADC to continuously convert on one channel
void SetupADC(void)
{
    Uint16 acqps;

    //determine minimum acquisition window (in SYSCLKS) based on resolution
    if(ADC_RESOLUTION_12BIT == AdcaRegs.ADCCTL2.bit.RESOLUTION){
        acqps = 32; //5ns * (n+1)
    }
    else { //resolution is 16-bit
        acqps = 32; //320ns
    }

    EALLOW;
    //顺序采样
//    AdcaRegs.ADCSAMPLEMODE.all = 0x0000;
/******************软件启动*******************/
    AdcaRegs.ADCINTSOCSEL1.all = 0x0000;
    AdcaRegs.ADCINTSOCSEL2.all = 0x0000;
    AdcbRegs.ADCINTSOCSEL1.all = 0x0000;
    AdcbRegs.ADCINTSOCSEL2.all = 0x0000;
    AdccRegs.ADCINTSOCSEL1.all = 0x0000;
    AdccRegs.ADCINTSOCSEL2.all = 0x0000;
    AdcdRegs.ADCINTSOCSEL1.all = 0x0000;
    AdcdRegs.ADCINTSOCSEL2.all = 0x0000;
/******************通道选择*******************/
    /****************A****************/
//    AdcaRegs.ADCSOC0CTL .bit.CHSEL = 0;
//    AdcaRegs.ADCSOC1CTL .bit.CHSEL = 1;
//    AdcaRegs.ADCSOC2CTL .bit.CHSEL = 2;
//    AdcaRegs.ADCSOC3CTL .bit.CHSEL = 3;
//    AdcaRegs.ADCSOC4CTL .bit.CHSEL = 4;
//    AdcaRegs.ADCSOC5CTL .bit.CHSEL = 5;
//    AdcaRegs.ADCSOC14CTL.bit.CHSEL = 14;
//    AdcaRegs.ADCSOC15CTL.bit.CHSEL = 15;
    /****************B****************/
    AdcbRegs.ADCSOC0CTL .bit.CHSEL = 0;
    AdcbRegs.ADCSOC1CTL .bit.CHSEL = 1;
    AdcbRegs.ADCSOC2CTL .bit.CHSEL = 2;
    AdcbRegs.ADCSOC3CTL .bit.CHSEL = 3;
    AdcbRegs.ADCSOC4CTL .bit.CHSEL = 4;
    AdcbRegs.ADCSOC5CTL .bit.CHSEL = 5;
    /****************C****************/
    AdccRegs.ADCSOC2CTL .bit.CHSEL = 2;
    AdccRegs.ADCSOC3CTL .bit.CHSEL = 3;
    AdccRegs.ADCSOC4CTL .bit.CHSEL = 4;
    AdccRegs.ADCSOC5CTL .bit.CHSEL = 5;
    /****************D****************/
//    AdcdRegs.ADCSOC0CTL .bit.CHSEL = 0;
//    AdcdRegs.ADCSOC1CTL .bit.CHSEL = 1;
//    AdcdRegs.ADCSOC2CTL .bit.CHSEL = 2;
//    AdcdRegs.ADCSOC3CTL .bit.CHSEL = 3;
    AdcdRegs.ADCSOC4CTL .bit.CHSEL = 4;
    AdcdRegs.ADCSOC5CTL .bit.CHSEL = 5;
/******************采样窗********************/
    /****************A****************/
//    AdcaRegs.ADCSOC0CTL .bit.ACQPS = acqps;
//    AdcaRegs.ADCSOC1CTL .bit.ACQPS = acqps;
//    AdcaRegs.ADCSOC2CTL .bit.ACQPS = acqps;
//    AdcaRegs.ADCSOC3CTL .bit.ACQPS = acqps;
//    AdcaRegs.ADCSOC4CTL .bit.ACQPS = acqps;
//    AdcaRegs.ADCSOC5CTL .bit.ACQPS = acqps;
//    AdcaRegs.ADCSOC14CTL.bit.ACQPS = acqps;
//    AdcaRegs.ADCSOC15CTL.bit.ACQPS = acqps;
    /****************B****************/
    AdcbRegs.ADCSOC0CTL .bit.ACQPS = acqps;
    AdcbRegs.ADCSOC1CTL .bit.ACQPS = acqps;
    AdcbRegs.ADCSOC2CTL .bit.ACQPS = acqps;
    AdcbRegs.ADCSOC3CTL .bit.ACQPS = acqps;
    AdcbRegs.ADCSOC4CTL .bit.ACQPS = acqps;
    AdcbRegs.ADCSOC5CTL .bit.ACQPS = acqps;
    /****************C****************/
    AdccRegs.ADCSOC2CTL .bit.ACQPS = acqps;
    AdccRegs.ADCSOC3CTL .bit.ACQPS = acqps;
    AdccRegs.ADCSOC4CTL .bit.ACQPS = acqps;
    AdccRegs.ADCSOC5CTL .bit.ACQPS = acqps;
    /****************D****************/
//    AdcdRegs.ADCSOC0CTL .bit.ACQPS = acqps;
//    AdcdRegs.ADCSOC1CTL .bit.ACQPS = acqps;
//    AdcdRegs.ADCSOC2CTL .bit.ACQPS = acqps;
//    AdcdRegs.ADCSOC3CTL .bit.ACQPS = acqps;
    AdcdRegs.ADCSOC4CTL .bit.ACQPS = acqps;
    AdcdRegs.ADCSOC5CTL .bit.ACQPS = acqps;

//    AdcaRegs.ADCSOC0CTL.bit.TRIGSEL= 0;
//    AdcaRegs.ADCSOC1CTL.bit.TRIGSEL= 0;
//    AdcaRegs.ADCSOC2CTL.bit.TRIGSEL= 0;
//    AdcaRegs.ADCSOC3CTL.bit.TRIGSEL= 0;
//    AdcaRegs.ADCSOC4CTL.bit.TRIGSEL= 0;
//    AdcaRegs.ADCSOC5CTL.bit.TRIGSEL= 0;
//    AdcaRegs.ADCSOC14CTL.bit.TRIGSEL= 0;
//    AdcaRegs.ADCSOC15CTL.bit.TRIGSEL= 0;
//    AdcbRegs.ADCSOC0CTL.bit.TRIGSEL= 0;
//*********************************************
//    AdcaRegs.ADCSOC0CTL .bit.TRIGSEL = 0x1a;              //begin conversion on pwm11 socb
//    AdcaRegs.ADCSOC1CTL .bit.TRIGSEL = 0x1a;              //begin conversion on pwm11 socb
//    AdcaRegs.ADCSOC2CTL .bit.TRIGSEL = 0x08;              //begin conversion on pwm2 socb
//    AdcaRegs.ADCSOC3CTL .bit.TRIGSEL = 0x08;              //begin conversion on pwm2 socb
//    AdcaRegs.ADCSOC4CTL .bit.TRIGSEL = 0x0c;              //begin conversion on pwm4 socb
//    AdcaRegs.ADCSOC5CTL .bit.TRIGSEL = 0x0c;              //begin conversion on pwm4 socb
//    AdcaRegs.ADCSOC14CTL.bit.TRIGSEL = 0x0a;              //begin conversion on pwm3 socb
//    AdcaRegs.ADCSOC15CTL.bit.TRIGSEL = 0x06;              //begin conversion on pwm1 socb
//    AdcbRegs.ADCSOC0CTL .bit.TRIGSEL = 0x18;              //begin conversion on pwm10 socb
//*********************************************
    AdcbRegs.ADCSOC0CTL.bit.TRIGSEL = 0;
    AdcbRegs.ADCSOC1CTL.bit.TRIGSEL = 0;
    AdcbRegs.ADCSOC2CTL.bit.TRIGSEL = 0;
    AdcbRegs.ADCSOC3CTL.bit.TRIGSEL = 0;
    AdcbRegs.ADCSOC4CTL.bit.TRIGSEL = 0;
    AdcbRegs.ADCSOC5CTL.bit.TRIGSEL = 0;

    AdccRegs.ADCSOC2CTL.bit.TRIGSEL = 0;
    AdccRegs.ADCSOC3CTL.bit.TRIGSEL = 0;
    AdccRegs.ADCSOC4CTL.bit.TRIGSEL = 0;
    AdccRegs.ADCSOC5CTL.bit.TRIGSEL = 0;
//    AdccRegs.ADCSOC4CTL.bit.TRIGSEL= 0;
//    AdccRegs.ADCSOC4CTL.bit.TRIGSEL= 0x18;              //begin conversion on pwm10 socb

//    AdcdRegs.ADCSOC0CTL.bit.TRIGSEL = 0;
//    AdcdRegs.ADCSOC1CTL.bit.TRIGSEL = 0;
//    AdcdRegs.ADCSOC2CTL.bit.TRIGSEL = 0;
//    AdcdRegs.ADCSOC3CTL.bit.TRIGSEL = 0;
    AdcdRegs.ADCSOC4CTL.bit.TRIGSEL = 0;
    AdcdRegs.ADCSOC5CTL.bit.TRIGSEL = 0;

    EDIS;
}

/*
* Loads INL trim values from OTP into the trim registers of the specified ADC.
* Use only as part of AdcSetMode function, since linearity trim correction
* is needed for some modes.
*/
void CalAdcINL(Uint16 adc)
{
    switch(adc){
        case ADC_ADCA:
            if(0xFFFF != *((Uint16*)CalAdcaINL)){
                //trim function is programmed into OTP, so call it
                (*CalAdcaINL)();
            }
            else {
                //do nothing, no INL trim function populated
            }
            break;
        case ADC_ADCB:
            if(0xFFFF != *((Uint16*)CalAdcbINL)){
                //trim function is programmed into OTP, so call it
                (*CalAdcbINL)();
            }
            else {
                //do nothing, no INL trim function populated
            }
            break;
        case ADC_ADCC:
            if(0xFFFF != *((Uint16*)CalAdccINL)){
                //trim function is programmed into OTP, so call it
                (*CalAdccINL)();
            }
            else {
                //do nothing, no INL trim function populated
            }
            break;
        case ADC_ADCD:
            if(0xFFFF != *((Uint16*)CalAdcdINL)){
                //trim function is programmed into OTP, so call it
                (*CalAdcdINL)();
            }
            else {
                //do nothing, no INL trim function populated
            }
            break;
    }
}

float Temp = 0.0,T1_tmp = 0.0;
void seq_adc( void )
{
    static float VA1_tmp = 0.0;
    static float VA2_tmp = 0.0;
//    static float VA3_tmp = 0.0;
//    static float VA4_tmp = 0.0;
//    static float VA5_tmp = 0.0;
//    static float VA6_tmp = 0.0;
//    static float VA7_tmp = 0.0;
//    static float VA8_tmp = 0.0;


//    float MU_tmp = 0.0,MU_tmp1 = 0.0,MU_tmp2 = 0.0,MU_tmp3 = 0.0;
//    float U_Slope = - 3.887559808612440191387559808612e-2;
//    float MI_tmp = 0.0;
        AdcaRegs.ADCSOCFRC1.all = 0x0000;  // Epwm trigger  Start SOC0-15 to begin ping-pong sampling
        AdcbRegs.ADCSOCFRC1.all = 0x003f;  // Epwm trigger  Start SOC0-15 to begin ping-pong sampling
        AdccRegs.ADCSOCFRC1.all = 0x003C;  // Epwm trigger  Start SOC0-15 to begin ping-pong sampling
        AdcdRegs.ADCSOCFRC1.all = 0x0030;  // Epwm trigger  Start SOC0-15 to begin ping-pong sampling

        /**************温度**************/
        VA1_tmp =  AdcdResultRegs.ADCRESULT4;
        VA2_tmp =  AdcdResultRegs.ADCRESULT5;

        Temp = 7500 / ((VA1_tmp + VA2_tmp)/2) * 4096 / 3.3 - 3500;

        T1_tmp = 3010.676 - 765.5221 * sqrt(17.53 - 0.00261 * Temp);
        /**************主电流**************/
//        VA5_tmp =  AdcdResultRegs.ADCRESULT0;
//        VA6_tmp =  AdcdResultRegs.ADCRESULT1;

        /**************主电压**************/
//        VA7_tmp    = AdcdResultRegs.ADCRESULT2;
//        VA8_tmp    = AdcdResultRegs.ADCRESULT3;
//        MU_tmp     = VA7_tmp + VA8_tmp;
//        MU_tmp1    = MU_tmp  / 2;
//        MU_tmp2    = MU_tmp1 - 27720.0;
//        MU_tmp3    = MU_tmp2 * U_Slope;
//        U_main_tmp = FirstOrderFilter_UM( MU_tmp3 );
}


