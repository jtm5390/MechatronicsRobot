///*
// * File:   driveTest.c
// * Author: jothm
// *
// * Created on October 23, 2025, 12:30 PM
// */
//
//#include <stdlib.h>
//#include <math.h>
//#include "Photodiode.h"
//#include "xc.h"
//
//#define NUM_ANALOG_CHANNELS 1
//#define PHOTODIODE ADC1BUF0 // pin 2
//
//#define LIMIT1 309 // 0.25V
//#define LIMIT2 619 // 0.5V
//#define LIMIT3 1240 // 1V
//#define LIMIT4 1860 // 1.5V
//#define LIMIT5 2481 // 2V
//
//Photodiode pd;
//
////typedef struct {
////    volatile unsigned int *readBit;
////} Photodiode;
////Photodiode pd;
//
////unsigned int getIR(Photodiode *sensor) {
////    return *(sensor->readBit);
////}
//
//void setup();
//
//int main(void) {
//    // SETUP
//    setup();
//    
//    // MAIN
//    
//    while(1) {
////        if(getIR(&pd) >= LIMIT1) _LATA0 = 1; // LED 1
////        else _LATA0 = 0;
//        if(getIR(&pd) >= LIMIT1) _LATA1 = 1;
//        else _LATA1 = 0;
//        if(getIR(&pd) >= LIMIT2) _LATB0 = 1;
//        else _LATB0 = 0;
//        if(getIR(&pd) >= LIMIT3) _LATB1 = 1;
//        else _LATB1 = 0;
//        if(getIR(&pd) >= LIMIT4) _LATB2 = 1;
//        else _LATB2 = 0;
//        if(getIR(&pd) >= LIMIT5) _LATA2 = 1;
//        else _LATA2 = 0;
//    }
//    
//    return 0;
//}
//
//void config_ad(void) {
//    _ADON = 0;          // AD1CON1<15> -- Turn off A/D during config
//    
//    // Clear all A/D registers
//    AD1CON1 = 0; 
//    AD1CON2 = 0; 
//    AD1CON3 = 0; 
//    AD1CON5 = 0; 
//    AD1CSSL = 0; 
//    AD1CSSH = 0; 
//    
//    // AD1CON1 register
//    _ADSIDL = 0;    // AD1CON1<13> -- A/D continues while in idle mode
//    _MODE12 = 1;    // AD1CON1<10> -- 12-bit A/D operation
//    _FORM = 0;      // AD1CON1<9:8> -- Unsigned integer output
//    _SSRC = 7;      // AD1CON1<7:4> -- Auto conversion (internal counter)
//    _ASAM = 1;      // AD1CON1<2> -- Auto sampling
//
//    // AD1CON2 register
//    _PVCFG = 0;     // AD1CON2<15:14> -- Use VDD as positive ref voltage
//    _NVCFG = 0;     // AD1CON2<13> -- Use VSS as negative ref voltage
//    _BUFREGEN = 1;  // AD1CON2<11> -- Result appears in buffer
//                    // location corresponding to channel, e.g., AN12
//                    // results appear in ADC1BUF12
//    _CSCNA = 1;     // AD1CON2<10> -- Scans inputs specified in AD1CSSx
//                    // registers
//    _SMPI = NUM_ANALOG_CHANNELS-1;    // AD1CON2<6:2> -- Results sent to buffer after n conversion
//                    // For example, if you are sampling 4 channels, you
//                    // should have _SMPI = 3;
//    _ALTS = 0;      // AD1CON2<0> -- Sample MUXA only
//
//    // AD1CON3 register -- Change _SAMC and _ADCS based on your
//    // selection of oscillator and postscaling
//    _ADRC = 0;      // AD1CON3<15> -- Use system clock
//    _SAMC = 1;      // AD1CON3<12:8> -- Auto sample every A/D period TAD
//    _ADCS = 0;      // AD1CON3<7:0> -- A/D period TAD = TCY
//    
//    // AD1CHS register
//    _CH0NA = 0;     // AD1CHS<7:5> -- Measure voltages relative to VSS
//
//    // AD1CSSL register
//    // SET THE BITS CORRESPONDING TO CHANNELS THAT YOU WANT
//    // TO SAMPLE
//    _CSS0 = 1; // AN0 - pin 2: Photodiode
//    
//    _ADON = 1;      // AD1CON1<15> -- Turn on A/D
//}
//
//void setup() {
//    _RCDIV = 0; // div by 1 postscaler Fosc = 500 kHz; Fcy = 250 kHz
//    
//    //timers
//    
//    // Analog Select
//    _ANSA0 = 1;
//    _ANSA1 = 0;
//    _ANSB0 = 0;
//    _ANSB1 = 0;
//    _ANSB2 = 0;
//    _ANSA2 = 0;
//    
//    // I/O
//    _TRISA0 = 1;
//    _TRISA1 = 0;
//    _TRISB0 = 0;
//    _TRISB1 = 0;
//    _TRISB2 = 0;
//    _TRISA2 = 0;
//    
//    config_ad();
//    
//    pd.readBit = &PHOTODIODE;
//    
//    // PWM
//}
