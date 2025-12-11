///*
// * File:   test.c
// * Author: jothm
// *
// * Created on December 8, 2025, 6:05 PM
// */
//
//
//#include "xc.h"
//
//#define FCY 250000UL
//#include "libpic30.h"
//
//void config(void) {
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
//    _SMPI = 4;    // AD1CON2<6:2> -- Results sent to buffer after n conversion
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
//    _CSS4 = 1; // AN4 - pin 6: IRRange
//    _CSS13 = 1; // AN13 - pin 7: IRRange
//    _CSS10 = 1; // AN10 - pin 17: Photodiode
//    _CSS9 = 1; // AN9 - pin 18: QRD
//    
//    _ADON = 1;      // AD1CON1<15> -- Turn on A/D
//}
//
//int main(void) {
//    OC1CON1 = 0x1C06;
//    OC1CON2 = 0x001F;
//    OC2CON1 = 0x1C06;
//    OC2CON2 = 0x001F;
//    OC3CON1 = 0x1C06;
//    OC3CON2 = 0x001F;
//    
//    // analog selections
//    _ANSA0 = 1; // pin 2
//    _ANSA1 = 0; // pin 3
//    _ANSB2 = 1; // pin 6
//    _ANSA2 = 1; // pin 7
//    _ANSA3 = 0; // pin 8
//    _ANSB12 = 0; // pin 15
//    _ANSB13 = 0; // pin 16
//    _ANSB14 = 1; // pin 17
//    _ANSB15 = 1; // pin 18
//    
//    // I/O
//    _TRISA0 = 1; // pin 2 - Photodiode
//    _TRISA1 = 1; // pin 3 - IR Prox
//    _TRISB2 = 1; // pin 6 - IR Range
//    _TRISA2 = 1; // pin 7 - IR Range
//    _TRISA3 = 1; // pin 8 - IR Prox
//    _TRISA4 = 1; // pin 10 - IR Prox
//    _TRISB7 = 1; // pin 11 - IR Prox
//    _TRISB8 = 0; // pin 12 - Laser
//    _TRISB9 = 0; // pin 13 - Solenoid
//    _TRISB12 = 0; // pin 15 - Motor Dir
//    _TRISB13 = 0; // pin 16 - Motor Dir
//    _TRISB14 = 1; // pin 17 - Photodiode
//    _TRISB15 = 1; // pin 18 - QRD
//    
//    config();
//    
//    while(1) {
//        _LATB8 = 1;
//        __delay_ms(1000);
//        _LATB8 = 0;
//        __delay_ms(1000);
//    }
//    
//    return 0;
//}
