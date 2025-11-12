///*
// * File:   IRRangeTest.c
// * Author: jothm
// *
// * Created on November 10, 2025, 10:59 AM
// */
//
//
//#include "xc.h"
//#include "IRRangeSensor.h"
//
//#define NUM_ANALOG_CHANNELS 3
//#define IRRANGE1 ADC1BUF4 // pin 6
//#define IRRANGE2 ADC1BUF13 // pin 7
//#define PHOTODIODE ADC1BUF10 // pin 17
//#define IR_LIMIT 1240 // about 25 cm or 1V
//#define PHOTODIODE_LIMIT 1240 // 1V
//
//IRRangeSensor sensor1;
//
//void config_ad(void);
//
//void setup() {
//    // Analog Select
//    _ANSB2 = 1; // pin 6
//    _ANSA2 = 1; // pin 7
//    _ANSB14 = 1; // pin 17
//    _ANSA0 = 0;
//    _ANSA1 = 0;
//    _ANSB1 = 0;
//    
//    // Trigger Select
//    _TRISB2 = 1;
//    _TRISA2 = 1;
//    _TRISB14 = 1;
//    _TRISA0 = 0;
//    _TRISA1 = 0;
//    _TRISB1 = 0;
//    
//    // Analog Setup
//    config_ad();
//    
//    sensor1.readBit = &IRRANGE1;
//}
//
//int main(void) {
//    // SETUP
//    setup();
//    
//    while(1) {
//        if(getRangeValue(&sensor1) < IR_LIMIT) {
//            _LATA0 = 1;
//        } else _LATA0 = 0;
//        if(IRRANGE2 < IR_LIMIT) {
//            _LATA1 = 1;
//        } else _LATA1 = 0;
//        if(PHOTODIODE < PHOTODIODE_LIMIT) {
//            _LATB1 = 1;
//        } else _LATB1 = 0;
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
//    _CSS4 = 1; // AN4 - pin 6: IRRange
//    _CSS13 = 1; // AN13 - pin 7: IRRange
//    _CSS10 = 1; // AN10 - pin 17: Photodiode
//    
//    _ADON = 1;      // AD1CON1<15> -- Turn on A/D
//}