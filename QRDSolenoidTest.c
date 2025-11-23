///*
// * File:   QRDSolenoidTest.c
// * Author: jothm
// *
// * Created on November 18, 2025, 11:34 PM
// */
//
//
//#include "xc.h"
//#define FCY 250000UL
//#include "libpic30.h"
//
//#define QRD ADC1BUF9 // AN9, pin 18
//#define SOLENOID_REG PORTB
//#define SOLENOID_BIT 9
//#define NUM_ANALOG_CHANNELS 1
//#define QRD_WHITE_LIMIT 2481 // 2V
//
//typedef struct {
//    volatile uint16_t *readPin;
//} QRDSensor;
//
//typedef struct {
//    volatile uint16_t *reg;
//    unsigned int bit;
//} Solenoid;
//
//unsigned int getQRDValue(QRDSensor *sensor) {
//    return *(sensor->readPin);
//}
//
//unsigned int seesWhiteBall(QRDSensor *sensor) {
//    return (getQRDValue(sensor) < QRD_WHITE_LIMIT);
//}
//
//void setSolenoid(int onOff, Solenoid *solenoid) {
//    if(onOff) *(solenoid->reg) |= (1 << solenoid->bit);
//    else *(solenoid->reg) &= ~(1 << solenoid->bit);
//}
//
//QRDSensor qrd;
//Solenoid solenoid;
//
//void configadc();
//
//void setup() {
//    _ANSB15 = 1; // RB15, AN9, pin 18
//    _TRISB15 = 1;
//    _TRISB9 = 0; // pin 13
//    
//    configadc();
//    
//    solenoid.reg = &SOLENOID_REG;
//    solenoid.bit = SOLENOID_BIT;
//    qrd.readPin = &QRD;
//    setSolenoid(0, &solenoid);
//}
//
//int main(void) {
//    setup();
//    
//    while(1) {
//        if(seesWhiteBall(&qrd)) setSolenoid(0, &solenoid);
////        if(ADC1BUF9 <= QRD_WHITE_LIMIT) setSolenoid(0, &solenoid);
//        else setSolenoid(1, &solenoid);
////        if(seesWhiteBall(&qrd)) _LATA0 = 1;
////        else _LATA0 = 0;
////        setSolenoid(0, &solenoid);
////        __delay_ms(1000);
////        setSolenoid(1, &solenoid);
////        __delay_ms(1000);
//    }
//    
//    return 0;
//}
//
//void configadc(void) {
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
////    _CSS4 = 1; // AN4 - pin 6: IRRange
////    _CSS13 = 1; // AN13 - pin 7: IRRange
////    _CSS10 = 1; // AN10 - pin 17: Photodiode
//    _CSS9 = 1; // AN9 - pin 18: QRD
//    
//    _ADON = 1;      // AD1CON1<15> -- Turn on A/D
//}