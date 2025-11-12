///*
// * File:   IRProxTest.c
// * Author: jothm
// *
// * Created on November 3, 2025, 3:34 PM
// */
//
//
//#include "xc.h"
//
//#define IR_REG PORTA
//#define LEFT_IR_BIT 3
//#define CENTER_IR_BIT 1
//#define RIGHT_IR_BIT 0
//
//typedef struct {
//    volatile unsigned int *readReg;
//    unsigned int bit;
//} IRProximitySensor;
//IRProximitySensor centerLineDetector, leftLineDetector, rightLineDetector;
//
//void setup();
//int detectsLine();
//
//int main(void) {
//    // SETUP
//    setup();
//    
//    while(1) {
//        if(detectsLine(&leftLineDetector)) _LATB0 = 1;
//        else _LATB0 = 0;
//        if(detectsLine(&centerLineDetector)) _LATB1 = 1;
//        else _LATB1 = 0;
//        if(detectsLine(&rightLineDetector)) _LATB2 = 1;
//        else _LATB2 = 0;
//    }
//    return 0;
//}
//
//void setup() {  
//    _RCDIV = 0;
//    U2MODE = 0;
//    U2STA = 0;
//
//
//    
//    _ANSA0 = 0;
//    _ANSA1 = 0;
//    _ANSB0 = 0;
//    _ANSB1 = 0;
//    _ANSB2 = 0;
//    _ANSA3 = 0;
//    
//    _TRISA3 = 1; // IR Prox Sensor
//    _TRISA0 = 1; // IR Prox Sensor
//    _TRISA1 = 1; // IR Prox Sensor
//    _TRISB0 = 0; // LED
//    _TRISB1 = 0; // LED
//    _TRISB2 = 0; // LED
//    
//    centerLineDetector.readReg = &IR_REG;
//    centerLineDetector.bit = CENTER_IR_BIT;
//    leftLineDetector.readReg = &IR_REG;
//    leftLineDetector.bit = LEFT_IR_BIT;
//    rightLineDetector.readReg = &IR_REG;
//    rightLineDetector.bit = RIGHT_IR_BIT;
//}
//
//int detectsLine(IRProximitySensor *sensor) {
//    return !(*sensor->readReg & (1 << sensor->bit));
//}