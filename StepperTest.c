///* 
// * File:   StepperTest.c
// * Author: jothm
// *
// * Created on October 20, 2025, 3:14 PM
// */
//
//#include <stdio.h>
//#include <stdlib.h>
//#include <xc.h>
//#include "StepperMotor.h"
//
//#pragma config FNOSC = LPFRC
//
///*
// * 
// */
//int main(int argc, char** argv) {
//    // SETUP 
//    // I/O
//    ANSA = 0;
//    ANSB = 0;
//    _TRISB8 = 1; // button on pin 12
//    
//    // pwm
//    OC1CON1 = 0x1C06;
//    OC1CON2 = 0x001F;
//    
//    StepperMotor motor1;
//    motor1.motorDirection = FORWARD;
//    motor1.dirPin = _LATB9; // pin 13
//    motor1.pwmPin = OC1RS; // pwm pin 14 frequency
//    motor1.dutyCyclePin = OC1R; // pwm pin 14 duty cycle
//    motor1.wheelDiameter = 3; // 3 inch diameter wheel
//    motor1.countsPerRev = 200;
//    motor1.rps = 0;
//    
//    
//    
//    while(1) {
//        if(_RB8) {
////            setRPS(1, &motor1);
//            setPWMTest(200, 0.5, &motor1, &_RCDIV, &_FOSCSEL);
//        }
//        else {
////            setRPS(0, &motor1);
//            OC1R = 0;
//        }
//    }
//    
//    return 0;
//}
//
