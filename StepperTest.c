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
//#define FCY 250000UL
//#include "libpic30.h"
//
//#pragma config FNOSC = LPFRC
//
//#define MOTOR_DIRECTION_REGISTER LATB
//#define LEFT_MOTOR_DIRECTION_BIT 12
//#define RIGHT_MOTOR_DIRECTION_BIT 13
//#define MOTOR_PWM OC1RS
//#define MOTOR_DUTY_CYCLE OC1R
//#define TICKS_PER_REV 200
//#define WHEEL_DIAMETER 3.5
//#define MICROSTEP 0.5
//
//void setup();
//
//int main(int argc, char** argv) {
//    // SETUP 
//    setup();
//    
//    StepperMotor leftMotor;
//    setupMotor(FORWARD, &MOTOR_DIRECTION_REGISTER, LEFT_MOTOR_DIRECTION_BIT, &MOTOR_PWM, &MOTOR_DUTY_CYCLE, TICKS_PER_REV, WHEEL_DIAMETER, MICROSTEP, &leftMotor);
//    
//    setSpeed(12, &leftMotor);
////    testSetRPS(1, &leftMotor);
//    __delay_ms(2000);
//    setSpeed(0, &leftMotor);
//    __delay_ms(2000);
//    setSpeed(-6, &leftMotor);
////    testSetRPS(-0.5, &leftMotor);
//    __delay_ms(2000);
//    setSpeed(0, &leftMotor);
////    testSetRPS(0, &leftMotor);
//    
//    while(1) setSpeed(0, &leftMotor);
////    while(1) testSetRPS(0, &leftMotor);
//    
//    return 0;
//}
//
//void setup() {
//    // setup pwm
//    OC1CON1 = 0x1C06;
//    OC1CON2 = 0x001F;
//    
//    // direction pins
//    _ANSB12 = 0; // digital pins
//    _ANSB13 = 0;
//    _TRISB12 = 0;
//    _TRISB13 = 0;
//}