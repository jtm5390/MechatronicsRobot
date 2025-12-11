///*
// * File:   LaserServoTest.c
// * Author: jothm
// *
// * Created on December 1, 2025, 4:57 PM
// */
//
//#include "xc.h"
//#include "Servo.h"
//#include "Laser.h"
//
//#define FCY 250000UL
//#include "libpic30.h"
//
//#define SERVO_PWM OC3RS
//#define SERVO_DUTY_CYCLE OC3R
//#define SERVO_MAX_ANGLE 180
//#define SERVO_MIN_DUTY_CYCLE 200
//#define SERVO_MAX_DUTY_CYCLE 650
//#define LASER_REGISTER PORTB
//#define LASER_BIT 8
//
//Servo myServo;
//Laser myLaser;
//
//void setup() {
////    _ANSB1 = 0; // servo
////    _TRISB1 = 0; // servo
//    OC1CON1 = 0;
//    OC1CON2 = 0;
//    OC2CON1 = 0;
//    OC2CON2 = 0;
//    OC3CON1 = 0;
//    OC3CON2 = 0;
//    ANSA = 0;
//    ANSB = 0;
//    TRISA = 0;
//    TRISB = 0;
//    LATA = 0;
//    LATB = 0;
//    _TRISB8 = 0;
//    _RCDIV = 0;
//    
//    OC1CON1 = 0x1C06;
//    OC1CON2 = 0x001F;
//    OC2CON1 = 0x1C06;
//    OC2CON2 = 0x001F;
//    OC3CON1 = 0x1C06;
//    OC3CON2 = 0x001F;
//    
//    setupServo(&SERVO_PWM, &SERVO_DUTY_CYCLE, SERVO_MAX_ANGLE, SERVO_MIN_DUTY_CYCLE, SERVO_MAX_DUTY_CYCLE, &myServo);
//    setupLaser(&LASER_REGISTER, LASER_BIT, &myLaser);
//    
//    OC1RS = 1249;
//    OC1R = 0;
//    OC2RS = 1249;
//    OC2R = 0;
//    OC3RS = 4999;
//    OC3R = 0;
//}
//
//int main(void) {
//    setup();
////    setLaser(1, &myLaser);
////    while(1);
//    for(int i = 135; i > 25; i -= 2) {
////            OC3R = i;
//        setAngle(i, &myServo);
//        __delay_ms(100);
//    }
//    
//    while(1) {
////        setLaser(1, &myLaser);
////        __delay_ms(500);
////        setLaser(0, &myLaser);
////        __delay_ms(500);
////        for(int i = 0; i < 100; i += 5) {
//////            OC3R = i;
////            setAngle(i, &myServo);
////            __delay_ms(100);
////        }
////        for(int i = 135; i > 25; i -= 2) {
//////            OC3R = i;
////            setAngle(i, &myServo);
////            __delay_ms(100);
////        }
//    }
//    
//    return 0;
//}