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
//#define LASER_REGISTER PORTB
//#define LASER_BIT 8
//
//Servo myServo;
//Laser myLaser;
//
//void setup() {
//    _ANSB1 = 0; // servo
//    _TRISB1 = 0; // servo
//    
//    OC3CON1 = 0x1C06;
//    OC3CON2 = 0x001F;
//    
//    setupServo(&SERVO_PWM, &SERVO_DUTY_CYCLE, 180, 100, 375, &myServo);
//    setupLaser(&LASER_REGISTER, LASER_BIT, &myLaser);
//}
//
//int main(void) {
//    setup();
//
//    
//    OC3RS = 4999;
//    while(1) {
//        setLaser(1, &myLaser);
//        __delay_ms(500);
//        setLaser(0, &myLaser);
//        __delay_ms(500);
//        for(int i = 0; i < 180; i++) {
//            setAngle(i, &myServo);
//            __delay_ms(100);
//        }
//        for(int i = 180; i > 0; i--) {
//            setAngle(i, &myServo);
//            __delay_ms(100);
//        }
//    }
//    
//    return 0;
//}