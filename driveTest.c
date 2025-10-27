/*
 * File:   driveTest.c
 * Author: jothm
 *
 * Created on October 23, 2025, 12:30 PM
 */

#include <math.h>
#include "xc.h"

#pragma config FNOSC = LPFRC // 500 kHz osc

#define WHEEL_DIAMETER 3.5
#define TICKS_PER_REV 200
#define TURN_RADIUS 4.25
#define FCY 250000UL

#include "libpic30.h"

void setSpeed(float speedInPerSec);
//void turn(float angle, float turnSpeedInPerSec);

int main(void) {
    // SETUP
    _RCDIV = 0; // div by 1 postscaler Fosc = 500 kHz; Fcy = 250 kHz
    
    // I/O
    
    // PWM
    OC1CON1 = 0x1C06;
    OC1CON2 = 0x001F;
    OC2CON1 = 0x1C06;
    OC2CON2 = 0x001F;
    OC1RS = 1249; // Fpwm = 200 --> 1 RPS
    OC1R = 0; // keep motor off for now
    OC2RS = 1249;
    OC2R = 0;
    
    // drive forward
    setSpeed(4);
    // delay
    __delay_ms(5000);
    setSpeed(0);
    
    // turn 90 deg
//    turn(90);
//    
//    // drive forward
//    setSpeed(4);
//    // delay
//    setSpeed(0);
//    
//    // turn 180 deg
//    turn(180);
//    
//    // drive forward
//    setSpeed(4);
//    // delay
//    setSpeed(0);
    
    return 0;
}

//void setRPS(float rps, unsigned int *motorPWM, unsigned int *motorDutyCycle) {
//    float fpwm = rps * TICKS_PER_REV;
//    *motorPWM = (int)((250000/fpwm)-1);
//    *motorDutyCycle = (int)(OC1RS/2.0);
//}

void setRPS(float rps) {
    float fpwm = rps * TICKS_PER_REV;
    OC1RS = (int)((250000/fpwm)-1);
    OC1R = (int)(OC1RS/2.0);
}

void setSpeed(float speedInPerSec) {
    float rps = speedInPerSec/(M_PI*WHEEL_DIAMETER);
//    setRPS(rps, &OC1RS, &OC1R);
    setRPS(rps);
}

//void turn(float angle, float turnSpeedInPerSec) {
//    float distToTravel = (angle/360)*2*M_PI*TURN_RADIUS; // use this to determine how many ticks to go
//    int ticksToTravel = (int)(TICKS_PER_REV * distToTravel / (M_PI*WHEEL_DIAMETER));
//    float motorInPerSec = turnSpeedInPerSec * TURN_RADIUS;
//    float motorRPS = motorInPerSec / (M_PI*WHEEL_DIAMETER);
//    OC1RS, OC2RS = (int)((250000/motorRPS)-1);
//    OC1R, OC2R = (int)(OC1RS/2.0);
//    // set motor directions
//    if(angle > 0) {
//        _LATB12 = 1;
//        _LATB13 = 0;
//    } else {
//        _LATB12 = 0;
//        _LATB13 = 1;
//    }
//}