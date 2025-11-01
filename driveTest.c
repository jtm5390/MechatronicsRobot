/*
 * File:   driveTest.c
 * Author: jothm
 *
 * Created on October 23, 2025, 12:30 PM
 */

#include <stdlib.h>
#include <math.h>
#include "xc.h"

#pragma config FNOSC = LPFRC // 500 kHz osc

#define WHEEL_DIAMETER 3.5 // drive motor diameter
#define TICKS_PER_REV 200 // ticks per rev for the stepper motors
#define TURN_RADIUS 4.5 // radius of robot's turn (from center to wheel)
#define FCY 250000UL // Fcy
#define MICROSTEP 0.5 // match the stepper driver switches
#define MOTOR1_DIR _LATB12 // left motor direction
#define MOTOR2_DIR _LATB13 // right motor direction
#define MOTOR_PWM OC1RS // motor pwm register
#define MOTOR_DUTY_CYCLE OC1R // motor duty cycle register
#define MOTOR_TIMER TMR1 // timer used for the motor

#include "libpic30.h"

void setSpeed(float speedInPerSec);
void turn(float angle, float turnSpeedInPerSec);
void setup();

int main(void) {
    // SETUP
    setup();
    
    // MAIN
    // ------------------------------ Autonomous Code ---------------------------
    setSpeed(12); // about 1 rps forward
    __delay_ms(2000); // wait 2 seconds
    setSpeed(0); // brake
    
    turn(90, 12); // turn 90 degrees
    
    setSpeed(12);
    __delay_ms(2000);
    setSpeed(0);
    
    turn(180, 6); // turn 180 degrees, half speed
    
    setSpeed(12);
    __delay_ms(2000);
    setSpeed(0);
    
    while(1) setSpeed(0); // end of program, brake
    return 0;
}

void setRPS(float rps, volatile unsigned int *motorPWM, volatile unsigned int *motorDutyCycle) {
    MOTOR1_DIR = (rps > 0)? 0:1, MOTOR2_DIR = (rps > 0)? 0:1; // determine forward or reverse
    if((int)(rps) == 0) {
        *motorPWM = 1249; // some arbitrary pwm signal
        *motorDutyCycle = 0; // turn off motors (brake)
    } else {
        float fpwm = rps * TICKS_PER_REV / MICROSTEP; // calculate Fpwm
        *motorPWM = (int)((FCY/fpwm)-1); // calculate OCxRS
        *motorDutyCycle = (int)((*motorPWM)/2.0); // calculate OCxR
    }
}

void setSpeed(float speedInPerSec) {
    float rps = speedInPerSec/(M_PI*WHEEL_DIAMETER); // calculate motor RPS
    setRPS(rps, &MOTOR_PWM, &MOTOR_DUTY_CYCLE);
}

void turn(float angle, float turnSpeedInPerSec) {
    float distToTravel = (angle/360.0)*2.0*M_PI*TURN_RADIUS; // distance for the motor to travel
    int ticksToTravel = (int)(TICKS_PER_REV * distToTravel / (M_PI*WHEEL_DIAMETER)); // ticks for the motor to travel
    float motorFPWM = turnSpeedInPerSec*TICKS_PER_REV / (M_PI*WHEEL_DIAMETER*MICROSTEP); // Fpwm
    float timeToTravel = ticksToTravel/motorFPWM; // time it takes for motor to travel the number of ticks
    int timerCount = 2*timeToTravel * (int)(FCY/64.0); // timer counts it takes to take the time needed to travel the needed distance on the motor
    // NOTE: the timer count is specifically for timer 1 with a 1:64 prescaler in this case; look into making this generalized
    // NOTE: I had to double the time for turning, not entirely sure why...
    
    // set motor directions
    if(angle > 0) {
        MOTOR1_DIR = 1;
        MOTOR2_DIR = 0;
    } else {
        MOTOR1_DIR = 0;
        MOTOR2_DIR = 1;
    }
    MOTOR_PWM = (int)((FCY/motorFPWM)-1);
    MOTOR_DUTY_CYCLE = (int)(MOTOR_PWM/2.0);
    MOTOR_TIMER = 0;
    while(MOTOR_TIMER < timerCount){
        MOTOR_DUTY_CYCLE = (int)(MOTOR_PWM/2.0);
    } // wait until we completely turn
    setSpeed(0);
}

void setup() {
    _RCDIV = 0; // div by 1 postscaler Fosc = 500 kHz; Fcy = 250 kHz
    
    //timers
    T1CON = 0x0220; // NOTE: 1:64 prescaler
    _TON = 1; // turn timer on
    
    // I/O
    _TRISB12 = 0;
    _TRISB13 = 0;
    
    // PWM
    OC1CON1 = 0x1C06;
    OC1CON2 = 0x001F;
    OC1RS = 1249; // Fpwm = 200 --> 1 RPS
    OC1R = 0; // keep motor off for now
}