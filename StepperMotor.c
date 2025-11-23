#include <xc.h>
#include <math.h>
#include <stdint.h>

#define FCY 250000UL

typedef enum {FORWARD, REVERSE} direction;

typedef struct {
    direction direction;
    volatile uint16_t *dirReg, *pwmPin, *dutyCyclePin;
    unsigned int countsPerRev, dirBit;
    float rps, wheelDiameter, microstep;
} StepperMotor;

void setupMotor(direction dir, volatile uint16_t *dirReg, unsigned int dirBit, volatile uint16_t *pwmPin, volatile uint16_t *dutyCyclePin, unsigned int countsPerRev, float wheelDiameter, float microstep, StepperMotor *motor) {
    motor->direction = dir;
    motor->dirReg = dirReg;
    motor->pwmPin = pwmPin;
    motor->dutyCyclePin = dutyCyclePin;
    motor->countsPerRev = countsPerRev;
    motor->dirBit = dirBit;
    motor->wheelDiameter = wheelDiameter;
    motor->microstep = microstep;
    motor->rps = 0;
}

void setDirection(direction dir, StepperMotor *motor) {
    motor->direction = dir;
}

void setRPS(float rps, StepperMotor *motor) {
    if(rps > 0) *(motor->dirReg) &= ~(1 << motor->dirBit); // set direction bit to 0
    else *(motor->dirReg) |= (1 << motor->dirBit); // set direction bit to 1
    if((motor->direction) == REVERSE) {
        *(motor->dirReg) ^= (1 << motor->dirBit); // reverse the direction if the motor is flagged as reversed
    }
    if(rps == 0) {
        *(motor->pwmPin) = 1249; // some arbitrary pwm signal
        *(motor->dutyCyclePin) = 0; // turn off motors (brake)
    } else {
        float fpwm = fabs(rps) * motor->countsPerRev / motor->microstep; // calculate Fpwm
        *(motor->pwmPin) = (int)((FCY/fpwm)-1); // calculate OCxRS
        *(motor->dutyCyclePin) = (int)(*(motor->pwmPin)/2.0); // calculate OCxR
    }
}

void setSpeed(float speedInPerSec, StepperMotor *motor) {
    float rps = speedInPerSec/(M_PI * motor->wheelDiameter); // calculate motor RPS
    setRPS(rps, motor);
}