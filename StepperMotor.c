//#include <xc.h>
//#include <math.h>
//#include <stdint.h>
//
//#define FCY 250000UL
//
//typedef enum {FORWARD, REVERSE} direction;
//
//typedef struct {
//    direction direction;
//    volatile unsigned int *dirPin, *pwmPin, *dutyCyclePin;
//    int countsPerRev;
//    float rps, wheelDiameter, microstep;
//} StepperMotor;
//
//void setupMotor(direction dir, volatile unsigned int *dirPin, volatile unsigned int *pwmPin, volatile unsigned int *dutyCyclePin, int countsPerRev, float wheelDiameter, float microstep, StepperMotor *motor) {
//    motor->direction = dir;
//    motor->dirPin = dirPin;
//    motor->pwmPin = pwmPin;
//    motor->dutyCyclePin = dutyCyclePin;
//    motor->countsPerRev = countsPerRev;
//    motor->wheelDiameter = wheelDiameter;
//    motor->microstep = microstep;
//    motor->rps = 0;
//}
//
//void setDirection(direction dir, StepperMotor *motor) {
//    motor->direction = dir;
//}
//
//void setRPS(float rps, StepperMotor *motor) {
//    *(motor->dirPin) = (rps > 0)? 0:1; // determine forward or reverse
//    if((motor->direction) == REVERSE) {
//        *(motor->dirPin) = !*(motor->dirPin); // reverse the direction if the motor is flagged as reversed
//    }
//    if((int)(rps) == 0) {
//        *(motor->pwmPin) = 1249; // some arbitrary pwm signal
//        *(motor->dutyCyclePin) = 0; // turn off motors (brake)
//    } else {
//        float fpwm = rps * motor->countsPerRev / motor->microstep; // calculate Fpwm
//        *(motor->pwmPin) = (int)((FCY/fpwm)-1); // calculate OCxRS
//        *(motor->dutyCyclePin) = (int)(*(motor->pwmPin)/2.0); // calculate OCxR
//    }
//}
//
//void setSpeed(float speedInPerSec, StepperMotor *motor) {
//    float rps = speedInPerSec/(M_PI * motor->wheelDiameter); // calculate motor RPS
//    setRPS(rps, motor);
//}