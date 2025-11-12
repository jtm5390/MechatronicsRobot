///* 
// * File:   StepperMotor.h
// * Author: jothm
// *
// * Created on October 20, 2025, 3:15 PM
// */
//
//#ifndef STEPPERMOTOR_H
//#define	STEPPERMOTOR_H
//
//typedef enum {FORWARD, REVERSE} direction;
//
//typedef struct {
//    direction direction;
//    volatile unsigned int *dirReg, *pwmPin, *dutyCyclePin;
//    unsigned int countsPerRev, dirBit;
//    float rps, wheelDiameter, microstep;
//} StepperMotor;
//
//void setupMotor(direction dir, volatile unsigned int *dirReg, unsigned int dirBit, volatile unsigned int *pwmPin, volatile unsigned int *dutyCyclePin, int countsPerRev, float wheelDiameter, float microstep, StepperMotor *motor);
//void setDirection(direction dir, StepperMotor *motor);
//void setRPS(float rps, StepperMotor *motor);
//void setSpeed(float speedInPerSec, StepperMotor *motor);
//
//#endif	/* STEPPERMOTOR_H */
//
