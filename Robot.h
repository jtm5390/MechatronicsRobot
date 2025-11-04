/* 
 * File:   Robot.h
 * Author: jothm
 *
 * Created on October 4, 2025, 1:19 PM
 */

#ifndef ROBOT_H
#define	ROBOT_H

#include "StepperMotor.h"
#include "IRProximitySensor.h"

typedef struct {
    StepperMotor leftMotor, rightMotor;
    float turnRadius;
    volatile unsigned int *motorTimer;
    
    IRProximitySensor leftLineDetector, centerLineDetector, rightLineDetector;
} Robot_t;

extern Robot_t Robot;

void setupRobot();
void turn(float angle, float turnSpeedInPerSec);
void setDriveSpeed(float speedInPerSec);

#endif	/* ROBOT_H */

