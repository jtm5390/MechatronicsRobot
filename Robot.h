///* 
// * File:   Robot.h
// * Author: jothm
// *
// * Created on October 4, 2025, 1:19 PM
// */
//
//#ifndef ROBOT_H
//#define	ROBOT_H
//
//#include <xc.h>
//#include "StepperMotor.h"
//#include "IRProximitySensor.h"
//
//#define FCY 250000UL // Fcy = 250 kHz
//
//typedef struct {
//    StepperMotor leftMotor, rightMotor;
//    float turnRadius;
//    volatile unsigned int *motorTimer;
//    enum {LINE_FOLLOW, CANYON_NAVIGATE, GRAB_BALL, DEPOSIT_BALL, PARK_AND_TRANSMIT} State;
//    IRProximitySensor leftLineDetector, centerLineDetector, rightLineDetector;
//} Robot_t;
//
//extern Robot_t Robot;
//
//void setupRobot();
//void updateState();
//void turn(float angle, float turnSpeedInPerSec);
//void setDriveSpeed(float speedInPerSec);
//
//#endif	/* ROBOT_H */
//
