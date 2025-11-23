/* 
 * File:   Robot.h
 * Author: jothm
 *
 * Created on October 4, 2025, 1:19 PM
 */

#ifndef ROBOT_H
#define	ROBOT_H

#include <xc.h>
#include "StepperMotor.h"
#include "IRProximitySensor.h"
#include "IRRangeSensor.h"
#include "Photodiode.h"
#include "PIDController.h"
#include "Solenoid.h"
#include "QRDSensor.h"
#include <math.h>

#define FRONT_IR_LIMIT 2000 // just under 15 cm or ~1.65V
#define LEFT_IR_LIMIT 870 // around 0.7v
#define FCY 250000UL // Fcy = 250 kHz
#include "libpic30.h"

typedef struct {
    StepperMotor leftMotor, rightMotor;
    volatile uint16_t *motorTimer, grabbedBall, inCanyon, traversedCanyon, depositedBall;
    enum {LINE_FOLLOW, CANYON_NAVIGATE, EXIT_CANYON, GRAB_BALL, DEPOSIT_BALL, PARK_AND_TRANSMIT, STOP} state;
    IRProximitySensor leftLineDetector, centerLineDetector, rightLineDetector;
    IRRangeSensor frontRange, leftRange;
    Photodiode sampleCollectionDetector;
    PIDController lineFollowingPID;
    Solenoid solenoid;
    QRDSensor qrd;
} Robot_t;

extern Robot_t Robot;

void configADC();
void setupRobot();
void setDriveSpeed(float speedInPerSec);
void updateState();
void setTurnSpeed(float turnSpeedInPerSec);
void turn(float angle, float turnSpeedInPerSec);
void driveDistance(float distanceIn, float speedInPerSec);

#endif	/* ROBOT_H */

