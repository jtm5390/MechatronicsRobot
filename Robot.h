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
#include "Servo.h"
#include "Laser.h"
#include <math.h>

#define FRONT_IR_LIMIT 1750 // just under 15 cm or ~1.65V
#define NEW_FRONT_ENTER_IR_LIMIT 580
#define NEW_FRONT_IR_LIMIT 1860
#define LEFT_IR_LIMIT 850//805 // around 0.7v
#define NEW_LEFT_IR_LIMIT 1100
#define SATELLITE_IR_LIMIT 1000
#define FCY 250000UL // Fcy = 250 kHz
#include "libpic30.h"

typedef struct {
    StepperMotor leftMotor, rightMotor;
    volatile uint16_t *motorTimer, grabbedBall, traversedCanyon, depositedBall, transmitting, deployed;
    enum {LINE_FOLLOW, CANYON_NAVIGATE, EXIT_CANYON, GRAB_BALL, DEPOSIT_BALL, PARK_AND_TRANSMIT, START, STOP} state;
    IRProximitySensor leftLineDetector, centerLineDetector, rightLineDetector, parkLineDetector;
    IRRangeSensor frontRange, leftRange;
    Photodiode sampleCollectionDetector, satelliteDetector;
    PIDController lineFollowingPID;
    Solenoid solenoid;
    QRDSensor qrd;
    Servo servo;
    Laser laser;
} Robot_t;

extern Robot_t Robot;

void configADC();
void setupRobot();
void setDriveSpeed(float speedInPerSec);
void brake();
void updateState();
void setTurnSpeed(float turnSpeedInPerSec);
void turn(float angle, float turnSpeedInPerSec);
void turnOneWheel(float angle, float turnSpeedInPerSec);
void driveDistance(float distanceIn, float speedInPerSec);

#endif	/* ROBOT_H */

