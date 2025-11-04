#include "xc.h"
#include "StepperMotor.h"
#include "IRProximitySensor.h"
#include <math.h>

#define FCY 250000UL // Fcy

#define WHEEL_DIAMETER 3.5 // drive motor diameter
#define COUNTS_PER_REV 200 // ticks per rev for the stepper motors
#define TURN_RADIUS 4.5 // radius of robot's turn (from center to wheel)
#define MICROSTEP 0.5 // match the stepper driver switches
#define LMOTOR_DIR_BIT 12 // left motor direction
#define RMOTOR_DIR_BIT 13 // right motor direction
#define MOTOR_DIR_REG LATB
#define MOTOR_PWM OC1RS // motor pwm register
#define MOTOR_DUTY_CYCLE OC1R // motor duty cycle register
#define MOTOR_TIMER TMR1 // timer used for the motor

#define LEFT_CENTER_IR_PROX_REG PORTA
#define RIGHT_IR_PROX_REG PORTB
#define LEFT_IR_PROX_BIT 0
#define CENTER_IR_PROX_BIT 1
#define RIGHT_IR_PROX_BIT 0

typedef struct {
    // add sensors, motors, etc here
    StepperMotor leftMotor, rightMotor;
    float turnRadius;
    volatile unsigned int *motorTimer;
    
    IRProximitySensor leftLineDetector, centerLineDetector, rightLineDetector;
} Robot_t;

Robot_t Robot;

void setupRobot() {
    // setup timer
    T1CON = 0x0220;
    _TON = 1;
    Robot.motorTimer = &MOTOR_TIMER;
    
    // setup pwm
    OC1CON1 = 0x1C06;
    OC1CON2 = 0x001F;
    
    // analog selections
    _ANSB12 = 0;
    _ANSB13 = 0;
    _ANSA0 = 0;
    _ANSA1 = 0;
    _ANSB0 = 0;
    
    // I/O
    _TRISB12 = 0;
    _TRISB13 = 0;
    _TRISA0 = 1;
    _TRISA1 = 1;
    _TRISB0 = 1;
    
    // motors
    setupMotor(FORWARD, &MOTOR_DIR_REG, LMOTOR_DIR_BIT, &MOTOR_PWM, &MOTOR_DUTY_CYCLE, COUNTS_PER_REV, WHEEL_DIAMETER, MICROSTEP, &(Robot.leftMotor));
    setupMotor(FORWARD, &MOTOR_DIR_REG, RMOTOR_DIR_BIT, &MOTOR_PWM, &MOTOR_DUTY_CYCLE, COUNTS_PER_REV, WHEEL_DIAMETER, MICROSTEP, &(Robot.rightMotor));
    
    // sensors
    setupIRProximitySensor(&LEFT_CENTER_IR_PROX_REG, LEFT_IR_PROX_BIT, &Robot.leftLineDetector);
    setupIRProximitySensor(&LEFT_CENTER_IR_PROX_REG, CENTER_IR_PROX_BIT, &Robot.centerLineDetector);
    setupIRProximitySensor(&RIGHT_IR_PROX_REG, RIGHT_IR_PROX_BIT, &Robot.rightLineDetector);
}

void turn(float angle, float turnSpeedInPerSec) {
    float distToTravel = (angle/360.0)*2.0*M_PI * Robot.leftMotor.countsPerRev; // distance for the motor to travel
    int ticksToTravel = (int)(Robot.leftMotor.countsPerRev * distToTravel / (M_PI * Robot.leftMotor.wheelDiameter)); // ticks for the motor to travel
    float motorFPWM = turnSpeedInPerSec * Robot.leftMotor.countsPerRev / (M_PI * Robot.leftMotor.wheelDiameter * Robot.leftMotor.microstep); // Fpwm
    float timeToTravel = ticksToTravel/motorFPWM; // time it takes for motor to travel the number of ticks
    int timerCount = 2*timeToTravel * (int)(FCY/64.0); // timer counts it takes to take the time needed to travel the needed distance on the motor
    // NOTE: the timer count is specifically for timer 1 with a 1:64 prescaler in this case; look into making this generalized
    
    // set motor directions
    if(angle > 0) {
        *(Robot.leftMotor.dirReg) |= (1 << Robot.leftMotor.dirBit);
        *(Robot.rightMotor.dirReg) &= ~(1 << Robot.rightMotor.dirBit);
    } else {
        *(Robot.leftMotor.dirReg) &= ~(1 << Robot.leftMotor.dirBit);
        *(Robot.rightMotor.dirReg) |= (1 << Robot.rightMotor.dirBit);
    }
    *(Robot.leftMotor.pwmPin) = (int)((FCY/motorFPWM)-1); // both motors have the same pwm register for now
    *(Robot.leftMotor.dutyCyclePin) = (int)(*(Robot.leftMotor.pwmPin)/2.0); // both motoros have the same duty cycle register for now
    *(Robot.motorTimer) = 0;
    while(*(Robot.motorTimer) < timerCount){
        *(Robot.leftMotor.dutyCyclePin) = (int)(*(Robot.leftMotor.pwmPin)/2.0);
    } // wait until we completely turn
    setSpeed(0, &(Robot.leftMotor));
    setSpeed(0, &(Robot.rightMotor));
}

void setDriveSpeed(float speedInPerSec) {
    float leftRPS = speedInPerSec / (M_PI * Robot.leftMotor.wheelDiameter);
    float rightRPS = speedInPerSec / (M_PI * Robot.rightMotor.wheelDiameter);
    setSpeed(leftRPS, &(Robot.leftMotor));
    setSpeed(rightRPS, &(Robot.rightMotor));
}