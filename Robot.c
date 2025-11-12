//#include "xc.h"
//#include "StepperMotor.h"
//#include "IRProximitySensor.h"
//#include "PIDController.h"
//#include <math.h>
//
//#define FCY 250000UL // Fcy
//
//#define WHEEL_DIAMETER 3.5 // drive motor diameter
//#define COUNTS_PER_REV 200 // ticks per rev for the stepper motors
//#define TURN_RADIUS 4.5 // radius of robot's turn (from center to wheel)
//#define MICROSTEP 0.5 // match the stepper driver switches
//#define MOTOR_DIR_REG LATB
//#define LMOTOR_DIR_BIT 12 // left motor direction
//#define RMOTOR_DIR_BIT 13 // right motor direction
//#define RMOTOR_PWM OC1RS // motor pwm register
//#define RMOTOR_DUTY_CYCLE OC1R // motor duty cycle register
//#define LMOTOR_PWM OC2RS
//#define LMOTOR_DUTY_CYCLE OC2R
//#define MOTOR_TIMER TMR1 // timer used for the motor
//
//#define IR_PROX_REG PORTA
//#define LEFT_IR_PROX_BIT 3 // pin 8
//#define CENTER_IR_PROX_BIT 0 // pin 2
//#define RIGHT_IR_PROX_BIT 1 // pin 3
//
//typedef struct {
//    // add sensors, motors, etc here
//    StepperMotor leftMotor, rightMotor;
//    float turnRadius;
//    volatile unsigned int *motorTimer;
//    enum {LINE_FOLLOW, CANYON_NAVIGATE, GRAB_BALL, DEPOSIT_BALL, PARK_AND_TRANSMIT} state;
//    IRProximitySensor leftLineDetector, centerLineDetector, rightLineDetector;
//    PIDController lineFollowingPID;
//} Robot_t;
//
//Robot_t Robot;
//
//void setupRobot() {
//    // setup timer
//    _RCDIV = 0;
//    T1CON = 0x0220;
//    _TON = 1;
//    Robot.motorTimer = &MOTOR_TIMER;
//    
//    // setup pwm
//    OC1CON1 = 0x1C06;
//    OC1CON2 = 0x001F;
//    OC2CON1 = 0x1C06;
//    OC2CON2 = 0x001F;
//    
//    // analog selections
//    _ANSA0 = 0;
//    _ANSA1 = 0;
//    _ANSA3 = 0;
//    _ANSB12 = 0;
//    _ANSB13 = 0;
//    
//    // I/O
//    _TRISA0 = 1;
//    _TRISA1 = 1;
//    _TRISA3 = 1;
//    _TRISB12 = 0;
//    _TRISB13 = 0;
//    
//    // motors
//    setupMotor(FORWARD, &MOTOR_DIR_REG, LMOTOR_DIR_BIT, &LMOTOR_PWM, &LMOTOR_DUTY_CYCLE, COUNTS_PER_REV, WHEEL_DIAMETER, MICROSTEP, &(Robot.leftMotor));
//    setupMotor(FORWARD, &MOTOR_DIR_REG, RMOTOR_DIR_BIT, &RMOTOR_PWM, &RMOTOR_DUTY_CYCLE, COUNTS_PER_REV, WHEEL_DIAMETER, MICROSTEP, &(Robot.rightMotor));
//    
//    // sensors
//    setupIRProximitySensor(&IR_PROX_REG, LEFT_IR_PROX_BIT, &Robot.leftLineDetector);
//    setupIRProximitySensor(&IR_PROX_REG, CENTER_IR_PROX_BIT, &Robot.centerLineDetector);
//    setupIRProximitySensor(&IR_PROX_REG, RIGHT_IR_PROX_BIT, &Robot.rightLineDetector);
//    
//    // PID Controllers
//    setupPID(0, 0, 0, 0, &Robot.lineFollowingPID); // TODO: figure out how to fully implement this controller
//}
//
//void updateState() {
//    Robot.state = LINE_FOLLOW;
//    // do some checks to see what state to be in
//}
//
//void turn(float angle, float turnSpeedInPerSec) {
//    // assume identical motors and settings
//    float distToTravel = (angle/360.0)*2.0*M_PI * Robot.leftMotor.countsPerRev; // distance for the motor to travel
//    int ticksToTravel = (int)(Robot.leftMotor.countsPerRev * distToTravel / (M_PI * Robot.leftMotor.wheelDiameter)); // ticks for the motor to travel
//    float motorFPWM = turnSpeedInPerSec * Robot.leftMotor.countsPerRev / (M_PI * Robot.leftMotor.wheelDiameter * Robot.leftMotor.microstep); // Fpwm
//    float timeToTravel = ticksToTravel/motorFPWM; // time it takes for motor to travel the number of ticks
//    int timerCount = 2*timeToTravel * (int)(FCY/64.0); // timer counts it takes to take the time needed to travel the needed distance on the motor
//    // NOTE: the timer count is specifically for timer 1 with a 1:64 prescaler in this case; look into making this generalized
//    
//    // set motor directions
//    if(angle > 0) {
//        *(Robot.leftMotor.dirReg) |= (1 << Robot.leftMotor.dirBit);
//        *(Robot.rightMotor.dirReg) &= ~(1 << Robot.rightMotor.dirBit);
//    } else {
//        *(Robot.leftMotor.dirReg) &= ~(1 << Robot.leftMotor.dirBit);
//        *(Robot.rightMotor.dirReg) |= (1 << Robot.rightMotor.dirBit);
//    }
//    *(Robot.leftMotor.pwmPin) = (int)((FCY/motorFPWM)-1); // both motors have the same pwm register for now
//    *(Robot.rightMotor.pwmPin) = (int)((FCY/motorFPWM)-1);
//    *(Robot.leftMotor.dutyCyclePin) = (int)(*(Robot.leftMotor.pwmPin)/2.0); // both motoros have the same duty cycle register for now
//    *(Robot.rightMotor.dutyCyclePin) = (int)(*(Robot.rightMotor.pwmPin)/2.0);
//    *(Robot.motorTimer) = 0;
//    while(*(Robot.motorTimer) < timerCount){
//        *(Robot.leftMotor.dutyCyclePin) = (int)(*(Robot.leftMotor.pwmPin)/2.0);
//        *(Robot.rightMotor.dutyCyclePin) = (int)(*(Robot.rightMotor.pwmPin)/2.0);
//    } // wait until we completely turn
//    setSpeed(0, &(Robot.leftMotor));
//    setSpeed(0, &(Robot.rightMotor));
//}
//
//void setDriveSpeed(float speedInPerSec) {
//    setSpeed(speedInPerSec, &(Robot.leftMotor));
//    setSpeed(speedInPerSec, &(Robot.rightMotor));
//}