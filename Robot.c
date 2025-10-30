//#include "xc.h"
//#include "StepperMotor.h"
//#include <math.h>
//
//#define FCY 250000UL // Fcy
//#define WHEEL_DIAMETER 3.5 // drive motor diameter
//#define COUNTS_PER_REV 200 // ticks per rev for the stepper motors
//#define TURN_RADIUS 4.5 // radius of robot's turn (from center to wheel)
//#define MICROSTEP 0.5 // match the stepper driver switches
//#define LMOTOR_DIR _LATB12 // left motor direction
//#define RMOTOR_DIR _LATB13 // right motor direction
//#define MOTOR_PWM OC1RS // motor pwm register
//#define MOTOR_DUTY_CYCLE OC1R // motor duty cycle register
//#define MOTOR_TIMER TMR1 // timer used for the motor
//
//typedef struct {
//    // add sensors, motors, etc here
//    StepperMotor leftMotor;
//    StepperMotor rightMotor;
//    float turnRadius;
//    volatile unsigned int *leftMotorDirection, *rightMotorDirection, *motorTimer;
//} Robot;
//
//void setupRobot(Robot *robot) {
//    // setup timer
//    T1CON = 0x0220;
//    _TON = 1;
//    robot->motorTimer = &MOTOR_TIMER;
//    
//    // setup pwm
//    OC1CON1 = 0x1C06;
//    OC1CON2 = 0x001F;
//    
//    // direction pins
//    _TRISB12 = 0;
//    _TRISB13 = 0;
//    
//    // motors
//    setupMotor(FORWARD, LMOTOR_DIR, MOTOR_PWM, MOTOR_DUTY_CYCLE, COUNTS_PER_REV, WHEEL_DIAMETER, MICROSTEP, &(robot->leftMotor));
//    setupMotor(FORWARD, RMOTOR_DIR, MOTOR_PWM, MOTOR_DUTY_CYCLE, COUNTS_PER_REV, WHEEL_DIAMETER, MICROSTEP, &(robot->rightMotor));
//}
//
//void turn(float angle, float turnSpeedInPerSec, Robot *robot) {
//    float distToTravel = (angle/360.0)*2.0*M_PI * robot->leftMotor.countsPerRev; // distance for the motor to travel
//    int ticksToTravel = (int)(robot->leftMotor.countsPerRev * distToTravel / (M_PI * robot->leftMotor.wheelDiameter)); // ticks for the motor to travel
//    float motorFPWM = turnSpeedInPerSec * robot->leftMotor.countsPerRev / (M_PI * robot->leftMotor.wheelDiameter * robot->leftMotor.microstep); // Fpwm
//    float timeToTravel = ticksToTravel/motorFPWM; // time it takes for motor to travel the number of ticks
//    int timerCount = timeToTravel * (int)(FCY/64.0); // timer counts it takes to take the time needed to travel the needed distance on the motor
//    // NOTE: the timer count is specifically for timer 1 with a 1:64 prescaler in this case; look into making this generalized
//    
//    // set motor directions
//    if(angle > 0) {
//        *(robot->leftMotorDirection) = 1;
//        *(robot->rightMotorDirection) = 0;
//    } else {
//        *(robot->leftMotorDirection) = 0;
//        *(robot->rightMotorDirection) = 1;
//    }
//    *(robot->leftMotor.pwmPin) = (int)((FCY/motorFPWM)-1); // both motors have the same pwm register for now
//    *(robot->leftMotor.dutyCyclePin) = (int)(*(robot->leftMotor.pwmPin)/2.0); // both motoros have the same duty cycle register for now
//    *(robot->motorTimer) = 0;
//    while(*(robot->motorTimer) < timerCount){
//        *(robot->leftMotor.dutyCyclePin) = (int)(*(robot->leftMotor.pwmPin)/2.0);
//    } // wait until we completely turn
//    setSpeed(0, &(robot->leftMotor));
//    setSpeed(0, &(robot->rightMotor));
//}