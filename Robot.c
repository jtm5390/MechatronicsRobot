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
//    volatile unsigned int *motorTimer;
//} Robot_t;
//
//Robot_t Robot;
//
//void setupRobot() {
//    // setup timer
//    T1CON = 0x0220;
//    _TON = 1;
//    Robot.motorTimer = &MOTOR_TIMER;
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
//    setupMotor(FORWARD, LMOTOR_DIR, MOTOR_PWM, MOTOR_DUTY_CYCLE, COUNTS_PER_REV, WHEEL_DIAMETER, MICROSTEP, &(Robot.leftMotor));
//    setupMotor(FORWARD, RMOTOR_DIR, MOTOR_PWM, MOTOR_DUTY_CYCLE, COUNTS_PER_REV, WHEEL_DIAMETER, MICROSTEP, &(Robot.rightMotor));
//}
//
//void turn(float angle, float turnSpeedInPerSec) {
//    float distToTravel = (angle/360.0)*2.0*M_PI * Robot.leftMotor.countsPerRev; // distance for the motor to travel
//    int ticksToTravel = (int)(Robot.leftMotor.countsPerRev * distToTravel / (M_PI * Robot.leftMotor.wheelDiameter)); // ticks for the motor to travel
//    float motorFPWM = turnSpeedInPerSec * Robot.leftMotor.countsPerRev / (M_PI * Robot.leftMotor.wheelDiameter * Robot.leftMotor.microstep); // Fpwm
//    float timeToTravel = ticksToTravel/motorFPWM; // time it takes for motor to travel the number of ticks
//    int timerCount = 2*timeToTravel * (int)(FCY/64.0); // timer counts it takes to take the time needed to travel the needed distance on the motor
//    // NOTE: the timer count is specifically for timer 1 with a 1:64 prescaler in this case; look into making this generalized
//    
//    // set motor directions
//    if(angle > 0) {
//        *(Robot.leftMotor.dirPin) = 1;
//        *(Robot.rightMotor.dirPin) = 0;
//    } else {
//        *(Robot.leftMotor.dirPin) = 0;
//        *(Robot.rightMotor.dirPin) = 1;
//    }
//    *(Robot.leftMotor.pwmPin) = (int)((FCY/motorFPWM)-1); // both motors have the same pwm register for now
//    *(Robot.leftMotor.dutyCyclePin) = (int)(*(Robot.leftMotor.pwmPin)/2.0); // both motoros have the same duty cycle register for now
//    *(Robot.motorTimer) = 0;
//    while(*(Robot.motorTimer) < timerCount){
//        *(Robot.leftMotor.dutyCyclePin) = (int)(*(Robot.leftMotor.pwmPin)/2.0);
//    } // wait until we completely turn
//    setSpeed(0, &(Robot.leftMotor));
//    setSpeed(0, &(Robot.rightMotor));
//}
//
//void setDriveSpeed(float speedInPerSec) {
//    float leftRPS = speedInPerSec / (M_PI * Robot.leftMotor.wheelDiameter);
//    float rightRPS = speedInPerSec / (M_PI * Robot.rightMotor.wheelDiameter);
//    setSpeed(leftRPS, &(Robot.leftMotor));
//    setSpeed(rightRPS, &(Robot.rightMotor));
//}