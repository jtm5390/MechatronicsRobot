////#include "Robot.h"
//#include <xc.h>
//#include <math.h>
//#include <stdlib.h>
//
//#pragma config FNOSC = LPFRC // 500 kHz osc
//
//#define WHEEL_DIAMETER 3.5 // drive motor diameter
//#define TICKS_PER_REV 200 // ticks per rev for the stepper motors
//#define TURN_RADIUS 4.5 // radius of robot's turn (from center to wheel)
//#define FCY 250000UL // Fcy
//#define MICROSTEP 0.5 // match the stepper driver switches
//#define MOTOR1_DIR _LATB12 // left motor direction
//#define MOTOR2_DIR _LATB13 // right motor direction
//#define MOTOR_PWM OC1RS // motor pwm register
//#define MOTOR_DUTY_CYCLE OC1R // motor duty cycle register
//#define MOTOR_TIMER TMR1 // timer used for the motor
//
//#define IR_REG PORTA
//#define LEFT_IR_BIT 3
//#define CENTER_IR_BIT 1
//#define RIGHT_IR_BIT 0
//
//typedef struct {
//    volatile unsigned int *readReg;
//    unsigned int bit;
//} IRProximitySensor;
//extern IRProximitySensor centerLineDetector, leftLineDetector, rightLineDetector;
//
//typedef enum {LINE_FOLLOW, CANYON_NAVIGATE, GRAB_BALL, DEPOSIT_BALL, PARK_AND_TRANSMIT} State;
//
//void updateState(State *state) {
//    state = LINE_FOLLOW;
//    // do some checks to see what state to be in
//}
//
//void setSpeed(float speedInPerSec);
//void turn(float angle, float turnSpeedInPerSec);
//void setup();
//void straight();
//void correct_left();
//void correct_right();
//int detectsLine(IRProximitySensor *sensor);
//
//void lineFollow() {
//    //1 is dark and 0 is white
////    if (detectsLine(&centerLineDetector) && !detectsLine(&leftLineDetector) && !detectsLine(&rightLineDetector)){
////        straight();
////    }
////    else if (!detectsLine(&centerLineDetector) && detectsLine(&leftLineDetector) && !detectsLine(&rightLineDetector)){
////        correct_left();
////    }
////    else if (!detectsLine(&centerLineDetector) && !detectsLine(&leftLineDetector) && detectsLine(&rightLineDetector)){
////        correct_right();
////    }
//    if(detectsLine(&centerLineDetector)) straight();
//    else if(detectsLine(&leftLineDetector)) correct_left();
//    else if(detectsLine(&rightLineDetector)) correct_right();
//    else setSpeed(0);
//}
//
//void canyonNavigate() {
//
//}
//
//void grabBall() {
//
//}
//
//void depositBall() {
//
//}
//
//void parkAndTransmit() {
//
//}
//
//void setRPS(float rps, volatile unsigned int *motorPWM, volatile unsigned int *motorDutyCycle) {
//    MOTOR1_DIR = (rps > 0)? 0:1, MOTOR2_DIR = (rps > 0)? 0:1; // determine forward or reverse
//    if((int)(rps) == 0) {
//        *motorPWM = 1249; // some arbitrary pwm signal
//        *motorDutyCycle = 0; // turn off motors (brake)
//    } else {
//        float fpwm = rps * TICKS_PER_REV / MICROSTEP; // calculate Fpwm
//        *motorPWM = (int)((FCY/fpwm)-1); // calculate OCxRS
//        *motorDutyCycle = (int)((*motorPWM)/2.0); // calculate OCxR
//    }
//}
//
//void setSpeed(float speedInPerSec) {
//    float rps = speedInPerSec/(M_PI*WHEEL_DIAMETER); // calculate motor RPS
//    setRPS(rps, &MOTOR_PWM, &MOTOR_DUTY_CYCLE);
//}
//
//void turn(float angle, float turnSpeedInPerSec) {
//    float distToTravel = (abs(angle)/360.0)*2.0*M_PI*TURN_RADIUS; // distance for the motor to travel
//    int ticksToTravel = (int)(TICKS_PER_REV * distToTravel / (M_PI*WHEEL_DIAMETER)); // ticks for the motor to travel
//    float motorFPWM = turnSpeedInPerSec*TICKS_PER_REV / (M_PI*WHEEL_DIAMETER*MICROSTEP); // Fpwm
//    float timeToTravel = ticksToTravel/motorFPWM; // time it takes for motor to travel the number of ticks
//    int timerCount = 2*timeToTravel * (int)(FCY/64.0); // timer counts it takes to take the time needed to travel the needed distance on the motor
//    // NOTE: the timer count is specifically for timer 1 with a 1:64 prescaler in this case; look into making this generalized
//    // NOTE: I had to double the time for turning, not entirely sure why...
//    
//    // set motor directions
//    if(angle > 0) {
//        MOTOR1_DIR = 1;
//        MOTOR2_DIR = 0;
//    } else {
//        MOTOR1_DIR = 0;
//        MOTOR2_DIR = 1;
//    }
//    MOTOR_PWM = (int)((FCY/motorFPWM)-1);
//    MOTOR_DUTY_CYCLE = (int)(MOTOR_PWM/2.0);
//    MOTOR_TIMER = 0;
//    while(MOTOR_TIMER < timerCount){
//        MOTOR_DUTY_CYCLE = (int)(MOTOR_PWM/2.0);
//    } // wait until we completely turn
//    setSpeed(0);
//}
//
//void setup() {
//    _RCDIV = 0; // div by 1 postscaler Fosc = 500 kHz; Fcy = 250 kHz
//    
//    //timers
//    T1CON = 0x0220; // NOTE: 1:64 prescaler
//    _TON = 1; // turn timer on
//    
//    // Analog Select
//    _ANSA0 = 0;
//    _ANSA1 = 0;
//    _ANSA3 = 0;
//    _ANSB12 = 0;
//    _ANSB13 = 0;
//    
//    // I/O
//    _TRISB12 = 0; // Motor direction
//    _TRISB13 = 0; // Motor direction
//    _TRISA0 = 1; // IR Prox Sensor
//    _TRISA1 = 1; // IR Prox Sensor
//    _TRISA3 = 1; // IR Prox Sensor
//    
//    // IR Proximity Sensors
//    centerLineDetector.readReg = &IR_REG;
//    centerLineDetector.bit = CENTER_IR_BIT;
//    leftLineDetector.readReg = &IR_REG;
//    leftLineDetector.bit = LEFT_IR_BIT;
//    rightLineDetector.readReg = &IR_REG;
//    rightLineDetector.bit = RIGHT_IR_BIT;
//    
//    // PWM
//    OC1CON1 = 0x1C06;
//    OC1CON2 = 0x001F;
//    OC1RS = 1249; // Fpwm = 200 --> 1 RPS
//    OC1R = 0; // keep motor off for now
//}
//
//void straight(){
//     setSpeed(12); // about 1 rps forward
//}
//
//void correct_right(){
//    turn(2, 12); // turn 2 degrees CW at about 1 rps
//}
//
//void correct_left(){
//    turn(-2, 12); // turn 2 degrees CCW at about 1 rps
//}  
//
//int detectsLine(IRProximitySensor *sensor) {
//    return !(*sensor->readReg & (1 << sensor->bit));
//}