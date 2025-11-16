///*
// * File:   driveTest.c
// * Author: jothm
// *
// * Created on October 23, 2025, 12:30 PM
// */
//
//#include <stdlib.h>
//#include <math.h>
//#include "xc.h"
//#include "IRProximitySensor.h"
//#include "IRRangeSensor.h"
//
//// these lines are included in Config.c
////#pragma config FNOSC = LPFRC // 500 kHz osc
////#pragma config OSCIOFNC = OFF // disable osc on pin 8
//
//#define WHEEL_DIAMETER 3.5 // drive motor diameter
//#define TICKS_PER_REV 200 // ticks per rev for the stepper motors
//#define TURN_RADIUS 4.75 // radius of robot's turn (from center to wheel)
//#define TURN_FACTOR 1.0 // fudge factor to correct turning
//#define FCY 250000UL // Fcy
//#define MICROSTEP 0.5 // match the stepper driver switches
//#define MOTOR1_DIR _LATB12 // left motor direction
//#define MOTOR2_DIR _LATB13 // right motor direction
//#define RIGHT_MOTOR_PWM OC1RS // motor pwm register
//#define LEFT_MOTOR_PWM OC2RS
//#define RIGHT_MOTOR_DUTY_CYCLE OC1R // motor duty cycle register
//#define LEFT_MOTOR_DUTY_CYCLE OC2R
//#define MOTOR_TIMER TMR1 // timer used for the motor
//
//#define IR_REG PORTA
//#define LEFT_IR_BIT 3 // pin 8 - working and correct
//#define CENTER_IR_BIT 0 // pin 2
//#define RIGHT_IR_BIT 1 // pin 3 - working and correct
//
//#define LINE_FOLLOW_HIGH_SPEED 25.0
//#define LINE_FOLLOW_MED_SPEED 10.0
//#define LINE_FOLLOW_LOW_SPEED 6.0
//#define HIGH_ADJ_FACTOR 0.6
//#define MED_ADJ_FACTOR 0.35
//#define LOW_ADJ_FACTOR 0.3
//
//#define CANYON_HIGH_SPEED 25.0
//#define CANYON_MED_SPEED 15.0
//#define CANYON_LOW_SPEED 6.0
//
//#define NUM_ANALOG_CHANNELS 3
//#define FRONT_RANGE ADC1BUF4 // pin 6
//#define FRONT_NOISE 1613 // noise around 1.3V - consider an average value funciton
//#define LEFT_RANGE ADC1BUF13 // pin 7
//#define LEFT_NOISE 1551 // noise for left range around 1.25V
//#define PHOTODIODE ADC1BUF10 // pin 17
//#define IR_LIMIT 2000 // just over 16 cm or ~1.65V
//#define PHOTODIODE_LIMIT 1240 // 1V
//
//#include "libpic30.h"
//
//IRProximitySensor leftLineDetector, centerLineDetector, rightLineDetector;
//IRRangeSensor frontRange, leftRange;
//
//void setSpeed(float speedInPerSec);
//void turn(float angle, float turnSpeedInPerSec);
//void lineFollowCorrectRightBias(float speedInPerSec, float speedAdj);
//void setup();
//
//int main(void) {
//    // SETUP
//    setup();
//    
//    // MAIN
//    // ------------------------------ Autonomous Code ---------------------------
////    setSpeed(12); // about 1 rps forward
////    __delay_ms(2000); // wait 2 seconds
////    setSpeed(0); // brake
////    
////    turn(90, 12); // turn 90 degrees
////    
////    setSpeed(12);
////    __delay_ms(2000);
////    setSpeed(0);
////    
////    turn(180, 6); // turn 180 degrees, half speed
////    
////    setSpeed(12);
////    __delay_ms(2000);
////    setSpeed(0);
//    
////    turn(90, 6);
//    
//    while(1) {
//        // line following
//        if(detectsLine(&centerLineDetector)) { // center sensor sees a line
//            if(detectsLine(&leftLineDetector)) { // left sensor also sees a line
//                // turn left
//                lineFollowCorrectRightBias(LINE_FOLLOW_HIGH_SPEED, -LINE_FOLLOW_HIGH_SPEED*MED_ADJ_FACTOR);
//            } else if(detectsLine(&rightLineDetector)) { // right sensor also sees a line
//                // turn right
//                lineFollowCorrectRightBias(LINE_FOLLOW_HIGH_SPEED, LINE_FOLLOW_HIGH_SPEED*MED_ADJ_FACTOR);
//            } else setSpeed(LINE_FOLLOW_HIGH_SPEED);
//        } else if(detectsLine(&leftLineDetector)) { // only the left sensor sees a line
//            // turn left
//            lineFollowCorrectRightBias(LINE_FOLLOW_HIGH_SPEED, -LINE_FOLLOW_HIGH_SPEED*HIGH_ADJ_FACTOR);
//        } else if(detectsLine(&rightLineDetector)) { // only the right sensor sees a line
//            // turn right
//            lineFollowCorrectRightBias(LINE_FOLLOW_HIGH_SPEED, LINE_FOLLOW_HIGH_SPEED*HIGH_ADJ_FACTOR);
//        } // do whatever we were doing last if we don't see the line
////        else lineFollowCorrectRightBias(LINE_FOLLOW_HIGH_SPEED, LINE_FOLLOW_HIGH_SPEED*LOW_ADJ_FACTOR); // search if no line is seen
//    }
////    
////    while(1) {
////        updateRange(&frontRange);
////        updateRange(&leftRange);
////        if(getRangeValue(&frontRange) > IR_LIMIT) {
////            setSpeed(15);
////        } else if(getRangeValue(&leftRange) > IR_LIMIT) {
////            setSpeed(-15);
////        } else setSpeed(0);
////    }
//    
////    while(1) {
////        updateRange(&frontRange);
////        updateRange(&leftRange);
////        if(seesWall(IR_LIMIT, &frontRange)) {
////            setSpeed(15);
////        } else if(seesWall(IR_LIMIT, &leftRange)) {
////            setSpeed(-15);
////        } else setSpeed(0);
////    }
//    
//    // CANYON NAV
////    while (1) {
////        // TODO: finish testing and fixing this algorithm, it's on the way to working
////        updateRange(&frontRange);
////        updateRange(&leftRange);
////        if(seesWall(IR_LIMIT, &frontRange)) { // if we see a wall in front of us
////            if(seesWall(IR_LIMIT, &leftRange)) { // there is a wall to the left
////                // turn right
////                setSpeed(0);
////                __delay_ms(25); // pause before starting turn -- attempt to not skip any stepper signals
////                turn(90, CANYON_MED_SPEED);
////            } else {
////                // turn left
////                setSpeed(0);
////                __delay_ms(25);
////                turn(-90, CANYON_MED_SPEED);
////            }
////            // update at least one set of ranges after we turn to discard noise
////            __delay_ms(100); // wait a bit before taking new readings - also to eliminate noise
////            updateRange(&frontRange);
////            updateRange(&leftRange);
////        } else {
////            // go straight
////            setSpeed(CANYON_MED_SPEED);
////        }
////    }
////    while(1);
//    return 0;
//}
//
//void setRPS(float rps, volatile unsigned int *motorPWM, volatile unsigned int *motorDutyCycle) {
//    MOTOR1_DIR = (rps > 0)? 0:1, MOTOR2_DIR = (rps > 0)? 0:1; // determine forward or reverse
//    if(rps == 0) {
//        *motorPWM = 1249; // some arbitrary pwm signal
//        *motorDutyCycle = 0; // turn off motors (brake)
//    } else {
//        float fpwm = fabs(rps) * TICKS_PER_REV / MICROSTEP; // calculate Fpwm
//        *motorPWM = (int)((FCY/fpwm)-1); // calculate OCxRS
//        *motorDutyCycle = (int)((*motorPWM)/2.0); // calculate OCxR
//    }
//}
//
//void setMotorSpeed(float speedInPerSec, volatile unsigned int *pwm, volatile unsigned int *dc) {
//    float rps = speedInPerSec/(M_PI*WHEEL_DIAMETER); // calculate motor RPS
//    setRPS(rps, pwm, dc);
//}
//
//void setSpeed(float speedInPerSec) {
//    setMotorSpeed(speedInPerSec, &LEFT_MOTOR_PWM, &LEFT_MOTOR_DUTY_CYCLE);
//    setMotorSpeed(speedInPerSec, &RIGHT_MOTOR_PWM, &RIGHT_MOTOR_DUTY_CYCLE);
//}
//
//void lineFollowCorrectRightBias(float speedInPerSec, float speedAdj) {
//    setMotorSpeed(speedInPerSec + speedAdj, &LEFT_MOTOR_PWM, &LEFT_MOTOR_DUTY_CYCLE);
//    setMotorSpeed(speedInPerSec - speedAdj, &RIGHT_MOTOR_PWM, &RIGHT_MOTOR_DUTY_CYCLE);
//}
//
//void turn(float angle, float turnSpeedInPerSec) {
//    float distToTravel = (fabs(angle)/360.0)*2.0*M_PI*TURN_RADIUS; // distance for the motor to travel
//    int ticksToTravel = (int)(TICKS_PER_REV * distToTravel / (M_PI*WHEEL_DIAMETER)); // ticks for the motor to travel
//    float motorFPWM = turnSpeedInPerSec*TICKS_PER_REV / (M_PI*WHEEL_DIAMETER*MICROSTEP); // Fpwm
//    float timeToTravel = ticksToTravel/motorFPWM; // time it takes for motor to travel the number of ticks
//    int timerCount = (int)(TURN_FACTOR*2.0*timeToTravel * (FCY/64.0)); // timer counts it takes to take the time needed to travel the needed distance on the motor
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
//    LEFT_MOTOR_PWM = (int)((FCY/motorFPWM)-1);
//    RIGHT_MOTOR_PWM = (int)((FCY/motorFPWM)-1);
//    LEFT_MOTOR_DUTY_CYCLE = (int)(LEFT_MOTOR_PWM/2.0);
//    RIGHT_MOTOR_DUTY_CYCLE = (int)(RIGHT_MOTOR_PWM/2.0);
//    MOTOR_TIMER = 0;
//    while(MOTOR_TIMER < timerCount){
//        LEFT_MOTOR_DUTY_CYCLE = (int)(LEFT_MOTOR_PWM/2.0);
//        RIGHT_MOTOR_DUTY_CYCLE = (int)(RIGHT_MOTOR_PWM/2.0);
//    } // wait until we completely turn
//    setSpeed(0);
//}
//
//void config_ad(void) {
//    _ADON = 0;          // AD1CON1<15> -- Turn off A/D during config
//    
//    // Clear all A/D registers
//    AD1CON1 = 0; 
//    AD1CON2 = 0; 
//    AD1CON3 = 0; 
//    AD1CON5 = 0; 
//    AD1CSSL = 0; 
//    AD1CSSH = 0; 
//    
//    // AD1CON1 register
//    _ADSIDL = 0;    // AD1CON1<13> -- A/D continues while in idle mode
//    _MODE12 = 1;    // AD1CON1<10> -- 12-bit A/D operation
//    _FORM = 0;      // AD1CON1<9:8> -- Unsigned integer output
//    _SSRC = 7;      // AD1CON1<7:4> -- Auto conversion (internal counter)
//    _ASAM = 1;      // AD1CON1<2> -- Auto sampling
//
//    // AD1CON2 register
//    _PVCFG = 0;     // AD1CON2<15:14> -- Use VDD as positive ref voltage
//    _NVCFG = 0;     // AD1CON2<13> -- Use VSS as negative ref voltage
//    _BUFREGEN = 1;  // AD1CON2<11> -- Result appears in buffer
//                    // location corresponding to channel, e.g., AN12
//                    // results appear in ADC1BUF12
//    _CSCNA = 1;     // AD1CON2<10> -- Scans inputs specified in AD1CSSx
//                    // registers
//    _SMPI = NUM_ANALOG_CHANNELS-1;    // AD1CON2<6:2> -- Results sent to buffer after n conversion
//                    // For example, if you are sampling 4 channels, you
//                    // should have _SMPI = 3;
//    _ALTS = 0;      // AD1CON2<0> -- Sample MUXA only
//
//    // AD1CON3 register -- Change _SAMC and _ADCS based on your
//    // selection of oscillator and postscaling
//    _ADRC = 0;      // AD1CON3<15> -- Use system clock
//    _SAMC = 1;      // AD1CON3<12:8> -- Auto sample every A/D period TAD
//    _ADCS = 0;      // AD1CON3<7:0> -- A/D period TAD = TCY
//    
//    // AD1CHS register
//    _CH0NA = 0;     // AD1CHS<7:5> -- Measure voltages relative to VSS
//
//    // AD1CSSL register
//    // SET THE BITS CORRESPONDING TO CHANNELS THAT YOU WANT
//    // TO SAMPLE
//    _CSS4 = 1; // AN4 - pin 6: IRRange
//    _CSS13 = 1; // AN13 - pin 7: IRRange
//    _CSS10 = 1; // AN10 - pin 17: Photodiode
//    
//    _ADON = 1;      // AD1CON1<15> -- Turn on A/D
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
//    _TRISB12 = 0;
//    _TRISB13 = 0;
//    _TRISA0 = 1; // IR Prox Sensor - not working
//    _TRISA1 = 1; // IR Prox Sensor
//    _TRISA3 = 1; // IR Prox Sensor
//    
//    config_ad();
//    
//    centerLineDetector.readReg = &IR_REG;
//    centerLineDetector.bit = CENTER_IR_BIT;
//    leftLineDetector.readReg = &IR_REG;
//    leftLineDetector.bit = LEFT_IR_BIT;
//    rightLineDetector.readReg = &IR_REG;
//    rightLineDetector.bit = RIGHT_IR_BIT;
//    
//    frontRange.readBit = &FRONT_RANGE;
//    leftRange.readBit = &LEFT_RANGE;
//    
//    // PWM
//    OC1CON1 = 0x1C06;
//    OC1CON2 = 0x001F;
//    LEFT_MOTOR_PWM = 1249; // Fpwm = 200 --> 1 RPS
//    LEFT_MOTOR_DUTY_CYCLE = 0; // keep motor off for now
//    OC2CON1 = 0x1C06;
//    OC2CON2 = 0x001F;
//    RIGHT_MOTOR_PWM = 1249; // Fpwm = 200 --> 1 RPS
//    RIGHT_MOTOR_DUTY_CYCLE = 0; // keep motor off for now
//}
