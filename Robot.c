#include "xc.h"
#include "StepperMotor.h"
#include "IRProximitySensor.h"
#include "IRRangeSensor.h"
#include "PIDController.h"
#include <math.h>

#define FCY 250000UL // Fcy

#define NUM_ANALOG_CHANNELS 3

#define WHEEL_DIAMETER 3.5 // drive motor diameter
#define COUNTS_PER_REV 200 // ticks per rev for the stepper motors
#define TURN_RADIUS 4.75 // radius of robot's turn (from center to wheel)
#define TURN_FACTOR 1.0
#define MICROSTEP 0.5 // match the stepper driver switches
#define MOTOR_DIR_REG LATB
#define LMOTOR_DIR_BIT 12 // left motor direction
#define RMOTOR_DIR_BIT 13 // right motor direction
#define RMOTOR_PWM OC1RS // motor pwm register - pin 14
#define RMOTOR_DUTY_CYCLE OC1R // motor duty cycle register
#define LMOTOR_PWM OC2RS // pin 4
#define LMOTOR_DUTY_CYCLE OC2R
#define MOTOR_TIMER TMR1 // timer used for the motor

#define IR_PROX_REG PORTA
#define LEFT_IR_PROX_BIT 3 // pin 8
#define CENTER_IR_PROX_BIT 0 // pin 2
#define RIGHT_IR_PROX_BIT 1 // pin 3
#define FRONT_RANGE ADC1BUF4 // pin 6
#define LEFT_RANGE ADC1BUF13 // pin 7
#define PHOTODIODE ADC1BUF10 // pin 17

typedef struct {
    // add sensors, motors, etc here
    StepperMotor leftMotor, rightMotor;
    float turnRadius;
    volatile unsigned int *motorTimer;
    enum {LINE_FOLLOW, CANYON_NAVIGATE, GRAB_BALL, DEPOSIT_BALL, PARK_AND_TRANSMIT} state;
    IRProximitySensor leftLineDetector, centerLineDetector, rightLineDetector;
    IRRangeSensor frontRange, leftRange;
    PIDController lineFollowingPID;
} Robot_t;

Robot_t Robot;

void configADC(void) {
    _ADON = 0;          // AD1CON1<15> -- Turn off A/D during config
    
    // Clear all A/D registers
    AD1CON1 = 0; 
    AD1CON2 = 0; 
    AD1CON3 = 0; 
    AD1CON5 = 0; 
    AD1CSSL = 0; 
    AD1CSSH = 0; 
    
    // AD1CON1 register
    _ADSIDL = 0;    // AD1CON1<13> -- A/D continues while in idle mode
    _MODE12 = 1;    // AD1CON1<10> -- 12-bit A/D operation
    _FORM = 0;      // AD1CON1<9:8> -- Unsigned integer output
    _SSRC = 7;      // AD1CON1<7:4> -- Auto conversion (internal counter)
    _ASAM = 1;      // AD1CON1<2> -- Auto sampling

    // AD1CON2 register
    _PVCFG = 0;     // AD1CON2<15:14> -- Use VDD as positive ref voltage
    _NVCFG = 0;     // AD1CON2<13> -- Use VSS as negative ref voltage
    _BUFREGEN = 1;  // AD1CON2<11> -- Result appears in buffer
                    // location corresponding to channel, e.g., AN12
                    // results appear in ADC1BUF12
    _CSCNA = 1;     // AD1CON2<10> -- Scans inputs specified in AD1CSSx
                    // registers
    _SMPI = NUM_ANALOG_CHANNELS-1;    // AD1CON2<6:2> -- Results sent to buffer after n conversion
                    // For example, if you are sampling 4 channels, you
                    // should have _SMPI = 3;
    _ALTS = 0;      // AD1CON2<0> -- Sample MUXA only

    // AD1CON3 register -- Change _SAMC and _ADCS based on your
    // selection of oscillator and postscaling
    _ADRC = 0;      // AD1CON3<15> -- Use system clock
    _SAMC = 1;      // AD1CON3<12:8> -- Auto sample every A/D period TAD
    _ADCS = 0;      // AD1CON3<7:0> -- A/D period TAD = TCY
    
    // AD1CHS register
    _CH0NA = 0;     // AD1CHS<7:5> -- Measure voltages relative to VSS

    // AD1CSSL register
    // SET THE BITS CORRESPONDING TO CHANNELS THAT YOU WANT
    // TO SAMPLE
    _CSS4 = 1; // AN4 - pin 6: IRRange
    _CSS13 = 1; // AN13 - pin 7: IRRange
    _CSS10 = 1; // AN10 - pin 17: Photodiode
    
    _ADON = 1;      // AD1CON1<15> -- Turn on A/D
}

void setupRobot() {
    // setup timer
    _RCDIV = 0;
    T1CON = 0x0220;
    _TON = 1;
    Robot.motorTimer = &MOTOR_TIMER;
    
    // setup pwm
    OC1CON1 = 0x1C06;
    OC1CON2 = 0x001F;
    OC2CON1 = 0x1C06;
    OC2CON2 = 0x001F;
    
    // analog selections
    _ANSA0 = 0; // pin 1
    _ANSA1 = 0; // pin 2
    _ANSA3 = 0; // pin 8
    _ANSB12 = 0; // pin 15
    _ANSB13 = 0; // pin 16
    
    // I/O
    _TRISA0 = 1; // pin 1
    _TRISA1 = 1; // pin 2
    _TRISA3 = 1; // pin 8
    _TRISB12 = 0; // pin 15
    _TRISB13 = 0; // pin 16

    // ANALOG SETUP
    configADC();
    
    // motors
    setupMotor(FORWARD, &MOTOR_DIR_REG, LMOTOR_DIR_BIT, &LMOTOR_PWM, &LMOTOR_DUTY_CYCLE, COUNTS_PER_REV, WHEEL_DIAMETER, MICROSTEP, &(Robot.leftMotor));
    setupMotor(FORWARD, &MOTOR_DIR_REG, RMOTOR_DIR_BIT, &RMOTOR_PWM, &RMOTOR_DUTY_CYCLE, COUNTS_PER_REV, WHEEL_DIAMETER, MICROSTEP, &(Robot.rightMotor));
    
    // sensors
    setupIRProximitySensor(&IR_PROX_REG, LEFT_IR_PROX_BIT, &Robot.leftLineDetector);
    setupIRProximitySensor(&IR_PROX_REG, CENTER_IR_PROX_BIT, &Robot.centerLineDetector);
    setupIRProximitySensor(&IR_PROX_REG, RIGHT_IR_PROX_BIT, &Robot.rightLineDetector);
    setupIRRangeSensor(&FRONT_RANGE, &Robot.frontRange);
    setupIRRangeSensor(&LEFT_RANGE, &Robot.leftRange);
    
    // PID Controllers
    setupPID(8, 0.5, 0.05, 5.0, &Robot.lineFollowingPID); // P = 8inPerSec / in, I = 0.5, D = 0.05, I_CAP = 5.0 - suggestions from CHAT
}

void updateState() {
    Robot.state = LINE_FOLLOW;
    // do some checks to see what state to be in
}

void setDriveSpeed(float speedInPerSec) {
    setSpeed(speedInPerSec, &(Robot.leftMotor));
    setSpeed(speedInPerSec, &(Robot.rightMotor));
}

void turn(float angle, float turnSpeedInPerSec) {
    // assume identical motors and settings
    float distToTravel = (fabs(angle)/360.0)*2.0*M_PI * TURN_RADIUS; // distance for the motor to travel
    int ticksToTravel = (int)(Robot.leftMotor.countsPerRev * distToTravel / (M_PI * Robot.leftMotor.wheelDiameter)); // ticks for the motor to travel
    float motorFPWM = turnSpeedInPerSec * Robot.leftMotor.countsPerRev / (M_PI * Robot.leftMotor.wheelDiameter * Robot.leftMotor.microstep); // Fpwm
    float timeToTravel = ticksToTravel/motorFPWM; // time it takes for motor to travel the number of ticks
    int timerCount = (int)(TURN_FACTOR*2.0*timeToTravel * (FCY/64.0)); // timer counts it takes to take the time needed to travel the needed distance on the motor
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
    *(Robot.rightMotor.pwmPin) = (int)((FCY/motorFPWM)-1);
    *(Robot.leftMotor.dutyCyclePin) = (int)(*(Robot.leftMotor.pwmPin)/2.0); // both motoros have the same duty cycle register for now
    *(Robot.rightMotor.dutyCyclePin) = (int)(*(Robot.rightMotor.pwmPin)/2.0);
    *(Robot.motorTimer) = 0;
    while(*(Robot.motorTimer) < timerCount){
        *(Robot.leftMotor.dutyCyclePin) = (int)(*(Robot.leftMotor.pwmPin)/2.0);
        *(Robot.rightMotor.dutyCyclePin) = (int)(*(Robot.rightMotor.pwmPin)/2.0);
    } // wait until we completely turn
    setDriveSpeed(0);
}