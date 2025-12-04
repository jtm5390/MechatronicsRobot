#include "xc.h"
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

#define FCY 250000UL // Fcy
#include "libpic30.h"

#define NUM_ANALOG_CHANNELS 5

// motors
#define WHEEL_DIAMETER 3.5 // drive motor diameter
#define COUNTS_PER_REV 200 // ticks per rev for the stepper motors
#define TURN_RADIUS 4.75 // radius of robot's turn (from center to wheel)
#define TURN_FACTOR 0.96
#define DISTANCE_FACTOR 1.05
#define MICROSTEP 0.5 // match the stepper driver switches
#define MOTOR_DIR_REG LATB
#define LMOTOR_DIR_BIT 13 // left motor direction
#define RMOTOR_DIR_BIT 12 // right motor direction
#define RMOTOR_PWM OC1RS // motor pwm register - pin 14
#define RMOTOR_DUTY_CYCLE OC1R // motor duty cycle register
#define LMOTOR_PWM OC2RS // pin 4
#define LMOTOR_DUTY_CYCLE OC2R
#define MOTOR_TIMER TMR1 // timer used for the motor

// servo
#define SERVO_PWM OC3RS // pin 5
#define SERVO_DUTY_CYCLE OC3R
#define SERVO_MAX_ANGLE 180.0
#define SERVO_MIN_DUTY_CYCLE 100.0
#define SERVO_MAX_DUTY_CYCLE 375.0

// line sensors
#define IR_PROX_REG PORTA
#define CENTER_IR_PROX_REG PORTB
#define LEFT_IR_PROX_BIT 3 // pin 8
#define CENTER_IR_PROX_BIT 7 // pin 11
#define RIGHT_IR_PROX_BIT 1 // pin 3
#define PARK_LINE_IR_PROX_BIT 4 // pin 10

// wall sensors
#define FRONT_RANGE ADC1BUF4 // pin 6
#define LEFT_RANGE ADC1BUF13 // pin 7
#define FRONT_IR_LIMIT 2000 // just under 15 cm or ~1.65V
#define LEFT_IR_LIMIT 870 // around 0.7v

// photodiodes
#define SATELLITE_PHOTODIODE ADC1BUF0 // pin 2
#define SATELLITE_IR_LIMIT 1000
#define SAMPLE_COLLECTION_PHOTODIODE ADC1BUF10 // pin 17
#define SAMPLE_COLLECTION_IR_VALUE 1860 // 1.5V

// other
#define QRD ADC1BUF9 // AN9, pin 18
#define SOLENOID_REG PORTB
#define SOLENOID_BIT 9
#define LASER_REG PORTB
#define LASER_BIT 8 // pin 12

typedef struct {
    // add sensors, motors, etc here
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
    _CSS0 = 1; // AN0 - pin 2: Photodiode
    _CSS4 = 1; // AN4 - pin 6: IRRange
    _CSS13 = 1; // AN13 - pin 7: IRRange
    _CSS10 = 1; // AN10 - pin 17: Photodiode
    _CSS9 = 1; // AN9 - pin 18: QRD
    
    _ADON = 1;      // AD1CON1<15> -- Turn on A/D
}

void setupRobot() {
    // Initial States
    Robot.state = START;
    Robot.grabbedBall = 0;
    Robot.traversedCanyon = 0;
    Robot.depositedBall = 0;
    Robot.deployed = 0;
    Robot.transmitting = 0;
    
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
    OC3CON1 = 0x1C06;
    OC3CON2 = 0x001F;
    
    // analog selections
    _ANSA0 = 0; // pin 2
    _ANSA1 = 1; // pin 3
    _ANSB2 = 1; // pin 6
    _ANSA2 = 1; // pin 7
    _ANSA3 = 0; // pin 8
    _ANSB12 = 0; // pin 15
    _ANSB13 = 0; // pin 16
    _ANSB14 = 1; // pin 17
    _ANSB15 = 1; // pin 18
    
    // I/O
    _TRISA0 = 1; // pin 2 - Photodiode
    _TRISA1 = 1; // pin 3 - IR Prox
    _TRISB2 = 1; // pin 6 - IR Range
    _TRISA2 = 1; // pin 7 - IR Range
    _TRISA3 = 1; // pin 8 - IR Prox
    _TRISA4 = 1; // pin 10 - IR Prox
    _TRISB7 = 1; // pin 11 - IR Prox
    _TRISB8 = 0; // pin 12 - Laser
    _TRISB9 = 0; // pin 13 - Solenoid
    _TRISB12 = 0; // pin 15 - Motor Dir
    _TRISB13 = 0; // pin 16 - Motor Dir
    _TRISB14 = 1; // pin 17 - Photodiode
    _TRISB15 = 1; // pin 18 - QRD

    // ANALOG SETUP
    configADC();
    
    // motors
    setupMotor(FORWARD, &MOTOR_DIR_REG, LMOTOR_DIR_BIT, &LMOTOR_PWM, &LMOTOR_DUTY_CYCLE, COUNTS_PER_REV, WHEEL_DIAMETER, MICROSTEP, &(Robot.leftMotor));
    setupMotor(FORWARD, &MOTOR_DIR_REG, RMOTOR_DIR_BIT, &RMOTOR_PWM, &RMOTOR_DUTY_CYCLE, COUNTS_PER_REV, WHEEL_DIAMETER, MICROSTEP, &(Robot.rightMotor));
    setupServo(&SERVO_PWM, &SERVO_DUTY_CYCLE, SERVO_MAX_ANGLE, SERVO_MIN_DUTY_CYCLE, SERVO_MAX_DUTY_CYCLE, &Robot.servo);
    setupSolenoid(&SOLENOID_REG, SOLENOID_BIT, &Robot.solenoid);
    setSolenoid(1, &Robot.solenoid);
    
    // sensors
    setupIRProximitySensor(&IR_PROX_REG, LEFT_IR_PROX_BIT, &Robot.leftLineDetector);
    setupIRProximitySensor(&CENTER_IR_PROX_REG, CENTER_IR_PROX_BIT, &Robot.centerLineDetector);
    setupIRProximitySensor(&IR_PROX_REG, RIGHT_IR_PROX_BIT, &Robot.rightLineDetector);
    setupIRProximitySensor(&IR_PROX_REG, PARK_LINE_IR_PROX_BIT, &Robot.parkLineDetector);
    setupIRRangeSensor(&FRONT_RANGE, &Robot.frontRange);
    setupIRRangeSensor(&LEFT_RANGE, &Robot.leftRange);
    setupPhotodiode(&SAMPLE_COLLECTION_PHOTODIODE, &Robot.sampleCollectionDetector);
    setupPhotodiode(&SATELLITE_PHOTODIODE, &Robot.satelliteDetector);
    setupQRD(&QRD, &Robot.qrd);
    setupLaser(&LASER_REG, LASER_BIT, &Robot.laser);
    
    // PID Controllers
//    setupPID(10.0, 0.0, 0.0, 5.0, &Robot.lineFollowingPID);
//    setupPID(6.5, 0.5, 0.05, 5.0, &Robot.lineFollowingPID); // P = 8inPerSec / in, I = 0.5, D = 0.05, I_CAP = 5.0
    setupPID(11.5, 1.0, 0.0, 5.0, &Robot.lineFollowingPID);
}

void setDriveSpeed(float speedInPerSec) {
    setSpeed(speedInPerSec, &(Robot.leftMotor));
    setSpeed(speedInPerSec, &(Robot.rightMotor));
}

void brake() {
    setDriveSpeed(0);
}

void updateState() {
    switch(Robot.state) {
        case LINE_FOLLOW:
            // check for IR emitter to go to the grab ball state
            if(!Robot.depositedBall && !Robot.grabbedBall && seesIR(SAMPLE_COLLECTION_IR_VALUE, &Robot.sampleCollectionDetector)) Robot.state = GRAB_BALL;
            else if(!Robot.depositedBall && Robot.grabbedBall) {
                updateRange(&Robot.leftRange);
                updateRange(&Robot.leftRange);
                updateRange(&Robot.leftRange);
                if(seesWall(SAMPLE_COLLECTION_IR_VALUE, &Robot.leftRange)) Robot.state = DEPOSIT_BALL;
            }
            if(!Robot.traversedCanyon && !seesLine(&Robot.leftLineDetector) && !seesLine(&Robot.centerLineDetector) && !seesLine(&Robot.rightLineDetector)) {
                // this is probably enough to go into canyon navigation, but we'll double check
                updateRange(&Robot.leftRange);
                updateRange(&Robot.leftRange);
                updateRange(&Robot.leftRange);
                if(seesWall(LEFT_IR_LIMIT, &Robot.leftRange)) Robot.state = CANYON_NAVIGATE;
            }
            if(Robot.deployed && Robot.grabbedBall && Robot.depositedBall && Robot.traversedCanyon) {
                if(seesLine(&Robot.parkLineDetector)) Robot.state = PARK_AND_TRANSMIT;
            }
            break;
        case CANYON_NAVIGATE:
            // if we see the line, exit the canyon
            if(seesLine(&Robot.centerLineDetector)) Robot.state = EXIT_CANYON;
            break;
        case EXIT_CANYON:
            // if we finished traversing the canyon, line follow
            if(Robot.traversedCanyon) Robot.state = LINE_FOLLOW;
            break;
        case GRAB_BALL:
            // if we grabbed the ball, line follow
            if(Robot.grabbedBall) Robot.state = LINE_FOLLOW;
            break;
        case DEPOSIT_BALL:
            // if we deposited the ball, line follow
            if(Robot.depositedBall) Robot.state = LINE_FOLLOW;
            break;
        case PARK_AND_TRANSMIT:
            // if we are transmitting, end the routine
            if(Robot.transmitting) Robot.state = STOP;
            break;
        case START:
            // if we have left the lander, line follow
            if(Robot.deployed) Robot.state = LINE_FOLLOW;
            break;
        case STOP:
            // do nothing
            break;
        default:
            break;
    }
}

int convertDistanceToTimeCount(float distanceIn, float speedInPerSec) {
    int ticksToTravel = (int)(Robot.leftMotor.countsPerRev * distanceIn / (M_PI * Robot.leftMotor.wheelDiameter)); // ticks for the motor to travel
    float motorFPWM = fabs(speedInPerSec) * Robot.leftMotor.countsPerRev / (M_PI * Robot.leftMotor.wheelDiameter * Robot.leftMotor.microstep); // Fpwm
    float timeToTravel = ticksToTravel/motorFPWM; // time it takes for motor to travel the number of ticks
    return (int)(2.0*timeToTravel*(FCY/64.0));
}

void setTurnSpeed(float turnSpeedInPerSec) {
    setSpeed(turnSpeedInPerSec, &Robot.leftMotor);
    setSpeed(-turnSpeedInPerSec, &Robot.rightMotor);
}

void turn(float angle, float turnSpeedInPerSec) {
    // assume identical motors and settings
    float distToTravel = (fabs(angle)/360.0)*2.0*M_PI * TURN_RADIUS; // distance for the motor to travel
    int timerCount = convertDistanceToTimeCount(distToTravel*TURN_FACTOR, turnSpeedInPerSec);
    
    // set motor directions
    if(angle < 0) turnSpeedInPerSec *= -1.0; // update for CW vs CCW

    // default to turning CW
    setSpeed(turnSpeedInPerSec, &Robot.leftMotor);
    setSpeed(-turnSpeedInPerSec, &Robot.rightMotor);
    *(Robot.motorTimer) = 0;
    while(*(Robot.motorTimer) < timerCount);
    brake();
}

void turnOneWheel(float angle, float turnSpeedInPerSec) {
    float distToTravel = (fabs(angle)/360.0)*2.0*M_PI * TURN_RADIUS;
    int timerCount = convertDistanceToTimeCount(distToTravel*TURN_FACTOR, turnSpeedInPerSec);
    
    if(angle > 0) {
        if(turnSpeedInPerSec > 0) { // left wheel forward
            setSpeed(turnSpeedInPerSec, &Robot.leftMotor);
            setSpeed(0, &Robot.rightMotor);
        } else { // right wheel backward
            setSpeed(0, &Robot.leftMotor);
            setSpeed(turnSpeedInPerSec, &Robot.rightMotor);
        }
    } else {
        if(turnSpeedInPerSec > 0) { // right wheel forward
            setSpeed(0, &Robot.leftMotor);
            setSpeed(turnSpeedInPerSec, &Robot.rightMotor);
        } else { // left wheel backward
            setSpeed(turnSpeedInPerSec, &Robot.leftMotor);
            setSpeed(0, &Robot.rightMotor);
        }
    }
    
    *Robot.motorTimer = 0;
    while(*Robot.motorTimer < timerCount);
    brake();
}

void driveDistance(float distanceIn, float speedInPerSec) {
    int timerCount = convertDistanceToTimeCount(distanceIn*DISTANCE_FACTOR, speedInPerSec);
    
    setDriveSpeed(speedInPerSec);
    *(Robot.motorTimer) = 0;
    while(*(Robot.motorTimer) < timerCount);
    brake();
}