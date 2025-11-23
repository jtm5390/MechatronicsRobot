#include "Robot.h"

#define LINE_FOLLOW_HIGH_SPEED 25.0 // need to test with 30
#define LINE_FOLLOW_MED_SPEED 15.0
#define LINE_FOLLOW_LOW_SPEED 6.0
#define HIGH_ADJ_FACTOR 0.5
#define MED_ADJ_FACTOR 0.3
#define LOW_ADJ_FACTOR 0.3

#define CANYON_HIGH_SPEED 25.0
#define CANYON_MED_SPEED 15.0
#define CANYON_LOW_SPEED 6.0
#define STD_ERROR 0.875 // distance between sensors

#define QRD_WHITE_LIMIT 2481 // 2V


void stop() {
    setDriveSpeed(0);
}

void lineFollowCorrectRightBias(float speedInPerSec, float speedAdj) {
    setSpeed(speedInPerSec + speedAdj, &(Robot.leftMotor));
    setSpeed(speedInPerSec - speedAdj, &(Robot.rightMotor));
}

void lineFollow() {
    // disable PID
    if(T2CONbits.TON == 1) {
        T2CONbits.TON = 0;
        _T2IE = 0;
    }
    // line following
    if(seesLine(&Robot.centerLineDetector)) { // center sensor sees a line
        if(seesLine(&Robot.leftLineDetector)) { // left sensor also sees a line
            // turn left
            lineFollowCorrectRightBias(LINE_FOLLOW_HIGH_SPEED, -LINE_FOLLOW_HIGH_SPEED*MED_ADJ_FACTOR);
        } else if(seesLine(&Robot.rightLineDetector)) { // right sensor also sees a line
            // turn right
            lineFollowCorrectRightBias(LINE_FOLLOW_HIGH_SPEED, LINE_FOLLOW_HIGH_SPEED*MED_ADJ_FACTOR);
        } else setDriveSpeed(LINE_FOLLOW_HIGH_SPEED); // straight
    } else if(seesLine(&Robot.leftLineDetector)) { // only the left sensor sees a line
        // turn left
        lineFollowCorrectRightBias(LINE_FOLLOW_HIGH_SPEED, -LINE_FOLLOW_HIGH_SPEED*HIGH_ADJ_FACTOR);
    } else if(seesLine(&Robot.rightLineDetector)) { // only the right sensor sees a line
        // turn right
        lineFollowCorrectRightBias(LINE_FOLLOW_HIGH_SPEED, LINE_FOLLOW_HIGH_SPEED*HIGH_ADJ_FACTOR);
    }
}

void lineFollowPID() {
    // enable PID
    if(T2CONbits.TON == 0) {
        resetPID(&Robot.lineFollowingPID);
        _T2IE = 1;
        T2CONbits.TON = 1;
    }
    if(Robot.lineFollowingPID.setpoint != 0) Robot.lineFollowingPID.setpoint = 0;
    
    if(seesLine(&Robot.centerLineDetector)) {
        if(seesLine(&Robot.leftLineDetector)) { // err = .875/2
            Robot.lineFollowingPID.pointValue = STD_ERROR/2.0;
        } else if(seesLine(&Robot.rightLineDetector)) { // err = -.875/2
            Robot.lineFollowingPID.pointValue = -STD_ERROR/2.0;
        } else Robot.lineFollowingPID.pointValue = 0;
    } else if(seesLine(&Robot.leftLineDetector)) { // err = .875
        Robot.lineFollowingPID.pointValue = STD_ERROR;
    } else if(seesLine(&Robot.rightLineDetector)) { // err = -.875
        Robot.lineFollowingPID.pointValue = -STD_ERROR;
    }
    calculatePID(&Robot.lineFollowingPID);
    lineFollowCorrectRightBias(LINE_FOLLOW_HIGH_SPEED, Robot.lineFollowingPID.value); // value should be kP, i.e. 10
}

void canyonNavigate() {
    // TODO: finish testing and fixing this algorithm, it's on the way to working
    updateRange(&Robot.frontRange);
    updateRange(&Robot.leftRange);
    if(seesWall(FRONT_IR_LIMIT, &Robot.frontRange)) { // if we see a wall in front of us
        if(seesWall(LEFT_IR_LIMIT, &Robot.leftRange)) { // there is a wall to the left
            // turn right
            setDriveSpeed(0);
            __delay_ms(5); // pause before starting turn -- attempt to not skip any stepper signals
            turn(90, CANYON_MED_SPEED);
        } else {
            // turn left
            setDriveSpeed(0);
            __delay_ms(5);
            turn(-90, CANYON_MED_SPEED);
        }
        // update at least one set of ranges after we turn to discard noise
        __delay_ms(10); // wait a bit before taking new readings - also to eliminate noise
        updateRange(&Robot.frontRange);
        updateRange(&Robot.leftRange);
    } else {
        // go straight
        setDriveSpeed(CANYON_MED_SPEED);
    }
}

void exitCanyon() {
    setDriveSpeed(0);
    // flood the left range sensor with new values
    updateRange(&Robot.leftRange);
    updateRange(&Robot.leftRange);
    updateRange(&Robot.leftRange);
    driveDistance(8, 10);
    __delay_ms(5); // brief pause
    float turnSpeed = -6;
    if(seesWall(LEFT_IR_LIMIT, &Robot.leftRange)) turnSpeed = 6;
    while(!seesLine(&Robot.centerLineDetector)) setTurnSpeed(turnSpeed);
    Robot.inCanyon = 0; // no longer in canyon
//    Robot.state = STOP;
    stop();
    __delay_ms(100); // pause before switching states
}

void grabBall() {

}

void depositBall() {
    stop(); // stop after seeing the buckets
    __delay_ms(25); // brief pause
    driveDistance(10, 8.5); // reposition
    __delay_ms(25); // brief pause
    float turnAngle = (seesWhiteBall(QRD_WHITE_LIMIT, &Robot.qrd))? 65:-65; // determine which bucket to dump in
    turn(turnAngle, 15); // turn towards the bucket
    driveDistance(5, -8); // go to the bucket
    __delay_ms(100); // brief pause before dropping the ball
    setSolenoid(0, &Robot.solenoid); // deposit ball
    __delay_ms(1000);
    driveDistance(5, 10); // leave bucket
    while(!seesLine(&Robot.centerLineDetector)) setTurnSpeed((-turnAngle/fabs(turnAngle))*10.0); // find line
    stop();
    Robot.depositedBall = 1;
}

void parkAndTransmit() {

}