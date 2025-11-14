#include "Robot.h"
#include "libpic30.h"

#define LINE_FOLLOW_HIGH_SPEED 25.0 // need to test with 30
#define LINE_FOLLOW_MED_SPEED 10.0
#define LINE_FOLLOW_LOW_SPEED 6.0
#define HIGH_ADJ_FACTOR 0.6
#define MED_ADJ_FACTOR 0.35
#define LOW_ADJ_FACTOR 0.3

#define CANYON_HIGH_SPEED 25.0
#define CANYON_MED_SPEED 15.0
#define CANYON_LOW_SPEED 6.0
#define STD_ERROR 0.875 // distance between sensors
#define IR_LIMIT 2000 // just over 16 cm or ~1.65V

void lineFollowCorrectRightBias(float speedInPerSec, float speedAdj) {
    setSpeed(speedInPerSec + speedAdj, &Robot.leftMotor);
    setSpeed(speedInPerSec - speedAdj, &Robot.rightMotor);
}

void lineFollow() {
    // line following
    if(detectsLine(&Robot.centerLineDetector)) { // center sensor sees a line
        if(detectsLine(&Robot.leftLineDetector)) { // left sensor also sees a line
            // turn left
            lineFollowCorrectRightBias(LINE_FOLLOW_HIGH_SPEED, -LINE_FOLLOW_HIGH_SPEED*MED_ADJ_FACTOR);
        } else if(detectsLine(&Robot.rightLineDetector)) { // right sensor also sees a line
            // turn right
            lineFollowCorrectRightBias(LINE_FOLLOW_HIGH_SPEED, LINE_FOLLOW_HIGH_SPEED*MED_ADJ_FACTOR);
        } else setDriveSpeed(LINE_FOLLOW_HIGH_SPEED); // straight
    } else if(detectsLine(&Robot.leftLineDetector)) { // only the left sensor sees a line
        // turn left
        lineFollowCorrectRightBias(LINE_FOLLOW_HIGH_SPEED, -LINE_FOLLOW_HIGH_SPEED*HIGH_ADJ_FACTOR);
    } else if(detectsLine(&Robot.rightLineDetector)) { // only the right sensor sees a line
        // turn right
        lineFollowCorrectRightBias(LINE_FOLLOW_HIGH_SPEED, LINE_FOLLOW_HIGH_SPEED*HIGH_ADJ_FACTOR);
    }
//    else lineFollowCorrectRightBias(LINE_FOLLOW_MED_SPEED, LINE_FOLLOW_MED_SPEED*LOW_ADJ_FACTOR); // search for line
}

void lineFollowPID() {
    if(detectsLine(&Robot.centerLineDetector)) {
        if(detectsLine(&Robot.leftLineDetector)) { // err = -.875/2
            Robot.lineFollowingPID.pointValue = -STD_ERROR/2.0;
        } else if(detectsLine(&Robot.rightLineDetector)) { // err = .875/2
            Robot.lineFollowingPID.pointValue = STD_ERROR/2.0;
        } Robot.lineFollowingPID.pointValue = 0;
    } else if(detectsLine(&Robot.leftLineDetector)) { // err = -.875
        Robot.lineFollowingPID.pointValue = -STD_ERROR;
    } else if(detectsLine(&Robot.rightLineDetector)) { // err = .875
        Robot.lineFollowingPID.pointValue = STD_ERROR;
    }
    calculatePID(Robot.lineFollowingPID.pointValue, &Robot.lineFollowingPID);
    lineFollowCorrectRightBias(LINE_FOLLOW_HIGH_SPEED, Robot.lineFollowingPID.value);
}

void canyonNavigate() {
    // TODO: finish testing and fixing this algorithm, it's on the way to working
    updateRange(&Robot.frontRange);
    updateRange(&Robot.leftRange);
    if(seesWall(IR_LIMIT, &Robot.frontRange)) { // if we see a wall in front of us
        if(seesWall(IR_LIMIT, &Robot.leftRange)) { // there is a wall to the left
            // turn right
            setDriveSpeed(0);
//            __delay_ms(5); // pause before starting turn -- attempt to not skip any stepper signals
            turn(90, CANYON_MED_SPEED);
        } else {
            // turn left
            setDriveSpeed(0);
//            __delay_ms(5);
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

void grabBall() {

}

void depositBall() {

}

void parkAndTransmit() {

}