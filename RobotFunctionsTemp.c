#include "Robot.h"

typedef enum {LINE_FOLLOW, CANYON_NAVIGATE, GRAB_BALL, DEPOSIT_BALL, PARK_AND_TRANSMIT} State;

void updateState(State *state) {
    *state = LINE_FOLLOW;
    // do some checks to see what state to be in
}

void straight();
void correct_left();
void correct_right();

void lineFollow() {
    if (detectsLine(&Robot.centerLineDetector) && !detectsLine(&Robot.leftLineDetector) && !detectsLine(&Robot.rightLineDetector)){
        straight();
    }
    if (!detectsLine(&Robot.centerLineDetector) && detectsLine(&Robot.leftLineDetector) && !detectsLine(&Robot.rightLineDetector)){
        correct_left();
    }
    if (!detectsLine(&Robot.centerLineDetector) && !detectsLine(&Robot.leftLineDetector) && detectsLine(&Robot.rightLineDetector)){
        correct_right();
    }
}

void canyonNavigate() {

}

void grabBall() {

}

void depositBall() {

}

void parkAndTransmit() {

}

void straight(){
     setDriveSpeed(12); // about 1 rps forward
}

void correct_right(){
    turn(2, 12); // turn 2 degrees CW at about 1 rps
}

void correct_left(){
    turn(-2, 12); // turn 2 degrees CCW at about 1 rps
}  