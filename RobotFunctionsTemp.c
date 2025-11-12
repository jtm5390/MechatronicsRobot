//#include "Robot.h"
//
//#define LINE_FOLLOW_HIGH_SPEED 30.0
//#define LINE_FOLLOW_MED_SPEED 10.0
//#define LINE_FOLLOW_LOW_SPEED 6.0
//#define HIGH_ADJ_FACTOR 0.6
//#define MED_ADJ_FACTOR 0.35
//#define LOW_ADJ_FACTOR 0.3
//
//void lineFollowCorrectRightBias(float speedInPerSec, float speedAdj) {
//    setSpeed(speedInPerSec + speedAdj, &Robot.leftMotor);
//    setSpeed(speedInPerSec - speedAdj, &Robot.rightMotor);
//}
//
//void lineFollow() {
//    // line following
//    if(detectsLine(&Robot.centerLineDetector)) { // center sensor sees a line
//        if(detectsLine(&Robot.leftLineDetector)) { // left sensor also sees a line
//            // turn left
//            lineFollowCorrectRightBias(LINE_FOLLOW_MED_SPEED, -LINE_FOLLOW_MED_SPEED*MED_ADJ_FACTOR);
//        } else if(detectsLine(&Robot.rightLineDetector)) { // right sensor also sees a line
//            // turn right
//            lineFollowCorrectRightBias(LINE_FOLLOW_MED_SPEED, LINE_FOLLOW_MED_SPEED*MED_ADJ_FACTOR);
//        } else setDriveSpeed(LINE_FOLLOW_MED_SPEED); // straight
//    } else if(detectsLine(&Robot.leftLineDetector)) { // only the left sensor sees a line
//        // turn left
//        lineFollowCorrectRightBias(LINE_FOLLOW_MED_SPEED, -LINE_FOLLOW_MED_SPEED*HIGH_ADJ_FACTOR);
//    } else if(detectsLine(&Robot.rightLineDetector)) { // only the right sensor sees a line
//        // turn right
//        lineFollowCorrectRightBias(LINE_FOLLOW_MED_SPEED, LINE_FOLLOW_MED_SPEED*HIGH_ADJ_FACTOR);
//    } else lineFollowCorrectRightBias(LINE_FOLLOW_MED_SPEED, LINE_FOLLOW_MED_SPEED*LOW_ADJ_FACTOR); // search for line
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