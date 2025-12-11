///*
// * File:   RobotTest.c
// * Author: jothm
// *
// * Created on November 2, 2025, 11:53 AM
// */
//
//
//#include "RobotFunctions.h"
//
//int main(void) {
//    // SETUP
//    setupRobot();
////    setupPID(15.0, 0.5, 0.05, 5.0, &Robot.lineFollowingPID); // P = 8inPerSec / in, I = 0.5, D = 0.05, I_CAP = 5.0
////    setupPID(11.5, 1.0, 0.0, 5.0, &Robot.lineFollowingPID); // PI controller
////    setupPID(11.5, 1.0, 0.05, 5.0, &Robot.lineFollowingPID); // PID controller
////    setupPID(17.0, 0.0, 3.5, 4.0, &Robot.lineFollowingPID); // P = 8inPerSec / in, I = 0.5, D = 0.05, I_CAP = 5.0
//    setupPID(10.5, 1.0, 0.0, 5.0, &Robot.lineFollowingPID); // PI controller
//    brake();
//    __delay_ms(1000);
//    findSatellite();
//    
//    while(1) {
////        lineFollow();
////        canyonNavigate();
//        setAngle(50, &Robot.servo);
//    }
////    while(1) setDriveSpeed(0);
//    return 0;
//}