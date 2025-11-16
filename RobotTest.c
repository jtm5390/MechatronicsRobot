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
//    setupPID(11.5, 1.0, 0.0, 5.0, &Robot.lineFollowingPID); // PI controller
////    setupPID(11.5, 1.0, 0.05, 5.0, &Robot.lineFollowingPID); // PID controller
////    setupPID(17.0, 0.0, 3.5, 4.0, &Robot.lineFollowingPID); // P = 8inPerSec / in, I = 0.5, D = 0.05, I_CAP = 5.0
//    
////    setDriveSpeed(12);
////    __delay_ms(2000);
////    setDriveSpeed(0);
////    __delay_ms(2000);
////    turn(90, 6);
////    driveDistance(8, 10);
//    
////    setDriveSpeed(10); // ramp up speed
////    __delay_ms(25);
//    Robot.lineFollowingPID.setpoint = 0;
//    while(1) {
//        lineFollowPID();
////        lineFollow();
////        canyonNavigate();
//    }
////    while(1) setDriveSpeed(0);
//    return 0;
//}