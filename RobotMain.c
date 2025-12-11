/* 
 * File:   RobotMain.c
 * Author: jothm
 *
 * Created on October 4, 2025, 12:07 PM
 */

#include <xc.h>
#include "RobotFunctions.h"

int main(int argc, char** argv) {
    // SETUP
    setupRobot();
//    setupPID(11.4, 1.0, 0.03, 5.0, &Robot.lineFollowingPID); //0.035// PID controller
//    setupPID(11.0, 1.0, 0.15, 5.0, &Robot.lineFollowingPID);
    setupPID(10.5, 1.0, 0.0, 5.0, &Robot.lineFollowingPID); // PI controller
    
    // MAIN
    while(1) {
        updateState();
        switch(Robot.state) {
            case LINE_FOLLOW:
                lineFollowPID();
//                lineFollow();
//                while(1) testDrive();
                break;
            case CANYON_NAVIGATE:
                canyonNavigate();
                break;
            case EXIT_CANYON:
                exitCanyon();
                break;
            case GRAB_BALL:
                grabBall();
                break;
            case DEPOSIT_BALL:
                depositBall();
                break;
            case PARK_AND_TRANSMIT:
                parkAndTransmit();
                break;
            case START:
                start();
                break;
            case STOP:
                stop();
                while(1);
                break;
            default:
                break;
        }
    }
    
    return 0;
}
