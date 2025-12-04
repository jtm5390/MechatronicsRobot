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
//    setupPID(11.5, 1.0, 0.05, 5.0, &Robot.lineFollowingPID); // PID controller
    setupPID(11.5, 1.0, 0.0, 5.0, &Robot.lineFollowingPID); // PI controller
    
    // MAIN
    while(1) {
        updateState();
        switch(Robot.state) {
            case LINE_FOLLOW:
                lineFollowPID();
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
            case STOP:
                stop();
                break;
            default:
                break;
        }
    }
    
    return 0;
}
