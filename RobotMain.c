///* 
// * File:   RobotMain.c
// * Author: jothm
// *
// * Created on October 4, 2025, 12:07 PM
// */
//
//#include <stdio.h>
//#include <stdlib.h>
//#include <xc.h>
//#include "RobotFunctions.h"
//
//State opState = LINE_FOLLOW;
//
///*
// * 
// */
//int main(int argc, char** argv) {
//    
//    while(1) {
//        updateState(&opState);
//        switch(opState) {
//            case LINE_FOLLOW:
//                lineFollow();
//                break;
//            case CANYON_NAVIGATE:
//                canyonNavigate();
//                break;
//            case GRAB_BALL:
//                grabBall();
//                break;
//            case DEPOSIT_BALL:
//                depositBall();
//                break;
//            case PARK_AND_TRANSMIT:
//                parkAndTransmit();
//                break;
//            default:
//                break;
//        }
//    }
//    
//    return 0;
//}
