/*
 * File:   RobotTest.c
 * Author: jothm
 *
 * Created on November 2, 2025, 11:53 AM
 */


#include "xc.h"
#include "Robot.h"
#define FCY 250000UL
#include "libpic30.h"

int main(void) {
    // SETUP
    setupRobot();
    
    setDriveSpeed(12);
    __delay_ms(2000);
    setDriveSpeed(0);
    __delay_ms(2000);
    turn(90, 6);
    
    while(1) setDriveSpeed(0);
    return 0;
}