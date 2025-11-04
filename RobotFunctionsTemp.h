/* 
 * File:   RobotFunctionsTemp.h
 * Author: jothm
 *
 * Created on November 3, 2025, 11:16 PM
 */

#ifndef ROBOTFUNCTIONSTEMP_H
#define	ROBOTFUNCTIONSTEMP_H

typedef enum {LINE_FOLLOW, CANYON_NAVIGATE, GRAB_BALL, DEPOSIT_BALL, PARK_AND_TRANSMIT} State;

void updateState(State *state);
void straight();
void correct_left();
void correct_right();
void lineFollow();
void canyonNavigate();
void grabBall();
void depositBall();
void parkAndTransmit();

#endif	/* ROBOTFUNCTIONSTEMP_H */

