/* 
 * File:   PIDController.h
 * Author: jothm
 *
 * Created on October 21, 2025, 2:16 PM
 */

#ifndef PIDCONTROLLER_H
#define	PIDCONTROLLER_H

typedef struct {
    volatile float value, setpoint, error, tempI, prevError, timeAtLastCalculation, P, I, D, pointValue;
    float kP, kI, kD, I_CAP;
} PIDController;

uint32_t currentTimeMillis();
void resetTimeMillis();
void calculatePID(PIDController *controller);
void resetPID(PIDController *controller);
void setupPID(float kp, float ki, float kd, float iCap, PIDController *controller);

#endif	/* PIDCONTROLLER_H */

