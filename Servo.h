/* 
 * File:   Servo.h
 * Author: jothm
 *
 * Created on December 1, 2025, 5:32 PM
 */

#ifndef SERVO_H
#define	SERVO_H

#include <stdint.h>

typedef struct {
    volatile uint16_t *pwmPin, *dutyCyclePin;
    float angle, minDutyCycle, maxDutyCycle, maxAngle;
} Servo;

void setupServo(volatile uint16_t *pwm, volatile uint16_t *dc, float maxAngle, float minDC, float maxDC, Servo *servo);
void setServoPWM(Servo *servo);
void setAngle(float angle, Servo *servo);

#endif	/* SERVO_H */

