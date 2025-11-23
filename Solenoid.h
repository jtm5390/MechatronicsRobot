/* 
 * File:   Solenoid.h
 * Author: jothm
 *
 * Created on November 22, 2025, 9:04 AM
 */

#ifndef SOLENOID_H
#define	SOLENOID_H

#include <stdint.h>

typedef struct {
    volatile uint16_t *reg;
    unsigned int bit;
} Solenoid;

void setSolenoid(int onOff, Solenoid *solenoid);
void setupSolenoid(volatile uint16_t *reg, unsigned int bit, Solenoid *solenoid);

#endif	/* SOLENOID_H */

