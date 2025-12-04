/* 
 * File:   Laser.h
 * Author: jothm
 *
 * Created on December 1, 2025, 5:38 PM
 */

#ifndef LASER_H
#define	LASER_H

#include <stdint.h>

typedef struct {
    volatile uint16_t *reg;
    unsigned int bit;
} Laser;

void setupLaser(volatile uint16_t *reg, unsigned int bit, Laser *laser);
void setLaser(unsigned int onOff, Laser *laser);

#endif	/* LASER_H */

