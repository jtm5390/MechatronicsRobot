/* 
 * File:   Photodiode.h
 * Author: jothm
 *
 * Created on November 14, 2025, 2:39 PM
 */

#ifndef PHOTODIODE_H
#define	PHOTODIODE_H

#include <stdint.h>

typedef struct {
    volatile uint16_t *readBit;
} Photodiode;

void setupPhotodiode(volatile uint16_t *readBit, Photodiode *sensor);
unsigned int getIR(Photodiode *sensor);
unsigned int seesIR(unsigned int detectionRangeValue, Photodiode *sensor);

#endif	/* PHOTODIODE_H */

