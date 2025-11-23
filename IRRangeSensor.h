/* 
 * File:   IRRangeSensor.h
 * Author: jothm
 *
 * Created on November 12, 2025, 11:04 AM
 */

#ifndef IRRANGESENSOR_H
#define	IRRANGESENSOR_H

#include <stdint.h>

#define BUFFER_SIZE 3

typedef struct { 
    volatile uint16_t *readBit;
    volatile unsigned int rangeBuffer[BUFFER_SIZE]; // store a buffer of the last 20 range values
    volatile unsigned int rangeAverage;
    unsigned int lastUpdated;
} IRRangeSensor;

void updateRange(IRRangeSensor *sensor);
unsigned int getRangeValue(IRRangeSensor *sensor);
unsigned int seesWall(unsigned int withinRangeValue, IRRangeSensor *sensor);
void setupIRRangeSensor(volatile uint16_t *readBit, IRRangeSensor *sensor);

#endif	/* IRRANGESENSOR_H */

