/* 
 * File:   IRRangeSensor.h
 * Author: jothm
 *
 * Created on November 12, 2025, 11:04 AM
 */

#ifndef IRRANGESENSOR_H
#define	IRRANGESENSOR_H

typedef struct { 
    volatile unsigned int *readBit;
} IRRangeSensor;

unsigned int getRangeValue(IRRangeSensor *sensor);

#endif	/* IRRANGESENSOR_H */

