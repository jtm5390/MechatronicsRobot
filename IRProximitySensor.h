/* 
 * File:   IRProximitySensor.h
 * Author: jothm
 *
 * Created on November 3, 2025, 10:57 PM
 */

#ifndef IRPROXIMITYSENSOR_H
#define	IRPROXIMITYSENSOR_H

typedef struct {
    volatile unsigned int *readReg;
    unsigned int bit;
} IRProximitySensor;

void setupIRProximitySensor(volatile unsigned int *readReg, unsigned int bit, IRProximitySensor *sensor);
int detectsLine(IRProximitySensor *sensor);

#endif	/* IRPROXIMITYSENSOR_H */

