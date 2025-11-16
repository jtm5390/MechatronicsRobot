/* 
 * File:   Photodiode.h
 * Author: jothm
 *
 * Created on November 14, 2025, 2:39 PM
 */

#ifndef PHOTODIODE_H
#define	PHOTODIODE_H

typedef struct {
    volatile unsigned int *readBit;
} Photodiode;

void setupPhotodiode(volatile unsigned int *readBit, Photodiode *sensor);
unsigned int getIR(Photodiode *sensor);
unsigned int seesIR(unsigned int detectionRangeValue, Photodiode *sensor);

#endif	/* PHOTODIODE_H */

