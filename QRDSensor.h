/* 
 * File:   QRDSensor.h
 * Author: jothm
 *
 * Created on November 22, 2025, 9:02 AM
 */

#ifndef QRDSENSOR_H
#define	QRDSENSOR_H

#include <stdint.h>

typedef struct {
    volatile uint16_t *readPin;
} QRDSensor;

unsigned int getQRDValue(QRDSensor *sensor);
unsigned int seesWhiteBall(unsigned int qrdWhiteLimit, QRDSensor *sensor);
void setupQRD(volatile uint16_t *readPin, QRDSensor *sensor);

#endif	/* QRDSENSOR_H */

