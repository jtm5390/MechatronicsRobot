#include <stdint.h>

typedef struct {
    volatile uint16_t *readPin;
} QRDSensor;

unsigned int getQRDValue(QRDSensor *sensor) {
    return *(sensor->readPin);
}

unsigned int seesWhiteBall(unsigned int qrdWhiteLimit, QRDSensor *sensor) {
    return (getQRDValue(sensor) < qrdWhiteLimit);
}

void setupQRD(volatile uint16_t *readPin, QRDSensor *sensor) {
    sensor->readPin = readPin;
}