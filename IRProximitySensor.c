#include <xc.h>

typedef struct {
    volatile unsigned int *readReg;
    unsigned int bit;
} IRProximitySensor;

void setupIRProximitySensor(volatile unsigned int *readReg, unsigned int bit, IRProximitySensor *sensor) {
    sensor->readReg = readReg;
    sensor->bit = bit;
}

int detectsLine(IRProximitySensor *sensor) {
    return !(*sensor->readReg & (1 << sensor->bit));
}