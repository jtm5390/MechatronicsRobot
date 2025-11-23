#include <stdint.h>

typedef struct {
    volatile uint16_t *readBit;
} Photodiode;

void setupPhotodiode(volatile uint16_t *readBit, Photodiode *pd) {
    pd->readBit = readBit;
}

unsigned int getIR(Photodiode *sensor) {
    return *(sensor->readBit);
}

unsigned int seesIR(unsigned int detectionRangeValue, Photodiode *sensor) {
    return (getIR(sensor) > detectionRangeValue);
}