typedef struct {
    volatile unsigned int *readBit;
} Photodiode;

void setupPhotodiode(volatile unsigned int *readBit, Photodiode *pd) {
    pd->readBit = readBit;
}

unsigned int getIR(Photodiode *sensor) {
    return *(sensor->readBit);
}

unsigned int seesIR(unsigned int detectionRangeValue, Photodiode *sensor) {
    return (getIR(sensor) > detectionRangeValue);
}