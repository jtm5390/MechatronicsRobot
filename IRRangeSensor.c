#define BUFFER_SIZE 3

typedef struct { 
    volatile unsigned int *readBit;
    volatile unsigned int rangeBuffer[BUFFER_SIZE]; // store a buffer of the last 20 range values
    volatile unsigned int rangeAverage;
    unsigned int lastUpdated;
} IRRangeSensor;

void updateRange(IRRangeSensor *sensor) {
    sensor->rangeBuffer[sensor->lastUpdated++] = *(sensor->readBit);
    if(sensor->lastUpdated >= BUFFER_SIZE) sensor->lastUpdated = 0;
}
void updateAverage(IRRangeSensor *sensor) {
    float tempSum = 0;
    int i = 0;
    for(i = 0; i < BUFFER_SIZE; i++) {
        tempSum += sensor->rangeBuffer[i];
    }
    sensor->rangeAverage = (int)(tempSum/(float)(i));
}
unsigned int getRangeValue(IRRangeSensor *sensor) {
    updateAverage(sensor); // only calculate our average when we need it
    return sensor->rangeAverage;
}
unsigned int seesWall(unsigned int withinRangeValue, IRRangeSensor *sensor) {
    return (getRangeValue(sensor) > withinRangeValue);
}

void setupIRRangeSensor(volatile unsigned int *readBit, IRRangeSensor *sensor) {
    sensor->readBit = readBit;
    sensor->rangeAverage = 0;
    sensor->lastUpdated = 0;
    for(int i = 0; i < BUFFER_SIZE; i++) {
        sensor->rangeBuffer[i] = 0;
    }
}