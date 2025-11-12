

typedef struct { 
    volatile unsigned int *readBit;
} IRRangeSensor;

unsigned int getRangeValue(IRRangeSensor *sensor) {return *(sensor->readBit);}