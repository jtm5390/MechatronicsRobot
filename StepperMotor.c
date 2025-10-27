//#include <xc.h>
//#include <math.h>
//#include <stdint.h>
//
//typedef enum {FORWARD, REVERSE} direction;
//
//typedef struct {
//    direction motorDirection;
//    unsigned int *dirPin;
//    unsigned int *pwmPin;
//    unsigned int *dutyCyclePin;
//    int countsPerRev;
//    float rps;
//    float wheelDiameter;
//} StepperMotor;
//
//void setPWM(float frequencyHz, float dutyCycle, StepperMotor *motor) {
//    float frequencyCy = 8e6;
//    int preScaler = 1;
//    switch(_RCDIV) {
//        case 0b111:
//            preScaler = 256;
//            break;
//        case 0b110:
//            preScaler = 64;
//            break;
//        case 0b101:
//            preScaler = 32;
//            break;
//        case 0b100:
//            preScaler = 16;
//            break;
//        case 0b011:
//            preScaler = 8;
//            break;
//        case 0b010:
//            preScaler = 4;
//            break;
//        case 0b001:
//            preScaler = 2;
//            break;
//    }
//    switch(_FOSCSEL){
//        case FNOSC_LPFRC:
//            frequencyCy = 500e3;
//            break;
//        case FNOSC_LPRC:
//            frequencyCy = 31e3;
//            break;
//    }
//    frequencyCy /= preScaler;
//    *(motor->pwmPin) = round((frequencyCy/frequencyHz)-1);
//    *(motor->dutyCyclePin) = round(dutyCycle*(*(motor->pwmPin)));
//}
//
//void setPWMTest(float frequencyHz, float dutyCycle, StepperMotor *motor, unsigned int *rcdiv, unsigned int *fnosc) {
//    float frequencyCy = 8e6;
//    int preScaler = 1;
//    switch(*rcdiv) {
//        case 0b111:
//            preScaler = 256;
//            break;
//        case 0b110:
//            preScaler = 64;
//            break;
//        case 0b101:
//            preScaler = 32;
//            break;
//        case 0b100:
//            preScaler = 16;
//            break;
//        case 0b011:
//            preScaler = 8;
//            break;
//        case 0b010:
//            preScaler = 4;
//            break;
//        case 0b001:
//            preScaler = 2;
//            break;
//    }
//    switch(*fnosc){
//        case FNOSC_LPFRC:
//            frequencyCy = 500e3;
//            break;
//        case FNOSC_LPRC:
//            frequencyCy = 31e3;
//            break;
//    }
//    frequencyCy /= (float)(preScaler);
//    *(motor->pwmPin) = round((frequencyCy/frequencyHz)-1);
//    *(motor->dutyCyclePin) = round(dutyCycle*(*(motor->pwmPin)));
//}
//
//void setRPS(float rps, StepperMotor *motor) {
//    setPWM(rps*motor->countsPerRev, 0.5, motor);
//}
//
//void setDirection(direction dir, StepperMotor *motor) {
//    motor->motorDirection = dir;
//}
