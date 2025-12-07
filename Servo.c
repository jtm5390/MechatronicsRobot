#include <stdint.h>

// duty cycle bounds -> 100 to 325

typedef struct {
    volatile uint16_t *pwmPin, *dutyCyclePin;
    float angle, minDutyCycle, maxDutyCycle, maxAngle;
} Servo;

void setupServo(volatile uint16_t *pwm, volatile uint16_t *dc, float maxAngle, float minDC, float maxDC, Servo *servo) {
    servo->pwmPin = pwm;
    servo->dutyCyclePin = dc;
    servo->angle = 0;
    servo->maxAngle = maxAngle;
    servo->minDutyCycle = minDC;
    servo->maxDutyCycle = maxDC;
}

void setServoPWM(Servo *servo) {
    *servo->pwmPin = 4999; // calculated for Fpwm = 50Hz and Fcy = 250000
    *servo->dutyCyclePin = (int)((servo->maxDutyCycle - servo->minDutyCycle)*(servo->angle / servo->maxAngle) + servo->minDutyCycle); // conversion function: y=275(x/180) + 100
}

void setAngle(float angle, Servo *servo) {
    servo->angle = angle;
    setServoPWM(servo);
}