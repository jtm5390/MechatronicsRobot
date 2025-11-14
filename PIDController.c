#include <xc.h>
#include <stdint.h>

volatile uint32_t millis = 0;

void __attribute__((__interrupt__, auto_psv)) _T2Interrupt(void) {
    millis++;               // increment every 1 ms
    _T2IF = 0;      // clear Timer2 interrupt flag
}

uint32_t currentTimeMillis() { return millis; }
void resetTimeMillis() { millis = 0; }

typedef struct {
    float value, kP, kI, kD, setpoint, error, tempI, prevError, timeAtLastCalculation, P, I, D, I_CAP, pointValue;
} PIDController;

void calculatePID(float pointValue, PIDController *controller) {
    float deltaTime = (currentTimeMillis() - controller->timeAtLastCalculation)/1000.0; // figure out time calculation here
    if(deltaTime <= 0.0f) deltaTime = 0.001f;
    controller->error = controller->setpoint - pointValue;
    controller->P = controller->kP * controller->error;
    controller->tempI = controller->kI * controller->error * deltaTime;
    controller->I += controller->tempI;
    if(controller->kI == 0) controller->I = 0;
    if(controller->I > 0 && controller->I > controller->I_CAP) controller->I = controller->I_CAP;
    else if(controller->I < 0 && controller->I < -controller->I_CAP) controller->I = -controller->I_CAP;
    controller->D = controller->kD * (controller->prevError - controller->error) / deltaTime;
    controller->prevError = controller->error;
    controller->timeAtLastCalculation = currentTimeMillis(); // figure out current time
    controller->value = controller->P + controller->I + controller->D;
}

void resetPID(PIDController *controller) {
    controller->P = 0;
    controller->I = 0;
    controller->D = 0;
    controller->timeAtLastCalculation = 0;
    controller->pointValue = controller->setpoint;
}

void initPIDTimer() {
    T2CONbits.T32 = 0; // set as 16-bit timers
    T2CONbits.TON = 0;      // stop timer
    T2CONbits.TCS = 0;      // internal clock (Fcy)
    T2CONbits.TCKPS = 0b01; // prescaler 1:8
    PR2 = 31;               // for 1 ms interrupt @ 250 kHz Fcy
    TMR2 = 0;               // clear timer
    _T2IF = 0;      // clear interrupt flag
    _T2IE = 1;      // enable Timer2 interrupt
    T2CONbits.TON = 1;      // start timer
}

void setupPID(float kp, float ki, float kd, float iCap, PIDController *controller) {
    resetPID(controller);
    controller->kP = kp;
    controller->kI = ki;
    controller->kD = kd;
    controller->I_CAP = iCap;
    controller->pointValue = 0;
    initPIDTimer();
}