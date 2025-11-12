//#include <xc.h>
//#include <stdint.h>
//
//volatile uint32_t millis = 0;
//
//void __attribute__((__interrupt__, auto_psv)) _T1Interrupt(void) {
//    millis++;               // increment every 1 ms
//    _T1IF = 0;      // clear Timer1 interrupt flag
//}
//
//uint32_t currentTimeMillis() { return millis; }
//void resetTimeMillis() { millis = 0; }
//
//typedef struct {
//    float value, kP, kI, kD, setpoint, error, tempI, prevError, timeAtLastCalculation, P, I, D, I_CAP;
//} PIDController;
//
//void calculatePID(float pointValue, PIDController *controller) {
//    float deltaTime = (currentTimeMillis() - controller->timeAtLastCalculation)/1000.0; // figure out time calculation here
//    controller->error = controller->setpoint - pointValue;
//    controller->P = controller->kP * controller->error;
//    controller->tempI = controller->kI * controller->error * deltaTime;
//    controller->I += controller->tempI;
//    if(controller->kI == 0) controller->I = 0;
//    if(controller->I > 0 && controller->I > controller->I_CAP) controller->I = controller->I_CAP;
//    else if(controller->I < 0 && controller->I < -controller->I_CAP) controller->I = -controller->I_CAP;
//    controller->D = controller->kD * (controller->prevError - controller->error) / deltaTime;
//    controller->prevError = controller->error;
//    controller->timeAtLastCalculation = currentTimeMillis(); // figure out current time
//    controller->value = controller->P + controller->I + controller->D;
//}
//
//void resetPID(PIDController *controller) {
//    controller->P = 0;
//    controller->I = 0;
//    controller->D = 0;
//    controller->timeAtLastCalculation = 0;
//}
//
//void initPIDTimer() {
//    _TON = 0;      // stop timer
//    _TCS = 0;      // internal clock (Fcy)
//    _TCKPS = 0b01; // prescaler 1:8
//    PR1 = 31;               // for 1 ms interrupt @ 250 kHz Fcy
//    TMR1 = 0;               // clear timer
//    _T1IF = 0;      // clear interrupt flag
//    _T1IE = 1;      // enable Timer1 interrupt
//    _TON = 1;      // start timer
//}
//
//void setupPID(float kp, float ki, float kd, float iCap, PIDController *controller) {
//    resetPID(controller);
//    controller->kP = kp;
//    controller->kI = ki;
//    controller->kD = kd;
//    controller->I_CAP = iCap;
//    initPIDTimer();
//}