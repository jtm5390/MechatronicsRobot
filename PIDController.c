//typedef struct {
//    float value, kP, kI, kd, setpoint, error, tempI, prevError, timeAtLastCalculation, P, I, D, I_CAP;
//} PIDController;
//
//void calculatePID(float pointValue, PIDController *controller) {
//    float deltaTime = time - controller->timeAtLastCalculation; // figure out time calculation here
//    controller->error = controller->setpoint - pointValue;
//    controller->P = controller->kP * controller->error;
//    controller->tempI = controller->kI * controller->error * deltaTime;
//    controller->I += controller->tempI;
//    if(controller->kI == 0) controller->I = 0;
//    if(controller->I > 0 && controller->I > controller->I_CAP) controller->I = controller->I_CAP;
//    else if(controller->I < 0 && controller->I < -controller->I_CAP) controller->I = -controller->I_CAP;
//    controller->D = controller->kd * (controller->prevError - controller->error) / deltaTime;
//    controller->prevError = controller->error;
//    controller->timeAtLastCalculation = time; // figure out current time
//    controller->value = controller->P + controller->I + controller->D;
//}
//
//void resetPID(PIDController *controller) {
//    controller->P = 0;
//    controller->I = 0;
//    controller->D = 0;
//    controller->timeAtLastCalculation = 0;
//}