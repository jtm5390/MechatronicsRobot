#include "Robot.h"

#define LINE_FOLLOW_HIGH_SPEED 20.0//25.0
#define LINE_FOLLOW_MED_SPEED 15.0
#define LINE_FOLLOW_LOW_SPEED 6.0
#define HIGH_ADJ_FACTOR 0.5
#define MED_ADJ_FACTOR 0.3
#define LOW_ADJ_FACTOR 0.3

#define CANYON_HIGH_SPEED 20.0
#define CANYON_MED_SPEED 15.0
#define CANYON_LOW_SPEED 6.0
#define STD_ERROR 0.9 // distance between line sensors

#define QRD_WHITE_LIMIT 2481 // 2V


void stop() {
    brake();
}

void start() {
    __delay_ms(100);
//    setAngle(120, &Robot.servo); // having issues with servo starting properly
    driveDistance(30, 20);
    while(!seesLine(&Robot.centerLineDetector)) setTurnSpeed(-10);
    stop();
    __delay_ms(50);
    Robot.deployed = 1;
}

void lineFollowCorrectRightBias(float speedInPerSec, float speedAdj) {
    if(fabs(speedAdj) <= 0.2) speedAdj = 0.0; // fix for 0 errors
    setSpeed(speedInPerSec + speedAdj, &(Robot.leftMotor));
    setSpeed(speedInPerSec - speedAdj, &(Robot.rightMotor));
}

void lineFollow() {
    // disable PID
    if(T2CONbits.TON == 1) {
        T2CONbits.TON = 0;
        _T2IE = 0;
    }
    // line following
    if(seesLine(&Robot.centerLineDetector)) { // center sensor sees a line
        if(seesLine(&Robot.leftLineDetector)) { // left sensor also sees a line
            // turn left
            lineFollowCorrectRightBias(LINE_FOLLOW_HIGH_SPEED, -LINE_FOLLOW_HIGH_SPEED*MED_ADJ_FACTOR);
        } else if(seesLine(&Robot.rightLineDetector)) { // right sensor also sees a line
            // turn right
            lineFollowCorrectRightBias(LINE_FOLLOW_HIGH_SPEED, LINE_FOLLOW_HIGH_SPEED*MED_ADJ_FACTOR);
        } else setDriveSpeed(LINE_FOLLOW_HIGH_SPEED); // straight
    } else if(seesLine(&Robot.leftLineDetector)) { // only the left sensor sees a line
        // turn left
        lineFollowCorrectRightBias(LINE_FOLLOW_HIGH_SPEED, -LINE_FOLLOW_HIGH_SPEED*HIGH_ADJ_FACTOR);
    } else if(seesLine(&Robot.rightLineDetector)) { // only the right sensor sees a line
        // turn right
        lineFollowCorrectRightBias(LINE_FOLLOW_HIGH_SPEED, LINE_FOLLOW_HIGH_SPEED*HIGH_ADJ_FACTOR);
    }
}

void lineFollowPID() {
    // enable PID
    if(T2CONbits.TON == 0) {
        resetPID(&Robot.lineFollowingPID);
        _T2IE = 1;
        T2CONbits.TON = 1;
    }
    if(Robot.lineFollowingPID.setpoint != 0) Robot.lineFollowingPID.setpoint = 0;
    
    if(seesLine(&Robot.centerLineDetector)) {
        if(seesLine(&Robot.leftLineDetector)) { // err = .875/2
            Robot.lineFollowingPID.pointValue = STD_ERROR/2.0;
        } else if(seesLine(&Robot.rightLineDetector)) { // err = -.875/2
            Robot.lineFollowingPID.pointValue = -STD_ERROR/2.0 /*- .2*/;
        } else Robot.lineFollowingPID.pointValue = 0;        
    } else if(seesLine(&Robot.leftLineDetector)) { // err = .875
        Robot.lineFollowingPID.pointValue = STD_ERROR;
    } else if(seesLine(&Robot.rightLineDetector)) { // err = -.875
        Robot.lineFollowingPID.pointValue = -STD_ERROR /*- .35*/;
    } /*else {
        Robot.lineFollowingPID.pointValue = 0;
    }*/
    calculatePID(&Robot.lineFollowingPID);
    if(seesLine(&Robot.centerLineDetector) && !seesLine(&Robot.leftLineDetector) && !seesLine(&Robot.rightLineDetector)) Robot.lineFollowingPID.value = 0;
    lineFollowCorrectRightBias(LINE_FOLLOW_HIGH_SPEED, Robot.lineFollowingPID.value);
}

void canyonNavigate() {
    /*if(!Robot.traversedCanyon) { // sanity check*/
        // TODO: finish testing and fixing this algorithm, it's on the way to working
        // idea: turn the correct way until the sensor sees we can go forward, then go forward until we need to adjust again
        updateRange(&Robot.frontRange);
//        updateRange(&Robot.frontRange);
        updateRange(&Robot.leftRange);
//        updateRange(&Robot.leftRange);
        if(seesWall(NEW_FRONT_IR_LIMIT, &Robot.frontRange)) {
            if(seesWall(NEW_LEFT_IR_LIMIT, &Robot.leftRange)) {
                // turn right
                while(seesWall(NEW_FRONT_IR_LIMIT, &Robot.frontRange)){
                    updateRange(&Robot.frontRange);
                    setTurnSpeed(CANYON_MED_SPEED);
                } stop();
            } else {
                // turn left
                turn(-85, CANYON_MED_SPEED);
//                while(seesWall(NEW_FRONT_IR_LIMIT, &Robot.frontRange)){
//                    updateRange(&Robot.frontRange);
//                    setTurnSpeed(-CANYON_MED_SPEED-2.0);
//                } stop();
            }
        } else {
            // go straight
            setDriveSpeed(CANYON_MED_SPEED);
        }
//        if(seesWall(FRONT_IR_LIMIT, &Robot.frontRange)) { // if we see a wall in front of us
//            if(seesWall(LEFT_IR_LIMIT, &Robot.leftRange)) { // there is a wall to the left
//                // turn right
//                setDriveSpeed(0);
//                __delay_ms(5); // pause before starting turn -- attempt to not skip any stepper signals
//                turn(90, CANYON_MED_SPEED);
//            } else {
//                // turn left
//                setDriveSpeed(0);
//                __delay_ms(5);
//                turn(-90, CANYON_MED_SPEED);
//            }
//            // update at least one set of ranges after we turn to discard noise
//            __delay_ms(10); // wait a bit before taking new readings - also to eliminate noise
//            updateRange(&Robot.frontRange);
//            updateRange(&Robot.leftRange);
//        } else {
//            // go straight
//            setDriveSpeed(CANYON_MED_SPEED);
//        }
    /*}*/
}

void exitCanyon() {
    //if(!Robot.traversedCanyon) { // sanity check
        stop();
        // flood the left range sensor with new values
        updateRange(&Robot.leftRange);
        updateRange(&Robot.leftRange);
        updateRange(&Robot.leftRange);
        driveDistance(8, 10);
        float turnSpeed = -6;
        if(seesWall(NEW_LEFT_IR_LIMIT, &Robot.leftRange)) turnSpeed = 6;
        while(!seesLine(&Robot.centerLineDetector)) setTurnSpeed(turnSpeed);
        stop();
        __delay_ms(100); // pause before switching states
        Robot.traversedCanyon = 1;
    //}
}

void grabBall() {
    stop();
    __delay_ms(25);
    driveDistance(7.5, 8.5);
    turnOneWheel(-90, -15);
    driveDistance(10, -10);
    __delay_ms(1500);
    driveDistance(15, 10);
    while(!seesLine(&Robot.centerLineDetector)) setTurnSpeed(10);
    stop();
    Robot.grabbedBall = 1;
}

void depositBall() {
    stop(); // stop after seeing the buckets
    __delay_ms(25); // brief pause
    driveDistance(10, 8.5); // reposition
    __delay_ms(25); // brief pause
    float turnAngle = (seesWhiteBall(QRD_WHITE_LIMIT, &Robot.qrd))? 65:-65; // determine which bucket to dump in
    turn(turnAngle, 15); // turn towards the bucket
    driveDistance(6, -8); // go to the bucket
    __delay_ms(100); // brief pause before dropping the ball
    setSolenoid(0, &Robot.solenoid); // deposit ball
    __delay_ms(1000);
    driveDistance(5, 10); // leave bucket
    while(!seesLine(&Robot.centerLineDetector)) setTurnSpeed((-turnAngle/fabs(turnAngle))*10.0); // find line
    stop();
    Robot.depositedBall = 1;
}

void findSatellite() {
    for(int i = 135; i > 25; i -= 3) {
        if(seesIR(SATELLITE_IR_LIMIT, &Robot.satelliteDetector)) break;
        setAngle(i, &Robot.servo);
        __delay_ms(50);
    }
    __delay_ms(250);
//    setAngle(50, &Robot.servo);
//    setLaser(1, &Robot.laser);
}

void parkAndTransmit() {
    setSolenoid(0, &Robot.solenoid);
    setAngle(110, &Robot.servo);
    stop();
    driveDistance(2, 10);
    __delay_ms(10);
    turn(87, 15);
    __delay_ms(10);
    driveDistance(36, -10);
    __delay_ms(10);
    brake();
    __delay_ms(1000);
    while(!seesIR(SATELLITE_IR_LIMIT, &Robot.satelliteDetector)) {
        Robot.servo.angle = Robot.servo.angle - 2.0;
        setServoPWM(&Robot.servo);
        __delay_ms(25);
    }
    Robot.servo.angle = Robot.servo.angle - 5.0;
    setServoPWM(&Robot.servo);
    __delay_ms(250);
    setLaser(1, &Robot.laser);
    Robot.transmitting = 1;
}

void testDrive() {
//    Robot.lineFollowingPID.value = 0;
//    lineFollowCorrectRightBias(LINE_FOLLOW_HIGH_SPEED, Robot.lineFollowingPID.value);
    setDriveSpeed(25);
    __delay_ms(5000);
    setSpeed(-10, &Robot.leftMotor);
    setSpeed(-10, &Robot.rightMotor);
    __delay_ms(5000);
    brake();
}
