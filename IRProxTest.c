/*
 * File:   IRProxTest.c
 * Author: jothm
 *
 * Created on November 3, 2025, 3:34 PM
 */


#include "xc.h"

#pragma config FNOSC = LPFRC

#define IR_REG PORTA
#define IR_BIT 0

typedef struct {
    volatile unsigned int *readReg;
    unsigned int bit;
} IRProximitySensor;
IRProximitySensor centerLineDetector;

void setup();
int detectsLine();

int main(void) {
    // SETUP
    setup();
    
    while(1) {
        if(detectsLine()) {
            _LATA1 = 1;
        } else _LATA1 = 0;
//        _LATA1 = 1;
    }
    return 0;
}

void setup() {  
    _RCDIV = 0;
    _ANSA0 = 0;
    _ANSA1 = 0;
    _TRISA0 = 1;
    _TRISA1 = 0; // LED output
    
    centerLineDetector.readReg = &IR_REG;
    centerLineDetector.bit = IR_BIT;
}

int detectsLine() {
    return !(*centerLineDetector.readReg & (1 << centerLineDetector.bit));
}