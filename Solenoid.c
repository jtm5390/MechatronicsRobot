#include <stdint.h>

typedef struct {
    volatile uint16_t *reg;
    unsigned int bit;
} Solenoid;

void setSolenoid(int onOff, Solenoid *solenoid) {
    if(onOff) *(solenoid->reg) |= (1 << solenoid->bit);
    else *(solenoid->reg) &= ~(1 << solenoid->bit);
}

void setupSolenoid(volatile uint16_t *reg, unsigned int bit, Solenoid *solenoid) {
    solenoid->reg = reg;
    solenoid->bit = bit;
}