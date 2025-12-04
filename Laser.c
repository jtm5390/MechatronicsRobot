#include <stdint.h>

typedef struct {
    volatile uint16_t *reg;
    unsigned int bit;
} Laser;

void setupLaser(volatile uint16_t *reg, unsigned int bit, Laser *laser) {
    laser->reg = reg;
    laser->bit = bit;
}

void setLaser(unsigned int onOff, Laser *laser) {
    if(onOff) *(laser->reg) |= (1 << laser->bit);
    else *(laser->reg) &= ~(1 << laser->bit);
}