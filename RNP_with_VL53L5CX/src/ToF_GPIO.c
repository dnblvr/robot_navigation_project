

#include "../inc/ToF_GPIO.h"

/**
 * powers on the sensor by configuring LPn and I2C_RST
 * To initialize:
 *      LPn     -> P4.6
 *      I2C_RST -> P4.5
 */
void config_VL53L5CX_pins(void) {
//    P4.6 and P4.5 as digital outputs

    P4->SEL0 &= ~0x60;
    P4->SEL1 &= ~0x60;

    P4->DIR  |=  0x60;


    // clear OUT ports
    P4->OUT  &= ~0x60;

}


void VL53L5CX_wake(void) {
//    P4.6 and P4.5 as digital outputs

    // set LPn so it's not in low-power
    P4->OUT |=  0x60;

}


void VL53L5CX_sleep(void) {
//    P4.6 and P4.5 as digital outputs

    // clear LPn so it's in low-power
    P4->OUT &= ~0x40;

//    // Configure the pins by setting P1.(7-6) high so I2C is busy
//    P1->OUT  |=  0xC0;

}


void VL53L5CX_reset(void) {

    // set I2C_RST so it resets
    P4->OUT |=  0x20;
}


void VL53L5CX_no_reset(void) {

    // clear I2C_RST so it does not reset
    P4->OUT &= ~0x20;
}
