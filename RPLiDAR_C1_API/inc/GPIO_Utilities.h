/**
 * @file GPIO_Utilities.h
 *
 * @brief   a function purely meant to appease the Power (ULP) Advice
 *
 * @details It is not recommended to have the functions declared at the headers
 *          but it will suffice for this application.
 */

#ifndef __INC_GPIO_UTILITIES_H__
#define __INC_GPIO_UTILITIES_H__


/**
 * @brief Initialize unused GPIO ports to reduce power consumption
 *
 * This function configures all unused GPIO pins as outputs with low state to
 *  eliminate floating inputs and reduce current consumption. This satisfies
 *  the MSP432 ULP (Ultra-Low Power) Advisor requirements.
 *
 * @details Initializes ALL ports to GPIO mode with outputs low by default.
 *      It also appears that these registers are assigned directly instead of
 *      friendly-modified. This suffices because this will be the default
 *      setting until otherwise replaced by bitmasked values by their
 *      respective drivers.
 */
void Init_Unused_Ports(void) {


    // Port 1
    P1->SEL0 = 0x00;
    P1->SEL1 = 0x00;
    P1->DIR  = 0xFF;
    P1->OUT  = 0x00;

    // Port 2: P2.0/P2.1/P2.2 (RGB LED), P2.6/P2.7 (Timer PWM) are used
    P2->SEL0 = 0x00;
    P2->SEL1 = 0x00;
    P2->DIR  = 0xFF;
    P2->OUT  = 0x00;

    // Port 3: P3.2/P3.3 (UART A2), P3.6/P3.7 (Motor) are used
    P3->SEL0 = 0x00;
    P3->SEL1 = 0x00;
    P3->DIR  = 0xFF;
    P3->OUT  = 0x00;

    // Port 4: Bumper switches use some pins
    P4->SEL0 = 0x00;
    P4->SEL1 = 0x00;
    P4->DIR  = 0xFF;
    P4->OUT  = 0x00;

    // Port 5: Motor, Tachometer, Timer, Reflectance sensor use various pins
    P5->SEL0 = 0x00;
    P5->SEL1 = 0x00;
    P5->DIR  = 0xFF;
    P5->OUT  = 0x00;

    // Port 6: Completely unused
    P6->SEL0 = 0x00;
    P6->SEL1 = 0x00;
    P6->DIR  = 0xFF;
    P6->OUT  = 0x00;

    // Port 7: Reflectance sensor uses all 8 pins
    P7->SEL0 = 0x00;
    P7->SEL1 = 0x00;
    P7->DIR  = 0xFF;
    P7->OUT  = 0x00;

    // Port 8: Chassis board LEDs
    P8->SEL0 = 0x00;
    P8->SEL1 = 0x00;
    P8->DIR  = 0xFF;
    P8->OUT  = 0x00;

    // Port 9: PMOD LEDs (all pins), BLE UART (P9.6/P9.7), Reflectance (P9.2)
    P9->SEL0 = 0x00;
    P9->SEL1 = 0x00;
    P9->DIR  = 0xFF;
    P9->OUT  = 0x00;

    // Port 10: Buttons (P10.0-3), Timer capture (P10.4/P10.5)
    P10->SEL0 = 0x00;
    P10->SEL1 = 0x00;
    P10->DIR  = 0xFF;
    P10->OUT  = 0x00;

    // Port J: JTAG pins - initialize as outputs low
    PJ->SEL0 = 0x00;
    PJ->SEL1 = 0x00;
    PJ->DIR  = 0xFF;
    PJ->OUT  = 0x00;
}


/**
 * @brief set up all receive and send UART interrupts
 */
void Set_All_Interrupts_1(void) {


    EUSCI_A0->IE   |=  0x0003;  // printf()
    EUSCI_A2->IE   |=  0x0001;  // RPLiDAR C1
    EUSCI_A3->IE   |=  0x0001;  // BLE UART Friend

    // take out the register that assigns the interrupt EN of TIMER_A1
    // has no effect, it seems
//    TIMER_A1->CCTL[0]  |= 0x0010;

}

/**
 * @brief set up all receive and send UART interrupts
 * @details difference 1 & 2 is that it holds and releases the whole set of 
 *  EUSCI_Ax blocks while performing changes. In my experience, these steps are
 *  not necessary
 */
void Set_All_Interrupts_2(void) {

    EUSCI_A0->CTLW0    |=  0x01;
    EUSCI_A2->CTLW0    |=  0x01;
    EUSCI_A3->CTLW0    |=  0x01;


    EUSCI_A0->IE   |=  0x0001;  // printf()
    EUSCI_A2->IE   |=  0x0001;  // RPLiDAR C1
    EUSCI_A3->IE   |=  0x0001;  // BLE UART Friend
//    TIMER_A1->CCTL[0]  |= 0x0010;


    EUSCI_A0->CTLW0    &= ~0x01;
    EUSCI_A2->CTLW0    &= ~0x01;
    EUSCI_A3->CTLW0    &= ~0x01;

}


#endif /* __INC_GPIO_UTILITIES_H__ */
