/**
 * @file Reflectance_Sensor.c
 * @brief Source code for the Reflectance_Sensor driver.
 *
 * This file contains the function definitions for the Reflectance_Sensor driver.
 * It uses GPIO to interface with the 8-Channel QTRX Sensor Array module.
 *
 * The 8-Channel QTRX Sensor Array module uses the following pinout:
 *  - Reflectance Sensor Output 0   <-->  MSP432 LaunchPad Pin P7.0
 *  - Reflectance Sensor Output 1   <-->  MSP432 LaunchPad Pin P7.1
 *  - Reflectance Sensor Output 2   <-->  MSP432 LaunchPad Pin P7.2
 *  - Reflectance Sensor Output 3   <-->  MSP432 LaunchPad Pin P7.3
 *  - Reflectance Sensor Output 4   <-->  MSP432 LaunchPad Pin P7.4
 *  - Reflectance Sensor Output 5   <-->  MSP432 LaunchPad Pin P7.5
 *  - Reflectance Sensor Output 6   <-->  MSP432 LaunchPad Pin P7.6
 *  - Reflectance Sensor Output 7   <-->  MSP432 LaunchPad Pin P7.7
 *  - Reflectance Sensor Ctrl Even  <-->  MSP432 LaunchPad Pin P5.3
 *  - Reflectance Sensor Ctrl Odd   <-->  MSP432 LaunchPad Pin P9.2
 *  - Reflectance Sensor VCC        <-->  MSP432 LaunchPad 5V
 *  - Reflectance Sensor GND        <-->  MSP432 LaunchPad GND
 *
 * @note The typical sequence for reading the reflectance sensors is as follows:
 *          1. Turn on the IR LEDs
 *          2. Configure the sensor output pins as output GPIO and drive them high
 *          3. Allow at least 10 us for the sensor output to rise
 *          4. Configure the sensor output pins as input GPIO
 *          5. Measure the time for the voltage to decay by waiting for the GPIO lines to go low
 *          6. Turn off the IR LEDs
 *
 * @note For more information regarding the 8-Channel QTRX Sensor Array module,
 * refer to the product page: https://www.pololu.com/product/3672
 *
 * @author Aaron Nanas
 *
 */

#include "../inc/Reflectance_Sensor.h"

void Reflectance_Sensor_Init()
{
    // Configure P5.3 as an output GPIO pin by
    // clearing Bit 3 of the SEL0 and SEL1 registers for P5
    // and setting Bit 3 of the DIR register for P5
    P5->SEL0 &= ~0x08;
    P5->SEL1 &= ~0x08;
    P5->DIR |= 0x08;

    // Turn off the even-numbered IR LEDs by
    // clearing Bit 3 of the OUT register for P5
    P5->OUT &= ~0x08;

    // Configure P9.2 as an output GPIO pin by
    // clearing Bit 2 of the SEL0 and SEL1 registers for P9
    // and setting Bit 2 of the DIR register for P9
    P9->SEL0 &= ~0x04;
    P9->SEL1 &= ~0x04;
    P9->DIR |= 0x04;

    // Turn off the odd-numbered IR LEDs by
    // clearing Bit 2 of the OUT register for P9
    P9->OUT &= ~0x04;

    // Configure P7.0 - P7.7 as input GPIO pins by
    // clearing Bits 0 to 7 of the SEL0, SEL1, and DIR registers for P7
    P7->SEL0 &= ~0xFF;
    P7->SEL1 &= ~0xFF;
    P7->DIR &= ~0xFF;
}

uint8_t Reflectance_Sensor_Read(uint32_t time)
{
    // Turn on the even-numbered IR LEDs by
    // setting Bit 3 of the OUT register for P5
    P5->OUT |= 0x08;

    // Turn on the odd-numbered IR LEDs by
    // setting Bit 2 of the OUT register for P9
    P9->OUT |= 0x04;

    // Configure P7.0 - P7.7 as output GPIO pins by
    // setting Bits 0 to 7 of the DIR register for P7
    P7->DIR |= 0xFF;

    // Set the P7.0 - P7.7 pins to high by
    // setting Bits 0 to 7 of the OUT register for P7
    P7->OUT |= 0xFF;

    // Wait for 10 us using the Clock_Delay1us function
    Clock_Delay1us(10);

    // After waiting 10 us, configure P7.0 - P7.7 as input GPIO pins
    // by clearing Bits 0 to 7 of the DIR register for P7
    P7->DIR &= ~0xFF;

    // Call the Clock_Delay1us function and pass in the "time" input parameter
    Clock_Delay1us(time);

    // Declare a local uint8_t variable called "reflectance_value"
    // and assign it the value of P7->IN, which is the 8-bit data
    // read from P7. This represents the reflectance sensor value
    // Note: "1" indicates black while "0" indicates white
    uint8_t reflectance_value = P7->IN;

    // Turn off the even-numbered IR LEDs by
    // clearing Bit 3 of the OUT register for P5
    P5->OUT &= ~0x08;

    // Turn off the odd-numbered IR LEDs by
    // clearing Bit 2 of the OUT register for P9
    P9->OUT &= ~0x04;

    // Return the local variable, "reflectance_value"
    return reflectance_value;
}

void Reflectance_Sensor_Start()
{
    // Turn on the even-numbered IR LEDs by setting Bit 3 of the OUT register for P5
    P5->OUT |= 0x08;

    // Turn on the odd-numbered IR LEDs by setting Bit 2 of the OUT register for P9
    P9->OUT |= 0x04;

    // Configure P7.0 - P7.7 as output GPIO pins by setting Bits 0 to 7 of the DIR register for P7
    P7->DIR |= 0xFF;

    // Set the P7.0 - P7.7 pins to high by setting Bits 0 to 7 of the OUT register for P7
    P7->OUT |= 0xFF;

    // Wait for 10 us using the Clock_Delay1us function
    Clock_Delay1us(10);

    // After waiting 10 us, configure P7.0 - P7.7 as input GPIO pins
    // by clearing Bits 0 to 7 of the DIR register for P7
    P7->DIR &= ~0xFF;
}

uint8_t Reflectance_Sensor_End()
{
    // Declare a local uint8_t variable called "reflectance_value"
    // and assign it the value of P7->IN, which is the 8-bit data
    // read from P7. This represents the reflectance sensor value
    // Note: "1" indicates black while "0" indicates white
    uint8_t reflectance_value = P7->IN;

    // Turn off the even-numbered IR LEDs by clearing Bit 3 of the OUT register for P5
    P5->OUT &= ~0x08;

    // Turn off the odd-numbered IR LEDs by clearing Bit 2 of the OUT register for P9
    P9->OUT &= ~0x04;

    // Return the local variable, "reflectance_value"
    return reflectance_value;
}
