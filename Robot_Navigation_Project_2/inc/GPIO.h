/**
 * @file GPIO.h
 * @brief Header file for the GPIO driver.
 *
 * This file contains the function definitions for the GPIO driver.
 * It interfaces with the following:
 *  - User buttons and LEDs of the TI MSP432 LaunchPad
 *  - PMOD SWT (4 Slide Switches)
 *  - PMOD 8LD (8 LEDs)
 *
 * To verify the pinout of the user buttons and LEDs, refer to the MSP432P401R SimpleLink Microcontroller LaunchPad Development Kit User's Guide
 * Link: https://docs.rs-online.com/3934/A700000006811369.pdf
 *
 * For more information regarding the PMODs used in this lab, visit the following links:
 *  - PMOD SWT: https://digilent.com/reference/pmod/pmodswt/reference-manual
 *  - PMOD 8LD: https://digilent.com/reference/pmod/pmod8ld/reference-manual
 *
 * @note The user buttons, located at P1.1 and P1.4, are configured with negative logic
 * as the default setting. When the buttons are pressed, they connect to GND. Refer to the
 * schematic found in the MSP432P401R LaunchPad User's Guide.
 *
 * @author Aaron Nanas
 *
 */

#ifndef INC_GPIO_H_
#define INC_GPIO_H_

#include <stdint.h>
#include "msp.h"
#include "Clock.h"

// Constant definitions for the built-in red LED
extern const uint8_t RED_LED_OFF;
extern const uint8_t RED_LED_ON;

// Constant definitions for the RGB LED colors
extern const uint8_t RGB_LED_OFF;
extern const uint8_t RGB_LED_RED;
extern const uint8_t RGB_LED_GREEN;
extern const uint8_t RGB_LED_YELLOW;
extern const uint8_t RGB_LED_BLUE;
extern const uint8_t RGB_LED_PINK;
extern const uint8_t RGB_LED_SKY_BLUE;
extern const uint8_t RGB_LED_WHITE;

// Constant definitions for the PMOD 8LD module
extern const uint8_t PMOD_8LD_ALL_OFF;
extern const uint8_t PMOD_8LD_ALL_ON;
extern const uint8_t PMOD_8LD_0_3_ON;
extern const uint8_t PMOD_8LD_4_7_ON;

/**
 * @brief The LED1_Init function initializes the built-in red LED (P1.0).
 *
 * This function initializes the built-in red LED located at pin P1.0
 * and configures it as a GPIO pin. It sets the direction of the pin as output.
 *
 * @param None
 *
 * @return None
 */
void LED1_Init();

/**
 * @brief The LED1_Output function sets the output of the built-in red LED.
 *
 * This function sets the output of the built-in red LED based on the value of the input, led_value.
 * A bitwise AND operation (& 0xFE) is performed to mask the first bit (LSB) of the output register
 * to preserve the state of other pins connected to Port 1 while keeping the LED pin unaffected.
 * Then, a bitwise OR operation is performed with led_value to set the LED pin to the desired state
 * specified by led_value.
 *
 * @param led_value An 8-bit unsigned integer that determines the output of the built-in red LED. To turn off
 *                  the LED, set led_value to 0. Otherwise, setting led_value to 1 turns on the LED.
 *
 * @return None
 */
void LED1_Output(uint8_t led_value);

/**
 * @brief Toggles the built-in red LED (P1.0).
 *
 * This function toggles the state of the built-in red LED by performing a bitwise XOR operation between the current output
 * of Port 1 (P1->OUT) and the value 0x01. The purpose of the bitwise XOR operation is to flip the least significant
 * bit corresponding to LED1 while preserving the state of other pins connected to Port 1.
 *
 * @param None
 *
 * @return None
 */
void LED1_Toggle();

/**
 * @brief The LED1_Status function indicates the status of the built-in red LED
 * located at pin P1.0.
 *
 * @param None
 *
 * @return uint8_t The value representing the status of the built-in red LED.
 *  - 0: Red LED OFF
 *  - 1: Red LED ON
 *
 */
uint8_t LED1_Status();

/**
 * @brief The LED2_Init function initializes the RGB LED (P2.0 - P2.2).
 *
 * This function initializes the following RGB LED, configures the pins as GPIO pins with high drive strength,
 * and sets the direction of the pins as output. The RGB LED is off by default upon initialization.
 *  - RGBLED_RED      (P2.0)
 *  - RGBLED_GREEN    (P2.1)
 *  - RGBLED_BLUE     (P2.2)
 *
 * @param None
 *
 * @return None
 */
void LED2_Init();

/**
 * @brief The LED2_Output function sets the output of the RGB LED.
 *
 * This function sets the output of the RGB LED based on the value of the input, led_value.
 * A bitwise AND operation (& 0xF8) is performed to mask the lower three bits of the output register
 * to preserve the state of other pins connected to Port 2 while keeping the RGB LED pin unaffected.
 * Then, a bitwise OR operation is performed with led_value to set the RGB LED pin to the desired state
 * specified by led_value.
 *
 * @param led_value An 8-bit unsigned integer that determines the output of the RGB LED. To turn off
 *                  the RGB LED, set led_value to 0. The following values determine the color of the RGB LED:
 *
 *  Color       LED(s)   led_value
 *  Off         ---         0x00
 *  Red         R--         0x01
 *  Green       -G-         0x02
 *  Yellow      RG-         0x03
 *  Blue        --B         0x04
 *  Pink        R-B         0x05
 *  Sky Blue    -GB         0x06
 *  White       RGB         0x07
 *
 * @return None
 */
void LED2_Output(uint8_t led_value);

/**
 * @brief Toggles the RGB LED based on the input provided.
 *
 * This function toggles the RGB LED by performing a bitwise XOR operation between the current output
 * of Port 2 (P2->OUT) and the provided led_value. The purpose of the bitwise XOR operation is to flip the bits
 * corresponding to the RGB LED while preserving the state of other pins connected to Port 2.
 *
 * @param led_value An 8-bit unsigned integer that determines the color to display on the RGB LED.
 *
 * @return None
 */
void LED2_Toggle(uint8_t led_value);

/**
 * @brief The LED2_Status function indicates the status of the RGB LED
 * located at pins P2.0 - P2.2.
 *
 * @param None
 *
 * @return uint8_t The value representing the status of the RGB LED.
 *
 *  Color       LED(s)   rgb_led_value
 *  Off         ---         0x00
 *  Red         R--         0x01
 *  Green       -G-         0x02
 *  Yellow      RG-         0x03
 *  Blue        --B         0x04
 *  Pink        R-B         0x05
 *  Sky Blue    -GB         0x06
 *  White       RGB         0x07
 *
 */
uint8_t LED2_Status();

/**
 * @brief The Buttons_Init function initializes the user buttons (P1.1 and P1.4).
 *
 * This function initializes the user buttons located at pins P1.1 and P1.4.
 * It configures the pins as GPIO input pins with pull-up resistors enabled.
 *
 * @param None
 *
 * @return None
 */
void Buttons_Init();

/**
 * @brief The Get_Buttons_Status reads the status of the user buttons (P1.1 and P1.4) and returns it.
 *
 * This function reads the status of the user buttons located at pins P1.1 and P1.4.
 * It indicates whether or not the buttons are pressed and returns the status.
 * A bitwise AND operation (& 0x12) is performed to mask the unused bits in the input register.
 *
 * @param None
 *
 * @return Indicates the status of the buttons.
 *  - 0x00: Button 1 and Button 2 are pressed
 *  - 0x10: Button 1 is pressed while Button 2 is not pressed
 *  - 0x02: Button 1 is not pressed while Button 2 is pressed
 *  - 0x12: Button 1 and Button 2 are not pressed
 */
uint8_t Get_Buttons_Status();

/**
 * @brief The PMOD_8LD_Init function initializes the pins (P9.0 - P9.7) used by the Digilent PMOD 8LD module.
 *
 * This function initializes the pins, P9.0 through P9.7, which will be used to drive the eight
 * LEDs on the Digilent PMOD 8LD module. It configures the pins as GPIO output pins with high drive strength.
 *
 * The following connections must be made:
 *  - PMOD LED0   <-->  MSP432 LaunchPad Pin P9.0
 *  - PMOD LED1   <-->  MSP432 LaunchPad Pin P9.1
 *  - PMOD LED2   <-->  MSP432 LaunchPad Pin P9.2
 *  - PMOD LED3   <-->  MSP432 LaunchPad Pin P9.3
 *  - PMOD Pin 5  <-->  MSP432 LaunchPad GND
 *  - PMOD Pin 6  <-->  MSP432 LaunchPad VCC (3.3V)
 *  - PMOD LED4   <-->  MSP432 LaunchPad Pin P9.4
 *  - PMOD LED5   <-->  MSP432 LaunchPad Pin P9.5
 *  - PMOD LED6   <-->  MSP432 LaunchPad Pin P9.6
 *  - PMOD LED7   <-->  MSP432 LaunchPad Pin P9.7
 *  - PMOD Pin 11 <-->  MSP432 LaunchPad GND
 *  - PMOD Pin 12 <-->  MSP432 LaunchPad VCC (3.3V)
 *
 * @param None
 *
 * @return None
 */
void PMOD_8LD_Init();

/**
 * @brief The PMOD_8LD_Output function sets the output of the eight LEDs on the PMOD 8LD module.
 *
 * This function sets the output value of the PMOD 8LD module by writing the provided led_value to the
 * corresponding output pins. It then reads back the actual value written to the PMOD 8LD module and returns it.
 *
 * @param led_value An 8-bit unsigned integer representing the desired output value for the PMOD 8LD module.
 *
 * @return The output value written to the PMOD 8LD module.
 *         0: LED Off
 *         1: LED On
 */
uint8_t PMOD_8LD_Output(uint8_t led_value);

/**
 * @brief The PMOD_SWT_Init function initializes the pins (P10.0 - P10.3) used by the Digilent PMOD SWT module.
 *
 * This function initializes the pins, P10.0 through P10.3, which will be used to receive the inputs
 * from the four slide switches on the PMOD SWT module. It configures the pins as GPIO input pins.
 *
 * The following connections must be made:
 *  - PMOD SWT1   <-->  MSP432 LaunchPad Pin P10.0
 *  - PMOD SWT2   <-->  MSP432 LaunchPad Pin P10.1
 *  - PMOD SWT3   <-->  MSP432 LaunchPad Pin P10.2
 *  - PMOD SWT4   <-->  MSP432 LaunchPad Pin P10.3
 *  - PMOD Pin 5  <-->  MSP432 LaunchPad GND
 *  - PMOD Pin 6  <-->  MSP432 LaunchPad VCC (3.3V)
 *
 * @param None
 *
 * @return None
 */
void PMOD_SWT_Init();

/**
 * @brief The Get_PMOD_SWT_Status function gets the input values of the PMOD SWT.
 *
 * This function reads the values of the four slide switches on the PMOD SWT
 * from the P10 IN register. A bitwise AND operation (& 0xF) is performed to mask
 * the upper four bits of the input register, keeping only the lower four bits
 * that correspond to the switch positions.
 *
 * @param None
 *
 * @return Indicates the status of the switches.
 *
 *  SWT1    SWT2    SWT3    SWT4    switch_status
 *  ----    ----    ----    ----    -------------
 *  OFF     OFF     OFF     OFF         0x00
 *  ON      OFF     OFF     OFF         0x01
 *  OFF     ON      OFF     OFF         0x02
 *  ON      ON      OFF     OFF         0x03
 *  OFF     OFF     ON      OFF         0x04
 *  ON      OFF     ON      OFF         0x05
 *  OFF     ON      ON      OFF         0x06
 *  ON      ON      ON      OFF         0x07
 *  OFF     OFF     OFF     ON          0x08
 *  ON      OFF     OFF     ON          0x09
 *  OFF     ON      OFF     ON          0x0A
 *  ON      ON      OFF     ON          0x0B
 *  OFF     OFF     ON      ON          0x0C
 *  ON      OFF     ON      ON          0x0D
 *  OFF     ON      ON      ON          0x0E
 *  ON      ON      ON      ON          0x0F
 *
 */
uint8_t Get_PMOD_SWT_Status();

/**
 * @brief The LED_Pattern_1 function sets the output of the user LEDs based on the status of the user buttons.
 *
 * This function sets the output of both the built-in red LED (P1.0) and the RGB LED (P2.0 - P2.2) based on
 * the status of the Button 1 (P1.1) and Button 2 (P1.4).
 *
 * @param button_status An 8-bit unsigned integer that indicates the status of Button 1 and Button 2.
 *                      The two user LEDs are controlled based on the value of button_status.
 *
 *  button_status      LED 1 Color      RGB LED Color
 *  -------------      -----------      -------------
 *      0x00               Red              Red
 *      0x10               Red              Off
 *      0x02               Off              Green
 *      0x12               Off              Off
 *
 *
 * @return None
 */
void LED_Pattern_1(uint8_t button_status);

/**
 * @brief The LED_Pattern_2 function controls the user LEDs and the eight LEDs on the PMOD 8LD module.
 *
 * This function turns on LED1 with a red color, sets the RGB LED to display a red color,
 * and then initiates a binary counter pattern on the PMOD 8LD module. The counter starts from 0
 * and increments up to 255 (0xFF) with a delay of 100 ms between each count. The sequence stops if
 * a specific switch status is detected or if led_count has reached 0xFF.
 *
 *
 * @param None
 *
 * @return None
 */
void LED_Pattern_2();

/**
 * @brief The LED_Controller function selects and executes an appropriate LED pattern based on button and switch statuses.
 *
 * This function determines the LED pattern to execute based on the given button status and switch status.
 * It calls different pattern functions based on the switch status value.
 *
 * @param button_status An 8-bit unsigned integer representing the status of the user buttons. This value is used to determine
 *                      the LED pattern in some cases.
 * @param switch_status An 8-bit unsigned integer representing the status of the switches on the PMOD SWT. This value is used
 *                      to select the LED pattern to execute.
 *
 * @return None
 */
void LED_Controller(uint8_t button_status, uint8_t switch_status);

/**
 * @brief The Chassis_Board_LEDs_Init function initializes the LEDs on the chassis board.
 *
 * This function initializes the following LEDs on the chassis board and configures the pins as
 * output GPIO pins. The LEDs are off by default upon initialization.
 *  - P8.0  (Front Left Yellow LED)
 *  - P8.5  (Front Right Yellow LED)
 *  - P8.6  (Back Left Red LED)
 *  - P8.7  (Back Right Red LED)
 *
 * @param None
 *
 * @return None
 */
void Chassis_Board_LEDs_Init();

#endif /* INC_GPIO_H_ */
