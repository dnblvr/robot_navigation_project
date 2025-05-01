/**
 * @file Bumper_Switches.h
 * @brief Header file for the Bumper_Switches driver.
 *
 * This file contains the function definitions for the Bumper_Switches driver.
 * It is used to demonstrate external I/O-triggered interrupts.
 * The driver interfaces with the following:
 *  - Pololu Left Bumper Switches (https://www.pololu.com/product/3673)
 *  - Pololu Right Bumper Switches (https://www.pololu.com/product/3674)
 *
 * To verify the pinout used for the Pololu Bumper Switches, refer to the MSP432P401R SimpleLink Microcontroller LaunchPad Development Kit User's Guide
 * Link: https://docs.rs-online.com/3934/A700000006811369.pdf
 *
 * The following pins are used when the Bumper Switches are connected to the TI MSP432 LaunchPad:
 *  - BUMP_0                    <-->    MSP432 LaunchPad Pin P4.0
 *  - BUMP_1                    <-->    MSP432 LaunchPad Pin P4.2
 *  - BUMP_2                    <-->    MSP432 LaunchPad Pin P4.3
 *  - Right Bumper Switch GND   <-->    MSP432 LaunchPad GND
 *  - BUMP_3                    <-->    MSP432 LaunchPad Pin P4.5
 *  - BUMP_4                    <-->    MSP432 LaunchPad Pin P4.6
 *  - BUMP_5                    <-->    MSP432 LaunchPad Pin P4.7
 *  - Left Bumper Switch GND    <-->    MSP432 LaunchPad GND
 *
 * @note The Bumper Switches operate in an active low configuration.
 * When the switches are pressed, they connect to GND.
 *
 * @author Aaron Nanas
 *
 */

#ifndef INC_BUMPER_SWITCHES_H_
#define INC_BUMPER_SWITCHES_H_

#include <stdint.h>
#include "msp.h"

/**
 * @brief User-defined task function for handling Bumper Switch interrupt events.
 *
 * This is a user-defined function that can be assigned to the Bumper_Task pointer during initialization.
 * When a falling edge event is detected on any of the Bumper Switch pins (P4.7, P4.6, P4.5, P4.3, P4.2, or P4.0),
 * this function will be called, and the bumper_switch_state parameter will indicate which specific bumper switch triggered the interrupt.
 *
 * @param bumper_switch_state An 8-bit unsigned integer representing the Bumper Switch that triggered the interrupt.
 *                   The bit positions correspond to the following switches:
 *                   - Bit 7 (MSB): P4.7 Bumper Switch
 *                   - Bit 6: P4.6 Bumper Switch
 *                   - Bit 5: P4.5 Bumper Switch
 *                   - Bit 4: Reserved (unused)
 *                   - Bit 3: P4.3 Bumper Switch
 *                   - Bit 2: P4.2 Bumper Switch
 *                   - Bit 1: Reserved (unused)
 *                   - Bit 0 (LSB): P4.0 Bumper Switch
 *
 * @return None
 */
void (*Bumper_Task)(uint8_t bumper_switch_state);

/**
 * @brief
 *
 * @param task A pointer to the user-defined function that will be called on a falling edge event.
 *
 * @return None
 */
void Bumper_Switches_Init(void(*task)(uint8_t));

/**
 * @brief Read the current state of the 6 Bumper Switches which have negative logic behavior.
 *
 * This function reads the state of 6 switches (BUMP_0 to BUMP_5) connected to the TI MSP432 LaunchPad's P4 port.
 * The switches are expected to have negative logic behavior, and their states are returned as a 6-bit result
 * represented as a uint8_t variable. Each bit of the result corresponds to one switch, where Bit 5 represents BUMP_5,
 * Bit 4 represents BUMP_4, and so on until Bit 0, which represents BUMP_0.
 *
 *  @note The Bumper Switches have negative logic behavior, where a logic high indicates a released (open) switch,
 *       and a logic low represents a pressed (closed) switch.
 *
 * @return uint8_t The 6-bit positive logic result representing the current state of the switches (0 to 63).
 */
uint8_t Bumper_Read(void);

#endif /* INC_BUMPER_SWITCHES_H_ */
