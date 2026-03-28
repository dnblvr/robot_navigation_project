/**
 * @file LPUART8.h
 * @brief Teensy HAL for the LPUART8 driver.
 *
 * @details This is a direct port of RPLiDAR_A2_UART.h. The public API surface is intentionally kept somewhat identical to keep a consistent interface. Only the hardware seam is swapped.
 *
 * @note Call LPUART8_SetPort() before anything else, then
 *       LPUART8_Init() to start the serial port at 460,800 bps.
 *
 * @note For every ISR call, feed every incoming byte
 *       to LPUART8_ProcessByte(b).  The FSM will handle the rest.
 * 
 * @note LPUART modules are aligned according to Paul Stoffregen's IMXRT1060RM annotations. This is the table:
 *      | Module  | Serial Port |
 *      |---------|-------------|
 *      | LPUART1 |   Serial6   |
 *      | LPUART2 |   Serial3   |
 *      | LPUART3 |   Serial2   |
 *      | LPUART4 |   Serial4   |
 *      | LPUART5 |   Serial8   |
 *      | LPUART6 |   Serial1   |
 *      | LPUART7 |   Serial7   |
 *      | LPUART8 |   Serial5   |
 *
 * @author Gian Fajardo
 */

#ifndef __LPUART8_H__
#define __LPUART8_H__


#include <Arduino.h>
#include <stdint.h>
// #include <string.h>
// #include <assert.h>
// #include <math.h>


#include "LPUART8_Config.h"
// #include "data_structures.h"


// ----------------------------------------------------------------------------
//
//  UART LIFECYCLE
//
// ----------------------------------------------------------------------------


/**
 * @brief Bind a HardwareSerial port to the driver.
 *
 * @details Must be called before LPUART8_Init().  The driver stores a
 *  pointer to the supplied port; the caller is responsible for keeping the
 *  object alive (e.g. pass &Serial1 which is a forever-live global).
 *
 * @param port  Pointer to any HardwareSerial instance (Serial1…Serial8 on
 *              Teensy 4.x).
 */
void LPUART8_SetPort(HardwareSerial* port);

/**
 * @brief Open the serial port at LPUART8_BAUD (460,800).
 *
 * @note  Does NOT install the bare-metal RX ISR.  Call
 *        LPUART8_AttachISR() from setup() once all protocol init
 *        commands (STOP / RESET / GET_HEALTH / SCAN) have been sent.
 */
void LPUART8_Init(void);


/**
 * @brief Flush TX, disable TX interrupts, then replace HardwareSerial's
 *        LPUART6 vector with the bare-metal LPUART6_RX_ISR.
 *
 * @details Must be called from setup() AFTER Initialize_RPLiDAR_C1()
 *  returns.  Calling it earlier causes a TX-interrupt infinite loop:
 *  HardwareSerial re-enables CTRL.TIE when it queues TX bytes, but our
 *  RX-only ISR never clears TIE, so LPUART6_IRQ fires endlessly and
 *  starves the CPU (including USB serial).
 */
void LPUART8_AttachISR(void);


/**
 * @brief Close the serial port.
 *
 * @note  On Teensy, HardwareSerial::end() flushes TX and disables the LPUART
 *        peripheral.  The RPLiDAR C1 motor will keep spinning.
 */
void LPUART8_Stop(void);


/**
 * @brief Re-open the serial port at LPUART8_BAUD after a Stop().
 */
void LPUART8_Restart(void);


// ----------------------------------------------------------------------------
//
//  BYTE-LEVEL I/O  (analogous to EUSCI_A2_UART_OutChar / InChar)
//
// ----------------------------------------------------------------------------

/**
 * @brief Blocking transmit of one byte to the RPLiDAR C1.
 *
 * @param data   Byte to send.
 */
void LPUART8_OutChar(uint8_t data);


/**
 * @brief Blocking receive of one byte from the RPLiDAR C1.
 *
 * @return Received byte.
 */
uint8_t LPUART8_InChar(void);


// ----------------------------------------------------------------------------
//
//  BYTE PROCESSOR  (replaces EUSCIA2_IRQHandler)
//
// ----------------------------------------------------------------------------

/**
 * @brief Feed one received byte into the acquisition FSM.
 *
 * @details This is the direct replacement for EUSCIA2_IRQHandler().
 *
 * @param b  Byte freshly read from the serial port.
 * 
 * @return None
 */
void LPUART8_ProcessByte(uint8_t b);


// ----------------------------------------------------------------------------
//
//  INTERNAL DECLARATIONS
//
// ----------------------------------------------------------------------------

/** Raw 5-byte staging buffer and packed-word buffer (defined in .cpp) */
extern uint8_t*  RX_POINTER;
extern uint32_t* INTERM_POINTER;


#endif /* __LPUART8_H__ */
