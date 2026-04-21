/**
 * @file LPUART8.h
 * @brief Teensy HAL for the LPUART8 driver with emphasis on bare-metal RX
 *  handling.
 *
 * @details This is a direct reference of RPLiDAR_UART codebase. The public API
 *  surface is intentionally kept somewhat identical to keep a consistent
 *  interface. Only the hardware seam is swapped.
 *
 * @note Call LPUART8_SetPort() before anything else, then LPUART8_Init() to
 *  start the serial port at a given baud rate.
 *
 * @note For every ISR call, feed all FIFO bytes to LPUART8_ProcessByte(b).
 *  The FSM will handle the rest.
 * 
 * @note This is inspired by Paul Stoffregen's musing on his forums about
 *  barebones T4.0 serial drivers. He prefers that we not spend time writing a
 *  full drivers, considering that he spent a full year writing the core
 *  library. Instead, he prefers that we write drivers based on his Arduino
 *  Teensy core library and whittle down to the bare metal. This is part of the
 *  result of this approach. Over time, this driver will be completed with
 *  Arduino-independent functions and serve as a reference for LPUART handling.
 * 
 * @note LPUART modules are aligned to Serialx ports according to Paul
 *  Stoffregen's IMXRT1060RM annotations. This is the table for convenience:
 * 
 *      | Module  | Serial Port |
 *      | :-----: | :---------: |
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
#include <inEKF_se2.h>


#include "LPUART8_Config.h"

// ----------------------------------------------------------------------------
//
//  TASK INITIALIZATION
//
// ----------------------------------------------------------------------------

/**
 * @brief type definition of an UART ISR function pointer
 */
typedef void (*LPUART_ISR_Task)(volatile char *);


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
 * @brief   Open the serial port at the specified baud rate.
 * 
 * @note    Does NOT install the bare-metal RX ISR.  Call
 *          LPUART8_AttachISR() from setup() once all protocol init
 *          commands (STOP / RESET / GET_HEALTH / SCAN) have been sent.
 * 
 * @param baud_rate  Desired baud rate for the serial port.
 */
void LPUART8_Init(uint32_t baud_rate);


/**
 * @brief   Flush TX, disable TX interrupts, then replace HardwareSerial's
 *          LPUART8 vector with the bare-metal LPUART8_RX_ISR.
 * 
 * @details Must be called from setup() AFTER LPUART8_Init() returns.  Calling it earlier causes a TX-interrupt infinite loop:
 *  HardwareSerial re-enables CTRL.TIE when it queues TX bytes, but our
 *  RX-only ISR never clears TIE, so LPUART8_IRQ fires endlessly and
 *  starves the CPU (including USB serial).
 * 
 * @param task 
 */
void LPUART8_AttachISR(LPUART_ISR_Task task);


/**
 * @brief Close the serial port.
 *
 * @note  On Teensy, HardwareSerial::end() flushes TX and disables the LPUART
 *        peripheral.  The RPLiDAR C1 motor will keep spinning.
 */
void LPUART8_Stop(void);


/**
 * @brief Re-open the serial port at LPUART8_BAUD_RATE after a Stop().
 */
void LPUART8_Restart(void);


// ----------------------------------------------------------------------------
//
//  BYTE-LEVEL I/O
//
// ----------------------------------------------------------------------------

/**
 * @brief Blocking transmit of one byte to the LPUART8.
 *
 * @param data   Byte to send.
 */
void LPUART8_OutChar(uint8_t data);


/**
 * @brief Blocking receive of one byte from the LPUART8.
 *
 * @return Received byte.
 */
uint8_t LPUART8_InChar(void);


/**
 * @brief Blocking transmit of a null-terminated string to the LPUART8.
 * 
 * @note This is a simple wrapper around OutChar() for convenience.  It does NOT
 *       perform any formatting or buffering.
 * 
 * @param str  Null-terminated string to send.
 */
void LPUART8_OutString(const char* str);


/**
 * @brief Blocking receive of a string of specified length from the LPUART8.
 * 
 * @param buffer  Pointer to the buffer where received bytes will be stored.
 * @param length  Number of bytes to receive.
 */
void LPUART8_InString(uint8_t* buffer, size_t length);


// ----------------------------------------------------------------------------
//
//  BYTE PROCESSOR
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

/**
 * @brief extra
 * 
 * @return state_se2_t 
 */
state_se2_t Get_State_Request(void);

// ----------------------------------------------------------------------------
//
//  INTERNAL DECLARATIONS
//
// ----------------------------------------------------------------------------

/** Raw 5-byte staging buffer and packed-word buffer (defined in .cpp) */
// extern volatile char*  RX_POINTER;

uint8_t Check_UART_Data(
        volatile char  UART_Data_Buffer[],
           const char* data_string);


#endif /* __LPUART8_H__ */
