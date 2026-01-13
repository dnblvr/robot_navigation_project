/**
 * @file FunctionQueue.h
 * @brief Header file for the FunctionQueue library.
 *
 * This file contains the function definitions for the FunctionQueue library.
 *
 * It provides a way to queue function calls with associated messages for
 * processing in a circular buffer style.
 *
 * @note This library is designed for use with Arduino and compatible platforms.
 *
 * @author Gian Fajardo
 *
 */

#ifndef FUNCTION_QUEUE_H
#define FUNCTION_QUEUE_H

#include <Arduino.h>

/**
 * @brief Function pointer type for command handlers
 */
typedef void (*CommandHandler)(const String& message);


// Function pointer queue using arrays configured in a circular buffer style
/**
 * @brief Size of the function queue
 */
extern const uint8_t QUEUE_SIZE;


/**
 * @brief Queue state variables
 * @details queue_head      index of the function queue
 *          queue_tail      index of the function queue
 *          queue_count     number of functions in the queue
 */
extern volatile uint8_t queue_head;
extern volatile uint8_t queue_tail;
extern volatile uint8_t queue_count;


// Queue arrays
/**
 * @brief Queue arrays for both the message and the function (pointers)
 * @details queue_head      index of the function queue
 *          queue_tail      index of the function queue
 *          queue_count     number of functions in the queue
 */
extern CommandHandler function_queue[];
extern String queued_messages[];

// Queue management functions
bool enqueue_function(CommandHandler func, const String& message);
void process_function_queue();

#endif // FUNCTION_QUEUE_H