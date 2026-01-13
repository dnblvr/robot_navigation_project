#include "FunctionQueue.h"

// Function pointer queue using arrays configured in a circular buffer style
const uint8_t QUEUE_SIZE = 16;  // Adjust size as needed

// Message storage for queued functions
CommandHandler function_queue[QUEUE_SIZE];
String queued_messages[QUEUE_SIZE];

volatile uint8_t queue_head     = 0;
volatile uint8_t queue_tail     = 0;
volatile uint8_t queue_count    = 0;

/**
 * @brief Enqueue a function for later execution
 * 
 * @param func      function pointer to the command handler
 * @param message   the message to be processed by the command handler
 * @return true     if the function was successfully enqueued
 * @return false    if the queue is full
 */
bool enqueue_function(
    CommandHandler func,
    const String& message)
{
  if (queue_count >= QUEUE_SIZE) {
    Serial.println("Queue full! Dropping command.");
    return false;
  }
  
  // Store function pointer and message
  function_queue[queue_tail] = func;
  queued_messages[queue_tail] = message;
  
  // Update queue pointers, with wrap-around
  // queue_tail = (queue_tail + 1) % QUEUE_SIZE;
  queue_tail++;

  if (queue_tail >= QUEUE_SIZE) {
    queue_tail = 0;  // Wrap around to the beginning
  }
  
  queue_count++;
  
  return true;
}

void process_function_queue() {
  while (queue_count > 0) {
    // Get function and message from queue
    CommandHandler func = function_queue[queue_head];
    String message = queued_messages[queue_head];
    
    // Clear the slot
    function_queue[queue_head] = nullptr;
    queued_messages[queue_head] = "";
    
    // Update queue pointers
    queue_head = (queue_head + 1) % QUEUE_SIZE;
    queue_count--;
    
    // Execute the function
    if (func != nullptr) {
      func(message);
    }
  }
}
