#include <Arduino.h>
#include <iostream>
#include <string.h>
#include "FunctionQueue.h"
#include "matrix_examples.h"


#define MSP432_Serial Serial7
#define BLE_Serial    Serial5


#define RTS_Pin 22


// function declarations -----------------------------------------------------

// Command handler functions
void handle_BLE_command(const String& message);
void handle_MSP432_command(const String& message);

/**
 * @brief 
 * @todo implement this on a timer interrupt
 * 
 */
void initialize_BLE_UART();



void initialize_BLE_UART() {

  // MOD pin
  pinMode(RTS_Pin, OUTPUT);
  BLE_Serial.begin(9600);

  // test BLE UART friend configuration
  digitalWrite(RTS_Pin, HIGH); // HIGH = AT command mode
  delay(1000);

  BLE_Serial.write("ATZ\r\n");
  delay(3000);

  digitalWrite(RTS_Pin, LOW); // LOW = UART mode

  // while (!BLE_Serial);
}

// Command handler implementations
void handle_BLE_command(
    const String& message)
{

  Serial.println("Processing BLE command: " + message);
  
  // Parse different BLE command types
  if (message.indexOf("!B") != -1) {

    // Basic command
    char BLE_code[5] = {};
    strncpy(BLE_code, message.c_str(), 4);
    BLE_code[4] = '\0';
    
    Serial.println("BLE basic command received");
    Serial.print("Echoing: ");  Serial.println(BLE_code);
    
    MSP432_Serial.println(BLE_code);

    
  } else if (message.indexOf("!NAV") != -1) {

    // Navigation command
    Serial.println("BLE navigation command received");
    // Extract navigation parameters and process

    
  } else if (message.indexOf("!STOP") != -1) {

    // Emergency stop command
    Serial.println("BLE emergency stop received");
    MSP432_Serial.println("!STOP");

    
  } else if (message.indexOf("!STATUS") != -1) {
    // Status request command
    Serial.println("BLE status request received");
    // Send back robot status
    
  } else {
    Serial.println("Unknown BLE command: " + message);
  }
}

void handle_MSP432_command(const String& message) {
  Serial.println("Processing MSP432 command: " + message);
  
  // Add your MSP432 command processing logic here
  // For example, parse navigation commands, sensor data, etc.
}




void serialEvent5() {
  if (BLE_Serial.available()) {
    String message = BLE_Serial.readStringUntil('!');

    Serial.print("Received from BLE: ");
    Serial.println(message);

    // Add BLE command to function pointer queue
    enqueue_function(handle_BLE_command, message);
  }
}

/**
 * @brief This event is linked to the hardware serial port 7 (hooked up to the
 *    MSP432).
 * 
 */
void serialEvent7() {

  if (MSP432_Serial.available()) {

    String message = MSP432_Serial.readStringUntil('\n');

    Serial.print("Received from MSP432: ");
    Serial.println(message);

    // Add MSP432 command to function pointer queue
    enqueue_function(handle_MSP432_command, message);
  }
}



void setup() {

  // matrix_examples();

  // instantiate Serial port -------------------------------------------------
  Serial.begin(115200);
  
  Serial.println("Starting...");

  // MSP432 --> T4.1
  MSP432_Serial.begin(9600);
  // while (!MSP432_Serial);

  // T4.1 --> BLE UART Friend
  initialize_BLE_UART();

  Serial.println("Setup complete. Waiting for messages...");

}




void loop() {
  // Process queued function calls from serial events
  process_function_queue();
  
  // Small delay to prevent overwhelming the processor
  delay(10);
}
