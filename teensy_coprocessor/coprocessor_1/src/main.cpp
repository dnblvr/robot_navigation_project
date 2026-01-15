#include <Arduino.h>
#include <Wire.h>
#include <iostream>
#include <string.h>
#include <FunctionQueue.h>
#include <ICM_20948.h>
// #include "matrix_examples.h"
#include <IntervalTimer.h>

// #include <avr/io.h>
// #include <avr/interrupt.h>

#define DEBUG_OUTPUT



#define BLE_Serial      Serial5
#define MSP432_Serial   Serial7

#define USE_GRAVITY     1
#define USE_MAG_NORTH   1


#define RTS_Pin 22
#define SDA_PIN 18
#define SCL_PIN 19

// #define CALIBRATION_MODE 1



IntervalTimer     update_timer;
volatile uint8_t  task_flag = 0;
volatile uint32_t num_counts   = 0;

#define TASK_1_FLAG 0x01

/**
 * @brief Timer interrupt service routine
 * @note Sets a flag to indicate it's time to read the sensors
 */
void timer_ISR() {

    task_flag  |= TASK_1_FLAG;

    num_counts++;

}


// ICM-20948 configuration structures ----------------------------------------

icm_data_t icm_offsets = {
    .accel  = {0},
    .gyro   = {74, 85, -46},
    .temp   =  0,
    .counts =  0
};

ak_data_t ak_offsets = {
    .mag    = {0},
    .counts =  0
};

// ICM
sensor_config_t icm_config = {
    .i2c_address        = ICM20948_ADDR_ACCEL_GYRO_0,
    .wire_instance      = (void*) &Wire,
    .offset_instance    = (void*) &icm_offsets
};

// AK09916_
sensor_config_t ak_config = {
    .i2c_address        = ICM20948_ADDR_MAG,
    .wire_instance      = (void*) &Wire,
    .offset_instance    = (void*) &ak_offsets
};


// function declarations ------------------------------------------------------

/**
 * @brief 
 * @todo implement this on a timer interrupt
 */
void initialize_BLE_UART();

// Command handler functions
void handle_BLE_command(const String& message);
void handle_MSP432_command(const String& message);


// ----------------------------------------------------------------------------
// 
//  SETUP & LOOP
// 
// ----------------------------------------------------------------------------

void setup() {

    // matrix_examples();

    // instantiate Serial port ------------------------------------------------
    Serial.begin(115200);

    // allows time for Serial Monitor to open
    while (!Serial);
    
    // Serial.println("Starting...");

    // activate built-in LED to indicate setup in progress
    pinMode(LED_BUILTIN, OUTPUT);
    digitalWrite(LED_BUILTIN, HIGH);

    // MSP432 --> T4.1
    // MSP432_Serial.begin(9600);
    // while (!MSP432_Serial);

    // T4.1 --> BLE UART Friend
    // initialize_BLE_UART();
    
    // I2C initialization -----------------------------------------------------
    Wire.setSDA(SDA_PIN);
    Wire.setSCL(SCL_PIN);
    Wire.setClock(400000); // 400 kHz I2C
    Wire.begin();

    if (icm20948_init(&icm_config,
                      &ak_config,
                      (void*)&Serial) != 0)
    {
        Serial.println("ICM-20948 initialization failed!");
        while (1);
    }

    // Serial.printf("here 2\n");

    // icm20948_set_mag_rate(&ak_config, 10);


    // configure interrupt pin from ICM20948 ----------------------------------
    // pinMode(22, INPUT_PULLUP); // INT pin from ICM20948
    pinMode(22, INPUT); // INT pin from ICM20948

    attachInterrupt(digitalPinToInterrupt(22),
                    timer_ISR,
                    FALLING);
    
    
    // timer_ISR to run every 1 Hz
    // update_timer.begin(timer_ISR, (uint32_t)1000000);

    Serial.println("Setup complete. Waiting for messages...");
    
    digitalWrite(LED_BUILTIN, LOW);

    

#ifdef CALIBRATION_MODE
    Serial.println("CALIBRATION MODE ENABLED");

    int16_t gyro[3] = {0};

    icm20948_cal_gyro(&icm_config, gyro);

    Serial.printf("gyro = {%10.i, %10.i, %10.i}\n",
                  gyro[0], gyro[1], gyro[2]);

    // results from multiple runs:
    // 200 iters
    // gyro = [        74,         82,        -48]
    // gyro = [        74,         85,        -46]
    
    // 400 iters
    // gyro = [        75,         79,        -50]

#endif 

}


#ifdef CALIBRATION_MODE

void loop() {}

#else


void loop() {

    // Wait for interrupt
    asm("wfi");

    // if the task flag is not set, return and do not perform any tasks
    if (!(task_flag & TASK_1_FLAG))
        return;

    // clear the task flag
    task_flag &= ~TASK_1_FLAG;

    // proceed with the following tasks ---------------------------------------

    // vector_int_t accel = {0}, gyro = {0}, mag = {0};

    icm_data_t icm_data = {
            .accel  = {0},
            .gyro   = {0},
            .temp   =  0,
            .counts =  0};

    ak_data_t ak_data = {
            .mag    = {0},
            .counts =  0};

    // Process queued function calls from serial events
    // process_function_queue();

    // icm20948_read_raw_accel(&icm_config, accel);
    // icm20948_read_raw_gyro(&icm_config, gyro);
    // icm20948_read_raw_mag(&ak_config, mag);

    icm20948_record_data(&icm_config,
                         &icm_data);

    Serial.printf("accel = {%7.i, %7.i, %7.i}\t",
                  icm_data.accel.x, icm_data.accel.y, icm_data.accel.z);

    Serial.printf("gyro = {%7.i, %7.i, %7.i}\n",
                  icm_data.gyro.x, icm_data.gyro.y, icm_data.gyro.z);

    // Serial.printf("mag = {%7.i, %7.i, %7.i}\n",
    //               mag[0], mag[1], mag[2]);
}

#endif


// ----------------------------------------------------------------------------
// 
//  FUNCTION DEFINITIONS
// 
// ----------------------------------------------------------------------------

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

