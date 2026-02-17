#include <Arduino.h>
#include <Wire.h>
#include <iostream>
#include <string>

#include <FunctionQueue.h>
#include <ICM_20948.h>
#include <helper_3dmath.h>
// #include <MadgwickGDA.h>
#include <MadgwickECF.h>
// #include "matrix_examples.h"



// #define DEBUG_OUTPUT
// #define DEBUG_FINAL_OUTPUT
#define VISUALIZE_ORIENTATION

#define DATA_COLLECTION_MODE



#define BLE_Serial      Serial5
#define MSP432_Serial   Serial7


#define RTS_Pin 22
#define SDA_PIN 18
#define SCL_PIN 19

// #define CALIBRATION_MODE 1

/* ----------------------------------------------------------------------------
 * interrupt functions
 */

volatile uint8_t  task_flag     = 0;
volatile uint32_t num_counts    = 0;

#define DATA_READY_FLAG 0x01

/**
 * @brief user-defined macro to wait for interrupt Assembly instruction 
 * 
 * @note This macro wraps the assembly instruction "wfi" to improve code
 *  readability.
 */
#define WaitForInterrupt()  asm("wfi")

/**
 * @brief data-ready interrupt service routine
 * @note Sets a flag to indicate it's time to read the sensors
 */
void Data_Ready_ISR() {

    task_flag  |= DATA_READY_FLAG;

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


#define DELTA_T 0.01f // 10 ms sample rate


// orientation/motion vars
Quaternion q_imu;       // [w, x, y, z] quaternion container
VectorFloat aa;         // [x, y, z]    accel sensor measurements
VectorFloat av;         // [x, y, z]    angular velocity measurements
VectorFloat gravity;    // [x, y, z]    gravity vector
VectorInt16 aaReal;     // [x, y, z]    gravity-free accel sensor
VectorInt16 aaWorld;    // [x, y, z]    world-frame accel sensor measurements


// Madgwick_Filter orientation_filter(100 /*used to be 360*/, 14, 17.f, DELTA_T);
ExtendedComplementaryFilter orientation_filter(
        0.5f,   // K_norm,
        3.f,    // t_norm,
        15.f,   // K_init,
        (float)DELTA_T);  // dt


/**
 * @brief 
 */
#define MAX_BUFFER_SAMPLES 100

dataframe_t data_buffer[MAX_BUFFER_SAMPLES];

uint32_t buffer_count = 1;

// macro to check if a pin is disconnected (pulled high)
#define CONNECTED(pin) (digitalRead(pin) == HIGH)

// function declarations ------------------------------------------------------

/**
 * @brief 
 * @todo implement this on a timer interrupt
 */
void initialize_BLE_UART();

// Command handler functions
void handle_BLE_command(const String& message);
void handle_MSP432_command(const String& message);



void record_data_task();


// ----------------------------------------------------------------------------
// 
//  SETUP & LOOP
// 
// ----------------------------------------------------------------------------

void setup() {

    // matrix_examples();

    // instantiate Serial port ------------------------------------------------
    Serial.begin(115200);

    // allows time for the Serial Monitor and/or logic analyzer to open up
    // note: should be removed for production code
    while (!Serial);
    
    Serial.println("Starting...");

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

    // ICM-20948 initialization
    if (icm20948_init(&icm_config,
                       &ak_config,
                       (void*)&Serial) != 0)
    {
        Serial.println("ICM-20948 initialization failed!");
        while (1);
    }


    // configure interrupt pin from ICM20948 ----------------------------------
    // pinMode(22, INPUT_PULLUP); // INT pin from ICM20948
    pinMode(22, INPUT); // INT pin from ICM20948

    attachInterrupt(digitalPinToInterrupt(22),
                    Data_Ready_ISR,
                    FALLING);

    Serial.println("Setup complete. Waiting for messages...");
    
    // deactivate built-in LED to indicate setup complete
    digitalWrite(LED_BUILTIN, LOW);


    // setup debug printing via removable jumpers -----------------------------
    #ifdef DEBUG_OUTPUT

    pinMode(4, INPUT_PULLDOWN); // accel/gyro/mag output enable
    pinMode(5, INPUT_PULLDOWN); // quaternion output enable

    Serial.println("DEBUG OUTPUT ENABLED");
    #endif

    

    // q_imu = {1.0f, 0.0f, 0.0f, 0.0f};
    q_imu = Quaternion(1.0f, 0.0f, 0.0f, 0.0f);

    data_buffer[0] = {
            .accel  = {0, 0, 0},
            .gyro   = {0, 0, 0},
            .temp   =  0,
            .mag    = {0, 0, 0},
            .q      = Quaternion(1.0f, 0.0f, 0.0f, 0.0f),
            .counts =  0};

    

#ifdef CALIBRATION_MODE
    Serial.println("CALIBRATION MODE ENABLED");

    int16_t gyro[3] = {0};

    icm20948_cal_gyro(&icm_config, gyro);

    Serial.printf("gyro = {%10i, %10i, %10i}\n",
                  gyro[0], gyro[1], gyro[2]);

    // results from multiple runs:
    // 200 iters
    // gyro = [        74,         82,        -48]
    // gyro = [        74,         85,        -46]
    
    // 400 iters
    // gyro = [        75,         79,        -50]

#endif 

}



void loop() {

#ifndef CALIBRATION_MODE

    // Serial.printf("Loop count: %lu\n", num_counts);

    // Wait for interrupt
    WaitForInterrupt();

    // if the task flag is not set, return and do not perform any tasks
    if (task_flag & DATA_READY_FLAG) {

        // clear the task flag
        task_flag  &= ~DATA_READY_FLAG;

        // indicate data read in progress
        digitalWrite(LED_BUILTIN, HIGH);

        record_data_task();

        // indicate data read complete
        digitalWrite(LED_BUILTIN, LOW);
        
    }

    // Process queued function calls from serial events
    // process_function_queue();

#endif

}

#define ICM20948_OK 0x00

void record_data_task() {

    // Serial.printf("Recording data sample %lu\n", num_counts);

    dataframe_t* n   = &data_buffer[(buffer_count - 1) % MAX_BUFFER_SAMPLES];
    dataframe_t* np1 = &data_buffer[(buffer_count + 0) % MAX_BUFFER_SAMPLES];

    uint8_t status = ICM20948_OK;


    // read sensor data
    status |= icm20948_record_data(&icm_config, np1);
    status |= ak09916_record_data(&ak_config, np1);


    // increment sample count regardless of status
    np1->counts = num_counts++;

    // check for read errors
    if (status) {
        #ifdef DEBUG_OUTPUT
        Serial.printf("ICM-20948 read failed w/ status: 0x%02X\n",
                      status);
        #endif
        return;
    }

    np1->accel.x   /= 16384.0f;
    np1->accel.y   /= 16384.0f;
    np1->accel.z   /= 16384.0f;

    np1->gyro.x    /= 131.0f;
    np1->gyro.y    /= 131.0f;
    np1->gyro.z    /= 131.0f;

        
    // orientation_filter.update(&aa, &magS, &av, &q_imu);
    orientation_filter.update(np1, n);

    Get_Gravity(&(np1->q), &gravity);
    // Get_Linear_Accel(&aa, &gravity, &aaReal);
    // Get_World_Accel(&aaReal, &q_imu, &aaWorld);
    

#ifdef DEBUG_FINAL_OUTPUT
    // for slow debugging output
    if ((np1->counts % 30) == 0) {

        if (CONNECTED(4)) {

            // integer based printing
            // Serial.printf("accel = {%5i, %5i, %5i}\t",
            //             np1->accel.x, np1->accel.y, np1->accel.z);

            // Serial.printf("gyro = {%5i, %5i, %5i}\t",
            //             np1->gyro.x, np1->gyro.y, np1->gyro.z);

            // Serial.printf("mag = {%5i, %5i, %5i}\n",
            //             np1->mag.x, np1->mag.y, np1->mag.z);

            // float based printing
            Serial.printf("accel = {%5.2f, %5.2f, %5.2f}\t",
                        np1->accel.x, np1->accel.y, np1->accel.z);

            Serial.printf("gyro = {%5.2f, %5.2f, %5.2f}\t",
                        np1->gyro.x, np1->gyro.y, np1->gyro.z);

            Serial.printf("mag = {%5.2f, %5.2f, %5.2f}\n",
                        np1->mag.x, np1->mag.y, np1->mag.z);
        }

        if (CONNECTED(5)) {

            // quaternion output

            Serial.printf("%5u: q = ", np1->counts);
            np1->q.print(4);
            Serial.println();

        }

    }
#endif // DEBUG_FINAL_OUTPUT

#ifdef VISUALIZE_ORIENTATION
    if ((np1->counts % 3) == 0) {

        // Send quaternion as primary orientation data (smooth, no gimbal lock)
        // Format: Q,w,x,y,z
        Serial.printf("Q,%.6f,%.6f,%.6f,%.6f\n", np1->q.w, np1->q.x, np1->q.y, np1->q.z);
        
        // Optional: Send Euler angles for debugging/display purposes
        // Uncomment if you want to see Euler angles in the HUD
        // float euler[3];
        // Get_Euler(&q_imu, euler);
        // Serial.printf("E,%.2f,%.2f,%.2f\n", 
        //               euler[0] * 180.0f/PI,  // psi (yaw)
        //               euler[1] * 180.0f/PI,  // theta (pitch)
        //               euler[2] * 180.0f/PI); // phi (roll)
        
        // Format: G,x,y,z for gravity vector
        // Serial.printf("G,%.6f,%.6f,%.6f\n", gravity.x, gravity.y, gravity.z);
        
        // Format: M,x,y,z for magnetometer vector
        // Serial.printf("M,%.3f,%.3f,%.3f\n", np1->mag.x, np1->mag.y, np1->mag.z);
        
        // Format: A,x,y,z for world-frame acceleration
        // Serial.printf("A,%.6f,%.6f,%.6f\n", aaWorld.x, aaWorld.y, aaWorld.z);
    }

#endif

    // counter for data buffer
    buffer_count++;
}


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

