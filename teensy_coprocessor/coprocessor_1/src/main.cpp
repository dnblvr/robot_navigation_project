#include <Arduino.h>
#include <Wire.h>
#include <iostream>
#include <string>

#include <FunctionQueue.h>
#include <ICM_20948.h>
#include <helper_3dmath.h>
#include <MadgwickFilter.h>
// #include "matrix_examples.h"



// #define DEBUG_OUTPUT
#define DEBUG_FINAL_OUTPUT
// #define VISUALIZE_EULER_ANGLES



#define BLE_Serial      Serial5
#define MSP432_Serial   Serial7

#define USE_GRAVITY     1
#define USE_MAG_NORTH   1


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
Quaternion imu_q;       // [w, x, y, z] quaternion container
VectorInt16 aa;         // [x, y, z]    accel sensor measurements
VectorFloat gravity;    // [x, y, z]    gravity vector
VectorInt16 aaReal;     // [x, y, z]    gravity-free accel sensor
VectorInt16 aaWorld;    // [x, y, z]    world-frame accel sensor measurements
VectorFloat rawAV;      // [x, y, z]    raw angular velocity measurements


Madgwick_Filter orientation_filter(100 /*used to be 360*/, 14, 17.f, DELTA_T);


/**
 * @brief 
 */
#define MAX_BUFFER_SAMPLES 100

dataframe_t data_buffer[MAX_BUFFER_SAMPLES];


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
    
    digitalWrite(LED_BUILTIN, LOW);

    // imu_q = {1.0f, 0.0f, 0.0f, 0.0f};
    imu_q = Quaternion(1.0f, 0.0f, 0.0f, 0.0f);

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

void record_data_task() {

#define ICM20948_OK 0x00

    uint8_t status = ICM20948_OK;

    dataframe_t icm_data = {
            .accel  = {0},
            .gyro   = {0},
            .temp   =  0,
            .mag    = {0},
            .counts =  0};

    // read sensor data
    status |= icm20948_record_data(&icm_config, &icm_data);
    status |= ak09916_record_data(&ak_config, &icm_data);


    // increment sample count regardless of status
    icm_data.counts = num_counts++;

    // check for read errors
    if (status) {
        #ifdef DEBUG_OUTPUT
        Serial.printf("ICM-20948 read failed w/ status: 0x%02X\n",
                      status);
        #endif

        return;
    }

    VectorFloat magS;

    magS.x = (float)icm_data.mag.x;
    magS.y = (float)icm_data.mag.y;
    magS.z = (float)icm_data.mag.z;

    aa.x = icm_data.accel.x;
    aa.y = icm_data.accel.y;
    aa.z = icm_data.accel.z;

    rawAV.x = icm_data.gyro.x / 131.0f;
    rawAV.y = icm_data.gyro.y / 131.0f;
    rawAV.z = icm_data.gyro.z / 131.0f;

        
    orientation_filter.update(&aa, &magS, &imu_q, &rawAV);
        
    Get_Gravity(&imu_q, &gravity);
    Get_Linear_Accel(&aa, &gravity, &aaReal);
    Get_World_Accel(&aaReal, &imu_q, &aaWorld);
    

#ifdef DEBUG_FINAL_OUTPUT
    // for slow debugging output
    if ((icm_data.counts % 30) == 0) {

        // Serial.printf("accel = {%5i, %5i, %5i}\t",
        //             icm_data.accel.x, icm_data.accel.y, icm_data.accel.z);

        // Serial.printf("gyro = {%5i, %5i, %5i}\t",
        //             icm_data.gyro.x, icm_data.gyro.y, icm_data.gyro.z);

        // Serial.printf("mag = {%5i, %5i, %5i}\n",
        //             icm_data.mag.x, icm_data.mag.y, icm_data.mag.z);

        Serial.printf("%5u: q = ", icm_data.counts);
        imu_q.print(4);
        Serial.println();

    }
#endif // DEBUG_FINAL_OUTPUT

#ifdef VISUALIZE_EULER_ANGLES
    if ((icm_data.counts % 3) == 0) {     
        // Convert quaternion to Euler angles and send
        float euler[3];
        Get_Euler(&imu_q, euler);
        
        // Send Euler angles in degrees: E,psi,theta,phi (yaw,pitch,roll)
        Serial.printf("E,%.2f,%.2f,%.2f\n", 
                      euler[0] * 180.0f/PI,     // psi (yaw)
                      euler[1] * 180.0f/PI,     // theta (pitch)
                      euler[2] * 180.0f/PI);    // phi (roll)
        
        // Format: G,x,y,z for gravity vector
        Serial.printf("G,%.6f,%.6f,%.6f\n", gravity.x, gravity.y, gravity.z);
        
        // Format: M,x,y,z for magnetometer vector
        Serial.printf("M,%.3f,%.3f,%.3f\n", magS.x, magS.y, magS.z);
        
        // Format: A,x,y,z for world-frame acceleration
        Serial.printf("A,%.6f,%.6f,%.6f\n", aaWorld.x, aaWorld.y, aaWorld.z);
    }

#endif
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

