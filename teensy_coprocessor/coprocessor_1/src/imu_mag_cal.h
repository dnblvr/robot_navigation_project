/**
 * @file imu_mag_cal.h
 * @author your name (you@domain.com)
 * @brief IMU and magnetometer calibration header file
 * @version 0.1
 * @date 2026-04-23
 * 
 * @copyright Copyright (c) 2026
 * 
 */

#include <Arduino.h>
#include <Wire.h>

#include <ICM_20948.h>
#include <RPLiDAR_C1.h>
#include <inEKF_se2.h>


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

    static uint32_t internal_count = 0;

    internal_count++;

    // this makes sure that the data recording cooperative task runs at 10 Hz, which seems to be a reasonable rate for the MotionCal calibration tool.
    if (internal_count >= 10) {
        internal_count = 0;
        task_flag  |= DATA_READY_FLAG;
    }
    
    return;

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
    .i2c_address        = ICM20948_ADDR_ACCEL_GYRO_1,
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
// ExtendedComplementaryFilter orientation_filter(
//         0.5f,   // K_norm,
//         3.f,    // t_norm,
//         15.f,   // K_init,
//         (float)DELTA_T);  // dt


/**
 * @brief 
 */
#define MAX_BUFFER_SAMPLES 100

dataframe_t data_buffer[MAX_BUFFER_SAMPLES];

uint32_t buffer_count = 1;

// macro to check if a pin is disconnected (pulled high)
#define CONNECTED(pin) (digitalRead(pin) == HIGH)

// function declarations ------------------------------------------------------

// ----------------------------------------------------------------------------
//
//  Module-level state
//
// ----------------------------------------------------------------------------

/**
 * @brief FSM and buffer state for the RPLiDAR C1.
 *        Passed by pointer to Initialize_RPLiDAR_C1() and consulted by
 *        the application to detect PROCESSING frames.
 */
static C1_States rplidar_cfg;

#define RPLIDAR_Serial Serial1

void record_data_task();


// ----------------------------------------------------------------------------
// 
//  SETUP & LOOP
// 
// ----------------------------------------------------------------------------

void setup() {


    // instantiate Serial port ------------------------------------------------
    Serial.begin(115200);

    // allows time for the Serial Monitor and/or logic analyzer to open up
    // note: should be removed for production code
    while (!Serial);
    
    Serial.println("Starting...");

    // activate built-in LED to indicate setup in progress
    pinMode(LED_BUILTIN, OUTPUT);
    digitalWrite(LED_BUILTIN, HIGH);



    // Bind Serial1 to the RPLiDAR driver ---------------------------------
    RPLiDAR_UART_SetPort(&RPLIDAR_Serial);

    
    // Initialize scanner:
    //  - Configure_RPLiDAR_Struct(&rplidar_cfg)
    //  - RPLiDAR_UART_Init()   --> Serial1.begin(460800)
    //  - STOP --> RESET --> GET_HEALTH --> SCAN
    Serial.println("[1/3] Initializing RPLiDAR C1...");
    Initialize_RPLiDAR_C1(&rplidar_cfg);

    

    // I2C initialization -----------------------------------------------------
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

    // pinMode(4, INPUT_PULLDOWN); // accel/gyro/mag output enable
    // pinMode(5, INPUT_PULLDOWN); // quaternion output enable

    Serial.println("DEBUG OUTPUT ENABLED");
    #endif

    

    // q_imu = {1.0f, 0.0f, 0.0f, 0.0f};
    q_imu = Quaternion(1.0f, 0.0f, 0.0f, 0.0f);

    data_buffer[0] = {
            .accel  = {0, 0, 0},
            .gyro   = {0, 0, 0},
            .temp     =  0,
            .mag    = {0, 0, 0},
            .q        = Quaternion(1.0f, 0.0f, 0.0f, 0.0f),
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


    Serial.println("out setup()");

}


#define ICM20948_OK 0x00

void loop() {
    
    // Serial.printf("Loop count: %lu\n", num_counts);


    // Wait for interrupt
    WaitForInterrupt();

    // if the task flag is not set, return and do not perform any tasks
    if (task_flag & DATA_READY_FLAG) {

        // clear the task flag
        task_flag  &= ~DATA_READY_FLAG;

        // indicate data read in progress
        digitalWrite(LED_BUILTIN, HIGH);

        dataframe_t data;

        uint8_t status = ICM20948_OK;

        
        status |= icm20948_record_data(&icm_config, &data);
        status |= ak09916_record_data(&ak_config, &data);

        if (status) {
            return;
        }

        // #define UNCALIBRATED_OUTPUT
        // #define CALIBRATED_MOTIONCAL_OUTPUT
        // #define CALIBRATED_OUTPUT
        #define FXP_OUTPUT


        #ifdef UNCALIBRATED_OUTPUT

        int32_t ax = 0, ay = 0, az = 0,
                gx = 0, gy = 0, gz = 0,
                mx = 0, my = 0, mz = 0; 

        float cal_mx = 0, cal_my = 0, cal_mz = 0;

        // matrix that reshapes the raw elliptical model sensor data into the spherical model data expected in typical systems. this was made from tools like MotionCal, and is used to apply the factory calibration to the raw sensor data.
        float S[TOTAL] = { 1.055f,   -0.00325f, -0.0035f,
                          -0.00325f,  0.96625f,	 0.008f,
                          -0.0035f,	  0.008f,	 0.9815f};

        float hard_offset_x = -124.3925f;
        float hard_offset_y =  -66.3025f;
        float hard_offset_z =   57.36f;

        // scaling mag since AK09916 has a magnetic sensor sensitivity of 0.15 uT per LSB, and we want to work with integer values in microteslas (uT)
        mx = (int32_t)(data.mag.x * 1.5f);
        my = (int32_t)(data.mag.y * 1.5f);
        mz = (int32_t)(data.mag.z * 1.5f);

        Serial.printf("Raw:%i,%i,%i,%i,%i,%i,%i,%i,%i\r\n",
                      ax, ay, az,
                      gx, gy, gz,
                      mx, my, mz);

        #endif
        
        #ifdef CALIBRATED_MOTIONCAL_OUTPUT

        float   ax = 0, ay = 0, az = 0,
                gx = 0, gy = 0, gz = 0,
                mx = 0, my = 0, mz = 0; 

        float cal_mx = 0, cal_my = 0, cal_mz = 0;

        // matrix that reshapes the raw elliptical model sensor data into the spherical model data expected in typical systems. this was made from tools like MotionCal, and is used to apply the factory calibration to the raw sensor data.
        float S[TOTAL] = { 1.055f,   -0.00325f, -0.0035f,
                          -0.00325f,  0.96625f,	 0.008f,
                          -0.0035f,	  0.008f,	 0.9815f};

        float hard_offset_x = -124.3925f;
        float hard_offset_y =  -66.3025f;
        float hard_offset_z =   57.36f;

        // scaling mag since AK09916 has a magnetic sensor sensitivity of 0.15 uT per LSB, and we want to work with integer values in microteslas (uT)
        mx  = data.mag.x * 1.5f;
        my  = data.mag.y * 1.5f;
        mz  = data.mag.z * 1.5f; // 1. in counts

        // MotionCal reports hard iron in uT; mx/my/mz are in 0.1 uT/count, so
        // multiply offsets by 10 to match units before subtracting.
        mx -= (hard_offset_x * 10.0f);
        my -= (hard_offset_y * 10.0f);
        mz -= (hard_offset_z * 10.0f); // 2. still in counts

        // matrix transformation to apply factory calibration to raw mag data, reshaping the ellipsoid model into a sphere
        cal_mx  = mx*S[0] + my*S[1] + mz*S[2];
        cal_my  = mx*S[3] + my*S[4] + mz*S[5];
        cal_mz  = mx*S[6] + my*S[7] + mz*S[8]; // 3. still in counts


        Serial.printf("Raw:%.f,%.f,%.f,%.f,%.f,%.f,%.f,%.f,%.f\r\n",
                      ax, ay, az,
                      gx, gy, gz,
                      cal_mx, cal_my, cal_mz);

        #endif
        
        #ifdef CALIBRATED_OUTPUT

        float   ax = 0, ay = 0, az = 0,
                gx = 0, gy = 0, gz = 0,
                mx = 0, my = 0, mz = 0; 

        float cal_mx = 0, cal_my = 0, cal_mz = 0;

        // matrix that reshapes the raw elliptical model sensor data into the spherical model data expected in typical systems. this was made from tools like MotionCal, and is used to apply the factory calibration to the raw sensor data.
        float S[TOTAL] = { 1.055f,   -0.00325f, -0.0035f,
                          -0.00325f,  0.96625f,	 0.008f,
                          -0.0035f,	  0.008f,	 0.9815f};

        float hard_offset_x = -124.3925f;
        float hard_offset_y =  -66.3025f;
        float hard_offset_z =   57.36f;

        // scaling mag since AK09916 has a magnetic sensor sensitivity of 0.15 uT per LSB, and we want to work with integer values in microteslas (uT)
        mx  = data.mag.x * 0.15f;
        my  = data.mag.y * 0.15f;
        mz  = data.mag.z * 0.15f; // 1. in microteslas

        // MotionCal reports hard iron in uT; mx/my/mz are in 0.1 uT/count, so
        // multiply offsets by 10 to match units before subtracting.
        mx -= (hard_offset_x * 1.0f);
        my -= (hard_offset_y * 1.0f);
        mz -= (hard_offset_z * 1.0f); // 2. still in microteslas

        // matrix transformation to apply factory calibration to raw mag data, reshaping the ellipsoid model into a sphere
        cal_mx  = mx*S[0] + my*S[1] + mz*S[2];
        cal_my  = mx*S[3] + my*S[4] + mz*S[5];
        cal_mz  = mx*S[6] + my*S[7] + mz*S[8]; // 3. still in microteslas


        Serial.printf("Raw:%i,%i,%i,%i,%i,%i,%i,%i,%i\r\n",
                ax, ay, az,
                gx, gy, gz,
                cal_mx, cal_my, cal_mz);

        #endif

        // indicate data read complete
        digitalWrite(LED_BUILTIN, LOW);
        
    }

}


