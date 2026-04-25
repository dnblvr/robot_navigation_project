/**
 * @file imu_mag_cal.h
 * @author your name (you@domain.com)
 * @brief IMU and magnetometer calibration header file
 * @version 0.1
 * @date 2026-04-23
 * 
 * @copyright MIT license or whatever I see fit at the time of publication lmao
 * 
 */

#include <Arduino.h>
#include <Wire.h>

#include <ICM_20948.h>
#include <RPLiDAR_C1.h>
#include <inEKF_se2.h>
#include <arm_math.h>


/* ----------------------------------------------------------------------------
 * interrupt functions
 */

volatile uint8_t  task_flag     = 0;

/**
 * @brief cooperative task scheduling flag for data-ready ISR
 */
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
    
}


// #define UNCALIBRATED_OUTPUT
// #define CALIBRATED_MOTIONCAL_OUTPUT
// #define CALIBRATED_OUTPUT
#define FXP_OUTPUT

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

    
    // Initialize the 2D LiDAR scanner for any magnetic interference it may
    // cause to the IMU
    Serial.println("Initializing RPLiDAR C1...");
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

}

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


    #ifdef UNCALIBRATED_OUTPUT

        int32_t ax = 0, ay = 0, az = 0,
                gx = 0, gy = 0, gz = 0,
                mx = 0, my = 0, mz = 0; 

        float cal_mx = 0, cal_my = 0, cal_mz = 0;

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
        float S[TOTAL] = { 1.041f,  -0.009f,  -0.0005f,
                          -0.009f,	 0.985f,   0.0245f,
                          -0.0005f,	 0.0245f,  0.9765f};

        float hard_offset_x = -126.715f;
        float hard_offset_y =  -52.53f;
        float hard_offset_z =   89.245f;

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
        float S[TOTAL] = { 1.041f,  -0.009f,  -0.0005f,
                          -0.009f,	 0.985f,   0.0245f,
                          -0.0005f,	 0.0245f,  0.9765f};

        float hard_offset_x = -126.715f;
        float hard_offset_y =  -52.53f;
        float hard_offset_z =   89.245f;

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


        Serial.printf("Raw:%.f,%.f,%.f,%.f,%.f,%.f,%.f,%.f,%.f\r\n",
                      ax, ay, az,
                      gx, gy, gz,
                      cal_mx, cal_my, cal_mz);

    #endif

    #ifdef FXP_OUTPUT


        int16_t ax = 0, ay = 0, az = 0,
                gx = 0, gy = 0, gz = 0,
                mx = 0, my = 0, mz = 0; 

        int32_t cal_mx = 0, cal_my = 0, cal_mz = 0;

        // matrix that reshapes the raw elliptical model sensor data into the spherical model data expected in typical systems. these values were computed from MotionCal.
        // to get Q1.15, multiply each element by 32768 (2^15) and round to the nearest integer
        const int32_t S[TOTAL] = { 
                34111,	 -295,	  -16,  //  1.041f,  -0.009f,  -0.0005f,
                 -295,	32276,	  803,  // -0.009f,   0.985f,   0.0245f,
                  -16,	  803,	31998}; // -0.0005f,  0.0245f,  0.9765f};


        // hard iron offsets in counts (0.15 uT/count)
        int16_t hard_offset_x = -845;   // -126.715f;
        int16_t hard_offset_y = -350;   //  -52.53f;
        int16_t hard_offset_z =  595;   //   89.245f;

        // start by converting to Q16.0; mx/my/mz are in 0.1 uT/count (counts)
        mx  = (int16_t)(data.mag.x); 
        my  = (int16_t)(data.mag.y);
        mz  = (int16_t)(data.mag.z);

        // subtract hard iron offsets; in counts
        mx -= hard_offset_x;
        my -= hard_offset_y;
        mz -= hard_offset_z;

        // scaling mag counts to microTeslas (uT) since AK09916 has a magnetic sensor sensitivity of 0.15 uT per LSb.
        // mag counts (Q16.0) times 0.15 uT/count (Q1.15), shifted 7, gives Q8.8 mag values uT
        mx  = (int16_t)( ((int32_t)mx * 4915) >> 7 );
        my  = (int16_t)( ((int32_t)my * 4915) >> 7 );
        mz  = (int16_t)( ((int32_t)mz * 4915) >> 7 ); // in Q8.8 microteslas

        // matrix transformation to apply factory calibration to raw mag data,
        // reshaping the ellipsoid model for data gathering into a sphere
        // note: S is in int32_t Q1.15 format, so we can multiply directly
        // Q8.8 * Q1.15 = Q9.23 in int32
        cal_mx  = mx * S[0] + my * S[1] + mz * S[2];
        cal_my  = mx * S[3] + my * S[4] + mz * S[5];
        cal_mz  = mx * S[6] + my * S[7] + mz * S[8];

        // shift 15 for Q8.8 fixed point representation
        cal_mx  = cal_mx >> 15; 
        cal_my  = cal_my >> 15;
        cal_mz  = cal_mz >> 15;

        // @note: print formatting of int64_t causes problems because of stack misalignment on ARM
        // with switching to Q8.8, this is no longer a problem
        Serial.printf("Raw:%i,%i,%i,%i,%i,%i,%i,%i,%i\r\n",
                      ax, ay, az,
                      gx, gy, gz,
                      
                      // printing the whole numbers of Q8.8
                      cal_mx >> 8, cal_my >> 8, cal_mz >> 8);

    #endif

        // indicate data read complete
        digitalWrite(LED_BUILTIN, LOW);
        
    }

}
