/**
 * @file ICM_20948.cpp
 * @author your name (you@domain.com)
 * @brief 
 * @version 0.1
 * @date 2025-12-27
 */

#include "ICM_20948.h"
#include <Wire.h>

#define MAX_ITERS   400
#define INT_TYPE    uint32_t
#define DIMS        3

#define CHANGES 1

// #define DEBUG_OUTPUT 1


// ----------------------------------------------------------------------------
//
//  I2C INITIALIZATION FUNCTIONS
//
// ----------------------------------------------------------------------------


/**
 * @brief macro that waits until the LPI2C1 transmit FIFO has space for more
 *  data (less than 4 entries)
 */
#define wait_for_LPI2C1_FIFO_space()  while ((IMXRT_LPI2C1.MFSR & 0x07) >= 4)

/**
 * @brief Sends multiple bytes over I2C via direct LPI2C1 hardware access
 * 
 * @details Optimized Teensy 4.0 implementation which assumes:
 *   - LPI2C1 already initialized by Wire.begin()
 *   - Single-master bus (no arbitration needed)
 *   - FIFO depth = 4 entries (Teensy 4.0 spec)
 * Bloat removed: wait_idle, timeout tracking, yield(), error recovery,
 *                arbitration handling, pin timeout, buffer overflow checks
 * 
 * @param config    sensor configuration structure
 * @param data      data buffer to send
 * @param length    number of bytes to send
 */
void I2C_send_multiple(
        sensor_config_t*    config,
        uint8_t*            data,
        size_t              length)
{

    uint8_t address     = config->i2c_address;

    // Clear all status flags from previous transactions (SDF, NDF, ALF, FEF,
    // PLTF, DMF)
    IMXRT_LPI2C1.MSR    = 0x00007F00;

    // Write START + 7-bit address (shifted left with R/W=0 for write)
    IMXRT_LPI2C1.MTDR   = LPI2C_MTDR_CMD_START | (address << 1);

    // Write data bytes
    for (size_t i = 0; i < length; i++) {

        // wait until FIFO has space i.e. less than 4 bytes
        wait_for_LPI2C1_FIFO_space();

        // Write data byte with TRANSMIT command
        IMXRT_LPI2C1.MTDR   = LPI2C_MTDR_CMD_TRANSMIT | data[i];
    }

    // Ensure FIFO space, then write STOP condition
    wait_for_LPI2C1_FIFO_space();
    IMXRT_LPI2C1.MTDR       = LPI2C_MTDR_CMD_STOP;


    // Check for NACK error (device not responding)
    if (IMXRT_LPI2C1.MSR & LPI2C_MSR_NDF) {

        // Clear FIFO and abort on NACK
        IMXRT_LPI2C1.MCR   |= LPI2C_MCR_RTF | LPI2C_MCR_RRF;
        return;
    }

    // Wait for STOP completion, which ensures transaction finishes before
    // return
    while (!(IMXRT_LPI2C1.MSR & LPI2C_MSR_SDF));

    // Clear STOP flag so next transaction can start cleanly
    IMXRT_LPI2C1.MSR = LPI2C_MSR_SDF;
}


/**
 * @brief Reads multiple bytes over I2C after writing register address; uses 
 *  direct LPI2C1 hardware access
 * 
 * @details Optimized Teensy 4.0 implementation, which assumes:
 *  - LPI2C1 already initialized by Wire.begin()
 *  - Single-master bus (no arbitration needed)
 *  - FIFO depth = 4 entries (Teensy 4.0 spec)
 * Bloat removed: wait_idle, timeout tracking, yield(), error recovery,
 *                arbitration handling, pin timeout, buffer management
 * 
 * @param config    sensor configuration structure
 * @param reg_addr  register address to read from
 * @param data      buffer to store read data
 * @param length    number of bytes to read
 */
void I2C_read_register(
        sensor_config_t*    config,
        uint8_t             reg_addr,
        uint8_t*            data,
        size_t              length)
{

    uint8_t address     = config->i2c_address;

    // Clear all status flags from previous transactions (SDF, NDF, ALF, FEF, PLTF, DMF) by clearing the master status register MSR
    IMXRT_LPI2C1.MSR    = 0x00007F00;

    // Write START + 7-bit address (shifted left with R/W=0 for write)
    IMXRT_LPI2C1.MTDR   = LPI2C_MTDR_CMD_START | (address << 1);

    // Wait for TX FIFO space, then write register address with TRANSMIT command
    wait_for_LPI2C1_FIFO_space();
    IMXRT_LPI2C1.MTDR   = LPI2C_MTDR_CMD_TRANSMIT | reg_addr;

    // Write repeated START + 7-bit address (shifted left with R/W=1 for read)
    wait_for_LPI2C1_FIFO_space();
    IMXRT_LPI2C1.MTDR   = LPI2C_MTDR_CMD_START | (address << 1) | 1;

    // Write RECEIVE command (length - 1)
    wait_for_LPI2C1_FIFO_space();
    IMXRT_LPI2C1.MTDR   = LPI2C_MTDR_CMD_RECEIVE | (length - 1);

    // Write STOP condition
    wait_for_LPI2C1_FIFO_space();
    IMXRT_LPI2C1.MTDR   = LPI2C_MTDR_CMD_STOP;


    // Read data from RX FIFO
    for (size_t i = 0; i < length; i++) {

        // wait until data available
        while (((IMXRT_LPI2C1.MFSR >> 16) & 0x07) == 0);
        
        // Read data byte from receive register
        data[i]     = IMXRT_LPI2C1.MRDR & 0xFF;
    }

    // Check for NACK error (device not responding)
    if (IMXRT_LPI2C1.MSR & LPI2C_MSR_NDF) {

        // Clear FIFO and abort on NACK
        IMXRT_LPI2C1.MCR   |= LPI2C_MCR_RTF | LPI2C_MCR_RRF;
        return;
    }

    // Wait for STOP completion (ensures transaction finishes before return)
    while (!(IMXRT_LPI2C1.MSR & LPI2C_MSR_SDF));

    // Clear STOP flag so next transaction can start cleanly
    IMXRT_LPI2C1.MSR    = LPI2C_MSR_SDF;
}

/**
 * @brief Scans the I2C bus for connected devices and attempts direct register
 *  read
 * 
 * @param wire      Pointer to TwoWire instance
 * @param serial    Pointer to usb_serial_class instance for output
 */
void I2C_scan_bus(
        TwoWire*            wire,
        usb_serial_class*   serial)
{
    // Store found device addresses
    uint8_t buf;


    // Scan I2C bus
    serial->printf("Scanning I2C bus for devices...\n");

    for (uint8_t addr = 0x08; addr < 0x78; addr++) {
        wire->beginTransmission(addr);
        uint8_t error = wire->endTransmission();

        if (error == 0) {
            serial->printf("  Found device at 0x%02X\n", addr);
        }
    }


    // Try reading WHO_AM_I directly without writing register address first
    serial->printf("\nAttempting direct register read without prior write...\n");

    wire->requestFrom((uint8_t)0x68, (uint8_t)1, (bool)true);
    if (wire->available()) {
        buf = wire->read();
        serial->printf("Direct read from 0x68: 0x%02X\n", buf);
    } else {
        serial->printf("No response from direct read\n");
    }
}

// ----------------------------------------------------------------------------
//
//  SENSOR INITIALIZATION FUNCTIONS
//
// ----------------------------------------------------------------------------


static usb_serial_class* local_serial;

int8_t icm20948_init(
        sensor_config_t*    ag_config,
        sensor_config_t*    mag_config,
        void*               serial_usb)
{
    local_serial = (usb_serial_class*)serial_usb;

    uint8_t reg[6], buf;

    // I2C Bus Scan -----------------------------------------------------------
    // I2C_scan_bus(wire, local_serial);
    delay(1);


    // wake up accel/gyro
    // first write register then, write value
    reg[0] = PWR_MGMT_1; reg[1] = 0x00;
    I2C_send_multiple(ag_config, reg, 2);


    /** --------------------------------------------------------
     * switch to user bank to 0
     */
    reg[0] = REG_BANK_SEL; reg[1] = REG_BANK_0;
    I2C_send_multiple(ag_config, reg, 2);


    // auto select clock source
    reg[0] = PWR_MGMT_1; reg[1] = 0x01;
    I2C_send_multiple(ag_config, reg, 2);


    // disable accel/gyro once and allow time to settle
    reg[0] = PWR_MGMT_2; reg[1] = 0x3F;
    I2C_send_multiple(ag_config, reg, 2);
    delay(10);


    // enable accel/gyro again
    reg[0] = PWR_MGMT_2; reg[1] = 0x00;
    I2C_send_multiple(ag_config, reg, 2);


    // check if the accel/gyro can be accessed and give a 1 ms delay
    I2C_read_register(ag_config, WHO_AM_I_ICM20948, &buf, 1);
    delay(1);

    #if DEBUG_OUTPUT
    local_serial->printf("WHO_AM_I value: 0x%02X (expected 0xEA)\n", buf);
    #endif

    if (buf != 0xEA)
        return -1;
    
    delay(20);


    /** --------------------------------------------------------
     * switch to user bank 2 for gyro & accel config
     */
    reg[0] = REG_BANK_SEL; reg[1] = REG_BANK_2;
    I2C_send_multiple(ag_config, reg, 2);


    // gyro config
    // set full scale to +/- 250dps
    // set noise bandwidth to 
    // smaller bandwidth means lower noise level & slower max sample rate
    reg[0] = GYRO_CONFIG_1; reg[1] = 0x29;
    I2C_send_multiple(ag_config, reg, 2);


    // set gyro output data rate to 100Hz
    // output_data_rate = 1.125kHz / (1 + GYRO_SMPLRT_DIV)
    // 1125 / 11 = 100
    reg[0] = GYRO_SMPLRT_DIV; reg[1] = 11;
    I2C_send_multiple(ag_config, reg, 2);

    
    // accel config
    // set full scale to +-2g
    // set noise bandwidth to 136Hz
    reg[0] = ACCEL_CONFIG;     reg[1] = 0x01 | (0x02 << 3);
    I2C_send_multiple(ag_config, reg, 2);
    

    //  - set accel output data rate (ODR) to 100Hz
    //      - 16 bits for ACCEL_SMPLRT_DIV
    //      - output_data_rate = 1.125kHz / (1 + ACCEL_SMPLRT_DIV)
    //      - ACCEL_SMPLRT_DIV = (1.125 kHz / ODR) - 1
    //      -                  = 10.25 for 100Hz
    reg[0] = ACCEL_SMPLRT_DIV_2; reg[1] = 11;
    I2C_send_multiple(ag_config, reg, 2);
    delay(20);

    
    /** --------------------------------------------------------
     * switch back to user bank to 0 for magnetometer configuration
     */
    reg[0] = REG_BANK_SEL;      reg[1] = REG_BANK_0;
    I2C_send_multiple(ag_config, reg, 2);
        
    
    // in the meantime, prepare to wake up mag by bypassing the I2C master interface!
    // (INT_PIN_CFG, BYPASS_EN = 1)
    reg[0] = INT_PIN_CFG;       reg[1] = INT1_BYPASS_EN;
    I2C_send_multiple(ag_config, reg, 2);
    delay(20);


    // magnetometer initialization --------------------------------------------

    // while in `REG_BANK_0`, check if the magnetometer can be accessed and
    // give a 1 ms delay
    I2C_read_register(mag_config, AK09916_WHO_AM_I, &buf, 1);
    delay(1);

    #if DEBUG_OUTPUT
    local_serial->printf("MAG. WHO_AM_I: 0x%X\n", buf);
    #endif

    #define WHO_AM_I_AK09916 0x09

    if (buf != WHO_AM_I_AK09916)
        return -1;


    // config mag
    // set mag mode, to measure continuously in 100Hz
    reg[0] = AK09916_CONTROL_2; reg[1] = CONTINUOUS_100HZ;
    I2C_send_multiple(mag_config, reg, 2);

    
    // interrupt configuration ------------------------------------------------

    // enable data ready interrupt; temporarily disabled for this commit
    reg[0] = INT_ENABLE_1; reg[1] = RAW_DRDY_EN;
    I2C_send_multiple(ag_config, reg, 2);


    // allows I2C peripheral to generate interrupts through the same INT_1 pin
    // reg[0] = INT_ENABLE; reg[1] = I2C_MST_INT_EN;
    // I2C_send_multiple(ag_config, reg, 2);


    // configure interrupt pin: active-low, push-pull, latch until cleared
    reg[0] = INT_PIN_CFG; reg[1] = INT1_ACTL | INT1_BYPASS_EN;
    I2C_send_multiple(ag_config, reg, 2);


    return 0;

}



void icm20948_set_mag_rate(
        sensor_config_t*    mag_config,
        uint8_t             mode)
{
    // Single measurement              : mode = 0
    // Continuous measurement in  10 Hz: mode = 10
    // Continuous measurement in  20 Hz: mode = 20
    // Continuous measurement in  50 Hz: mode = 50
    // Continuous measurement in 100 Hz: mode = 100
    uint8_t reg[2];


    switch (mode) {
    
    // single-shot; after which it transitions to power-down
    case 0:     reg[1] = SINGLE_MODE;
                break;

    case 10:    reg[1] = CONTINUOUS_10HZ;
                break;

    case 20:    reg[1] = CONTINUOUS_20HZ;
                break;

    case 50:    reg[1] = CONTINUOUS_50HZ;
                break;

    case 100:   reg[1] = CONTINUOUS_100HZ;
                break;

    default:
#ifdef DEBUG_OUTPUT
        printf("error at icm20948_set_mag_mode: wrong mode %d\n", mode);
#endif
        return;
    }

    reg[0] = AK09916_CONTROL_2;
    I2C_send_multiple(mag_config, reg, 2);

    return;
}

// void icm20948_cal_mag_simple(
//         sensor_config_t*    mag_config,
//         int16_t             mag_bias[DIMS])
// {
//     // counters
//     INT_TYPE i, j;

//     int16_t buf[DIMS] = {0}, max[DIMS] = {0}, min[DIMS] = {0};

// #ifdef DEBUG_OUTPUT
//     printf("mag calibration: \nswing sensor for 360 deg\n");
// #endif

//     for (i = 0; i < MAX_ITERS; i++) {

//         icm20948_read_raw_mag(mag_config, buf);

//         for (j = 0; j < DIMS; j++) {
//             if (buf[j] > max[j])
//                 max[j] = buf[j];
//             if (buf[j] < min[j])
//                 min[j] = buf[j];
//         }

//         // sleep_ms(10);
//         delay(10);

//     }

//     for (i = 0; i < DIMS; i++)
//         mag_bias[i] = (max[i] + min[i]) / 2;

//     return;
// }


// ----------------------------------------------------------------------------
//
//  READ FUNCTIONS
//
// ----------------------------------------------------------------------------

uint8_t icm20948_record_data(
        sensor_config_t*    icm_config,
        dataframe_t*        data)
{
    #define ICM20948_FAILED 0x01


    // get offsets
    icm_data_t*   offsets       = (icm_data_t*)icm_config->offset_instance;
    vector_int_t* accel_offset  = &(offsets->accel);
    vector_int_t* gyro_offset   = &(offsets->gyro);

    uint8_t buf[14];
    
    // assume that the user bank is already set to 0. then, read accel, gyro,
    // temp in one I2C transaction
    I2C_read_register(icm_config, ACCEL_XOUT_H, buf, 14);

    data->accel.x   = (int16_t)(buf[ 0] << 8 | buf[ 1]);
    data->accel.y   = (int16_t)(buf[ 2] << 8 | buf[ 3]);
    data->accel.z   = (int16_t)(buf[ 4] << 8 | buf[ 5]);

    data->gyro.x    = (int16_t)(buf[ 6] << 8 | buf[ 7]);
    data->gyro.y    = (int16_t)(buf[ 8] << 8 | buf[ 9]);
    data->gyro.z    = (int16_t)(buf[10] << 8 | buf[11]);

    data->temp      = (int16_t)(buf[12] << 8 | buf[13]);

    
    // add offsets
    data->accel.x  -= accel_offset->x;
    data->accel.y  -= accel_offset->y;
    data->accel.z  -= accel_offset->z;

    data->gyro.x   -= gyro_offset->x;
    data->gyro.y   -= gyro_offset->y;
    data->gyro.z   -= gyro_offset->z;

    return 0;
}


uint8_t ak09916_record_data(
        sensor_config_t*    ak_config,
        dataframe_t*        data)
{
    #define AK09916_FAILED 0x02

    // get offsets
    ak_data_t* local_offsets    = (ak_data_t*)ak_config->offset_instance;
    // vector_int_t* mag_offset    = &(local_offsets->mag);

    uint8_t reg[7];

    // read ST1 and check if data is ready
    I2C_read_register(ak_config, AK09916_DATA_STATUS_1, reg, 1);

    if ((reg[0] & AK09916_ST1_DRDY) != AK09916_ST1_DRDY) {
        
        #ifdef DEBUG_OUTPUT
        local_serial->printf("  AK09916_DATA_STATUS_1 = %02X\nST1: Data is NOT ready\n", reg[0]);
        #endif
        return AK09916_FAILED;
    }

    // assume that the user bank is already set to 0. then, read mag in one I2C
    // transaction
    I2C_read_register(ak_config, AK09916_XOUT_L, reg, 7);


    // finish reading by getting ST2
    I2C_read_register(ak_config, AK09916_DATA_STATUS_2, &reg[7], 1);


    // process the data; 
    data->mag.x = (int16_t)(reg[1] << 8 | reg[0]);
    data->mag.y = (int16_t)(reg[3] << 8 | reg[2]);
    data->mag.z = (int16_t)(reg[5] << 8 | reg[4]);

    // check data overflow
    if ((reg[6] & 0x08) == 0x08) {
        
        #ifdef DEBUG_OUTPUT
        local_serial->printf("mag: ST2: Sensor overflow\n");
        #endif

        return AK09916_FAILED;
    }

    // offsets are incorporated later in later commits

    return 0;
}


// void icm20948_read_raw_accel(
//         sensor_config_t*    ag_config,
//         int16_t             accel[DIMS])
// {
//     // counter
//     INT_TYPE i;

//     uint8_t buf[6];

//     // relies on auto-increment of register address to record the high and low 
//     // bytes of each axis in the body-acceleration realm
//     I2C_read_register(ag_config, ACCEL_XOUT_H, buf, 6);

//     for (i = 0; i < DIMS; i++)
//         accel[i] = (int16_t)(buf[2*i] << 8 | buf[2*i + 1]);
    
//     return;
// }

// void icm20948_read_raw_gyro(
//         sensor_config_t*    ag_config,
//         int16_t             gyro[DIMS])
// {
//     // counter
//     INT_TYPE i;

//     uint8_t buf[6];

//     // relies on auto-increment of register address to record the high and low 
//     // bytes of each axis of the body-rotation rate sensor
//     I2C_read_register(ag_config, GYRO_XOUT_H, buf, 6);
    
//     for (i = 0; i < DIMS; i++)
//         gyro[i] = (int16_t)(buf[2*i] << 8 | buf[2*i + 1]);

//     return;
// }

// void icm20948_read_raw_temp(
//         sensor_config_t*    ag_config,
//         int16_t*            temp)
// {

//     uint8_t buf[6];

//     // relies on auto-increment of register address to record the high and low 
//     // bytes of the temperature sensor
//     I2C_read_register(ag_config, TEMP_OUT_H, buf, 2);
    
//     *temp = (buf[0] << 8 | buf[1]);

//     return;
// }

// void icm20948_read_raw_mag(
//         sensor_config_t*    mag_config,
//         int16_t             mag[DIMS])
// {
//     // counter
//     INT_TYPE i;

//     uint8_t buf[8];


//     // read ST1 and check if data is ready
//     I2C_read_register(mag_config, AK09916_DATA_STATUS_1, buf, 1);

//     if ((buf[0] & AK09916_ST1_DRDY) != AK09916_ST1_DRDY) {
        
//         #ifdef DEBUG_OUTPUT
//         local_serial->printf("  AK09916_DATA_STATUS_1 = %02X\nST1: Data is NOT ready\n", buf[0]);
//         #endif

//         return;
//     }

//     // relies on auto-increment of register address to record the high and low 
//     // bytes of each of the three axes of the magnetometer sensor
//     I2C_read_register(mag_config,
//                       AK09916_XOUT_L,
//                       &buf[1],
//                       6);

//     // finish reading by getting ST2
//     I2C_read_register(mag_config,
//                       AK09916_DATA_STATUS_2,
//                       &buf[7],
//                       1);

//     // local_serial->printf("  here mag read: %02X\n", buf[0]);

//     // print whole array for debugging
//     // local_serial->print("[");
//     // for (i = 0; i < 8; i++)
//     //     local_serial->printf("%02X, ", buf[i]);
//     // local_serial->print("]\n");

//     for (i = 0; i < DIMS; i++)
//         mag[i] = (int16_t)(buf[2*i + 2] << 8 | buf[2*i + 1]);

// #ifdef DEBUG_OUTPUT
//     if ((buf[6] & 0x08) == 0x08)
//         local_serial->printf("mag: ST1: Sensor overflow\n");

//     // printf below works only if we read 0x10
//     //if ((buf[0] & 0x01) == 0x01) printf("mag: ST1: Data overrun\n");
//     //if ((buf[0] & 0x02) != 0x02) printf("mag: ST1: Data is NOT ready\n");
// #endif

//     return;
// }

// void icm20948_cal_gyro(
//         sensor_config_t*    ag_config,
//         int16_t             gyro_bias[DIMS])
// {
//     // counters
//     INT_TYPE i, j;
    
//     int16_t buf[DIMS]  = {0};
//     int32_t bias[DIMS] = {0};

//     for (i = 0; i < MAX_ITERS; i++) {

//         icm20948_read_raw_gyro(ag_config, buf);

//         for (j = 0; j < DIMS; j++) {
//             bias[j] += buf[j];
//         }

//         delay(25);
//     }

//     for (i = 0; i < DIMS; i++)
//         gyro_bias[i] = (int16_t)(bias[i] / MAX_ITERS);
// }

// void icm20948_read_cal_gyro(
//         sensor_config_t*    ag_config,
//         int16_t             gyro[DIMS],
//         int16_t             bias[DIMS])
// {
//     // counters
//     INT_TYPE i;

//     icm20948_read_raw_gyro(ag_config, gyro);

//     for (i = 0; i < DIMS; i++)
//         gyro[i] -= bias[i];
// }



// void icm20948_read_cal_accel(
//         sensor_config_t*    ag_config, 
//         int16_t             accel[DIMS],
//         int16_t             bias[DIMS])
// {
//     // counters
//     INT_TYPE i;

//     icm20948_read_raw_accel(ag_config, accel);

//     for (i = 0; i < DIMS; i++)
//         accel[i] -= bias[i];
// }



// void icm20948_read_cal_mag(
//         sensor_config_t*    mag_config, 
//         int16_t             mag[DIMS],
//         int16_t             bias[DIMS])
// {
//     // counters
//     INT_TYPE i;

//     icm20948_read_raw_mag(mag_config, mag);

//     for (i = 0; i < DIMS; i++)
//         mag[i] -= bias[i];
// }

// void icm20948_read_temp_c(
//         sensor_config_t*    ag_config, 
//         float*              temp)
// {
//     int16_t tmp;

//     icm20948_read_raw_temp(ag_config, &tmp);

//     // temp  = ((raw_value - ambient_temp) / speed_of_sound) + 21
//     *temp = (((float)tmp - 21.0f) / 333.87) + 21.0f;
// }

