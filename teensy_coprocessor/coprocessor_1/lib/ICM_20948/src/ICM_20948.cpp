/**
 * @file ICM_20948.c
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

// #define CHANGES 1

#define DEBUG_OUTPUT 1

// ----------------------------------------------------------------------------
//
//  INITIALIZATION FUNCTIONS
//
// ----------------------------------------------------------------------------

void I2C_send_multiple(
        icm20948_config_t*  config,
        uint8_t             target_addr,
        uint8_t*            data,
        size_t              length)
{
    // Get Wire instance from config
    TwoWire* wire = (TwoWire*)config->wire_instance;

    // Send data over I2C (typically register address)
    wire->beginTransmission(target_addr);
    for (size_t i = 0; i < length; i++) {
        wire->write(data[i]);
    }
    wire->endTransmission(true);  // Send STOP condition
}


void I2C_read_register(
        icm20948_config_t*  config,
        uint8_t             target_addr,
        uint8_t             reg_addr,
        uint8_t*            data,
        size_t              length)
{
    // Get Wire instance from config
    TwoWire* wire = (TwoWire*)config->wire_instance;

    // Send register address
    wire->beginTransmission(target_addr);
    wire->write(reg_addr);
    wire->endTransmission(false);  // false = repeated START (no STOP)
    
    // Read response
    wire->requestFrom((uint8_t)target_addr, (uint8_t)length, (uint8_t)true);
    
    for (size_t i = 0; i < length; i++) {
        if (wire->available()) {
            data[i] = wire->read();
        }
    }
}


int8_t icm20948_init(icm20948_config_t* config)
{
    usb_serial_class* serial_usb = (usb_serial_class*)config->serial_usb;
    TwoWire* wire = (TwoWire*)config->wire_instance;

    uint8_t reg[2], buf;

    serial_usb->printf("ICM-20948 Initialization Started...\n");
    serial_usb->printf("Scanning I2C bus for devices...\n");

    // Scan I2C bus
    for (uint8_t addr = 0x08; addr < 0x78; addr++) {
        wire->beginTransmission(addr);
        uint8_t error = wire->endTransmission();

        if (error == 0) {
            serial_usb->printf("  Found device at 0x%02X\n", addr);
        }
    }

    serial_usb->printf("\nAttempting direct register read without prior write...\n");
    
    // Try reading WHO_AM_I directly without writing register address first
    wire->requestFrom((uint8_t)0x68, (uint8_t)1, (uint8_t)true);
    if (wire->available()) {
        buf = wire->read();
        serial_usb->printf("Direct read from 0x68: 0x%02X\n", buf);
    } else {
        serial_usb->printf("No response from direct read\n");
    }

    serial_usb->printf("here 0\n");

    delay(1);

    // wake up accel/gyro
    // first write register then, write value
    reg[0] = PWR_MGMT_1; reg[1] = 0x00;
    I2C_send_multiple(config, config->addr_accel_gyro, reg, 2);


    /** --------------------------------------------------------
     * switch to user bank to 0
     */
    reg[0] = REG_BANK_SEL; reg[1] = 0x00;
    I2C_send_multiple(config, config->addr_accel_gyro, reg, 2);


    // auto select clock source
    reg[0] = PWR_MGMT_1; reg[1] = 0x01;
    I2C_send_multiple(config, config->addr_accel_gyro, reg, 2);


    // disable accel/gyro once and allow time to settle
    reg[0] = PWR_MGMT_2; reg[1] = 0x3F;
    I2C_send_multiple(config, config->addr_accel_gyro, reg, 2);
    delay(10);


    // enable accel/gyro again
    reg[0] = PWR_MGMT_2; reg[1] = 0x00;
    I2C_send_multiple(config, config->addr_accel_gyro, reg, 2);


    serial_usb->printf("here 2\n");

    // check if the accel/gyro can be accessed and give a 1 ms delay
    I2C_read_register(config,
                      config->addr_accel_gyro,
                      WHO_AM_I_ICM20948,
                      &buf,
                      1);

    delay(1);

    #if DEBUG_OUTPUT
        serial_usb->printf("WHO_AM_I value: 0x%02X (expected 0xEA)\n", buf);
    #endif

    if (buf != 0xEA)
        return -1;


    serial_usb->printf("here 3\n");


    /** --------------------------------------------------------
     * switch to user bank 2 for gyro & accel config
     */
    reg[0] = REG_BANK_SEL; reg[1] = 0x20;
    I2C_send_multiple(config, config->addr_accel_gyro, reg, 2);


    // gyro config
    // set full scale to +/- 250dps
    // set noise bandwidth to 
    // smaller bandwidth means lower noise level & slower max sample rate
    reg[0] = GYRO_CONFIG_1; reg[1] = 0x29;
    I2C_send_multiple(config, config->addr_accel_gyro, reg, 2);


    // set gyro output data rate to 100Hz
    // output_data_rate = 1.125kHz / (1 + GYRO_SMPLRT_DIV)
    // 1125 / 11 = 100
    reg[0] = GYRO_SMPLRT_DIV; reg[1] = 0x0A;
    I2C_send_multiple(config, config->addr_accel_gyro, reg, 2);


    serial_usb->printf("here 3\n");

    
    // accel config
    // set full scale to +-2g
    // set noise bandwidth to 136Hz
    reg[0] = ACCEL_CONFIG; reg[1] = 0x11;
    I2C_send_multiple(config, config->addr_accel_gyro, reg, 2);

    

    // set accel output data rate to 100Hz
    //  - output_data_rate = 1.125kHz / (1 + ACCEL_SMPLRT_DIV)
    //  - 16 bits for ACCEL_SMPLRT_DIV
    reg[0] = ACCEL_SMPLRT_DIV_2; reg[1] = 0x0A;
    I2C_send_multiple(config, config->addr_accel_gyro, reg, 2);

    
    /** --------------------------------------------------------
     * switch back to user bank to 0
     */
    reg[0] = REG_BANK_SEL; reg[1] = 0x00;
    I2C_send_multiple(config, config->addr_accel_gyro, reg, 2);

    
    // wake up mag! (INT_PIN_CFG, BYPASS_EN = 1)
    reg[0] = INT_PIN_CFG; reg[1] = 0x02;
    I2C_send_multiple(config, config->addr_accel_gyro, reg, 2);


    // check if the magnetometer can be accessed and give a 1 ms delay
    I2C_read_register(config, config->addr_mag, AK09916_WHO_AM_I, &buf, 1);
    delay(1);

    #if DEBUG_OUTPUT
        serial_usb->printf("MAG. WHO_AM_I: 0x%X\n", buf);
    #endif

    if (buf != 0x09)
        return -1;


    // config mag
    // set mag mode, to measure continuously in 100Hz
    reg[0] = AK09916_CNTL2; reg[1] = 0x08;
    I2C_send_multiple(config, config->addr_mag, reg, 2);

    return 0;

}

void icm20948_set_mag_rate(
        icm20948_config_t*  config,
        uint8_t             mode)
{
    // Single measurement              : mode = 0
    // Continuous measurement in  10 Hz: mode = 10
    // Continuous measurement in  20 Hz: mode = 20
    // Continuous measurement in  50 Hz: mode = 50
    // Continuous measurement in 100 Hz: mode = 100
    uint8_t reg[2];


    switch (mode) {

    // single shot; after measure, transitions to power-down mode automatically
    case 0:
        reg[1] = 0x01;
        break;

    // 10Hz continuous --------------------------------------------------------
    case 10:
        reg[1] = 0x02;
        break;

    // 20Hz continuous --------------------------------------------------------
    case 20:
        reg[1] = 0x04;
        break;

    // 50Hz continuous --------------------------------------------------------
    case 50:
        reg[1] = 0x06;
        break;

    // 100Hz continuous -------------------------------------------------------
    case 100:
        reg[1] = 0x08;
        break;

    default:
#ifdef DEBUG_OUTPUT
        printf("error at icm20948_set_mag_mode: wrong mode %d\n", mode);
#endif
        return;
    }

    reg[0] = AK09916_CNTL2;
    // i2c_write_blocking(config->i2c, config->addr_mag, reg, 2, false);
    I2C_send_multiple(config, config->addr_mag, reg, 2);

    return;
}

void icm20948_cal_mag_simple(
        icm20948_config_t*  config,
        int16_t             mag_bias[DIMS])
        
{
    // counters
    INT_TYPE i, j;

    int16_t buf[DIMS] = {0}, max[DIMS] = {0}, min[DIMS] = {0};

#ifdef DEBUG_OUTPUT
    printf("mag calibration: \nswing sensor for 360 deg\n");
#endif

    for (i = 0; i < MAX_ITERS; i++) {

        icm20948_read_raw_mag(config, buf);

        for (j = 0; j < DIMS; j++) {
            if (buf[j] > max[j])
                max[j] = buf[j];
            if (buf[j] < min[j])
                min[j] = buf[j];
        }

        // sleep_ms(10);
        delay(10);

    }

    for (i = 0; i < DIMS; i++)
        mag_bias[i] = (max[i] + min[i]) / 2;

    return;
}

void icm20948_cal_accel(
        icm20948_config_t*  config,
        int16_t             accel_bias[DIMS])
{
    // counters
    INT_TYPE i, j;

    int16_t buf[DIMS]  = {0};
    int32_t bias[DIMS] = {0};

    for (i = 0; i < MAX_ITERS; i++) {

        icm20948_read_raw_accel(config, buf);

        for (j = 0; j < DIMS; j++) {

            if (j == 2)
                bias[j] += (buf[j] - 16384);
            else
                bias[j] += buf[j];

        }

        // sleep_ms(25);
        delay(25);

    }

    for (i = 0; i < DIMS; i++)
        accel_bias[i] = (int16_t)(bias[i] / MAX_ITERS);

    return;
}

// ----------------------------------------------------------------------------
//
//  READ FUNCTIONS
//
// ----------------------------------------------------------------------------

void icm20948_read_raw_accel(
        icm20948_config_t*  config,
        int16_t             accel[DIMS])
{
    // counter
    INT_TYPE i;

    uint8_t buf[6];

    // accel: 2 bytes each axis    
    I2C_read_register(config, config->addr_accel_gyro, ACCEL_XOUT_H, buf, 6);

    for (i = 0; i < DIMS; i++)
        accel[i] = (buf[2*i] << 8 | buf[2*i + 1]);
    
    return;
}

void icm20948_read_raw_gyro(
        icm20948_config_t*  config,
        int16_t             gyro[DIMS])
{
    // counter
    INT_TYPE i;

    uint8_t buf[6];

    // gyro: 2byte each axis
    I2C_read_register(config, config->addr_accel_gyro, GYRO_XOUT_H, buf, 6);
    
    for (i = 0; i < DIMS; i++)
        gyro[i] = (buf[2*i] << 8 | buf[2*i + 1]);

    return;
}

void icm20948_read_raw_temp(
        icm20948_config_t*  config,
        int16_t*            temp)
{

    uint8_t buf[6];
    
    I2C_read_register(config, config->addr_accel_gyro, TEMP_OUT_H, buf, 2);
    
    *temp = (buf[0] << 8 | buf[1]);

    return;
}

void icm20948_read_raw_mag(
        icm20948_config_t*  config,
        int16_t             mag[DIMS])
{
    usb_serial_class* serial_usb = (usb_serial_class*)config->serial_usb;
    
    // counter
    INT_TYPE i;

    uint8_t buf[8];


    // read ST1 and check if data is ready
    I2C_read_register(config, config->addr_mag, AK09916_DATA_STATUS_1, buf, 1);

    if ((buf[0] & AK09916_ST1_DRDY) != AK09916_ST1_DRDY) {
        serial_usb->printf("  AK09916_DATA_STATUS_1 = %02X\nST1: Data is NOT ready\n", buf[0]);
        return;
    }

    // read the next 6 bytes of mag data
    I2C_read_register(config, config->addr_mag, AK09916_XOUT_L, &buf[1], 6);

    // finish reading by getting ST2
    I2C_read_register(config, config->addr_mag, AK09916_DATA_STATUS_2, &buf[7], 1);


    // serial_usb->printf("  here mag read: %02X\n", buf[0]);

    // print whole array for debugging
    // serial_usb->print("[");
    // for (i = 0; i < 8; i++)
    //     serial_usb->printf("%02X, ", buf[i]);
    // serial_usb->print("]\n");

    for (i = 0; i < DIMS; i++)
        mag[i] = (buf[2*i + 2] << 8 | buf[2*i + 1]);

#ifdef DEBUG_OUTPUT
    if ((buf[6] & 0x08) == 0x08)
        serial_usb->printf("mag: ST1: Sensor overflow\n");

    // printf below works only if we read 0x10
    //if ((buf[0] & 0x01) == 0x01) printf("mag: ST1: Data overrun\n");
    //if ((buf[0] & 0x02) != 0x02) printf("mag: ST1: Data is NOT ready\n");
#endif

    return;
}

void icm20948_cal_gyro(
        icm20948_config_t*  config,
        int16_t             gyro_bias[DIMS])
{
    // counters
    INT_TYPE i, j;
    
    int16_t buf[DIMS]  = {0};
    int32_t bias[DIMS] = {0};

    for (i = 0; i < MAX_ITERS; i++) {

        icm20948_read_raw_gyro(config, buf);

        for (j = 0; j < DIMS; j++) {
            bias[j] += buf[j];
        }

        delay(25);
    }

    for (i = 0; i < DIMS; i++)
        gyro_bias[i] = (int16_t)(bias[i] / MAX_ITERS);
}

void icm20948_read_cal_gyro(
        icm20948_config_t*  config,
        int16_t             gyro[DIMS],
        int16_t             bias[DIMS])
{
    // counters
    INT_TYPE i;

    icm20948_read_raw_gyro(config, gyro);

    for (i = 0; i < DIMS; i++)
        gyro[i] -= bias[i];
}



void icm20948_read_cal_accel(
        icm20948_config_t*  config, 
        int16_t             accel[DIMS],
        int16_t             bias[DIMS])
{
    // counters
    INT_TYPE i;

    icm20948_read_raw_accel(config, accel);

    for (i = 0; i < DIMS; i++)
        accel[i] -= bias[i];
}



void icm20948_read_cal_mag(
        icm20948_config_t*  config, 
        int16_t             mag[DIMS],
        int16_t             bias[DIMS])
{
    // counters
    INT_TYPE i;

    icm20948_read_raw_mag(config, mag);

    for (i = 0; i < DIMS; i++)
        mag[i] -= bias[i];
}

void icm20948_read_temp_c(
        icm20948_config_t*  config, 
        float*              temp)
{
    int16_t tmp;

    icm20948_read_raw_temp(config, &tmp);

    // temp  = ((raw_value - ambient_temp) / speed_of_sound) + 21
    *temp = (((float)tmp - 21.0f) / 333.87) + 21.0f;
}

