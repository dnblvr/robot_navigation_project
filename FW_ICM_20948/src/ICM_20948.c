/**
 * @file ICM_20948.c
 * @author your name (you@domain.com)
 * @brief 
 * @version 0.1
 * @date 2025-12-27
 * 
 */
#include "./inc/ICM_20948.h"



#define INT_TYPE    uint32_t
#define DIMS        3


// void EUSCI_B1_I2C_Init();
//
// void EUSCI_B1_I2C_Send_A_Byte(
//         uint8_t slave_address,
//         uint8_t data);
//
// void EUSCI_B1_I2C_Send_Multiple_Bytes(
//         uint8_t     slave_address,
//         uint8_t*    data_buffer,
//         uint32_t    packet_length);
//
// uint8_t EUSCI_B1_I2C_Receive_A_Byte(uint8_t slave_address);
//
// void EUSCI_B1_I2C_Receive_Multiple_Bytes(
//         uint8_t     slave_address,
//         uint8_t*    data_buffer,
//         uint16_t    packet_length);

// ----------------------------------------------------------------------------
//
//  INITIALIZATION FUNCTIONS
//
// ----------------------------------------------------------------------------

int8_t icm20948_init(icm20948_config_t* config)
{

    uint8_t reg[2], buf;

    // wake up accel/gyro
    // first write register then, write value
    // reg[0] = PWR_MGMT_1; reg[1] = 0x00;
    // i2c_write_blocking(config->i2c, config->addr_accel_gyro, reg, 2, false);
    EUSCI_B1_I2C_Send_Multiple_Bytes(config->addr_accel_gyro,
                                     (uint8_t[]){PWR_MGMT_1, 0x00},
                                     2);



    /** --------------------------------------------------------
     * switch to user bank to 0
     */
    // reg[0] = REG_BANK_SEL; reg[1] = 0x00;
    // i2c_write_blocking(config->i2c, config->addr_accel_gyro, reg, 2, false);
    EUSCI_B1_I2C_Send_Multiple_Bytes(config->addr_accel_gyro,
                                     (uint8_t[]){REG_BANK_SEL, 0x00},
                                     2);


    // auto select clock source
    // reg[0] = PWR_MGMT_1; reg[1] = 0x01;
    // i2c_write_blocking(config->i2c, config->addr_accel_gyro, reg, 2, false);
    EUSCI_B1_I2C_Send_Multiple_Bytes(config->addr_accel_gyro,
                                     (uint8_t[]){PWR_MGMT_1, 0x01},
                                     2);


    // disable accel/gyro once
    // reg[0] = PWR_MGMT_2; reg[1] = 0x3F;
    // i2c_write_blocking(config->i2c, config->addr_accel_gyro, reg, 2, false);
    // sleep_ms(10);
    EUSCI_B1_I2C_Send_Multiple_Bytes(config->addr_accel_gyro,
                                     (uint8_t[]){PWR_MGMT_2, 0x3F},
                                     2);


    Clock_Delay1ms(10);

    // enable accel/gyro again
    // reg[0] = PWR_MGMT_2; reg[1] = 0x00;
    // i2c_write_blocking(config->i2c, config->addr_accel_gyro, reg, 2, false);
    EUSCI_B1_I2C_Send_Multiple_Bytes(config->addr_accel_gyro,
                                     (uint8_t[]){PWR_MGMT_2, 0x00},
                                     2);



//    printf("here 1\n");

    // check if the accel/gyro can be accessed
    // reg[0] = WHO_AM_I_ICM20948;
    // i2c_write_blocking(config->i2c, config->addr_accel_gyro, reg, 1, true);
    // i2c_read_blocking(config->i2c, config->addr_accel_gyro, &buf, 1, false);
    EUSCI_B1_I2C_Send_A_Byte(config->addr_accel_gyro, WHO_AM_I_ICM20948);
    buf = EUSCI_B1_I2C_Receive_A_Byte(config->addr_accel_gyro);
    

#ifdef DEBUG_OUTPUT
    printf("AG. WHO_AM_I: 0x%X\n", buf);
#endif

    if (buf != 0xEA)
        return -1;



    printf("here 1\n");


    /** --------------------------------------------------------
     * switch to user bank 2
     */
    // reg[0] = REG_BANK_SEL; reg[1] = 0x20;
    // i2c_write_blocking(config->i2c, config->addr_accel_gyro, reg, 2, false);
    EUSCI_B1_I2C_Send_Multiple_Bytes(config->addr_accel_gyro,
                                     (uint8_t[]){REG_BANK_SEL, 0x20},
                                     2);

    printf("here 2\n");

    // gyro config
    //
    // set full scale to +-
    // set noise bandwidth to 
    // smaller bandwidth means lower noise level & slower max sample rate
    // reg[0] = GYRO_CONFIG_1; reg[1] = 0x29;
    // i2c_write_blocking(config->i2c, config->addr_accel_gyro, reg, 2, false);
    EUSCI_B1_I2C_Send_Multiple_Bytes(config->addr_accel_gyro,
                                     (uint8_t[]){GYRO_CONFIG_1, 0x29},
                                     2);

    //
    // set gyro output data rate to 100Hz
    // output_data_rate = 1.125kHz / (1 + GYRO_SMPLRT_DIV)
    // 1125 / 11 = 100
    // reg[0] = GYRO_SMPLRT_DIV; reg[1] = 0x0A;
    // i2c_write_blocking(config->i2c, config->addr_accel_gyro, reg, 2, false);
    EUSCI_B1_I2C_Send_Multiple_Bytes(config->addr_accel_gyro,
                                     (uint8_t[]){GYRO_SMPLRT_DIV, 0x0A},
                                     2);

    printf("here 3\n");

    // accel config
    //
    // set full scale to +-2g
    // set noise bandwidth to 136Hz
    // reg[0] = ACCEL_CONFIG; reg[1] = 0x11;
    // i2c_write_blocking(config->i2c, config->addr_accel_gyro, reg, 2, false);
    EUSCI_B1_I2C_Send_Multiple_Bytes(config->addr_accel_gyro,
                                     (uint8_t[]){ACCEL_CONFIG, 0x11},
                                     2);

    //
    // set accel output data rate to 100Hz
    // output_data_rate = 1.125kHz / (1 + ACCEL_SMPLRT_DIV)
    // 16 bits for ACCEL_SMPLRT_DIV
    // reg[0] = ACCEL_SMPLRT_DIV_2; reg[1] = 0x0A;
    // i2c_write_blocking(config->i2c, config->addr_accel_gyro, reg, 2, false);
    EUSCI_B1_I2C_Send_Multiple_Bytes(config->addr_accel_gyro,
                                     (uint8_t[]){ACCEL_SMPLRT_DIV_2, 0x0A},
                                     2);

    
    /** --------------------------------------------------------
     * switch back to user bank to 0
     */
    // reg[0] = REG_BANK_SEL; reg[1] = 0x00;
    // i2c_write_blocking(config->i2c, config->addr_accel_gyro, reg, 2, false);
    EUSCI_B1_I2C_Send_Multiple_Bytes(config->addr_accel_gyro,
                                     (uint8_t[]){REG_BANK_SEL, 0x00},
                                     2);

    
    // wake up mag! (INT_PIN_CFG, BYPASS_EN = 1)
    // reg[0] = INT_PIN_CFG; reg[1] = 0x02;
    // i2c_write_blocking(config->i2c, config->addr_accel_gyro, reg, 2, false);
    EUSCI_B1_I2C_Send_Multiple_Bytes(config->addr_accel_gyro,
                                     (uint8_t[]){INT_PIN_CFG, 0x02},
                                     2);

    // check if the magnetometer can be accessed
    // reg[0] = 0x01;
    // i2c_write_blocking(config->i2c, config->addr_mag, reg, 1, true);
    // i2c_read_blocking(config->i2c, config->addr_mag, &buf, 1, false);
    EUSCI_B1_I2C_Send_A_Byte(config->addr_mag, 0x01);
    buf = EUSCI_B1_I2C_Receive_A_Byte(config->addr_mag);


#ifdef DEBUG_OUTPUT
    printf("MAG. WHO_AM_I: 0x%X\n", buf);
#endif

    if (buf != 0x09)
        return -1;

    // config mag
    //
    // set mag mode, to measure continuously in 100Hz
    // reg[0] = AK09916_CNTL2; reg[1] = 0x08;
    // i2c_write_blocking(config->i2c, config->addr_mag, reg, 2, false);
    EUSCI_B1_I2C_Send_Multiple_Bytes(config->addr_mag,
                                     (uint8_t[]){AK09916_CNTL2, 0x08},
                                     2);

    printf("here 5\n");

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

    // 100Hz continuous --------------------------------------------------------
    case 100:
        reg[1] = 0x08;
        break;

    default:
#ifdef DEBUG_OUTPUT
        printf("error at icm20948_set_mag_mode: wrong mode %d\n", mode);
#endif
        return;
    }

    // reg[0] = AK09916_CNTL2;
    // i2c_write_blocking(config->i2c, config->addr_mag, reg, 2, false);
    EUSCI_B1_I2C_Send_Multiple_Bytes(config->addr_mag,
                                     (uint8_t[]){AK09916_CNTL2, reg[1]},
                                     2);

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
    // uint8_t reg = ACCEL_XOUT_H;
    // i2c_write_blocking(config->i2c, config->addr_accel_gyro, &reg, 1, true);
    // i2c_read_blocking(config->i2c, config->addr_accel_gyro, buf, 6, false);

    EUSCI_B1_I2C_Send_A_Byte(config->addr_accel_gyro, ACCEL_XOUT_H);
    EUSCI_B1_I2C_Receive_Multiple_Bytes(config->addr_accel_gyro, buf, 6);

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
    // uint8_t reg = GYRO_XOUT_H;
    // i2c_write_blocking(config->i2c, config->addr_accel_gyro, &reg, 1, true);
    // i2c_read_blocking(config->i2c, config->addr_accel_gyro, buf, 6, false);

    EUSCI_B1_I2C_Send_A_Byte(config->addr_accel_gyro, GYRO_XOUT_H);
    EUSCI_B1_I2C_Receive_Multiple_Bytes(config->addr_accel_gyro, buf, 6);

    for (i = 0; i < DIMS; i++)
        gyro[i] = (buf[2*i] << 8 | buf[2*i + 1]);

    return;
}

void icm20948_read_raw_temp(
        icm20948_config_t*  config,
        int16_t*            temp)
{

    uint8_t buf[6];

    // uint8_t reg = TEMP_OUT_H;
    // i2c_write_blocking(config->i2c, config->addr_accel_gyro, &reg, 1, true);
    // i2c_read_blocking(config->i2c, config->addr_accel_gyro, buf, 2, false);

    EUSCI_B1_I2C_Send_A_Byte(config->addr_accel_gyro,
                             TEMP_OUT_H);

    EUSCI_B1_I2C_Receive_Multiple_Bytes(config->addr_accel_gyro,
                                        buf,
                                        2);

    *temp = (buf[0] << 8 | buf[1]);

    return;
}

void icm20948_read_raw_mag(
        icm20948_config_t*  config,
        int16_t             mag[DIMS])
{
    // counter
    INT_TYPE i;

    uint8_t buf[8];

    // uint8_t reg = AK09916_XOUT_L;
    // i2c_write_blocking(config->i2c, config->addr_mag, &reg, 1, true);
    // i2c_read_blocking(config->i2c, config->addr_mag, buf, 8, false);

    EUSCI_B1_I2C_Send_A_Byte(config->addr_mag, AK09916_XOUT_L);
    EUSCI_B1_I2C_Receive_Multiple_Bytes(config->addr_mag, buf, 8);

    for (i = 0; i < DIMS; i++)
        mag[i] = (buf[2*i + 1] << 8 | buf[2*i]);

#ifdef DEBUG_OUTPUT
    if ((buf[6] & 0x08) == 0x08)
        printf("mag: ST1: Sensor overflow\n");

    // printf below works only if we read 0x10
    //if ((buf[0] & 0x01) == 0x01) printf("mag: ST1: Data overrun\n");
    //if ((buf[0] & 0x02) != 0x02) printf("mag: ST1: Data is NOT ready\n");
#endif

    return;
}

void icm20948_cal_gyro(
        icm20948_config_t* config,
        int16_t gyro_bias[DIMS])
{
    const INT_TYPE MAX_ITERS = 200;

    // counters
    INT_TYPE i, j;
    
    int16_t buf[DIMS]  = {0};
    int32_t bias[DIMS] = {0};

    for (i = 0; i < MAX_ITERS; i++) {

        icm20948_read_raw_gyro(config, buf);

        for (j = 0; j < DIMS; j++) {
            bias[j] += buf[j];
        }

        // sleep_ms(25);
        Clock_Delay1ms(25);

    }

    for (i = 0; i < DIMS; i++)
        gyro_bias[i] = (int16_t)(bias[i] / 200);
    
    return;
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

    for (i = 0; i < 200; i++) {

        icm20948_read_raw_accel(config, buf);

        for (j = 0; j < DIMS; j++) {

            if (j == 2)
                bias[j] += (buf[j] - 16384);
            else
                bias[j] += buf[j];

        }

        // sleep_ms(25);
        Clock_Delay1ms(25);

    }

    for (i = 0; i < DIMS; i++)
        accel_bias[i] = (int16_t)(bias[i] / 200);

    return;
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

    for (i = 0; i < 1000; i++) {

        icm20948_read_raw_mag(config, buf);

        for (j = 0; j < DIMS; j++) {
            if (buf[j] > max[j])
                max[j] = buf[j];
            if (buf[j] < min[j])
                min[j] = buf[j];
        }

        // sleep_ms(10);
        Clock_Delay1ms(10);

    }

    for (i = 0; i < DIMS; i++)
        mag_bias[i] = (max[i] + min[i]) / 2;

    return;
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

    return;
}

void icm20948_read_temp_c(
        icm20948_config_t*  config, 
        float*              temp)
{
    int16_t tmp;

    icm20948_read_raw_temp(config, &tmp);

    // temp  = ((raw_value - ambient_temp) / speed_of_sound) + 21
    *temp = (((float)tmp - 21.0f) / 333.87) + 21.0f;

    return;
}

/**
 * @brief 
 * 
 * 
 * 
 */
