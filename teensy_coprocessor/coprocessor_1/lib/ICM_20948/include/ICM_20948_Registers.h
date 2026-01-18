/**
 * @file ICM_20948_Registers.h
 * @author @akionu (https://github.com/akionu)
 *         (modified by Gian Fajardo for the Teensy 4.1 platform)
 * @brief Register definitions for the ICM-20948 sensor
 * @date 2025-12-27 
 */


#ifndef __INC_ICM20948_H__
#define __INC_ICM20948_H__

// ----------------------------------------------------------------------------
//
//  REGISTER BITS
//
// ----------------------------------------------------------------------------

#define GET_BIT(value, mask)    ((value) & (mask) != 0)
#define SET_BIT(value, mask)    ((value) | (mask))
#define UNSET_BIT(value, mask)  ((value) & ~(mask))
#define BIT(n)                  (1 << (n))

// ----------------------------------------------------------------------------
//
//  AK09916 REGISTERS
//
// ----------------------------------------------------------------------------

/**
 * @note must wake up AK09916 before reading WHO_AM_I register! 
 */

#define AK09916_WHO_AM_I        0x01
#define AK09916_DATA_STATUS_1   0x10

#define AK09916_ST1_DRDY        0x01
#define AK09916_ST1_DOR         0x02

#define AK09916_DATA_STATUS_2   0x18

/**
 * @brief AK09916 Control Register 2
 * 
 * mode select B4-0:
 *  - `0b00000`: Power-down
 *  - `0b00001`: Single measurement
 *  - `0b00010`: Continuous measurement 1; 10 Hz
 *  - `0b00100`: Continuous measurement 2; 20 Hz
 *  - `0b00110`: Continuous measurement 3; 50 Hz
 *  - `0b01000`: Continuous measurement 4; 100 Hz
 *  - `0b10000`: Self-test mode
 */
#define AK09916_CONTROL_2       0x31

    #define SINGLE_MODE         BIT(0)
    #define CONTINUOUS_10HZ     BIT(1)
    #define CONTINUOUS_20HZ     BIT(2)
    #define CONTINUOUS_50HZ     (BIT(2) | BIT(1))
    #define CONTINUOUS_100HZ    BIT(3)
    #define SELF_TEST_MODE      BIT(4)

/**
 * @brief B0 -> software reset
 */
#define AK09916_CONTROL_3       0x32

    #define SOFT_RESET      BIT(0)

/** ---------------------------------------------------------------------------
 * @brief Data Output Registers
 */
#define AK09916_XOUT_L          0x11
#define AK09916_XOUT_H          0x12
#define AK09916_YOUT_L          0x13
#define AK09916_YOUT_H          0x14
#define AK09916_ZOUT_L          0x15
#define AK09916_ZOUT_H          0x16


// ----------------------------------------------------------------------------
//
//  ICM20948 Registers - User Bank 0
//
// ----------------------------------------------------------------------------

#define WHO_AM_I_ICM20948       0x00
#define USER_CTRL               0x03
#define LP_CONFIG		        0x05
#define PWR_MGMT_1              0x06
#define PWR_MGMT_2              0x07

/**
 * @brief interrupt configuration register address
 */
#define INT_PIN_CFG             0x0F

    #define INT1_ACTL       BIT(7)
    #define INT1_OPEN       BIT(6)
    #define INT1_LATCH_EN   BIT(5)

/**
 * @brief switches to I2C bypass mode, enabling direct access to the
 *  magnetometer 
 */
#define INT1_BYPASS_EN      BIT(1)

/**
 * @brief interrupt enable register address
 */
#define INT_ENABLE              0x10

    #define I2C_MST_INT_EN  BIT(0)

#define INT_ENABLE_1	        0x11
#define INT_ENABLE_2	        0x12
#define INT_ENABLE_3	        0x13
#define I2C_MST_STATUS          0x17

/**
 * @brief interrupt status register address
 */
#define INT_STATUS              0x19
#define INT_STATUS_1	        0x1A

    #define RAW_DRDY_EN     BIT(0)


#define INT_STATUS_2	        0x1B
#define INT_STATUS_3	        0x1C


/* ----------------------------------------------------------------------------
 * data registers 
 */

#define DELAY_TIME_H	        0x28
#define DELAY_TIME_L	        0x29
#define ACCEL_XOUT_H            0x2D
#define ACCEL_XOUT_L            0x2E
#define ACCEL_YOUT_H            0x2F
#define ACCEL_YOUT_L            0x30
#define ACCEL_ZOUT_H            0x31
#define ACCEL_ZOUT_L            0x32
#define GYRO_XOUT_H             0x33
#define GYRO_XOUT_L             0x34
#define GYRO_YOUT_H             0x35
#define GYRO_YOUT_L             0x36
#define GYRO_ZOUT_H             0x37
#define GYRO_ZOUT_L             0x38
#define TEMP_OUT_H              0x39
#define TEMP_OUT_L              0x3A

#define EXT_SENS_DATA_00        0x3B
#define EXT_SENS_DATA_01        0x3C
#define EXT_SENS_DATA_02        0x3D
#define EXT_SENS_DATA_03        0x3E
#define EXT_SENS_DATA_04        0x3F
#define EXT_SENS_DATA_05        0x40
#define EXT_SENS_DATA_06        0x41
#define EXT_SENS_DATA_07        0x42
#define EXT_SENS_DATA_08        0x43
#define EXT_SENS_DATA_09        0x44
#define EXT_SENS_DATA_10        0x45
#define EXT_SENS_DATA_11        0x46
#define EXT_SENS_DATA_12        0x47
#define EXT_SENS_DATA_13        0x48
#define EXT_SENS_DATA_14        0x49
#define EXT_SENS_DATA_15        0x4A
#define EXT_SENS_DATA_16        0x4B
#define EXT_SENS_DATA_17        0x4C
#define EXT_SENS_DATA_18        0x4D
#define EXT_SENS_DATA_19        0x4E
#define EXT_SENS_DATA_20        0x4F
#define EXT_SENS_DATA_21        0x50
#define EXT_SENS_DATA_22        0x51
#define EXT_SENS_DATA_23        0x52

#define FIFO_EN_1               0x66
#define FIFO_EN_2               0x67
#define FIFO_RST		        0x68
#define FIFO_MODE		        0x69
#define FIFO_COUNTH             0x70
#define FIFO_COUNTL             0x71
#define FIFO_R_W                0x72
#define DATA_RDY_STATUS	        0x74
#define FIFO_CFG		        0x76

/**
 * @brief register that selects different portions of the register map 
 */
#define REG_BANK_SEL	        0x7F

    #define REG_BANK_0      0x00
    #define REG_BANK_1      0x10
    #define REG_BANK_2      0x20
    #define REG_BANK_3      0x30


// ----------------------------------------------------------------------------
//
//  ICM20948 Registers - User Bank 1
//
// ----------------------------------------------------------------------------

#define SELF_TEST_X_GYRO  		0x02
#define SELF_TEST_Y_GYRO  		0x03
#define SELF_TEST_Z_GYRO  		0x04
#define SELF_TEST_X_ACCEL 		0x0E
#define SELF_TEST_Y_ACCEL 		0x0F
#define SELF_TEST_Z_ACCEL 		0x10
#define XA_OFFSET_H       		0x14
#define XA_OFFSET_L       		0x15
#define YA_OFFSET_H       		0x17
#define YA_OFFSET_L       		0x18
#define ZA_OFFSET_H       		0x1A
#define ZA_OFFSET_L       		0x1B
#define TIMEBASE_CORRECTION_PLL 0x28


// ----------------------------------------------------------------------------
//
//  ICM20948 Registers - User Bank 2
//
// ----------------------------------------------------------------------------

#define GYRO_SMPLRT_DIV        	0x00
#define GYRO_CONFIG_1      		0x01
#define GYRO_CONFIG_2      		0x02
#define XG_OFFSET_H       		0x03
#define XG_OFFSET_L       		0x04
#define YG_OFFSET_H       		0x05
#define YG_OFFSET_L       		0x06
#define ZG_OFFSET_H       		0x07
#define ZG_OFFSET_L       		0x08
#define ODR_ALIGN_EN			0x09
#define ACCEL_SMPLRT_DIV_1     	0x10
#define ACCEL_SMPLRT_DIV_2     	0x11
#define ACCEL_INTEL_CTRL		0x12
#define ACCEL_WOM_THR			0x13
#define ACCEL_CONFIG      		0x14
#define ACCEL_CONFIG_2     		0x15
#define FSYNC_CONFIG			0x52
#define TEMP_CONFIG				0x53
#define MOD_CTRL_USR			0x54


// ----------------------------------------------------------------------------
//
//  ICM20948 Registers - User Bank 3
//
// ----------------------------------------------------------------------------

#define I2C_MST_ODR_CONFIG		0x00
#define I2C_MST_CTRL       		0x01
#define I2C_MST_DELAY_CTRL 		0x02
#define I2C_SLV0_ADDR      		0x03
#define I2C_SLV0_REG       		0x04
#define I2C_SLV0_CTRL      		0x05
#define I2C_SLV0_DO        		0x06
#define I2C_SLV1_ADDR      		0x07
#define I2C_SLV1_REG       		0x08
#define I2C_SLV1_CTRL      		0x09
#define I2C_SLV1_DO        		0x0A
#define I2C_SLV2_ADDR      		0x0B
#define I2C_SLV2_REG       		0x0C
#define I2C_SLV2_CTRL      		0x0D
#define I2C_SLV2_DO        		0x0E
#define I2C_SLV3_ADDR      		0x0F
#define I2C_SLV3_REG       		0x10
#define I2C_SLV3_CTRL      		0x11
#define I2C_SLV3_DO        		0x12
#define I2C_SLV4_ADDR      		0x13
#define I2C_SLV4_REG       		0x14
#define I2C_SLV4_CTRL      		0x15
#define I2C_SLV4_DO        		0x16
#define I2C_SLV4_DI        		0x17



#endif /* __INC_ICM20948_H__ */
