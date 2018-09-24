/**
 *     mpu9150.c:  Driver for MPU-9150 9-axis Gyroscope/Accelerometer/Magnetometer
 *
 *     This driver supports various important functions of the MPU-9150, yet also
 *     includes "low-level" functions that allow the user to send commands/receive data
 *     from the module for functionality that is unimplemented.
 *
 *     This driver relies on STM32 HAL for functionality. Support for other STM32 libraries
 *     (i.e. LL or SPL) is unimplemented.
 *
 *     HOW TO USE THIS DRIVER:
 *     See mpu_common.h
 *
 *    ================================================================================
 *
 *     Author:               Georges Troulis
 *     Email:                gtroulis@ucsd.edu
 *     Driver Version:        0.1.0
 *     Last Revision Date:    08/26/2018
 *
 *    ================================================================================
 *
 *    Changelog:
 *      0.1.0:              Created this file
 *
 */

#include "mpu_common.h"
#include "stm32f4xx_hal.h"
#include "math.h"

#ifdef MPU_9150

/*------------------------------------------------------------*/
//  Device Registers for MPU-9150
/*------------------------------------------------------------*/

#define SELF_TESTX_REG    0x0D
#define SELF_TESTY_REG    0x0E
#define SELF_TESTZ_REG    0x0F
#define SELF_TESTA_REG    0x10

#define SMPLRT_REG        0x19
#define CFG_REG           0x1A
#define GYRO_CFG_REG      0x1B
#define ACCEL_CFG_REG     0x1C

#define FIFO_EN_REG       0x23
#define I2CMST_CTRL_REG   0x24
#define I2C_SLV0_ADDR_REG 0x25
#define I2C_SLV0_REG_REG  0x26
#define I2C_SLV0_CTRL_REG 0x27

#define I2CMST_STA_REG    0x36
#define INT_PIN_CFG_REG   0x37
#define INT_EN_REG        0x38
#define INT_STA_REG       0x3A

#define ACCEL_XOUTH_REG   0x3B
#define ACCEL_XOUTL_REG   0x3C
#define ACCEL_YOUTH_REG   0x3D
#define ACCEL_YOUTL_REG   0x3E
#define ACCEL_ZOUTH_REG   0x3F
#define ACCEL_ZOUTL_REG   0x40

#define TEMP_OUTH_REG     0x41
#define TEMP_OUTL_REG     0x42

#define GYRO_XOUTH_REG    0x43
#define GYRO_XOUTL_REG    0x44
#define GYRO_YOUTH_REG    0x45
#define GYRO_YOUTL_REG    0x46
#define GYRO_ZOUTH_REG    0x47
#define GYRO_ZOUTL_REG    0x48

// TODO LEFT OFF HERE
#define SIGPATH_RST_REG   0x68
#define USER_CTRL_REG     0x6A
#define PWR_MGMT1_REG     0x6B
#define PWR_MGMT2_REG     0x6C
#define FIFO_CNTH_REG     0x72
#define FIFO_CNTL_REG     0x73
#define FIFO_RW_REG       0x74
#define WHO_AM_I_REG      0x75

#define MPU_ADDR          0x68

/*------------------------------------------------------------*/
//  Config Options for device registers
/*------------------------------------------------------------*/

// Acceleration Full Scale Range, write to ACCEL_CFG_REG<4:3>
#define MPU_ACCEL_FS_2G   0x00
#define MPU_ACCEL_FS_4G   0x01
#define MPU_ACCEL_FS_8G   0x02
#define MPU_ACCEL_FS_16G  0x03

// Gyro Full Scale Range, write to GYRO_CFG_REG<4:3>
#define MPU_GYRO_FS_250DPS   0x00
#define MPU_GYRO_FS_500DPS   0x01
#define MPU_GYRO_FS_1000DPS  0x02
#define MPU_GYRO_FS_2000DPS  0x03

/*------------------------------------------------------------*/
//  Useful SPI Macros
/*------------------------------------------------------------*/

#define SPI_ADDR_ADD_W_BIT(x)    (x)
#define SPI_ADDR_ADD_R_BIT(x)    ((x) | 0x80)

/*------------------------------------------------------------*/
// Private Variables
/*------------------------------------------------------------*/
static float gyroFSScale = 0;
static float accelFSScale = 0;

#ifdef MPU_USE_SPI
  uint8_t spiTxBuf[32];
#endif

/**
 *   Initializes the accelerometer, and certain private variables
 *   pertaining to the functionality of this driver.
 *
 *  The parameters to MPU_SetAccelFSRange and MPU_SetGyroFSRange
 *  may change depending on the application, but both functions
 *  must be called at least once.
 */
HAL_StatusTypeDef MPU_Init(void) {
  HAL_StatusTypeDef status = 0;

  status |= MPU_SetAccelFSRange(MPU_ACCEL_FS_2G);
  status |= MPU_SetGyroFSRange(MPU_GYRO_FS_1000DPS);

  return (status == HAL_OK ? HAL_OK : HAL_ERROR);
}

/**
 *   Sets the full scale range of the accelerometer,
 *   and updates the scaling factor for acceleration values read
 *   from the sensor
 *
 *   fsRange:  The new full scale range of the accelerometer. Use the
 *             macros defined in mpu9150.h to ensure correct functionality
 */
HAL_StatusTypeDef MPU_SetAccelFSRange(uint8_t fsRange) {
  accelFSScale = pow(2, fsRange) / 16384;

  return MPU_WriteBits(ACCEL_CFG_REG, fsRange, 4, 2);
}


/**
 *   Sets the full scale range of the gyroscope,
 *   and updates the scaling factor for gyroscope values read
 *   from the sensor
 *
 *   fsRange:  The new full scale range of the gyroscope. Use the
 *             macros defined in mpu9150.h to ensure correct functionality
 */
HAL_StatusTypeDef MPU_SetGyroFSRange(uint8_t fsRange) {
  gyroFSScale = pow(2, fsRange) / 232;

  return MPU_WriteBits(GYRO_CFG_REG, fsRange, 4, 2);
}

/**
 *   Reads the x, y, and z gyroscope values in burst-read mode, and scales them
 *   according to the full scale range of the gyroscope.
 *
 *   The scaling factor that is applied is calculated during the call to
 *   MPU_SetAccelFSRange(). Therefore no mater what FS range is selected,
 *   the values returned from this function will be very similar, but with
 *   varying measurement precision
 *
 *   ax:  memory location into which to place the x gyroscope value
 *   ay:  memory location into which to place the y gyroscope value
 *   az:  memory location into which to place the z gyroscope value
 */
HAL_StatusTypeDef MPU_GetGyroscope(float *gx, float *gy, float *gz) {
  HAL_StatusTypeDef status;
  int16_t xRaw, yRaw, zRaw;

  status = MPU_GetGyroscopeRaw(&xRaw, &yRaw, &zRaw);

  *gx = ((float) xRaw) * gyroFSScale;
  *gy = ((float) yRaw) * gyroFSScale;
  *gz = ((float) zRaw) * gyroFSScale;

  return status;
}

/**
 *   Reads the x, y, and z accelerometer values in burst-read mode, and scales them
 *   according to the full scale range of the accelerometer.
 *
 *   The scaling factor that is applied is calculated during the call to
 *   MPU_SetAccelFSRange(). Therefore no mater what FS range is selected,
 *   the values returned from this function will be very similar, but with
 *   varying measurement precision
 *
 *   ax:  memory location into which to place the x acceleration
 *   ay:  memory location into which to place the y acceleration
 *   az:  memory location into which to place the z acceleration
 */
HAL_StatusTypeDef MPU_GetAccelerations(float *ax, float *ay, float *az) {
  HAL_StatusTypeDef status;
  int16_t xRaw, yRaw, zRaw;

  status = MPU_GetAccelerationsRaw(&xRaw, &yRaw, &zRaw);

  *ax = ((float) xRaw) * accelFSScale;
  *ay = ((float) yRaw) * accelFSScale;
  *az = ((float) zRaw) * accelFSScale;

  return status;
}


/**
 *   Reads the x, y, and z gyroscope values in burst-read mode
 *   The datasheet guarantees that all readings will be from the same sampling interval
 *   during a burst-read.
 *
 *   The gyroscope values are the raw bit values, and need scaling to be interpreted
 *   properly. The scaling depends on the full-scale range that is set in the init function
 *   of this driver
 *
 *   ax:  memory location into which to place the x gyroscope value
 *   ay:  memory location into which to place the y gyroscope value
 *   az:  memory location into which to place the z gyroscope value
 */
HAL_StatusTypeDef MPU_GetGyroscopeRaw(int16_t *gx, int16_t *gy, int16_t *gz) {
  HAL_StatusTypeDef status;
  uint8_t buf[6];

  status = MPU_ReadLen(GYRO_XOUTH_REG, 6, buf);
  *gx = ((uint16_t) buf[0] << 8) | buf[1];
  *gy = ((uint16_t) buf[2] << 8) | buf[3];
  *gz = ((uint16_t) buf[4] << 8) | buf[5];

  return status;
}

/**
 *   Reads the x, y, and z accelerometer values in burst-read mode
 *   The datasheet guarantees that all readings will be from the same sampling interval
 *   during a burst-read.
 *
 *   The acceleration values are the raw bit values, and need scaling to be interpreted
 *   properly. The scaling depends on the full-scale range that is set in the init function
 *   of this driver
 *
 *   ax:  memory location into which to place the x acceleration
 *   ay:  memory location into which to place the y acceleration
 *   az:  memory location into which to place the z acceleration
 */
HAL_StatusTypeDef MPU_GetAccelerationsRaw(int16_t *ax, int16_t *ay, int16_t *az) {
  HAL_StatusTypeDef status;
  uint8_t buf[6];

  status = MPU_ReadLen(ACCEL_XOUTH_REG, 6, buf);
  *ax = ((uint16_t) buf[0] << 8) | buf[1];
  *ay = ((uint16_t) buf[2] << 8) | buf[3];
  *az = ((uint16_t) buf[4] << 8) | buf[5];

  return status;
}


/**
 * TODO THIS METHOD IS MALFUNCTIONAL
 *
 *   Reads the internal temperature of the sensor
 *
 *   temp:  pointer to a variable into which to store the sensor temperature
 */
HAL_StatusTypeDef MPU_GetTemperature(float* temp) {
  HAL_StatusTypeDef status;
  uint8_t buf[2];
  uint16_t raw;

  status = MPU_ReadLen(TEMP_OUTH_REG, 2, buf);
  raw = ((uint16_t) buf[0] << 8) | buf[1];
  *temp = 36.53 + ((float) raw) / 340;
  *temp *= 100;

  return status;
}

/**
 *   For lazy debug purposes only, to test singular register reads without implementing
 *   an entire extra method to handle functionality. Has undocumented behaviour
 */

HAL_StatusTypeDef MPU_Surprise(uint16_t* output) {
  HAL_StatusTypeDef status;
  uint8_t buf[2];

  status = MPU_ReadLen(ACCEL_XOUTH_REG, 2, buf);
  *output = ((uint16_t) buf[0] << 8) | buf[1];

  return status;
}

#endif // USE_MPU_9150
