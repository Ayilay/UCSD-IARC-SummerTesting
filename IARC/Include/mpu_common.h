/**
 *     mpu_common.h:  Defines a common software interface for Invensense's MPU sensor modules
 *
 *     This driver supports various important functions of the MPU-sensors, yet also
 *     includes "low-level" functions that allow the user to send commands/receive data
 *     from the module for functionality that is unimplemented.
 *
 *     This driver relies on STM32 HAL for functionality. Support for other STM32 libraries
 *     (i.e. LL or SPL) is unimplemented.
 *
 *     HOW TO USE THIS DRIVER:
 *     (1) Configure HAL such that I2C or SPI is initialized, and ensure proper parameters are used
 *     (2) In mpu_common.h, define either MPU_USE_I2C or MPU_USE_SPI
 *     (3) In mpu_common.h, define either MPU_9255 or MPU_9150 depending on which device is used
 *     (4) In mpu_common.h, define MPU_I2C or MPU_SPI to alias the same hspi/hi2c module
 *           that HAL is configured to use with the sensor
 *     (5) In the desired mpu_XXXX.c file in MPU_Init(), ensure both MPU_SetAccelFSRange and MPU_SetGyroFSRange
 *           are called at least once and given user-desired arguments, otherwise this Driver File
 *           might result in unexpected behaviour.
 *
 *    ================================================================================
 *
 *     Author:               Georges Troulis
 *     Email:                gtroulis@ucsd.edu
 *     Driver Version:       0.3.0
 *     Last Revision Date:   10/31/2018
 *
 *    ================================================================================
 *
 *    Changelog:
 *      0.3.0:              Added MPU_SampleAndSerialize function to sample all MPU sensors
 *                           and serialize it to a single data packet
 *      0.2.0:              Disabled sleep mode in Init()
 */

#ifndef __MPU_COMMON_H
#define __MPU_COMMON_H

#include "main.h"

/*------------------------------------------------------------*/
//  (2) Define either MPU_USE_I2C or MPU_USE_SPI
/*------------------------------------------------------------*/

#define MPU_USE_I2C
//#define MPU_USE_SPI

/*------------------------------------------------------------*/
//  (3) Define which MPU device to use
/*------------------------------------------------------------*/

//#define MPU_9255
#define MPU_9150

/*------------------------------------------------------------*/
//  (4) Important Macro Definitions and Error Checking
/*------------------------------------------------------------*/

#if defined(MPU_USE_I2C)

  #include "i2c.h"

  // (4) Ensure the proper i2c peripheral is used
  #define MPU_I2C  hi2c1

#elif defined(MPU_USE_SPI)

  #include "spi.h"

  // (4) Ensure the proper spi peripheral is used
  #define MPU_SPI  hspi1

#else
  #error "Must define either MPU_USE_I2C or MPU_USE_SPI in main.h"
#endif

/*------------------------------------------------------------*/
//  More important macros, DO NOT MODIFY
/*------------------------------------------------------------*/

#if defined(MPU_USE_I2C) & defined(MPU_USE_SPI)
  #error "Must only define one of MPU_USE_I2C or MPU_USE_SPI"
#endif

#if defined(MPU_9255) & defined(MPU_9150)
  #error "Must only define a single MPU device, MPU_9150 or MPU_9255"
#endif

#if defined(MPU_9150) & defined(MPU_USE_SPI)
  #error "MPU 9150 doesn't support SPI"
#endif

#if defined(MPU_9150)
  #include "mpu9150.h"
#elif defined(MPU_9255)
  #include "mpu9255.h"
#endif

/*------------------------------------------------------------*/
//  Address Macros for SPI and I2C
/*------------------------------------------------------------*/

#define SPI_ADDR_ADD_W_BIT(x)    (x)
#define SPI_ADDR_ADD_R_BIT(x)    ((x) | 0x80)

#define MPU_ADDR               0x68
#define MPU_ADDR_W             (MPU_ADDR << 1)
#define MPU_ADDR_R             (MPU_ADDR << 1) | 1

/*------------------------------------------------------------*/
//  Config Options for device registers
/*------------------------------------------------------------*/

// Acceleration Full Scale Range, write to ACCEL_CFG1_REG<4:3>
#define MPU_ACCEL_FS_2G    0x00
#define MPU_ACCEL_FS_4G    0x01
#define MPU_ACCEL_FS_8G    0x02
#define MPU_ACCEL_FS_16G   0x03

// Gyro Full Scale Range, write to GYRO_CFG_REG<4:3>
#define MPU_GYRO_FS_250DPS  0x00
#define MPU_GYRO_FS_500DPS  0x01
#define MPU_GYRO_FS_1000DPS 0x02
#define MPU_GYRO_FS_2000DPS 0x03

/*------------------------------------------------------------*/
//  Internal Data Structures
/*------------------------------------------------------------*/

// Contains all the relevant sensor info with packet headers/footers
// to facilitate with streaming sensor data over a serial interface
typedef struct {
  uint32_t sod;
  float accelData[3];
  float gyroData[3];
  float magData[3];
  uint32_t eod;
} MPUDataPacket_t;

/*------------------------------------------------------------*/
//  Other Constants
/*------------------------------------------------------------*/

#define MPU_DEFAULT_TIMEOUT  200

// Serial Start of Data and End of Data frame headers (arbitrarily chosen)
#define MPU_SERIAL_SOD (0xFFDEADFF)
#define MPU_SERIAL_EOD (0xEEDDAA77)

/*------------------------------------------------------------*/
//  Public Interface Functions
/*------------------------------------------------------------*/

// Interface Funcs
HAL_StatusTypeDef MPU_Init(void);
HAL_StatusTypeDef MPU_GetTemperature(float* tempBuf);
HAL_StatusTypeDef MPU_GetGyroscope(float *gx, float *gy, float *gz);
HAL_StatusTypeDef MPU_GetAccelerations(float *ax, float *ay, float *az);
HAL_StatusTypeDef MPU_SetAccelFSRange(uint8_t range);
HAL_StatusTypeDef MPU_SetGyroFSRange(uint8_t range);

// Data Transportation Funcs
void MPU_SampleAndSerialize(MPUDataPacket_t* packet);

// Debug Funcs
HAL_StatusTypeDef MPU_GetGyroscopeRaw(int16_t *gx, int16_t *gy, int16_t *gz);
HAL_StatusTypeDef MPU_GetAccelerationsRaw(int16_t *ax, int16_t *ay, int16_t *az);
HAL_StatusTypeDef MPU_Surprise(uint16_t* output);

// Utility Funcs, only for Low-Level purposes
HAL_StatusTypeDef MPU_ReadLen(uint8_t reg, uint8_t len, uint8_t *buf);
HAL_StatusTypeDef MPU_WriteLen(uint8_t reg, uint8_t len, uint8_t *buf);
HAL_StatusTypeDef MPU_ReadByte(uint8_t reg, uint8_t* res);
HAL_StatusTypeDef MPU_WriteByte(uint8_t reg, uint8_t data);
HAL_StatusTypeDef MPU_WriteBits(uint8_t reg, uint8_t bitsToWrite, uint8_t bitStart, uint8_t numBits);


#endif
