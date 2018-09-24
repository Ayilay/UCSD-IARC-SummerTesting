/**
 * 		mpu_common.h:	Defines a common software interface for Invensense's MPU sensor modules
 *
 * 		This driver supports various important functions of the MPU-sensors, yet also
 * 		includes "low-level" functions that allow the user to send commands/receive data
 * 		from the module for functionality that is unimplemented.
 *
 * 		This driver relies on STM32 HAL for functionality. Support for other STM32 libraries
 * 		(i.e. LL or SPL) is unimplemented.
 *
 * 		HOW TO USE THIS DRIVER:
 * 		(1) Configure HAL such that I2C or SPI is initialized, and ensure proper parameters are used
 * 		(2) In mpu_common.h, define either MPU_USE_I2C or MPU_USE_SPI
 * 		(3) In mpu_common.h, define either MPU_9255 or MPU_9150 depending on which device is used
 * 		(4) In mpu_common.h, define MPU_I2C or MPU_SPI to alias the same hspi/hi2c module
 * 					that HAL is configured to use with the sensor
 * 		(5) In the desired mpu_XXXX.c file in MPU_Init(), ensure both MPU_SetAccelFSRange and MPU_SetGyroFSRange
 * 					are called at least once and given user-desired arguments, otherwise this Driver File
 * 					might result in unexpected behaviour.
 *
 *	  ================================================================================
 *
 * 		Author: 							Georges Troulis
 * 		Email:								gtroulis@ucsd.edu
 * 		Driver Version:				0.1.0
 * 		Last Revision Date:		08/26/2018
 *
 *	  ================================================================================
 *
 *		Changelog:
 *			0.1.0:							Renamed mpu9255.h to mpu_common.h to accommodate
 *													for multiple MPU sensors (MPU_9255 and MPU_9150)
 */

#ifndef __MPU_COMMON_H
#define __MPU_COMMON_H

#include "main.h"

/*------------------------------------------------------------*/
//	(2) Define either MPU_USE_I2C or MPU_USE_SPI
/*------------------------------------------------------------*/

#define MPU_USE_I2C
//#define MPU_USE_SPI

/*------------------------------------------------------------*/
//	(3) Define which MPU device to use
/*------------------------------------------------------------*/

//#define MPU_9255
#define MPU_9150


/*------------------------------------------------------------*/
//	(4) Important Macro Definitions and Error Checking
/*------------------------------------------------------------*/

#if defined(MPU_USE_I2C)

	#include "i2c.h"

	// (4) Ensure the proper i2c peripheral is used
	#define MPU_I2C	hi2c1

#elif defined(MPU_USE_SPI)

	#include "spi.h"

	// (4) Ensure the proper spi peripheral is used
	#define MPU_SPI	hspi1

#else
	#error "Must define either MPU_USE_I2C or MPU_USE_SPI in main.h"
#endif

#if defined(MPU_USE_I2C) & defined(MPU_USE_SPI)
	#error "Must only define one of MPU_USE_I2C or MPU_USE_SPI"
#endif

#if defined(MPU_9255) & defined(MPU_9150)
	#error "Must only define a single MPU device, MPU_9150 or MPU_9255"
#endif

#if defined(MPU_9150) & defined(MPU_USE_SPI)
	#error "MPU 9150 doesn't support SPI"
#endif

/*------------------------------------------------------------*/
//	Other Constants
/*------------------------------------------------------------*/

#define MPU_DEFAULT_TIMEOUT	200

/*------------------------------------------------------------*/
//	Public Interface Functions
/*------------------------------------------------------------*/

// Interface Funcs
HAL_StatusTypeDef MPU_Init(void);
HAL_StatusTypeDef MPU_GetTemperature(float* tempBuf);
HAL_StatusTypeDef MPU_GetGyroscope(float *gx, float *gy, float *gz);
HAL_StatusTypeDef MPU_GetAccelerations(float *ax, float *ay, float *az);
HAL_StatusTypeDef MPU_SetAccelFSRange(uint8_t range);
HAL_StatusTypeDef MPU_SetGyroFSRange(uint8_t range);

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
