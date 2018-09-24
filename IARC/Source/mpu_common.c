/**
 * 		mpu_common.c:	Defines functions that are common to all MPU sensors.
 *
 * 		This driver supports various important functions of the MPU-9255, yet also
 * 		includes "low-level" functions that allow the user to send commands/receive data
 * 		from the module for functionality that is unimplemented.
 *
 * 		This driver relies on STM32 HAL for functionality. Support for other STM32 libraries
 * 		(i.e. LL or SPL) is unimplemented.
 *
 * 		HOW TO USE THIS DRIVER:
 * 		See mpu_common.h
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
 *			0.1.0:							Added macro-dependent compilation flags and migrated all
 *													register definitions to this file
 */

#include <mpu_common.h>
#include "stm32f4xx_hal.h"
#include "math.h"

#define MPU_ADDR         0x68
#define MPU_ADDR_W       (MPU_ADDR << 1)
#define MPU_ADDR_R       (MPU_ADDR << 1) | 1

/*------------------------------------------------------------*/
// Private Variables
/*------------------------------------------------------------*/

#ifdef MPU_USE_SPI
	uint8_t spiTxBuf[32];
#endif

/**
 * 	Writes a bit pattern to a sub-section of a register. This is useful for writing
 * 	to registers where a certain configuration option does not span the entire register length.
 * 	Ex: To set the Accel Full Scale, only bits <4:3> of ACCEL_CFG1_REG need to be written.
 *
 * 	This function works by first reading the value of the register, then super-imposing the bits
 * 	to write onto the original register value, and finall writing the full register back to
 * 	the device.
 *
 * 	Clever bitwise logic is borrowed from the Open-Source I2CDevLib:
 * 	https://www.i2cdevlib.com/
 *
 *	reg:					The register to write to
 * 	bitsToWrite:	The relevant bits to write to the register sub-section
 * 	bitStart:			The location of the 1st bit in the register sub-section (from 0 to 7)
 * 	numBits:			The number of bits in this bit string (from 1 to 6)
 */
HAL_StatusTypeDef MPU_WriteBits(uint8_t reg, uint8_t bitsToWrite, uint8_t bitStart, uint8_t numBits) {
	uint8_t prevRegVal;
	HAL_StatusTypeDef status = MPU_ReadByte(reg, &prevRegVal);
	if (status != HAL_OK) {
		return status;
	}

  uint8_t mask = ((1 << numBits) - 1) << (bitStart - numBits + 1);
  bitsToWrite <<= (bitStart - numBits + 1); // shift data into correct position
  bitsToWrite &= mask; // zero all non-important bits in data
  prevRegVal &= ~(mask); // zero all important bits in existing byte
  prevRegVal |= bitsToWrite; // combine data with existing byte

  return MPU_WriteByte(reg, prevRegVal);
}


/**
 * 	Writes a single byte to a device register
 *
 * 	reg:	The device register to write to
 * 	data:	The data to write to the register (note, not a pointer)
 */
HAL_StatusTypeDef MPU_WriteByte(uint8_t reg, uint8_t data) {
	return MPU_WriteLen(reg, 1, &data);
}

/**
 * 	Reads a single byte from a device register
 *
 * 	reg:	The device register to read from
 * 	res:	The destination memory address into which to place the read result
 */
HAL_StatusTypeDef MPU_ReadByte(uint8_t reg, uint8_t* res) {
	return MPU_ReadLen(reg, 1, res);
}


/**
 *	Writes a certain number of bytes from a buffer into a given register
 *
 *	reg:	The register to write to
 *	len:	The number of bytes to write
 *	buf:	The buffer from which to read data (must be properly initialized)
 */
HAL_StatusTypeDef MPU_WriteLen(uint8_t reg, uint8_t len, uint8_t *buf) {
	HAL_StatusTypeDef status;
	#if defined(MPU_USE_I2C)
		status = HAL_I2C_Mem_Write(&MPU_I2C, MPU_ADDR_W, reg, I2C_MEMADD_SIZE_8BIT, buf, len, MPU_DEFAULT_TIMEOUT);
	#elif defined(MPU_USE_SPI)

		// The first byte of the transfer is the register address plus a write bit
		spiTxBuf[0] = SPI_ADDR_ADD_W_BIT(reg);

		// Initiate the SPI transfer
		HAL_GPIO_WritePin(Gyro_CS_GPIO_Port, Gyro_CS_Pin, GPIO_PIN_RESET);

		// Transmit the register address to read from. If not ok status, end
		// SPI transfer early and return
		status = HAL_SPI_Transmit(&MPU_SPI, spiTxBuf, 1, MPU_DEFAULT_TIMEOUT);
		if (status != HAL_OK) {
			HAL_GPIO_WritePin(Gyro_CS_GPIO_Port, Gyro_CS_Pin, GPIO_PIN_SET);
			return status;
		}

		// Write the actual data
		status = HAL_SPI_Transmit(&MPU_SPI, buf, len, MPU_DEFAULT_TIMEOUT);

		// Finalize the SPI transfer
		HAL_GPIO_WritePin(Gyro_CS_GPIO_Port, Gyro_CS_Pin, GPIO_PIN_SET);
	#endif
	return status;
}

/**
 *	Reads a certain number of bytes from a given register into a buffer
 *
 *	reg:	The register to read from
 *	len:	The number of bytes to read
 *	buf:	The buffer into which to place data (must be properly initialized)
 */
HAL_StatusTypeDef MPU_ReadLen(uint8_t reg, uint8_t len, uint8_t *buf) {
	HAL_StatusTypeDef status;
	#if defined(MPU_USE_I2C)
		status = HAL_I2C_Mem_Read(&MPU_I2C, MPU_ADDR_R, reg, I2C_MEMADD_SIZE_8BIT, buf, len, MPU_DEFAULT_TIMEOUT);
	#elif defined(MPU_USE_SPI)

		// The first byte of the transfer is the register address plus a read bit
		spiTxBuf[0] = SPI_ADDR_ADD_R_BIT(reg);

		// Initiate the SPI transfer
		HAL_GPIO_WritePin(Gyro_CS_GPIO_Port, Gyro_CS_Pin, GPIO_PIN_RESET);

		// Transmit the register address to read from. If not ok status, end
		// SPI transfer early and return
		status = HAL_SPI_Transmit(&MPU_SPI, spiTxBuf, 1, MPU_DEFAULT_TIMEOUT);
		if (status != HAL_OK) {
			HAL_GPIO_WritePin(Gyro_CS_GPIO_Port, Gyro_CS_Pin, GPIO_PIN_SET);
			return status;
		}

		// Read the actual data
		status = HAL_SPI_Receive(&MPU_SPI, buf, len, MPU_DEFAULT_TIMEOUT);

		// Finalize the SPI transfer
		HAL_GPIO_WritePin(Gyro_CS_GPIO_Port, Gyro_CS_Pin, GPIO_PIN_SET);
	#endif

	return status;
}
