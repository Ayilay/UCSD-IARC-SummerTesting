/**
 * 		mpu9255.c:	Driver for MPU-9255 9-axis Gyroscope/Accelerometer/Magnetometer
 *
 * 		This driver supports various important functions of the MPU-9255, yet also
 * 		includes "low-level" functions that allow the user to send commands/receive data
 * 		from the module for functionality that is unimplemented.
 *
 * 		This driver relies on STM32 HAL for functionality. Support for other STM32 libraries
 * 		(i.e. LL or SPL) is unimplemented.
 *
 * 		HOW TO USE THIS DRIVER:
 * 		(1) Configure HAL such that I2C or SPI is initialized, and ensure proper parameters are used
 * 		(2) In mpu9255.h, define either MPU_USE_I2C or MPU_USE_SPI
 * 		(3) In mpu9255.h, define MPU_I2C or MPU_SPI to alias the same hspi/hi2c module
 * 					that HAL is configured to use with the sensor
 * 		(4) In mpu9255.c in MPU_Init(), ensure both MPU_SetAccelFSRange and MPU_SetGyroFSRange
 * 					are called at least once and given user-desired arguments, otherwise this Driver File
 * 					might result in unexpected behaviour.
 *
 *		FILE INFO:
 * 		Author: Georges Troulis
 * 		Email:	gtroulis@ucsd.edu
 * 		Driver Version Number:	0.0.3
 * 		Latest Revision Date:		08/26/2018
 *
 *		Changelog:
 */

#include "mpu9255.h"
#include "stm32f4xx_hal.h"
#include "math.h"

/*------------------------------------------------------------*/
// Private Variables
/*------------------------------------------------------------*/
static float gyroFSScale = 0;
static float accelFSScale = 0;

#ifdef MPU_USE_SPI
	uint8_t spiTxBuf[32];
#endif

/**
 * 	Initializes the accelerometer, and certain private variables
 * 	pertaining to the functionality of this driver.
 *
 *	The parameters to MPU_SetAccelFSRange and MPU_SetGyroFSRange
 *	may change depending on the application, but both functions
 *	must be called at least once.
 */
HAL_StatusTypeDef MPU_Init(void) {
	HAL_StatusTypeDef status = 0;

	status |= MPU_SetAccelFSRange(MPU_ACCEL_FS_2G);
	status |= MPU_SetGyroFSRange(MPU_GYRO_FS_1000DPS);

	return (status == HAL_OK ? HAL_OK : HAL_ERROR);
}

/**
 * 	Sets the full scale range of the accelerometer,
 * 	and updates the scaling factor for acceleration values read
 * 	from the sensor
 *
 * 	fsRange:	The new full scale range of the accelerometer. Use the
 * 						macros defined in mpu9255.h to ensure correct functionality
 */
HAL_StatusTypeDef MPU_SetAccelFSRange(uint8_t fsRange) {
	accelFSScale = pow(2, fsRange) / 16384;

	return MPU_WriteBits(MPU_ACCEL_CFG1_REG, fsRange, 4, 2);
}


/**
 * 	Sets the full scale range of the gyroscope,
 * 	and updates the scaling factor for gyroscope values read
 * 	from the sensor
 *
 * 	fsRange:	The new full scale range of the gyroscope. Use the
 * 						macros defined in mpu9255.h to ensure correct functionality
 */
HAL_StatusTypeDef MPU_SetGyroFSRange(uint8_t fsRange) {
	gyroFSScale = pow(2, fsRange) / 232;

	return MPU_WriteBits(MPU_GYRO_CFG_REG, fsRange, 4, 2);
}

/**
 * 	Reads the x, y, and z gyroscope values in burst-read mode, and scales them
 * 	according to the full scale range of the gyroscope.
 *
 * 	The scaling factor that is applied is calculated during the call to
 * 	MPU_SetAccelFSRange(). Therefore no mater what FS range is selected,
 * 	the values returned from this function will be very similar, but with
 * 	varying measurement precision
 *
 * 	ax:	memory location into which to place the x gyroscope value
 * 	ay:	memory location into which to place the y gyroscope value
 * 	az:	memory location into which to place the z gyroscope value
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
 * 	Reads the x, y, and z accelerometer values in burst-read mode, and scales them
 * 	according to the full scale range of the accelerometer.
 *
 * 	The scaling factor that is applied is calculated during the call to
 * 	MPU_SetAccelFSRange(). Therefore no mater what FS range is selected,
 * 	the values returned from this function will be very similar, but with
 * 	varying measurement precision
 *
 * 	ax:	memory location into which to place the x acceleration
 * 	ay:	memory location into which to place the y acceleration
 * 	az:	memory location into which to place the z acceleration
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
 * 	Reads the x, y, and z gyroscope values in burst-read mode
 * 	The datasheet guarantees that all readings will be from the same sampling interval
 * 	during a burst-read.
 *
 * 	The gyroscope values are the raw bit values, and need scaling to be interpreted
 * 	properly. The scaling depends on the full-scale range that is set in the init function
 * 	of this driver
 *
 * 	ax:	memory location into which to place the x gyroscope value
 * 	ay:	memory location into which to place the y gyroscope value
 * 	az:	memory location into which to place the z gyroscope value
 */
HAL_StatusTypeDef MPU_GetGyroscopeRaw(int16_t *gx, int16_t *gy, int16_t *gz) {
	HAL_StatusTypeDef status;
	uint8_t buf[6];

	status = MPU_ReadLen(MPU_GYRO_XOUTH_REG, 6, buf);
	*gx = ((uint16_t) buf[0] << 8) | buf[1];
	*gy = ((uint16_t) buf[2] << 8) | buf[3];
	*gz = ((uint16_t) buf[4] << 8) | buf[5];

	return status;
}

/**
 * 	Reads the x, y, and z accelerometer values in burst-read mode
 * 	The datasheet guarantees that all readings will be from the same sampling interval
 * 	during a burst-read.
 *
 * 	The acceleration values are the raw bit values, and need scaling to be interpreted
 * 	properly. The scaling depends on the full-scale range that is set in the init function
 * 	of this driver
 *
 * 	ax:	memory location into which to place the x acceleration
 * 	ay:	memory location into which to place the y acceleration
 * 	az:	memory location into which to place the z acceleration
 */
HAL_StatusTypeDef MPU_GetAccelerationsRaw(int16_t *ax, int16_t *ay, int16_t *az) {
	HAL_StatusTypeDef status;
	uint8_t buf[6];

	status = MPU_ReadLen(MPU_ACCEL_XOUTH_REG, 6, buf);
	*ax = ((uint16_t) buf[0] << 8) | buf[1];
	*ay = ((uint16_t) buf[2] << 8) | buf[3];
	*az = ((uint16_t) buf[4] << 8) | buf[5];

	return status;
}


/**
 * TODO THIS METHOD IS MALFUNCTIONAL
 *
 * 	Reads the internal temperature of the sensor
 *
 * 	temp:	pointer to a variable into which to store the sensor temperature
 */
HAL_StatusTypeDef MPU_GetTemperature(float* temp) {
	HAL_StatusTypeDef status;
	uint8_t buf[2];
	uint16_t raw;

	status = MPU_ReadLen(MPU_TEMP_OUTH_REG, 2, buf);
	raw = ((uint16_t) buf[0] << 8) | buf[1];
	*temp = 36.53 + ((float) raw) / 340;
	*temp *= 100;

	return status;
}

/**
 * 	For lazy debug purposes only, to test singular register reads without implementing
 * 	an entire extra method to handle functionality. Has undocumented behaviour
 */

HAL_StatusTypeDef MPU_Surprise(uint16_t* output) {
	HAL_StatusTypeDef status;
	uint8_t buf[2];

	status = MPU_ReadLen(MPU_ACCEL_XOUTH_REG, 2, buf);
	*output = ((uint16_t) buf[0] << 8) | buf[1];

	return status;
}

/**
 * 	Writes a bit pattern to a sub-section of a register. This is useful for writing
 * 	to registers where a certain configuration option does not span the entire register length.
 * 	Ex: To set the Accel Full Scale, only bits <4:3> of MPU_ACCEL_CFG1_REG need to be written.
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
