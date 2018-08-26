#ifndef __MPU6500_H
#define __MPU9255_H

#include "main.h"

/*------------------------------------------------------------*/
//	Define either MPU_USE_I2C or MPU_USE_SPI
/*------------------------------------------------------------*/

//#define MPU_USE_I2C
#define MPU_USE_SPI

/*------------------------------------------------------------*/
//	Important Macro Definitions
/*------------------------------------------------------------*/

#if defined(MPU_USE_I2C)

	#include "i2c.h"

	// Ensure the proper i2c peripheral is used
	#define MPU_I2C	hi2c1

#elif defined(MPU_USE_SPI)

	#include "spi.h"

	// Ensure the proper spi peripheral is used
	#define MPU_SPI	hspi1

#else
	#error "Must define either MPU_USE_I2C or MPU_USE_SPI in main.h"
#endif

#if defined(MPU_USE_I2C) & defined(MPU_USE_SPI)
	#error "Must only define one of MPU_USE_I2C or MPU_USE_SPI"
#endif

#define SPI_ADDR_ADD_W_BIT(x)		(x)
#define SPI_ADDR_ADD_R_BIT(x)		((x) | 0x80)

/*------------------------------------------------------------*/
//	Device Registers for MPU-9255
/*------------------------------------------------------------*/

#define MPU_SELF_TESTX_G_REG	0x00
#define MPU_SELF_TESTY_G_REG	0x01
#define MPU_SELF_TESTZ_G_REG	0x02
#define MPU_SELF_TESTX_A_REG	0x0D
#define MPU_SELF_TESTY_A_REG	0x0E
#define MPU_SELF_TESTZ_A_REG	0x0F

#define MPU_XG_OFFSET_H				0x13
#define MPU_XG_OFFSET_L				0x14
#define MPU_YG_OFFSET_H				0x15
#define MPU_YG_OFFSET_L				0x16
#define MPU_ZG_OFFSET_H				0x17
#define MPU_ZG_OFFSET_L				0x18

#define MPU_SMPLRT_REG				0x19
#define MPU_CFG_REG						0x1A
#define MPU_GYRO_CFG_REG			0x1B
#define MPU_ACCEL_CFG1_REG		0x1C
#define MPU_ACCEL_CFG2_REG		0x1D

#define MPU_FIFO_EN_REG				0x23
#define MPU_I2CMST_CTRL_REG		0x24

#define MPU_I2CMST_STA_REG		0x36
#define MPU_INT_PIN_CFG_REG		0x37
#define MPU_INT_EN_REG				0x38
#define MPU_INT_STA_REG				0x3A

#define MPU_ACCEL_XOUTH_REG		0x3B
#define MPU_ACCEL_XOUTL_REG		0x3C
#define MPU_ACCEL_YOUTH_REG		0x3D
#define MPU_ACCEL_YOUTL_REG		0x3E
#define MPU_ACCEL_ZOUTH_REG		0x3F
#define MPU_ACCEL_ZOUTL_REG		0x40

#define MPU_TEMP_OUTH_REG			0x41
#define MPU_TEMP_OUTL_REG			0x42

#define MPU_GYRO_XOUTH_REG		0x43
#define MPU_GYRO_XOUTL_REG		0x44
#define MPU_GYRO_YOUTH_REG		0x45
#define MPU_GYRO_YOUTL_REG		0x46
#define MPU_GYRO_ZOUTH_REG		0x47
#define MPU_GYRO_ZOUTL_REG		0x48

#define MPU_SIGPATH_RST_REG		0x68
#define MPU_MDETECT_CTRL_REG	0x69
#define MPU_USER_CTRL_REG			0x6A
#define MPU_PWR_MGMT1_REG			0x6B
#define MPU_PWR_MGMT2_REG			0x6C
#define MPU_FIFO_CNTH_REG			0x72
#define MPU_FIFO_CNTL_REG			0x73
#define MPU_FIFO_RW_REG				0x74
#define MPU_WHO_AM_I_REG			0x75

#define MPU_XA_OFFSET_H_REG		0x77
#define MPU_XA_OFFSET_L_REG		0x78
#define MPU_YA_OFFSET_H_REG		0x7A
#define MPU_YA_OFFSET_L_REG		0x7B
#define MPU_ZA_OFFSET_H_REG		0x7D
#define MPU_ZA_OFFSET_L_REG		0x7E

#define MPU_ADDR              0x68
#define MPU_ADDR_W            (MPU_ADDR << 1)
#define MPU_ADDR_R            (MPU_ADDR << 1) | 1

/*------------------------------------------------------------*/
//	Config Options for certain device registers
/*------------------------------------------------------------*/

// Acceleration Full Scale Range, write to MPU_ACCEL_CFG_REG<4:3>
#define MPU_ACCEL_FS_2G		0x00
#define MPU_ACCEL_FS_4G		0x01
#define MPU_ACCEL_FS_8G		0x02
#define MPU_ACCEL_FS_16G	0x03

// Gyro Full Scale Range, write to MPU_GYRO_CFG_REG<4:3>
#define MPU_GYRO_FS_250DPS	0x00
#define MPU_GYRO_FS_500DPS	0x01
#define MPU_GYRO_FS_1000DPS	0x02
#define MPU_GYRO_FS_2500DPS	0x03


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
