#include "mpu6050.h"
#include "stm32f4xx_hal.h"

HAL_StatusTypeDef MPU_Init() {
	// set clock source to MPU6050_CLOCK_PLL_XGYRO
	MPU_Write_Bits(MPU_PWR_MGMT1_REG, 0x01, 2, 2);

	// set FS Gyro Range to 0x01
	MPU_Set_Gyro_Fsr(0x01);

	// set FS Accel Range to 0x01
	MPU_Set_Accel_Fsr(0x01);

	// disable sleep
	MPU_Write_Bits(MPU_PWR_MGMT1_REG, 0, 6, 1);
}

HAL_StatusTypeDef MPU_Set_Gyro_Fsr(uint8_t fsr) {
	return MPU_Write_Byte(MPU_GYRO_CFG_REG, fsr << 3);
}

HAL_StatusTypeDef MPU_Set_Accel_Fsr(uint8_t fsr) {
	return MPU_Write_Byte(MPU_ACCEL_CFG_REG, fsr << 3);
}

HAL_StatusTypeDef MPU_Set_LPF(uint16_t lpf) {
	uint8_t data = 0;
	if (lpf >= 188)
		data = 1;
	else if (lpf >= 98)
		data = 2;
	else if (lpf >= 42)
		data = 2;
	else if (lpf >= 42)
		data = 3;
	else if (lpf >= 20)
		data = 4;
	else if (lpf >= 10)
		data = 5;
	else
		data = 6;
	return MPU_Write_Byte(MPU_CFG_REG, data);
}

HAL_StatusTypeDef MPU_Set_Rate(uint16_t rate) {
	uint8_t data;
	if (rate > 1000)
		rate = 1000;
	if (rate < 4)
		rate = 4;
	data = 1000 / rate - 1;
	return MPU_Write_Byte(MPU_SAMPLE_RATE_REG, data) || MPU_Set_LPF(rate / 2);
}

float MPU_Get_Temperature(void) {
	uint8_t buf[2];
	uint16_t raw;
	float temp;
	MPU_Read_Len(MPU_TEMP_OUTH_REG, 2, buf);
	raw = ((uint16_t) buf[0] << 8) | buf[1];
	temp = 36.53 + ((double) raw) / 340;
	return temp * 100;
}

HAL_StatusTypeDef MPU_Get_Gyroscope(uint16_t *gx, uint16_t *gy, uint16_t *gz) {
	uint8_t buf[6], res;
	HAL_StatusTypeDef hal_status;
	hal_status = MPU_Read_Len(MPU_GYRO_XOUTH_REG, 6, buf);
	if (hal_status == HAL_OK) {
		*gx = ((uint16_t) buf[0] << 8) | buf[1];
		*gy = ((uint16_t) buf[2] << 8) | buf[3];
		*gz = ((uint16_t) buf[4] << 8) | buf[5];
	}
	return hal_status;
}

int16_t MPU_GetAccelerationX() {
	uint8_t buffer[2] = {0};
	MPU_Read_Len(MPU_ACCEL_XOUTH_REG, 2, buffer);
	return (((int16_t)buffer[0]) << 8) | buffer[1];
}

HAL_StatusTypeDef MPU_Get_Accelerometer(uint16_t *ax, uint16_t *ay, uint16_t *az) {
	uint8_t buf[6];
	HAL_StatusTypeDef hal_status;
	hal_status = MPU_Read_Len(MPU_ACCEL_XOUTH_REG, 6, buf);
	if (hal_status == HAL_OK) {
		*ax = ((uint16_t) buf[0] << 8) | buf[1];
		*ay = ((uint16_t) buf[2] << 8) | buf[3];
		*az = ((uint16_t) buf[4] << 8) | buf[5];
	}
	return hal_status;
}

HAL_StatusTypeDef MPU_Write_Bits(uint8_t reg, uint8_t bitsToWrite, uint8_t bitStart, uint8_t numBits) {
	uint8_t prevRegVal;
	HAL_StatusTypeDef status = MPU_Read_Byte(reg, &prevRegVal);
	if (status != HAL_OK) {
		return status;
	}

  uint8_t mask = ((1 << numBits) - 1) << (bitStart - numBits + 1);
  bitsToWrite <<= (bitStart - numBits + 1); // shift data into correct position
  bitsToWrite &= mask; // zero all non-important bits in data
  prevRegVal &= ~(mask); // zero all important bits in existing byte
  prevRegVal |= bitsToWrite; // combine data with existing byte

  return MPU_Write_Byte(reg, prevRegVal);
}

HAL_StatusTypeDef MPU_Write_Byte(uint8_t reg, uint8_t data) {
	HAL_StatusTypeDef status = HAL_I2C_Mem_Write(&mpu_i2c, MPU_ADDR, reg, 1, &data, 1, MPU_DEFAULT_TIMEOUT);
	return status;
}

HAL_StatusTypeDef MPU_Read_Byte(uint8_t reg, uint8_t* res) {
	HAL_StatusTypeDef status = HAL_I2C_Mem_Read(&mpu_i2c, MPU_ADDR, reg, 1, res, 1, MPU_DEFAULT_TIMEOUT);
	return status;
}

HAL_StatusTypeDef MPU_Write_Len(uint8_t reg, uint8_t len, uint8_t *buf) {
	HAL_StatusTypeDef status = HAL_I2C_Master_Transmit(&mpu_i2c, MPU_ADDR, buf, len, MPU_DEFAULT_TIMEOUT);
	return status;
}

HAL_StatusTypeDef MPU_Read_Len(uint8_t reg, uint8_t len, uint8_t *buf) {
	HAL_StatusTypeDef status = HAL_I2C_Master_Receive(&mpu_i2c, MPU_ADDR, buf, len, MPU_DEFAULT_TIMEOUT);
	return status;
}

////////////////////////////////////////////////////////////////////////////////
//	Garbage/debug stuff
////////////////////////////////////////////////////////////////////////////////

/*
// Old init function
HAL_StatusTypeDef MPU_Init(void) {

	MPU_Write_Byte(MPU_PWR_MGMT1_REG, 0x80);
	HAL_Delay(100);
	MPU_Write_Byte(MPU_PWR_MGMT1_REG, 0x00);

	MPU_Set_Gyro_Fsr(0);
	MPU_Set_Accel_Fsr(0);
	MPU_Set_Rate(200);

	MPU_Write_Byte(MPU_INT_EN_REG, 0x00);
	MPU_Write_Byte(MPU_USER_CTRL_REG, 0x00);
	MPU_Write_Byte(MPU_FIFO_EN_REG, 0x00);
	MPU_Write_Byte(MPU_INTBP_CFG_REG, 0x80);

	uint8_t res;
	MPU_Read_Byte(MPU_DEVICE_ID_REG, &res);
	if (res<<1 == MPU_ADDR) {
		MPU_Write_Byte(MPU_PWR_MGMT1_REG, 0x02);
		MPU_Write_Byte(MPU_PWR_MGMT2_REG, 0x00);
		MPU_Set_Rate(200);
	}
	else {
		return HAL_OK;
	}

	return HAL_ERROR;
}
*/
