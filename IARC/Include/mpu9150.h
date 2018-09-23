/**
 *     mpu9150.h:  Defines device registers for the MPU9150 device.
 *
 *     This file is part of the mpu_common driver. See mpu_common.h for more info.
 *
 *     This file is included by mpu_common.h. Do not include this file manually.
 *
 *    ================================================================================
 *
 *     Author:               Georges Troulis
 *     Email:                gtroulis@ucsd.edu
 *
 *    ================================================================================
 *
 *    Changelog:
 */

#ifndef MPU9150_H
#define MPU9150_H

#define SELF_TESTX_REG    0x0D
#define SELF_TESTY_REG    0x0E
#define SELF_TESTZ_REG    0x0F
#define SELF_TESTA_REG    0x10

#define SMPLRT_REG        0x19
#define CFG_REG           0x1A
#define GYRO_CFG_REG      0x1B
#define ACCEL_CFG1_REG    0x1C  // Only 1 config register for this device

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

#define SIGPATH_RST_REG   0x68
#define USER_CTRL_REG     0x6A
#define PWR_MGMT1_REG     0x6B
#define PWR_MGMT2_REG     0x6C
#define FIFO_CNTH_REG     0x72
#define FIFO_CNTL_REG     0x73
#define FIFO_RW_REG       0x74
#define WHO_AM_I_REG      0x75

#define MPU_ADDR          0x68

#endif
