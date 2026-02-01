#ifndef IMU_H
#define IMU_H

#include "lgdxrobot2.h"

#define TIMEOUT_MS 100

#define REG_BANK_SEL 0x7F

// Bank 0
#define USER_CTRL 0x03
#define LP_CONFIG 0x05
#define PWR_MGMT_1 0x06
#define ACCEL_XOUT_H 0x2D

// Bank 1
#define XA_OFFS_H 0x14
#define XA_OFFS_L 0x15
#define YA_OFFS_H 0x17
#define YA_OFFS_L 0x18
#define ZA_OFFS_H 0x1A
#define ZA_OFFS_L 0x1B

// Bank 2
#define GYRO_SMPLRT_DIV 0x00
#define GYRO_CONFIG_1 0x01
#define XG_OFFS_USRH 0x03
#define XG_OFFS_USRL 0x04
#define YG_OFFS_USRH 0x05
#define YG_OFFS_USRL 0x06
#define ZG_OFFS_USRH 0x07
#define ZG_OFFS_USRL 0x08
#define ODR_ALIGN_EN 0x09
#define ACCEL_SMPLRT_DIV_1 0x10
#define ACCEL_SMPLRT_DIV_2 0x11
#define ACCEL_CONFIG 0x14

// Bank 3
#define I2C_MST_ODR_CONFIG 0x00
#define I2C_MST_CTRL 0x01
#define I2C_SLV0_ADDR 0x03
#define I2C_SLV0_REG 0x04
#define I2C_SLV0_CTRL 0x05
#define I2C_SLV0_DO 0x06
#define I2C_SLV4_ADDR 0x13
#define I2C_SLV4_REG 0x14
#define I2C_SLV4_CTRL 0x15
#define I2C_SLV4_DO 0x16

// Magentometer
#define MAG_STEP 0.15f
#define MAG_ADDRESS 0x0C
#define MAG_ST1 0x10
#define MAG_CNTL2 0x31
#define MAG_CNTL3 0x32

void IMU_Init(SPI_HandleTypeDef *hspi);
void IMU_Read_Start();
McuImuDof IMU_Get_Data();

#endif