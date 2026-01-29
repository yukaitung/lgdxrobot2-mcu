#ifndef IMU_H
#define IMU_H

#include "lgdxrobot2.h"

#define TIMEOUT_MS 100

#define REG_BANK_SEL 0x7F

// Bank 0
#define USER_CTRL 0x03
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

// Gyro Precision
#define GYRO_250_DPS 0x00
#define GYRO_500_DPS 0x01
#define GYRO_1000_DPS 0x02
#define GYRO_2000_DPS 0x03

// Accel Precision
#define ACCEL_2G 0x00
#define ACCEL_4G 0x01
#define ACCEL_8G 0x02
#define ACCEL_16G 0x03

// ROS
#define G_TO_M_S2 9.80665
#define DEG_TO_RAD 0.01745329252

void IMU_Init(SPI_HandleTypeDef *hspi);
void IMU_Read_Start();
McuImuDof IMU_Get_Data();

#endif