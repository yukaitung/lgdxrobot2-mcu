#ifndef __FLASH_H
#define __FLASH_H

#include "lgdxrobot2.h"

#define FLASH_DATA_MODIFIED 0xAA
#define FLAST_START_ADDRESS 0x08040000U

typedef struct {
	float p;
	float i;
	float d;
} _pid;

#pragma pack(push, 1)
typedef struct {
	uint8_t modified;
  // PID
	_pid pid[PID_LEVEL][API_MOTOR_COUNT];
	float pid_speed[PID_LEVEL];
	float motors_maximum_speed[API_MOTOR_COUNT];
	// Mag
	float mag_hard_iron_max[3];
	float mag_hard_iron_min[3];
	float mag_soft_iron_matrix[9];
} flash_data;
#pragma pack(pop)

flash_data Flash_Get();
void Flash_Save();

#endif