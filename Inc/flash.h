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
} flash_data;
#pragma pack(pop)

flash_data Flash_Get();
void Flash_Save();