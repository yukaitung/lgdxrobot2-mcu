#include "lgdxrobot2.h"

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