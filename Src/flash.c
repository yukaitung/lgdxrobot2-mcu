#include <string.h>
#include <stdbool.h>
#include "main.h"
#include "stm32f4xx_hal_flash.h"
#include "stm32f4xx_hal_flash_ex.h"
#include "flash.h"
#include "motor.h"

bool _data_empty = true;
flash_data _data = {0};

void _read_from_flash()
{
  if (_data_empty)
  {
		flash_data temp_data = {0};
    uint8_t *flash = (uint8_t*)(uintptr_t)FLAST_START_ADDRESS;
	  memcpy(&temp_data, flash, sizeof(flash_data));
		if (temp_data.modified == FLASH_DATA_MODIFIED)
		{
			_data = temp_data;
		}
    _data_empty = false;
  }
}

/*
 * Public Functions
 */

flash_data Flash_Get()
{
  _read_from_flash();
  return _data;
}

void Flash_Save()
{
	_data.modified = FLASH_DATA_MODIFIED;
  for(int level = 0; level < PID_LEVEL; level++)
	{
		for(int motor = 0; motor < API_MOTOR_COUNT; motor++)
		{
			_data.pid[level][motor].p = MOTOR_Get_Pid(motor, level, 0);
			_data.pid[level][motor].i = MOTOR_Get_Pid(motor, level, 1);
			_data.pid[level][motor].d = MOTOR_Get_Pid(motor, level, 2);
		}
		_data.pid_speed[level] = MOTOR_Get_Pid_Speed(level);
	}
	for (int motor = 0; motor < API_MOTOR_COUNT; motor++)
	{
		_data.motors_maximum_speed[motor] = MOTOR_Get_Maximum_Speed(motor);
	}
	uint8_t* data = (uint8_t*) &_data;
	HAL_FLASH_Unlock();
	FLASH_Erase_Sector(6, FLASH_VOLTAGE_RANGE_3);
	for(int i = 0; i < sizeof(flash_data); i++)
	{
		HAL_FLASH_Program(FLASH_TYPEPROGRAM_BYTE, FLAST_START_ADDRESS + i, data[i]);
	}
	HAL_FLASH_Lock();
}