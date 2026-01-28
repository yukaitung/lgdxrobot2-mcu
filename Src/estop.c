
#include <stdbool.h>

#include "main.h"
#include "estop.h"

bool emergency_stops_enabled[emergency_stops_count] = {false, false, false};

void _set_led()
{
	HAL_GPIO_WritePin(D1_GPIO_Port, D1_Pin, GPIO_PIN_SET);
}

void _reset_led()
{
	for(int i = 0; i < emergency_stops_count; i++)
	{
		if(emergency_stops_enabled[i])
			return;
	}
	HAL_GPIO_WritePin(DRxSTBY_GPIO_Port, DRxSTBY_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(D1_GPIO_Port, D1_Pin, GPIO_PIN_RESET);
}

void ESTOP_Init()
{
	_reset_led();
}

void ESTOP_Enable(int type)
{
	if (type < 0 || type > emergency_stops_count)
		return;
	
	HAL_GPIO_WritePin(DRxSTBY_GPIO_Port, DRxSTBY_Pin, GPIO_PIN_RESET);
	_set_led();
	emergency_stops_enabled[type] = true;
}

void ESTOP_Disable(int type)
{
	if (type < 0 || type > emergency_stops_count)
		return;
	
	emergency_stops_enabled[type] = false;
	_reset_led();
}

bool ESTOP_Get_Status(int type)
{
	if (type < 0 || type > emergency_stops_count)
		return false;

	return emergency_stops_enabled[type];
}