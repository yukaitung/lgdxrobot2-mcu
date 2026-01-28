#include <math.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "configuation.h"
#include "main.h"
#include "stm32f4xx_hal_flash.h"
#include "stm32f4xx_hal_flash_ex.h"
#include "motor.h"

// Constant from motor.h
PID_SPEED
MOTOR_MAX_SPEED
PID_KP
PID_KI
PID_KD

int pwm_counter_max = 0;
int encoder_counter_max = 0;

uint32_t pid_elapsed = 0;
uint32_t pid_last_tick = 0;
float pid_accumulate_error[API_MOTOR_COUNT] = {0, 0, 0, 0};
float pid_last_error[API_MOTOR_COUNT] = {0, 0, 0, 0};
int pid_output[API_MOTOR_COUNT] = {0, 0, 0, 0};

float transform[3] = {0, 0, 0};
float motors_forward_kinematic[3] = {0, 0, 0};

float motors_actual_velocity[API_MOTOR_COUNT] = {0, 0, 0, 0}; // The velocity measured by the encoder
float motors_last_velocity[API_MOTOR_COUNT] = {0, 0, 0, 0}; // For discarding the anomaly
float motors_target_velocity[API_MOTOR_COUNT] = {0, 0, 0, 0}; // The velocity set by the user
float motors_desire_velocity[API_MOTOR_COUNT] = {0, 0, 0, 0}; // For slow down gradually

uint16_t encoder_value[API_MOTOR_COUNT] = {0, 0, 0, 0};
uint16_t encoder_last_value[API_MOTOR_COUNT] = {0, 0, 0, 0};

TIM_HandleTypeDef *pwm_htim;
TIM_HandleTypeDef *encoders_htim[API_MOTOR_COUNT];

uint32_t flash_start_address = 0x08040000U;

typedef struct {
	float p;
	float i;
	float d;
} _pid;

#pragma pack(push, 1)
typedef struct {
	uint8_t modified;
	_pid pid[PID_LEVEL][API_MOTOR_COUNT];
	float pid_speed[PID_LEVEL];
	float motors_maximum_speed[API_MOTOR_COUNT];
} _pid_save;
#pragma pack(pop)

/*
 * Private Functions
 */

int _safe_unsigned_subtract(int a, int b, int limit)
{
	limit++;
	// assume limit = 65536
	// 0 - 65535
	if(a < b && b - a > limit)
		return(a + limit - b);
	// 65535 - 0
	if(a > b && a - b > limit)
		return(limit - a + b);
	return a - b;
}



void _set_ccr(int motor, int ccr)
{
	if(ccr > pwm_counter_max)
	{
		ccr = pwm_counter_max;
	}
	switch(motor)
	{
		case 0:
			TIM2->CCR4 = ccr;
			break;
		case 1:
			TIM2->CCR2 = ccr;
			break;
		case 2:
			TIM2->CCR1 = ccr;
			break;
		case 3:
			TIM2->CCR3 = ccr;
			break;
	}
}

void _set_direction(int motor, bool forward)
{
	switch(motor)
	{
		case 0:
			if(forward)
			{
				HAL_GPIO_WritePin(DR1BIN2_GPIO_Port, DR1BIN2_Pin, GPIO_PIN_SET);
				HAL_GPIO_WritePin(DR1BIN1_GPIO_Port, DR1BIN1_Pin, GPIO_PIN_RESET);
			}
			else
			{
				HAL_GPIO_WritePin(DR1BIN2_GPIO_Port, DR1BIN2_Pin, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(DR1BIN1_GPIO_Port, DR1BIN1_Pin, GPIO_PIN_SET);
			}
			break;
		case 1:
			if(forward)
			{
				HAL_GPIO_WritePin(DR2AIN1_GPIO_Port, DR2AIN1_Pin, GPIO_PIN_SET);
				HAL_GPIO_WritePin(DR2AIN2_GPIO_Port, DR2AIN2_Pin, GPIO_PIN_RESET);
			}
			else
			{
				HAL_GPIO_WritePin(DR2AIN1_GPIO_Port, DR2AIN1_Pin, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(DR2AIN2_GPIO_Port, DR2AIN2_Pin, GPIO_PIN_SET);
			}
			break;
		case 2:
			if(forward)
			{
				HAL_GPIO_WritePin(DR1AIN2_GPIO_Port, DR1AIN2_Pin, GPIO_PIN_SET);
				HAL_GPIO_WritePin(DR1AIN1_GPIO_Port, DR1AIN1_Pin, GPIO_PIN_RESET);
			}
			else
			{
				HAL_GPIO_WritePin(DR1AIN2_GPIO_Port, DR1AIN2_Pin, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(DR1AIN1_GPIO_Port, DR1AIN1_Pin, GPIO_PIN_SET);
			}
			break;
		case 3:
			if(forward)
			{
				HAL_GPIO_WritePin(DR2BIN1_GPIO_Port, DR2BIN1_Pin, GPIO_PIN_SET);
				HAL_GPIO_WritePin(DR2BIN2_GPIO_Port, DR2BIN2_Pin, GPIO_PIN_RESET);
			}
			else
			{
				HAL_GPIO_WritePin(DR2BIN1_GPIO_Port, DR2BIN1_Pin, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(DR2BIN2_GPIO_Port, DR2BIN2_Pin, GPIO_PIN_SET);
			}
			break;
	}
}

void _handle_user_velocity(int motor, float target_velocity)
{
	target_velocity >= 0 ? _set_direction(motor, true) : _set_direction(motor, false);
	target_velocity = fabs(target_velocity);
	
	if (target_velocity == 0)
	{
		_set_ccr(motor, 0);
		motors_actual_velocity[motor] = 0;
		motors_desire_velocity[motor] = 0;
		motors_last_velocity[motor] = 0;
		motors_target_velocity[motor] = 0;
		pid_accumulate_error[motor] = 0;
		pid_last_error[motor] = 0;
		pid_output[motor] = 0;
		return;
	}

	target_velocity = fabs(target_velocity);
	if (target_velocity >= fabs(motors_actual_velocity[motor]))
	{
		// Speed up, we can achieve the velocity immediately
		motors_desire_velocity[motor] = target_velocity;
	}
	else
	{
		// Speed down, we need to slow down gradually
		motors_desire_velocity[motor] = fabs(motors_actual_velocity[motor]) - MOTOR_SPEED_RAMP;
		if (motors_desire_velocity[motor] < target_velocity)
		{
			// But not slower than target velocity
			motors_desire_velocity[motor] = target_velocity;
		}
	}
}

/*
 * Public Functions
 */

void _read_pid_from_flash()
{
	_pid_save pid_save = {0};
	uint8_t *flash = (uint8_t*)(uintptr_t)flash_start_address;
	memcpy(&pid_save, flash, sizeof(_pid_save));
	if (pid_save.modified == MCU_HEADER1)
	{
		for(int level = 0; level < PID_LEVEL; level++)
		{
			for(int motor = 0; motor < API_MOTOR_COUNT; motor++)
			{
				motors_Kp[level][motor] = pid_save.pid[level][motor].p;
				motors_Ki[level][motor] = pid_save.pid[level][motor].i;
				motors_Kd[level][motor] = pid_save.pid[level][motor].d;
			}
			pid_speed[level] = pid_save.pid_speed[level];
		}
		for (int motor = 0; motor < API_MOTOR_COUNT; motor++)
		{
			motor_max_speed[motor] = pid_save.motors_maximum_speed[motor];
		}
	}
}

void MOTOR_Init(TIM_HandleTypeDef *pwm_htim1, TIM_HandleTypeDef *e1_htim, TIM_HandleTypeDef *e2_htim, TIM_HandleTypeDef *e3_htim, TIM_HandleTypeDef *e4_htim)
{
	_read_pid_from_flash();

	pwm_htim = pwm_htim1;
	encoders_htim[0] = e1_htim;
	encoders_htim[1] = e2_htim;
	encoders_htim[2] = e3_htim;
	encoders_htim[3] = e4_htim;

	pwm_counter_max = pwm_htim->Init.Period;
	encoder_counter_max = encoders_htim[0]->Init.Period;

	HAL_TIM_PWM_Start(pwm_htim, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(pwm_htim, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(pwm_htim, TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(pwm_htim, TIM_CHANNEL_4);
	for(int i = 0; i < API_MOTOR_COUNT; i++)
	{
		HAL_TIM_Encoder_Start(encoders_htim[i], TIM_CHANNEL_ALL);
	}

	pid_last_tick = HAL_GetTick();
	
	HAL_GPIO_WritePin(RS1_GPIO_Port, RS1_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(DRxSTBY_GPIO_Port, DRxSTBY_Pin, GPIO_PIN_SET);
}

uint32_t MOTOR_Get_Pid_Elapsed()
{
	return pid_elapsed;
}

float MOTOR_Get_Transform(int axis)
{
	if (axis < 0 || axis > 2)
		return 0;
	return transform[axis];
}

float MOTOR_Get_Forward_Kinematic(int axis)
{
	if (axis < 0 || axis > 2)
		return 0;
	return motors_forward_kinematic[axis];
}

float MOTOR_Get_Actual_Velocity(int motor)
{
	if (motor < 0 || motor > API_MOTOR_COUNT)
		return 0;
	return motors_actual_velocity[motor];
}

float MOTOR_Get_Desired_Velocity(int motor)
{
	if (motor < 0 || motor > API_MOTOR_COUNT)
		return 0;
	return motors_desire_velocity[motor];
}

float MOTOR_Get_Target_Velocity(int motor)
{
	if (motor < 0 || motor > API_MOTOR_COUNT)
		return 0;
	return motors_target_velocity[motor];
}

float MOTOR_Get_Pid_Speed(int level)
{
	if (level < 0 || level > PID_LEVEL)
		return 0;
	return pid_speed[level];
}

float MOTOR_Get_Maximum_Speed(int motor)
{
	if (motor < 0 || motor > API_MOTOR_COUNT)
		return 0;
	return motor_max_speed[motor];
}

float MOTOR_Get_Pid(int motor, int level, int k)
{
	if (motor < 0 || motor > API_MOTOR_COUNT)
		return 0;
	if (level < 0 || level > PID_LEVEL)
		return 0;

	switch (k)
	{
		case 0: // Kp
			return motors_Kp[level][motor];
		case 1: // Ki
			return motors_Ki[level][motor];
		case 2: // Kd
			return motors_Kd[level][motor];
	}
	return 0;
}

void MOTOR_Set_Ik(float velocity_x, float velocity_y, float velocity_w)
{
	motors_target_velocity[0] = (1 / WHEEL_RADIUS) * (velocity_x - velocity_y - (CHASSIS_LX + CHASSIS_LY) * velocity_w);
	motors_target_velocity[1] = (1 / WHEEL_RADIUS) * (velocity_x + velocity_y + (CHASSIS_LX + CHASSIS_LY) * velocity_w);
	motors_target_velocity[2] = (1 / WHEEL_RADIUS) * (velocity_x + velocity_y - (CHASSIS_LX + CHASSIS_LY) * velocity_w);
	motors_target_velocity[3] = (1 / WHEEL_RADIUS) * (velocity_x - velocity_y + (CHASSIS_LX + CHASSIS_LY) * velocity_w);
	for(int i = 0; i < API_MOTOR_COUNT; i++)
	{
		_handle_user_velocity(i, motors_target_velocity[i]);
	}
}

void MOTOR_Set_Single_Motor(int motor, float velocity)
{
	if (motor < 0 || motor > API_MOTOR_COUNT)
		return;

	motors_target_velocity[motor] = velocity;
	_handle_user_velocity(motor, motors_target_velocity[motor]);
}

void MOTOR_Set_Temporary_Pid_Speed(float level1, float level2, float level3)
{
	pid_speed[0] = level1;
	pid_speed[1] = level2;
	pid_speed[2] = level3;
}

void MOTOR_Set_Temporary_Pid(int motor, int level, float p, float i, float d)
{
	if (motor < 0 || motor > API_MOTOR_COUNT)
		return;
	if (level < 0 || level > PID_LEVEL)
		return;

	motors_Kp[level][motor] = p;
	motors_Ki[level][motor] = i;
	motors_Kd[level][motor] = d;
}

void MOTOR_Set_Temporary_Maximum_Speed(float speed1, float speed2, float speed3, float speed4)
{
	motor_max_speed[0] = speed1;
	motor_max_speed[1] = speed2;
	motor_max_speed[2] = speed3;
	motor_max_speed[3] = speed4;
}

void MOTOR_Save_Pid()
{
	_pid_save pid_save = {0};
	pid_save.modified = MCU_HEADER1;
	for(int level = 0; level < PID_LEVEL; level++)
	{
		for(int motor = 0; motor < API_MOTOR_COUNT; motor++)
		{
			pid_save.pid[level][motor].p = motors_Kp[level][motor];
			pid_save.pid[level][motor].i = motors_Ki[level][motor];
			pid_save.pid[level][motor].d = motors_Kd[level][motor];
		}
		pid_save.pid_speed[level] = pid_speed[level];
	}
	for (int motor = 0; motor < API_MOTOR_COUNT; motor++)
	{
		pid_save.motors_maximum_speed[motor] = motor_max_speed[motor];
	}
	uint8_t* data = (uint8_t*) &pid_save;
	HAL_FLASH_Unlock();
	FLASH_Erase_Sector(6, FLASH_VOLTAGE_RANGE_3);
	for(int i = 0; i < sizeof(_pid_save); i++)
	{
		HAL_FLASH_Program(FLASH_TYPEPROGRAM_BYTE, flash_start_address + i, data[i]);
	}
	HAL_FLASH_Lock();
}

void MOTOR_Reset_Transform()
{
	for(int i = 0; i < 3; i++)
		transform[i] = 0;
}

_pid get_pid_from_linear_interpolation(int lower_level, int higher_level, int motor, float alpha)
{
	if (alpha <= 0.0f) alpha = 0.0f;
	if (alpha >= 1.0f) alpha = 1.0f;

	_pid pid = {0, 0, 0};
	pid.p = motors_Kp[lower_level][motor] + (alpha * (motors_Kp[higher_level][motor] - motors_Kp[lower_level][motor]));
	pid.i = motors_Ki[lower_level][motor] + (alpha * (motors_Ki[higher_level][motor] - motors_Ki[lower_level][motor]));
	pid.d = motors_Kd[lower_level][motor] + (alpha * (motors_Kd[higher_level][motor] - motors_Kd[lower_level][motor]));
	return pid;
}

_pid _get_pid_from_speed(int motor, float speed)
{
	_pid pid = {0, 0, 0};
	if (speed <= pid_speed[0])
	{
		pid.p = motors_Kp[0][motor];
		pid.i = motors_Ki[0][motor];
		pid.d = motors_Kd[0][motor];
		return pid;
	}
	if (speed >= pid_speed[2])
	{
		pid.p = motors_Kp[2][motor];
		pid.i = motors_Ki[2][motor];
		pid.d = motors_Kd[2][motor];
		return pid;
	}

	if (speed <= pid_speed[1])
	{
		// Speed between level 0 and 1
		float a = (speed - pid_speed[0]) / (pid_speed[1] - pid_speed[0]);
		return get_pid_from_linear_interpolation(0, 1, motor, a);
	}
	else 
	{
		// Speed between level 1 and 2
		float a = (speed - pid_speed[1]) / (pid_speed[2] - pid_speed[1]);
		return get_pid_from_linear_interpolation(1, 2, motor, a);
	}
}

void MOTOR_PID()
{
	// Time elapsed
	uint32_t currentTick = HAL_GetTick();
	pid_elapsed = currentTick - pid_last_tick;
	float dt = (float) pid_elapsed / 1000.0f;
	pid_last_tick = currentTick;
	
	// PID calculation
	for(int i = 0; i < API_MOTOR_COUNT; i++)
	{
		// 1. Get encoder value
		encoder_value[i] = __HAL_TIM_GET_COUNTER(encoders_htim[i]);
		motors_actual_velocity[i] = (_safe_unsigned_subtract(encoder_value[i], encoder_last_value[i], encoder_counter_max) / (ENCODER_PPR * dt)) * (2 * M_PI);

			// Discard anomaly in encoder
		if(motors_actual_velocity[i] > motor_max_speed[i] || motors_actual_velocity[i] < -motor_max_speed[i])
			motors_actual_velocity[i] = motors_last_velocity[i];

		// 2. Calculate PID
			// Get PID from speed
		_pid pid = _get_pid_from_speed(i, motors_desire_velocity[i]);

			// Calaculate error and P
		float error = fabs(motors_desire_velocity[i]) - fabs(motors_actual_velocity[i]);
		float pid_p = pid.p * error;

			// Calcuater I using trapezoidal rule
		float pid_trapezoidal = 0.5 * (error + pid_last_error[i]) * dt;
		pid_accumulate_error[i] += pid_trapezoidal;
		float pid_i = pid.i * pid_accumulate_error[i];

			// Calculate D
		float pid_d = (error - pid_last_error[i]) / dt;

			// Calculate output
		pid_output[i] = roundf(((pid_p + pid_i + pid_d) / motor_max_speed[i]) * pwm_counter_max);
		pid_output[i] = motors_desire_velocity[i] != 0 ? abs(pid_output[i]) : 0;

			// Discard overshooting error
		if (pid_output[i] > pwm_counter_max)
		{
			pid_output[i] = pwm_counter_max;
			if (pid.i != 0.0f && error * pid.i > 0.0f)
			{
				pid_accumulate_error[i] -= pid_trapezoidal;
			}
		}
		else if (pid_output[i] < 0)
		{
			pid_output[i] = 0;
			if (pid.i != 0.0f && error * pid.i < 0.0f)
			{
				pid_accumulate_error[i] -= pid_trapezoidal;
			}
		}

			// Discard error at 0
		if (pid_output[i] == 0)
		{
			pid_accumulate_error[i] = 0;
		}

		// 3. Update State
			// For speed down
		if (motors_desire_velocity[i] != 0 && motors_desire_velocity[i] > fabs(motors_target_velocity[i]))
		{
			motors_desire_velocity[i] -= MOTOR_SPEED_RAMP;
			if (motors_desire_velocity[i] < fabs(motors_target_velocity[i]))
				motors_desire_velocity[i] = fabs(motors_target_velocity[i]);
		}

		pid_last_error[i] = error;
		encoder_last_value[i] = encoder_value[i];
		motors_last_velocity[i] = motors_actual_velocity[i];
	}
	
	// Apply PID
	for(int i = 0; i < API_MOTOR_COUNT; i++)
	{
		_set_ccr(i, pid_output[i]);
	}
	
	// Odometry information
	motors_forward_kinematic[0] = (motors_actual_velocity[0] + motors_actual_velocity[1] + motors_actual_velocity[2] + motors_actual_velocity[3]) * (WHEEL_RADIUS / 4);
	motors_forward_kinematic[1] = (-motors_actual_velocity[0] + motors_actual_velocity[1] + motors_actual_velocity[2] - motors_actual_velocity[3]) * (WHEEL_RADIUS / 4);
	motors_forward_kinematic[2] = (-motors_actual_velocity[0] + motors_actual_velocity[1] - motors_actual_velocity[2] + motors_actual_velocity[3]) * (WHEEL_RADIUS / (4 * (CHASSIS_LX + CHASSIS_LY )));
	transform[0] += (motors_forward_kinematic[0] * cos(transform[2]) - motors_forward_kinematic[1] * sin(transform[2])) * dt;
	transform[1] += (motors_forward_kinematic[0] * sin(transform[2]) + motors_forward_kinematic[1] * cos(transform[2])) * dt;
	transform[2] += (motors_forward_kinematic[2] * dt);
}