#include <math.h>
#include <stdlib.h>

#include "configuation.h"
#include "main.h"
#include "motor.h"

// Constant from motor.h
PID_LEVEL_VELOCITY
MOTOR_MAX_SPEED
PID_KP
PID_KI
PID_KD

int pwm_counter_max = 0;
int encoder_counter_max = 0;

uint32_t pid_last_tick = 0;
float pid_accumulate_error[API_MOTOR_COUNT] = {0, 0, 0, 0};
float pid_last_error[API_MOTOR_COUNT] = {0, 0, 0, 0};
int pid_output[API_MOTOR_COUNT] = {0, 0, 0, 0};

float transform[3] = {0, 0, 0};

float motors_actual_velocity[API_MOTOR_COUNT] = {0, 0, 0, 0}; // The velocity measured by the encoder
float motors_last_velocity[API_MOTOR_COUNT] = {0, 0, 0, 0}; // For discarding the anomaly
float motors_target_velocity[API_MOTOR_COUNT] = {0, 0, 0, 0}; // The velocity set by the user
float motors_desire_velocity[API_MOTOR_COUNT] = {0, 0, 0, 0}; // For slow down gradually

uint16_t encoder_value[API_MOTOR_COUNT] = {0, 0, 0, 0};
uint16_t encoder_last_value[API_MOTOR_COUNT] = {0, 0, 0, 0};

bool emergency_stops_enabled[emergency_stops_count] = {false, false};

TIM_HandleTypeDef *pwm_htim;
TIM_HandleTypeDef *encoders_htim[API_MOTOR_COUNT];

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

void _set_led(bool green)
{
	if (green)
	{
		HAL_GPIO_WritePin(D1_GPIO_Port, D1_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(D2_GPIO_Port, D2_Pin, GPIO_PIN_RESET);
	}
	else
	{
		HAL_GPIO_WritePin(D1_GPIO_Port, D1_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(D2_GPIO_Port, D2_Pin, GPIO_PIN_SET);
	}
}

void _reset_led()
{
	for(int i = 0; i < emergency_stops_count; i++)
	{
		if(emergency_stops_enabled[i])
			return;
	}
	HAL_GPIO_WritePin(DRxSTBY_GPIO_Port, DRxSTBY_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(D1_GPIO_Port, D1_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(D2_GPIO_Port, D2_Pin, GPIO_PIN_RESET);
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
			TIM2->CCR1 = ccr;
			break;
		case 1:
			TIM2->CCR2 = ccr;
			break;
		case 2:
			TIM2->CCR4 = ccr;
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
		motors_desire_velocity[motor] = 0;
		_set_ccr(motor, 0);
		pid_accumulate_error[motor] = 0;
		pid_last_error[motor] = 0;
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

void MOTOR_Init(TIM_HandleTypeDef *pwm_htim, TIM_HandleTypeDef *e1_htim, TIM_HandleTypeDef *e2_htim, TIM_HandleTypeDef *e3_htim, TIM_HandleTypeDef *e4_htim)
{
	pwm_htim = pwm_htim;
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
	_set_led(true);
}

float MOTOR_Get_Transform(int axis)
{
	return transform[axis];
}

float MOTOR_Get_Actual_Velocity(int motor)
{
	return motors_actual_velocity[motor];
}

float MOTOR_Get_Desired_Velocity(int motor)
{
	return motors_desire_velocity[motor];
}

float MOTOR_Get_Target_Velocity(int motor)
{
	return motors_target_velocity[motor];
}

float MOTOR_Get_Level_Velocity(int level)
{
	return level_velocity[level];
}

float MOTOR_Get_Pid(int motor, int level, int k)
{
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

bool MOTOR_Get_Emergency_Stop_Status(int type)
{
	return emergency_stops_enabled[type];
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
	motors_target_velocity[motor] = velocity;
	_handle_user_velocity(motor, motors_target_velocity[motor]);
}

void MOTOR_Set_Temporary_Level_Velocity(float level1, float level2, float level3)
{
	level_velocity[0] = level1;
	level_velocity[1] = level2;
	level_velocity[2] = level3;
}

void MOTOR_Set_Temporary_Pid(int motor, int level, float p, float i, float d)
{
	motors_Kp[level][motor] = p;
	motors_Ki[level][motor] = i;
	motors_Kd[level][motor] = d;
}

void MOTOR_Save_Pid()
{
	// TODO
}

void MOTOR_Set_Emergency_Stop(int type, bool enable)
{
	if (enable)
	{
		HAL_GPIO_WritePin(DRxSTBY_GPIO_Port, DRxSTBY_Pin, GPIO_PIN_RESET);
		_set_led(false);
		emergency_stops_enabled[type] = true;
	}
	else
	{
		emergency_stops_enabled[type] = false;
		_reset_led();
	}
}

void MOTOR_Reset_Transform()
{
	for(int i = 0; i < 3; i++)
		transform[i] = 0;
}

void MOTOR_PID()
{
	// Time elapsed
	uint32_t currentTick = HAL_GetTick();
	uint32_t pid_elapsed = currentTick - pid_last_tick;
	float dt = (float) pid_elapsed / 1000.0f;
	pid_last_tick = currentTick;
	
	// PID calculation
	for(int i = 0; i < API_MOTOR_COUNT; i++)
	{
		encoder_value[i] = __HAL_TIM_GET_COUNTER(encoders_htim[i]);
		motors_actual_velocity[i] = (_safe_unsigned_subtract(encoder_value[i], encoder_last_value[i], encoder_counter_max) / (ENCODER_PPR * dt)) * (2 * M_PI);

		// Discard anomaly in encoder
		if(motors_actual_velocity[i] > motor_max_speed[i] || motors_actual_velocity[i] < -motor_max_speed[i])
			motors_actual_velocity[i] = motors_last_velocity[i];
		
		// Calculate error
		float error = fabs(motors_desire_velocity[i]) - fabs(motors_actual_velocity[i]);
		pid_accumulate_error[i] = pid_accumulate_error[i] + error * dt;
		// Discard overshooting error
		if (pid_accumulate_error[i] > motor_max_speed[i] * 0.3f)
			pid_accumulate_error[i] = motor_max_speed[i] * 0.3f;
		if (pid_accumulate_error[i] < -motor_max_speed[i] * 0.3f)
			pid_accumulate_error[i] = -motor_max_speed[i] * 0.3f;
		float error_rate = (error - pid_last_error[i]) / dt;

		pid_output[i] = roundf(((motors_Kp[0][i] * error + motors_Ki[0][i] * pid_accumulate_error[i] + motors_Kd[0][i] * error_rate) / motor_max_speed[i]) * pwm_counter_max);
		pid_output[i] = motors_desire_velocity[i] != 0 ? abs(pid_output[i]) : 0;

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
	float motors_forward_kinematic[3] = {0, 0, 0};
	motors_forward_kinematic[0] = (motors_actual_velocity[0] + motors_actual_velocity[1] + motors_actual_velocity[2] + motors_actual_velocity[3]) * (WHEEL_RADIUS / 4);
	motors_forward_kinematic[1] = (-motors_actual_velocity[0] + motors_actual_velocity[1] + motors_actual_velocity[2] - motors_actual_velocity[3]) * (WHEEL_RADIUS / 4);
	motors_forward_kinematic[2] = (-motors_actual_velocity[0] + motors_actual_velocity[1] - motors_actual_velocity[2] + motors_actual_velocity[3]) * ((WHEEL_RADIUS * 2) / (M_PI * (CHASSIS_LX + CHASSIS_LY))); // Just guessing to use PI for fk
	transform[0] += (motors_forward_kinematic[0] * cos(transform[2]) - motors_forward_kinematic[1] * sin(transform[2])) * dt;
	transform[1] += (motors_forward_kinematic[0] * sin(transform[2]) + motors_forward_kinematic[1] * cos(transform[2])) * dt;
	transform[2] += (motors_forward_kinematic[2] * dt);
}