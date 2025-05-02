#include <math.h>
#include <stdlib.h>
#include "motor.h"
#include "main.h"

#define M_PI 3.14159265358979323846

// Constant from motor.h
MOTOR_MAX_SPEED
PID_KP
PID_KI
PID_KD

uint32_t pid_last_tick = 0;
uint32_t pid_elapsed = 0;
float pid_accumulate_error[WHEEL_COUNT] = {0, 0, 0, 0};
float pid_last_error[WHEEL_COUNT] = {0, 0, 0, 0};
float motor_velocity[WHEEL_COUNT] = {0, 0, 0, 0};
float motor_last_velocity[WHEEL_COUNT] = {0, 0, 0, 0}; // Discard anomaly in encoder
float motor_target_velocity[WHEEL_COUNT] = {0, 0, 0, 0};
uint16_t encoder_value[WHEEL_COUNT] = {0, 0, 0, 0};
uint16_t encoder_last_value[WHEEL_COUNT] = {0, 0, 0, 0};
float motor_transform[3] = {0, 0, 0};
float motor_forward_kinematic[3] = {0, 0, 0};
bool using_external_imu = false;
float accel_vector = 0;

enum __e_Stop {
  software_estop,
  hardware_estop,
	estop_count
} E_stop;
bool e_stop_enabled[estop_count] = {false, false};

TIM_HandleTypeDef *m_pwm_htim;
TIM_HandleTypeDef *motor_htim[WHEEL_COUNT];

//
// Private Function
//

int safe_unsigned_subtract(int a, int b, int limit)
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

void MOTOR_Set_Pwm(int motor, uint32_t CCR)
{
	if(CCR == 0)
	{
		pid_accumulate_error[motor] = 0;
	}
	if(CCR > m_pwm_htim->Init.Period)
	{
		CCR = m_pwm_htim->Init.Period;
	}
	switch(motor)
	{
		case 0:
			TIM2->CCR1 = CCR;
			break;
		case 1:
			TIM2->CCR2 = CCR;
			break;
		case 2:
			TIM2->CCR4 = CCR;
			break;
		case 3:
			TIM2->CCR3 = CCR;
			break;
	}
}

void MOTOR_Set_Direction(int motor, bool forward)
{
	switch(motor)
	{
		case 0:
			if(forward)
			{
				HAL_GPIO_WritePin(M1_IN2_GPIO_Port, M1_IN2_Pin, GPIO_PIN_SET);
				HAL_GPIO_WritePin(M1_IN1_GPIO_Port, M1_IN1_Pin, GPIO_PIN_RESET);
			}
			else
			{
				HAL_GPIO_WritePin(M1_IN2_GPIO_Port, M1_IN2_Pin, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(M1_IN1_GPIO_Port, M1_IN1_Pin, GPIO_PIN_SET);
			}
			break;
		case 1:
			if(forward)
			{
				HAL_GPIO_WritePin(M2_IN1_GPIO_Port, M2_IN1_Pin, GPIO_PIN_SET);
				HAL_GPIO_WritePin(M2_IN2_GPIO_Port, M2_IN2_Pin, GPIO_PIN_RESET);
			}
			else
			{
				HAL_GPIO_WritePin(M2_IN1_GPIO_Port, M2_IN1_Pin, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(M2_IN2_GPIO_Port, M2_IN2_Pin, GPIO_PIN_SET);
			}
			break;
		case 2:
			if(forward)
			{
				HAL_GPIO_WritePin(M3_IN2_GPIO_Port, M3_IN2_Pin, GPIO_PIN_SET);
				HAL_GPIO_WritePin(M3_IN1_GPIO_Port, M3_IN1_Pin, GPIO_PIN_RESET);
			}
			else
			{
				HAL_GPIO_WritePin(M3_IN2_GPIO_Port, M3_IN2_Pin, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(M3_IN1_GPIO_Port, M3_IN1_Pin, GPIO_PIN_SET);
			}
			break;
		case 3:
			if(forward)
			{
				HAL_GPIO_WritePin(M4_IN1_GPIO_Port, M4_IN1_Pin, GPIO_PIN_SET);
				HAL_GPIO_WritePin(M4_IN2_GPIO_Port, M4_IN2_Pin, GPIO_PIN_RESET);
			}
			else
			{
				HAL_GPIO_WritePin(M4_IN1_GPIO_Port, M4_IN1_Pin, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(M4_IN2_GPIO_Port, M4_IN2_Pin, GPIO_PIN_SET);
			}
			break;
	}
}

void MOTOR_Reset_LED()
{
	for(int i = 0; i < estop_count; i++)
	{
		if(e_stop_enabled[i])
			return;
	}
	HAL_GPIO_WritePin(L_GREEN_GPIO_Port, L_GREEN_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(L_RED_GPIO_Port, L_RED_Pin, GPIO_PIN_RESET);
}

//
// Public Function
//

void MOTOR_Init(TIM_HandleTypeDef *pwm_htim, TIM_HandleTypeDef *m1_htim, TIM_HandleTypeDef *m2_htim, TIM_HandleTypeDef *m3_htim, TIM_HandleTypeDef *m4_htim)
{
	m_pwm_htim = pwm_htim;
	motor_htim[0] = m1_htim;
	motor_htim[1] = m2_htim;
	motor_htim[2] = m3_htim;
	motor_htim[3] = m4_htim;
	HAL_TIM_PWM_Start(m_pwm_htim, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(m_pwm_htim, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(m_pwm_htim, TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(m_pwm_htim, TIM_CHANNEL_4);
	for(int i = 0; i < WHEEL_COUNT; i++)
	{
		HAL_TIM_Encoder_Start(motor_htim[i], TIM_CHANNEL_ALL);
		pid_last_tick = HAL_GetTick();
	}
	MOTOR_Set_Power(true);
}

float MOTOR_Get_Transform(int axis)
{
	return motor_transform[axis];
}

float MOTOR_Get_Fk(int axis)
{
	return motor_forward_kinematic[axis];
}

float MOTOR_Get_Velocity(int motor)
{
	return motor_velocity[motor];
}
	
float MOTOR_Get_Target_Velocity(int motor)
{
	return motor_target_velocity[motor];
}

float MOTOR_Get_PID(int pid, int motor)
{
	switch(pid)
	{
		case 0:
			return motor_kp[motor];
		case 1:
			return motor_ki[motor];
		case 2:
			return motor_kd[motor];
	}
	return -1;
}

bool MOTOR_Get_E_Stop_Status(int e_stop)
{
	return e_stop_enabled[e_stop];
}

void MOTOR_Set_Power(bool enable)
{
	if(enable)
	{
		HAL_GPIO_WritePin(BT1_SW_GPIO_Port, BT1_SW_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(L_GREEN_GPIO_Port, L_GREEN_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(L_RED_GPIO_Port, L_RED_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(D_STBY_GPIO_Port, D_STBY_Pin, GPIO_PIN_SET);
	}
	else
	{
		HAL_GPIO_WritePin(BT1_SW_GPIO_Port, BT1_SW_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(L_GREEN_GPIO_Port, L_GREEN_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(L_RED_GPIO_Port, L_RED_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(D_STBY_GPIO_Port, D_STBY_Pin, GPIO_PIN_RESET);
	}	
}

void MOTOR_Set_Ik(float velocity_x, float velocity_y, float velocity_w)
{
	motor_target_velocity[0] = (1 / WHEEL_RADIUS) * (velocity_x - velocity_y - (CHASSIS_LX + CHASSIS_LY) * velocity_w);
	motor_target_velocity[1] = (1 / WHEEL_RADIUS) * (velocity_x + velocity_y + (CHASSIS_LX + CHASSIS_LY) * velocity_w);
	motor_target_velocity[2] = (1 / WHEEL_RADIUS) * (velocity_x + velocity_y - (CHASSIS_LX + CHASSIS_LY) * velocity_w);
	motor_target_velocity[3] = (1 / WHEEL_RADIUS) * (velocity_x - velocity_y + (CHASSIS_LX + CHASSIS_LY) * velocity_w);
	for(int i = 0; i < WHEEL_COUNT; i++)
	{
		motor_target_velocity[i] >= 0 ? MOTOR_Set_Direction(i, true) : MOTOR_Set_Direction(i, false);
		if(motor_target_velocity[i] == 0)
			MOTOR_Set_Pwm(i, 0);
	}
}

void MOTOR_Set_Single_Velocity(int motor, float velocity)
{
	motor_target_velocity[motor] = velocity;
	motor_target_velocity[motor] >= 0 ? MOTOR_Set_Direction(motor, true) : MOTOR_Set_Direction(motor, false);
	if(motor_target_velocity[motor] == 0) 
		MOTOR_Set_Pwm(motor, 0);
}

void MOTOR_Set_PID(int motor, float kp, float ki, float kd)
{
	motor_kp[motor] = kp;
	motor_ki[motor] = ki;
	motor_kd[motor] = kd;
}

uint32_t MOTOR_Get_PID_elapsed()
{
	return pid_elapsed;
}

void MOTOR_Set_Software_E_Stop(bool enable)
{
	if(enable)
	{
		HAL_GPIO_WritePin(BT1_SW_GPIO_Port, BT1_SW_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(L_RED_GPIO_Port, L_RED_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(L_GREEN_GPIO_Port, L_GREEN_Pin, GPIO_PIN_RESET);
		e_stop_enabled[software_estop] = true;
		MOTOR_Set_Ik(0, 0, 0);
	}
	else 
	{
		HAL_GPIO_WritePin(BT1_SW_GPIO_Port, BT1_SW_Pin, GPIO_PIN_SET);
		e_stop_enabled[software_estop] = false;
		MOTOR_Reset_LED();
	}
}

void MOTOR_Set_Hardware_E_Stop(bool enable)
{
	if(enable)
	{
		HAL_GPIO_WritePin(L_RED_GPIO_Port, L_RED_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(L_GREEN_GPIO_Port, L_GREEN_Pin, GPIO_PIN_RESET);
		e_stop_enabled[hardware_estop] = true;
		MOTOR_Set_Ik(0, 0, 0);
	}
	else 
	{
		e_stop_enabled[hardware_estop] = false;
		MOTOR_Reset_LED();
	}
}

void MOTOR_Reset_Transform()
{
	for(int i = 0; i < 3; i++)
		motor_transform[i] = 0;
}

void MOTOR_Set_External_IMU(float ax, float ay, float az, float gz)
{
	using_external_imu = true;
	accel_vector = sqrt(ax * ax + ay * ay + az * az);
	motor_transform[2] += gz;
}

void MOTOR_PID()
{
	// Time elapsed
	uint32_t currentTick = HAL_GetTick();
	pid_elapsed = currentTick - pid_last_tick;
	float dt = (float) pid_elapsed / 1000.0;
	pid_last_tick = currentTick;
	
	// PID calculation
	int newPwm[WHEEL_COUNT] = {0, 0, 0, 0};
	for(int i = 0; i < WHEEL_COUNT; i++)
	{
		encoder_value[i] = __HAL_TIM_GET_COUNTER(motor_htim[i]);

		if(i % 2 == 0)
		{
			// Motor 0, 2
			motor_velocity[i] = (safe_unsigned_subtract(encoder_value[i], encoder_last_value[i], motor_htim[i]->Init.Period) / (ENCODER_PPR * dt)) * (2 * M_PI);
		}
		else
		{
			// Motor 1, 3
			motor_velocity[i] = (-1 * safe_unsigned_subtract(encoder_value[i], encoder_last_value[i], motor_htim[i]->Init.Period) / (ENCODER_PPR * dt)) * (2 * M_PI);
		}
		// Discard anomaly in encoder
		if(motor_velocity[i] > motor_max_speed[i] || motor_velocity[i] < -motor_max_speed[i])
			motor_velocity[i] = motor_last_velocity[i];
		
		// Calculate error 
		float error = fabs(motor_target_velocity[i]) - fabs(motor_velocity[i]);
		pid_accumulate_error[i] = pid_accumulate_error[i] + error * dt;
		// Discard overshooting error
		pid_accumulate_error[i] = fmax(pid_accumulate_error[i], -motor_max_speed[i]);
		pid_accumulate_error[i] = fmin(pid_accumulate_error[i], motor_max_speed[i]);
		float error_rate = (error - pid_last_error[i]) / dt;

		if (motor_target_velocity[i] >= PID_DEADZONE_VELOCITY)
		{
			int pid = roundf(((motor_kp[i] * error + motor_ki[i] * pid_accumulate_error[i] + motor_kd[i] * error_rate) / motor_max_speed[i]) * m_pwm_htim->Init.Period);
			newPwm[i] = motor_target_velocity[i] != 0 ? pid : 0;
		}
		else
		{
			int pid = roundf(((1 * error + 1 * pid_accumulate_error[i]) / motor_max_speed[i]) * m_pwm_htim->Init.Period);
			newPwm[i] = motor_target_velocity[i] != 0 ? pid : 0;
		}
		
		pid_last_error[i] = error;
		encoder_last_value[i] = encoder_value[i];
		motor_last_velocity[i] = motor_velocity[i];
	}
	
	// Apply PID
	for(int i = 0; i < WHEEL_COUNT; i++)
	{
		MOTOR_Set_Pwm(i, newPwm[i]);
	}
	
	// Odometry information
	motor_forward_kinematic[0] = (motor_velocity[0] + motor_velocity[1] + motor_velocity[2] + motor_velocity[3]) * (WHEEL_RADIUS / 4);
	motor_forward_kinematic[1] = (-motor_velocity[0] + motor_velocity[1] + motor_velocity[2] - motor_velocity[3]) * (WHEEL_RADIUS / 4);
	// Update w transform if no IMU using or accel meter reports non stop
	if(!using_external_imu || accel_vector >= IMU_STOP)
	{
		motor_transform[0] += (motor_forward_kinematic[0] * cos(motor_transform[2]) - motor_forward_kinematic[1] * sin(motor_transform[2])) * dt;
		motor_transform[1] += (motor_forward_kinematic[0] * sin(motor_transform[2]) + motor_forward_kinematic[1] * cos(motor_transform[2])) * dt;
	}
	// Update w transform if no IMU using
	if(!using_external_imu)
	{
		motor_forward_kinematic[2] = (-motor_velocity[0] + motor_velocity[1] - motor_velocity[2] + motor_velocity[3]) * ((WHEEL_RADIUS * 2) / (M_PI * (CHASSIS_LX + CHASSIS_LY))); // Just guessing to use PI for fk
		motor_transform[2] += (motor_forward_kinematic[2] * dt);
	}
}