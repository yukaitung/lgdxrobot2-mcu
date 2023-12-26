#include <math.h>
#include <stdlib.h>
#include "motor.h"
#include "main.h"

// Private Define
#define ENCODER_LIMIT 65536

// Edit
float motor_max_speed[WHEEL_COUNT] = {10.948, 11.424, 11.1066, 10.6306}; // By testing
float motor_min_step[WHEEL_COUNT] = {0.0001520576675, 0.0001586688704, 0.0001542604758, 0.0001476492729}; // MOTOR_MAX_SPEED / MAX_PWM_CCR

// Private Variables
TIM_HandleTypeDef *m_pwm_htim;
TIM_HandleTypeDef *motor_htim[WHEEL_COUNT];

// Motor
float motor_velocity[WHEEL_COUNT] = {0, 0, 0, 0};
float motor_last_velocity[WHEEL_COUNT] = {0, 0, 0, 0}; // Discard anomaly in encoder
float motor_target_velocity[WHEEL_COUNT] = {0, 0, 0, 0};
int motor_pwm[WHEEL_COUNT] = {0, 0, 0, 0};

// Encoder
uint16_t encoder_value[WHEEL_COUNT] = {0, 0, 0, 0};
uint16_t encoder_last_value[WHEEL_COUNT] = {0, 0, 0, 0};

// PID
float motor_kp[WHEEL_COUNT] = {0, 0, 0, 0};
float motor_ki[WHEEL_COUNT] = {0, 0, 0, 0};
float motor_kd[WHEEL_COUNT] = {0, 0, 0, 0};
float pid_accumulate_error[WHEEL_COUNT] = {0, 0, 0, 0};
float pid_last_error[WHEEL_COUNT] = {0, 0, 0, 0};

// Private Function
int safe_unsigned_subtract(int a, int b, int limit)
{
	// assume limit = 65536
	// 0 - 65535
	if(a < b && b - a > limit)
		return(a + limit - b);
	// 65535 - 0
	if(a > b && a - b > limit)
		return(limit - a + b);
	return a - b;
}

void MOTOR_Set_Pwm(int motor, int CCR)
{
	motor_pwm[motor] = CCR;
	if(CCR > MAX_PWM_CCR)
	{
		CCR = MAX_PWM_CCR;
		motor_pwm[motor] = MAX_PWM_CCR;
	}
	else if(CCR < 0)
	{
		CCR = 0;
		motor_pwm[motor] = 0;
	}
	switch(motor)
	{
		case 0:
			__HAL_TIM_SetCompare(m_pwm_htim, TIM_CHANNEL_1, CCR);
			break;
		case 1:
			__HAL_TIM_SetCompare(m_pwm_htim, TIM_CHANNEL_2, CCR);
			break;
		case 2:
			__HAL_TIM_SetCompare(m_pwm_htim, TIM_CHANNEL_4, CCR);
			break;
		case 3:
			__HAL_TIM_SetCompare(m_pwm_htim, TIM_CHANNEL_3, CCR);
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

// Public Function
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
	}
	MOTOR_Set_Power(true);
}

void MOTOR_Set_Power(bool enable)
{
	if(enable)
	{
		HAL_GPIO_WritePin(BT2_SW_GPIO_Port, BT2_SW_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(L_GREEN_GPIO_Port, L_GREEN_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(L_RED_GPIO_Port, L_RED_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(D_STBY_GPIO_Port, D_STBY_Pin, GPIO_PIN_SET);
	}
	else
	{
		HAL_GPIO_WritePin(BT2_SW_GPIO_Port, BT2_SW_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(L_GREEN_GPIO_Port, L_GREEN_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(L_RED_GPIO_Port, L_RED_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(D_STBY_GPIO_Port, D_STBY_Pin, GPIO_PIN_RESET);
	}	
}

void MOTOR_PID()
{
	for(int i = 0; i < WHEEL_COUNT; i++)
	{
		encoder_value[i] = __HAL_TIM_GET_COUNTER(motor_htim[i]);
		
		if(i % 2 == 0)
		{
			// Motor 0, 2
			motor_velocity[i] = (safe_unsigned_subtract(encoder_value[i], encoder_last_value[i], ENCODER_LIMIT) * ENCODER_MIN_ANGULAR) * TIME_FACTOR;
		}
		else
		{
			// Motor 1, 3
			motor_velocity[i] = (-1 * safe_unsigned_subtract(encoder_value[i], encoder_last_value[i], ENCODER_LIMIT) * ENCODER_MIN_ANGULAR) * TIME_FACTOR;
		}
		if(motor_velocity[i] > motor_max_speed[i] || motor_velocity[i] < -motor_max_speed[i])
			motor_velocity[i] = motor_last_velocity[i];

		// Calculate error
		float error = motor_target_velocity[i] - motor_velocity[i];
		pid_accumulate_error[i] += error;
		// Discard overshooting error
		if(motor_velocity[i] + error >= motor_max_speed[i])
			pid_accumulate_error[i] = motor_max_speed[i] - motor_velocity[i];
		else if(motor_velocity[i] + error <= -motor_max_speed[i])
			pid_accumulate_error[i] = -motor_max_speed[i] - motor_velocity[i];
		float error_rate = error - pid_last_error[i];
		
		// Apply PID
		int pid = roundf((motor_kp[i] * error + motor_ki[i] * pid_accumulate_error[i] + motor_kd[i] * error_rate) / motor_min_step[i]);
		MOTOR_Set_Pwm(i, pid);
		
		pid_last_error[i] = error;
		encoder_last_value[i] = encoder_value[i];
		motor_last_velocity[i] = motor_velocity[i];
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
		//MOTOR_Set_Pwm(i, abs((int) roundf(motor_target_velocity[i] / motor_min_step[i])));
	}
}

void MOTOR_Set_Single_Velocity(int motor, float velocity)
{
	motor_target_velocity[motor] = velocity;
	motor_target_velocity[motor] >= 0 ? MOTOR_Set_Direction(motor, true) : MOTOR_Set_Direction(motor, false);
	//MOTOR_Set_Pwm(motor, abs((int) roundf(motor_target_velocity[motor] / motor_min_step[motor])));
}

void MOTOR_Set_PID(int motor, float kp, float ki, float kd)
{
	motor_kp[motor] = kp;
	motor_ki[motor] = ki;
	motor_kd[motor] = kd;
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

int MOTOR_Get_PWM(int motor)
{
	return motor_pwm[motor];
}