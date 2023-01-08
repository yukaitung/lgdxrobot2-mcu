#include "motor.h"
#include "main.h"
// Private Define
#define ENCODER_LIMIT 65536

// Private Variables
TIM_HandleTypeDef *m_pwm_htim;
TIM_HandleTypeDef *motor_htim[WHEEL_COUNT];
// Motor
float motor_velocity[WHEEL_COUNT] = {0, 0, 0, 0};
float motor_target_velocity[WHEEL_COUNT] = {0, 0, 0, 0};
uint16_t motor_pwm[WHEEL_COUNT] = {0, 0, 0, 0};
// Encoder
uint16_t encoder_value[WHEEL_COUNT] = {0, 0, 0, 0};
uint16_t encoder_last_value[WHEEL_COUNT] = {0, 0, 0, 0};
// PID
float motor_kp[WHEEL_COUNT] = {0, 0, 0, 0};
float motor_ki[WHEEL_COUNT] = {0, 0, 0, 0};
float motor_kd[WHEEL_COUNT] = {0, 0, 0, 0};
float accumulate_error[WHEEL_COUNT] = {0, 0, 0, 0};
float last_error[WHEEL_COUNT] = {0, 0, 0, 0};

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
	if(CCR > MAX_PWM_CCR)
		CCR = MAX_PWM_CCR;
	if(CCR < 0)
		CCR = 0;
	switch(motor)
	{
		case 0:
			__HAL_TIM_SetCompare(m_pwm_htim, TIM_CHANNEL_1, CCR);
			break;
		case 1:
			__HAL_TIM_SetCompare(m_pwm_htim, TIM_CHANNEL_2, CCR);
			break;
		case 2:
			__HAL_TIM_SetCompare(m_pwm_htim, TIM_CHANNEL_3, CCR);
			break;
		case 3:
			__HAL_TIM_SetCompare(m_pwm_htim, TIM_CHANNEL_4, CCR);
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
	HAL_TIM_PWM_Start(m_pwm_htim, TIM_CHANNEL_ALL);
	for(int i = 0; i < WHEEL_COUNT; i++)
	{
		HAL_TIM_Encoder_Start(motor_htim[i], TIM_CHANNEL_ALL);
	}
	MOTOR_Set_Power(true);
	HAL_GPIO_WritePin(D_STBY_GPIO_Port, D_STBY_Pin, GPIO_PIN_SET);
}

void MOTOR_Set_Power(bool enable)
{
	if(enable)
	{
		HAL_GPIO_WritePin(BT2_SW_GPIO_Port, BT2_SW_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(L_GREEN_GPIO_Port, L_GREEN_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(L_RED_GPIO_Port, L_RED_Pin, GPIO_PIN_RESET);
	}
	else
	{
		HAL_GPIO_WritePin(BT2_SW_GPIO_Port, BT2_SW_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(L_GREEN_GPIO_Port, L_GREEN_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(L_RED_GPIO_Port, L_RED_Pin, GPIO_PIN_SET);
	}	
}

void MOTOR_PID()
{
	for(int i = 0; i < WHEEL_COUNT; i++)
	{
		encoder_value[i] = __HAL_TIM_GET_COUNTER(motor_htim[i]);
		
		if(i % 2 == 0)
		{
			// 0, 2
				motor_velocity[i] = safe_unsigned_subtract(encoder_value[i], encoder_last_value[i], ENCODER_LIMIT) * ENCODER_MIN_ANGULAR;
		}
		else
		{
			// 1, 3
				motor_velocity[i] = -1 * safe_unsigned_subtract(encoder_value[i], encoder_last_value[i], ENCODER_LIMIT) * ENCODER_MIN_ANGULAR;
		}
		
		// Calculate error
		float error = motor_target_velocity[i] - motor_velocity[i];
		accumulate_error[i] += error;
		if(accumulate_error[i] >= MOTOR_MAX_SPEED)
			accumulate_error[i] = MOTOR_MAX_SPEED;
		else if(accumulate_error[i] <= -MOTOR_MAX_SPEED)
			accumulate_error[i] = -MOTOR_MAX_SPEED;
		float error_rate = error - last_error[i];
		
		int pid = round((motor_kp[i] * error + motor_ki[i] * accumulate_error[i] + motor_kd[i] * error_rate) / MOTOR_MIN_STEP);
		//motor_pwm[i] += abs(pid);
		//MOTOR_Set_Pwm(i, motor_pwm[i]);
		
		last_error[i] = error;
		encoder_last_value[i] = encoder_value[i];
	}
}

void MOTOR_Set_Ik(float velocity_x, float velocity_y, float velocity_w)
{
	motor_target_velocity[0] = 1 / (WHEEL_RADIUS * GEAR_RATIO) * (velocity_x - velocity_y - (CHASSIS_LX + CHASSIS_LY) * velocity_w);
	motor_target_velocity[1] = 1 / (WHEEL_RADIUS * GEAR_RATIO) * (velocity_x + velocity_y + (CHASSIS_LX + CHASSIS_LY) * velocity_w);
	motor_target_velocity[2] = 1 / (WHEEL_RADIUS * GEAR_RATIO) * (velocity_x + velocity_y - (CHASSIS_LX + CHASSIS_LY) * velocity_w);
	motor_target_velocity[3] = 1 / (WHEEL_RADIUS * GEAR_RATIO) * (velocity_x - velocity_y + (CHASSIS_LX + CHASSIS_LY) * velocity_w);
	for(int i = 0; i < WHEEL_COUNT; i++)
	{
		if(motor_target_velocity[i] >= 0)
		{
			MOTOR_Set_Direction(i, true);
		}
		else
		{
			MOTOR_Set_Direction(i, false);
		}
		motor_pwm[i] = round(motor_target_velocity[i] / MOTOR_MIN_STEP);
		MOTOR_Set_Pwm(i, motor_pwm[i]);
	}
}

float *MOTOR_Get_Velocity()
{
	return motor_velocity;
}
	
float *MOTOR_Get_Target_Velocity()
{
	return motor_target_velocity;
}

uint16_t *MOTOR_Get_Pwm()
{
	return motor_pwm;
}