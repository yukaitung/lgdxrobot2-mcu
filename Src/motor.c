#include "motor.h"
#include "main.h"

// Private Variables
TIM_HandleTypeDef *m_pwm_htim;
TIM_HandleTypeDef *motor_htim[WHEEL_COUNT];
float motor_velocity[WHEEL_COUNT] = {0, 0, 0, 0};

// Private
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

uint32_t MOTOR_Get_Encoder(int motor)
{
	if(motor < 0)
		motor = 0;
	else if(motor > WHEEL_COUNT)
		motor = WHEEL_COUNT;
	return __HAL_TIM_GET_COUNTER(motor_htim[motor]);
}