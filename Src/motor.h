#ifndef __MOTOR_H
#define __MOTOR_H

#include "stm32f4xx_hal.h"
#include <stdbool.h>

// Constants
#define MAX_PWM_CCR 1028
#define CHASSIS_LX 0.237
#define CHASSIS_LY 0.287
#define WHEEL_RADIUS 0.0375
#define WHEEL_COUNT 4

// Hide Later
void MOTOR_Set_Pwm(int motor, int CCR);
void MOTOR_Set_Direction(int motor, bool forward);

void MOTOR_Init(TIM_HandleTypeDef *pwm_htim, TIM_HandleTypeDef *m1_htim, TIM_HandleTypeDef *m2_htim, TIM_HandleTypeDef *m3_htim, TIM_HandleTypeDef *m4_htim);
void MOTOR_Set_Power(bool enable);
uint32_t MOTOR_Get_Encoder(int motor);

#endif