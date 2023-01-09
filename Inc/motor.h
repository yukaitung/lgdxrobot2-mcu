#ifndef __MOTOR_H
#define __MOTOR_H

#include "stm32f4xx_hal.h"
#include <stdbool.h>

// Constants
#define MAX_PWM_CCR 1028
// Constants - Chassis Configuration
#define CHASSIS_LX 0.237
#define CHASSIS_LY 0.287
#define WHEEL_RADIUS 0.0375
#define WHEEL_COUNT 4
#define MOTOR_GEAR_RATIO 90
#define MOTOR_MAX_SPEED 11.205013785 // 107 RPM to rad/s
// Constants - Pre-Calculation
#define ENCODER_MIN_ANGULAR 0.00158666296 // 2pi / (3960)
#define MOTOR_MIN_STEP 0.0108998189 // MOTOR_MAX_SPEED / MAX_PWM_CCR
// Constants - PID Configuration
#define PID_RESPONSE_TIME_MS 10
#define TIME_FACTOR 100 // 1 second / PID_RESPONSE_TIME_MS

void MOTOR_Init(TIM_HandleTypeDef *pwm_htim, TIM_HandleTypeDef *m1_htim, TIM_HandleTypeDef *m2_htim, TIM_HandleTypeDef *m3_htim, TIM_HandleTypeDef *m4_htim);
void MOTOR_Set_Power(bool enable);
void MOTOR_PID();
void MOTOR_Set_Ik(float velocity_x, float velocity_y, float velocity_w);
float *MOTOR_Get_Velocity();
float *MOTOR_Get_Target_Velocity();
int *MOTOR_Pwm();

#endif