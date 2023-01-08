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
#define GEAR_RATIO 90
#define MOTOR_MAX_SPEED 33.5103216 // 320 RPM to rad/s
// Constants - Pre-Calculation
#define ENCODER_MIN_ANGULAR 0.00158666296 // 2pi / 3960
#define MOTOR_MIN_STEP 0.0325975891 // (Max RPM in rad/s) / (MAX_PWM_CCR)
// Constants - PID Configuration
#define PID_RESPONSE_TIME_MS 10

void MOTOR_Init(TIM_HandleTypeDef *pwm_htim, TIM_HandleTypeDef *m1_htim, TIM_HandleTypeDef *m2_htim, TIM_HandleTypeDef *m3_htim, TIM_HandleTypeDef *m4_htim);
void MOTOR_Set_Power(bool enable);
void MOTOR_PID();
void MOTOR_Set_Ik(float velocity_x, float velocity_y, float velocity_w);
float *MOTOR_Get_Velocity();
float *MOTOR_Get_Target_Velocity();
uint16_t *MOTOR_Get_Pwm();

#endif