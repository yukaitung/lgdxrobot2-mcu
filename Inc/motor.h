#ifndef __MOTOR_H
#define __MOTOR_H

#include "stm32f4xx_hal.h"
#include <stdbool.h>

// Constants
#define MAX_PWM_CCR 71999

// Constants - Chassis Configuration
#define CHASSIS_LX 0.237
#define CHASSIS_LY 0.287
#define WHEEL_RADIUS 0.0375
#define WHEEL_COUNT 4
#define MOTOR_GEAR_RATIO 90
// Constants - Pre-Calculation
#define ENCODER_MIN_ANGULAR 0.00158666296 // 2pi / (3960)

// Constants - PID Configuration
#define PID_RESPONSE_TIME_MS 20
#define TIME_FACTOR 50 // 1 second / PID_RESPONSE_TIME_MS

// Constructor
void MOTOR_Init(TIM_HandleTypeDef *pwm_htim, TIM_HandleTypeDef *m1_htim, TIM_HandleTypeDef *m2_htim, TIM_HandleTypeDef *m3_htim, TIM_HandleTypeDef *m4_htim);

// Get
float MOTOR_Get_Velocity(int motor);
float MOTOR_Get_Target_Velocity(int motor);
float MOTOR_Get_PID(int pid, int motor);
int MOTOR_Get_PWM(int motor);
int MOTOR_Get_E_Stop_Status(int status);

// Set
void MOTOR_Set_Power(bool enable);
void MOTOR_Set_Ik(float velocity_x, float velocity_y, float velocity_w);
void MOTOR_Set_Single_Velocity(int motor, float velocity);
void MOTOR_Set_PID(int motor, float kp, float ki, float kd);
void MOTOR_Set_Software_E_Stop(int enable);
void MOTOR_Set_Hardware_E_Stop(int enable);

// Functions
void MOTOR_PID();

#endif