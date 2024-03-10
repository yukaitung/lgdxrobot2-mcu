#ifndef __MOTOR_H
#define __MOTOR_H

#include "stm32f4xx_hal.h"
#include <stdbool.h>

// Constants - Chassis Configuration
#define CHASSIS_LX 0.237
#define CHASSIS_LY 0.287
#define WHEEL_RADIUS 0.0375
#define WHEEL_COUNT 4
#define MOTOR_GEAR_RATIO 90
#define MOTOR_MAX_SPEED float motor_max_speed[WHEEL_COUNT] = {10.948, 11.424, 11.1066, 10.6306}; // By testing

// Constants - Pre-Calculation
#define ENCODER_MIN_ANGULAR 0.00158666296 // 2pi / (3960)

// Constants - PID Configuration
#define PID_KP float motor_kp[WHEEL_COUNT] = {11.5, 2.2, 8.0, 10};
#define PID_KI float motor_ki[WHEEL_COUNT] = {1.1, 0.8, 0.9, 1.0};
#define PID_KD float motor_kd[WHEEL_COUNT] = {1, 2, 1, 1};

// Constants - IMU
#define IMU_STOP 9.73

//
// Function
//

// Constructor
void MOTOR_Init(TIM_HandleTypeDef *pwm_htim, TIM_HandleTypeDef *m1_htim, TIM_HandleTypeDef *m2_htim, TIM_HandleTypeDef *m3_htim, TIM_HandleTypeDef *m4_htim);

// Get
float MOTOR_Get_Transform(int axis);
float MOTOR_Get_Fk(int axis);
float MOTOR_Get_Velocity(int motor);
float MOTOR_Get_Target_Velocity(int motor);
float MOTOR_Get_PID(int pid, int motor);
uint32_t MOTOR_Get_PID_elapsed();
bool MOTOR_Get_E_Stop_Status(int e_stop);

// Set
void MOTOR_Set_Power(bool enable);
void MOTOR_Set_Ik(float velocity_x, float velocity_y, float velocity_w);
void MOTOR_Set_Single_Velocity(int motor, float velocity);
void MOTOR_Set_PID(int motor, float kp, float ki, float kd);
void MOTOR_Set_Software_E_Stop(bool enable);
void MOTOR_Set_Hardware_E_Stop(bool enable);
void MOTOR_Set_External_IMU(float ax, float ay, float az, float gz);
void MOTOR_Reset_Transform();

// Functions
void MOTOR_PID();

#endif