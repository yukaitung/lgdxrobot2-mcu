#ifndef __MOTOR_H
#define __MOTOR_H

#include "stm32f4xx_hal_tim.h"
#include "stm32f4xx_hal.h"
#include <stdbool.h>

void MOTOR_Init(TIM_HandleTypeDef *pwm_htim1, TIM_HandleTypeDef *e1_htim, TIM_HandleTypeDef *e2_htim, TIM_HandleTypeDef *e3_htim, TIM_HandleTypeDef *e4_htim);
uint32_t MOTOR_Get_Pid_Elapsed();
float MOTOR_Get_Transform(int axis);
float MOTOR_Get_Forward_Kinematic(int axis);
float MOTOR_Get_Actual_Velocity(int motor);
float MOTOR_Get_Desired_Velocity(int motor);
float MOTOR_Get_Target_Velocity(int motor);
float MOTOR_Get_Pid_Speed(int level);
float MOTOR_Get_Maximum_Speed(int motor);
float MOTOR_Get_Pid(int motor, int level, int k);
void MOTOR_Set_Ik(float velocity_x, float velocity_y, float velocity_w);
void MOTOR_Set_Single_Motor(int motor, float velocity);
void MOTOR_Set_Temporary_Pid_Speed(float level1, float level2, float level3);
void MOTOR_Set_Temporary_Pid(int motor, int level, float p, float i, float d);
void MOTOR_Set_Temporary_Maximum_Speed(float speed1, float speed2, float speed3, float speed4);
void MOTOR_Reset_Transform();
void MOTOR_PID();

#endif