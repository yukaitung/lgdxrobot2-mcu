#ifndef __MOTOR_H
#define __MOTOR_H

#include "stm32f4xx_hal_tim.h"
#include "stm32f4xx_hal.h"
#include <stdbool.h>

enum __emergency_stops {
	software_emergency_stop = 0,
	hardware_emergency_stop = 1,
	emergency_stops_count = 2
};

void MOTOR_Init(TIM_HandleTypeDef *pwm_htim, TIM_HandleTypeDef *e1_htim, TIM_HandleTypeDef *e2_htim, TIM_HandleTypeDef *e3_htim, TIM_HandleTypeDef *e4_htim);
float MOTOR_Get_Transform(int axis);
float MOTOR_Get_Actural_Velocity(int motor);
float MOTOR_Get_Level_Velocity(int level);
float MOTOR_Get_Pid(int motor, int level, int k);
bool MOTOR_Get_Emergency_Stop_Status(int type);
void MOTOR_Set_Ik(float velocity_x, float velocity_y, float velocity_w);
void MOTOR_Set_Single_Motor(int motor, float velocity);
void MOTOR_Set_Temporary_Level_Velocity(float level1, float level2, float level3);
void MOTOR_Set_Temporary_Pid(int motor, int level, float p, float i, float d);
void MOTOR_Save_Pid();
void MOTOR_Set_Emergency_Stop(int type, bool enable);
void MOTOR_Reset_Transform();
void MOTOR_PID();

#endif