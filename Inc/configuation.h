#ifndef __CONFIGURATION_H
#define __CONFIGURATION_H

#include "lgdxrobot2.h"

// Chassis Configuration
#define CHASSIS_LX 0.237f
#define CHASSIS_LY 0.287f
#define WHEEL_RADIUS 0.0375f

// Motor Configuration
#define ENCODER_PPR 3960
#define MOTOR_GEAR_RATIO 90
#define MOTOR_MAX_SPEED float motor_max_speed[API_MOTOR_COUNT] = {10.948f, 11.424f, 11.1066f, 10.6306f};

// PID Configuration
#define PID_LEVEL_VELOCITY float level_velocity[PID_LEVEL] = {0.0f, 0.0f, 0.0f};
#define PID_KP float motors_Kp[PID_LEVEL][API_MOTOR_COUNT] = { {1.0f, 1.0f, 1.0f, 1.0f}, {1.0f, 1.0f, 1.0f, 1.0f}, {1.0f, 1.0f, 1.0f, 1.0f} };
#define PID_KI float motors_Ki[PID_LEVEL][API_MOTOR_COUNT] = { {0, 0, 0, 0}, {0, 0, 0, 0}, {0, 0, 0, 0} };
#define PID_KD float motors_Kd[PID_LEVEL][API_MOTOR_COUNT] = { {0, 0, 0, 0}, {0, 0, 0, 0}, {0, 0, 0, 0} };
#define MOTOR_SPEED_RAMP 0.1f

// Power Monitoring
#define ENABLE_POWER_MONITORING

#endif