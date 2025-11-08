#ifndef __CONFIGURATION_H
#define __CONFIGURATION_H

#include "lgdxrobot2.h"

// Chassis Configuration
#define CHASSIS_LX 0.24f
#define CHASSIS_LY 0.24f
#define WHEEL_RADIUS 0.0375f

// Motor Configuration
#define ENCODER_PPR 3960
#define MOTOR_GEAR_RATIO 90
#define MOTOR_MAX_SPEED float motor_max_speed[API_MOTOR_COUNT] = { 12.8520f, 12.0586f, 11.7413f, 10.4720f };

// PID Configuration
#define PID_SPEED float pid_speed[PID_LEVEL] = { 2.0f, 5.0f, 10.0f };
#define PID_KP float motors_Kp[PID_LEVEL][API_MOTOR_COUNT] = { {1.0f, 1.0f, 1.0f, 1.0f}, {1.0f, 1.0f, 1.0f, 1.0f}, {1.0f, 1.0f, 1.0f, 1.0f} };
#define PID_KI float motors_Ki[PID_LEVEL][API_MOTOR_COUNT] = { {0, 0, 0, 0}, {0, 0, 0, 0}, {0, 0, 0, 0} };
#define PID_KD float motors_Kd[PID_LEVEL][API_MOTOR_COUNT] = { {0, 0, 0, 0}, {0, 0, 0, 0}, {0, 0, 0, 0} };
#define MOTOR_SPEED_RAMP 0.1f

// Power Monitoring
#define ENABLE_POWER_MONITORING
#define POWER_MAXIMUM_CURRENT 15.0f // A
#define POWER_MINIMUM_VOLTAGE 12.0f // V (Only for actuators battery)

#endif