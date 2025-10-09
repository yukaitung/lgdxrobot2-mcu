#pragma once

#include "lgdxrobot2.h"

// Chassis Configuration
#define CHASSIS_LX 0.237
#define CHASSIS_LY 0.287
#define WHEEL_RADIUS 0.0375

// Motor Configuration
#define ENCODER_PPR 3960
#define MOTOR_GEAR_RATIO 90
#define MOTOR_MAX_SPEED float motor_max_speed[API_MOTOR_COUNT] = {10.948, 11.424, 11.1066, 10.6306};

// PID Configuration
#define PID_KP float motor_p[PID_LEVEL][API_MOTOR_COUNT] = { {0, 0, 0, 0}, {0, 0, 0, 0}, {0, 0, 0, 0} };
#define PID_KI float motor_i[PID_LEVEL][API_MOTOR_COUNT] = { {0, 0, 0, 0}, {0, 0, 0, 0}, {0, 0, 0, 0} };
#define PID_KD float motor_d[PID_LEVEL][API_MOTOR_COUNT] = { {0, 0, 0, 0}, {0, 0, 0, 0}, {0, 0, 0, 0} };
#define MOTOR_SPEED_RAMP 0.1
