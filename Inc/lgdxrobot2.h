#pragma once

#include <cstdint>

#define PID_LEVEL 3
#define API_MOTOR_COUNT 4

#pragma pack(push, 1)

/*
 * MCU to PC communication
 */

typedef struct {
  float x;
  float y;
  float rotation;
} McuDof;

typedef struct {
  float voltage;
  float current;
} McuPower;

typedef struct {
  const uint8_t header1 = 0xAA;
  const uint8_t header2 = 0x55;
  const char type = 'D';
  McuDof transform;
  McuDof forward_kinematics;
  float target_motors_velocity[API_MOTOR_COUNT] = {0};
  float actural_motors_velocity[API_MOTOR_COUNT] = {0};
  McuPower battery1;
  McuPower battery2;
  bool software_emergency_stop_enabled;
  bool hardware_emergency_stop_enabled;
} McuData;

typedef struct {
  const uint8_t header1 = 0xAA;
  const uint8_t header2 = 0x55;
  const char type = 'P';
  uint32_t serial_number1;
  uint32_t serial_number2;
  uint32_t serial_number3;
} McuSerialNumber;

typedef struct {
  const uint8_t header1 = 0xAA;
  const uint8_t header2 = 0x55;
  const char type = 'S';
  float p[API_MOTOR_COUNT][PID_LEVEL] = {0};
  float i[API_MOTOR_COUNT][PID_LEVEL] = {0};
  float d[API_MOTOR_COUNT][PID_LEVEL] = {0};
} McuPid;

/*
 * PC to MCU communication
 */

// Emergency stop
typedef struct {
  const char command = 'E';
  bool enable;
} McuSoftwareEmergencyStopCommand;

// Motor control
typedef struct {
  const char command = 'I';
  McuDof velocity;
} McuInverseKinematicsCommand;

typedef struct {
  const char command = 'M';
  uint8_t motor;
  float velocity;
} McuMotorCommand;

// PID control
typedef struct {
  const char command = 'P';
} McuGetPidCommand;

typedef struct {
  const char command = 'Q';
  uint8_t motor;
  uint8_t level;
  float p;
  float i;
  float d;
} McuSetPidCommand;

typedef struct {
  const char command = 'R';
} McuSavePidCommand;

// Other
typedef struct {
  const char command = 'S';
} McuGetSerialNumberCommand;

typedef struct {
  const char command = 'T';
} McuResetTransformCommand;

#pragma pack(pop)