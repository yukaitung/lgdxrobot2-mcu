#ifndef POWER_H
#define POWER_H

// Maximum voltage of voltage divider / 4095 (12 bit)
#define VOLTAGE_PER_VALUE 0.00628571428
#define ESTOP_VOLTAGE_THRESHOLD 2.8

enum __batteries {
  logic_battery = 0,
  actuator_battery,
	battery_count
};

void POWER_Init(ADC_HandleTypeDef *hadc);
void POWER_Read_Start();
float POWER_Get_Value(int battery);

#endif