#include "main.h"
#include "power.h"
#include "estop.h"

ADC_HandleTypeDef *_hadc;
uint32_t _power_monitoring_buffer[3] = {0};
float _batteries_voltage[battery_count] = {0};

/*
 * Private Functions
 */

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
 _batteries_voltage[0] = _power_monitoring_buffer[0] * VOLTAGE_PER_VALUE;
 _batteries_voltage[1] = _power_monitoring_buffer[1] * VOLTAGE_PER_VALUE;
 float estop_voltage = _power_monitoring_buffer[2] * VOLTAGE_PER_VALUE;
 
 if (estop_voltage >= ESTOP_VOLTAGE_THRESHOLD)
 {
   ESTOP_Enable(hardware_emergency_stop);
 }
 else 
 {
   ESTOP_Disable(hardware_emergency_stop);
 }
}

/*
 * Public Functions
 */

void POWER_Init(ADC_HandleTypeDef *hadc)
{
  _hadc = hadc;
  POWER_Read_Start();
}

void POWER_Read_Start()
{
  HAL_ADC_Start_DMA(_hadc, _power_monitoring_buffer, 3);
}

float POWER_Get_Value(int battery)
{
  if (battery < 0 || battery > battery_count)
    return 0;
  return _batteries_voltage[battery];
}

