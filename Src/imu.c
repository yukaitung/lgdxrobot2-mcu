#include "main.h"
#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_conf.h"

#include "imu.h"
#include "flash.h"
#include "power.h"
#include <math.h>
#include <stdint.h>

#define CABLICATE_COUNT 100
#define IMU_READ_SIZE 23

const uint8_t _gyro_precision = GYRO_500_DPS;
const uint8_t _accel_precision = ACCEL_2G;

enum __imu_steps {
  get_accel_gyro_addr = 0,
  get_accel_gyro_read,
  read_power,
  done
};
int _current_step = done;

SPI_HandleTypeDef *_hspi;
McuImuData _imu_data = {0};
uint8_t _buffer[IMU_READ_SIZE] = {0};
float _mag_hard_iron_max[3] = {0};
float _mag_hard_iron_min[3] = {0};
float _mag_soft_iron_matrix[9] = {1, 0, 0, 0, 1, 0, 0, 0, 1};
float _accel_offset[3] = {0};
float _gyro_offset[3] = {0};

float _accel_history[3][CABLICATE_COUNT] = {0};
float _gyro_history[3][CABLICATE_COUNT] = {0};
float _mag_history[3][CABLICATE_COUNT] = {0};
int _history_index = 0;

/*
 * Private Functions
 */

void _select()
{
  HAL_GPIO_WritePin(SPI2_CS_GPIO_Port, SPI2_CS_Pin, GPIO_PIN_RESET);
}

void _unselect()
{
  HAL_GPIO_WritePin(SPI2_CS_GPIO_Port, SPI2_CS_Pin, GPIO_PIN_SET);
}

void _select_bank(uint8_t bank)
{
  uint8_t value = bank << 4;
  uint8_t addr = REG_BANK_SEL;
  _select();
  HAL_SPI_Transmit(_hspi, &addr, 1, TIMEOUT_MS);
  HAL_SPI_Transmit(_hspi, &value, 1, TIMEOUT_MS);
  _unselect();
}

void _write(uint8_t bank, uint8_t addr, uint8_t value)
{
  _select_bank(bank);
  _select();
  HAL_SPI_Transmit(_hspi, &addr, 1, TIMEOUT_MS);
  HAL_SPI_Transmit(_hspi, &value, 1, TIMEOUT_MS);
  _unselect();
}

uint8_t _read(uint8_t bank, uint8_t addr)
{
  _select_bank(bank);
  uint8_t buf = 0;
  addr |= 0x80;
  _select();
  HAL_SPI_Transmit(_hspi, &addr, 1, TIMEOUT_MS);
  HAL_SPI_Receive(_hspi, &buf, 1, TIMEOUT_MS);
  _unselect();
  return buf;
}

void _mag_write(uint8_t addr, uint8_t value)
{
  _write(3, I2C_SLV0_ADDR, MAG_ADDRESS);
  _write(3, I2C_SLV0_REG, addr);
  _write(3, I2C_SLV0_DO, value);
  _write(3, I2C_SLV0_CTRL, 0x80 | 0x01);
}

void _mag_read(uint8_t addr, uint8_t len)
{
  _write(3, I2C_SLV0_ADDR, 0x80 | MAG_ADDRESS);
  _write(3, I2C_SLV0_REG, addr);
  _write(3, I2C_SLV0_CTRL, 0x80 | len);
}

float _get_accel(uint8_t high, uint8_t low, int axis)
{
  int16_t value = (high << 8) | low;
  switch (_accel_precision) 
	{
    case ACCEL_2G:
      return (double)value * (2 / 32768.0) * G_TO_MS2 - _accel_offset[axis];
    case ACCEL_4G:
      return (double)value * (4 / 32768.0) * G_TO_MS2 - _accel_offset[axis];
    case ACCEL_8G:
      return (double)value * (8 / 32768.0) * G_TO_MS2 - _accel_offset[axis];
    case ACCEL_16G:
      return (double)value * (16 / 32768.0) * G_TO_MS2 - _accel_offset[axis];
    default:
      return 0.0;
	}
}

float _get_gyro(uint8_t high, uint8_t low, int axis)
{
  int16_t value = (high << 8) | low;
	switch (_gyro_precision) 
	{
    case GYRO_250_DPS:
      return (double)value * (250 / 32768.0) * DEG_TO_RAD - _gyro_offset[axis];
    case GYRO_500_DPS:
      return (double)value * (500 / 32768.0) * DEG_TO_RAD - _gyro_offset[axis];
    case GYRO_1000_DPS:
      return (double)value * (1000 / 32768.0) * DEG_TO_RAD - _gyro_offset[axis];
    case GYRO_2000_DPS:
      return (double)value * (2000 / 32768.0) * DEG_TO_RAD - _gyro_offset[axis];
    default:
      return 0.0;
	}
}

float _get_mag(uint8_t high, uint8_t low, int axis)
{
  int16_t temp = (high << 8) | low;
  float value = (float)temp * TO_M_TESLA;
  double hardX = value - (_mag_hard_iron_max[0] + _mag_hard_iron_min[0]) / 2.0;
	double hardY = value - (_mag_hard_iron_max[1] + _mag_hard_iron_min[1]) / 2.0;
	double hardZ = value - (_mag_hard_iron_max[2] + _mag_hard_iron_min[2]) / 2.0;
	if (axis == 0)
	{
		return _mag_soft_iron_matrix[0] * hardX + _mag_soft_iron_matrix[1] * hardY + _mag_soft_iron_matrix[2] * hardZ;
	}
	else if (axis == 1)
	{
		return _mag_soft_iron_matrix[3] * hardX + _mag_soft_iron_matrix[4] * hardY + _mag_soft_iron_matrix[5] * hardZ;
	}
	else if (axis == 2)
	{
		return _mag_soft_iron_matrix[6] * hardX + _mag_soft_iron_matrix[7] * hardY + _mag_soft_iron_matrix[8] * hardZ;
	}
	return 0.0;
}

void _covariance_matrix(float accelX, float accelY, float accelZ, 
  float gyroX, float gyroY, float gyroZ, 
  float magX, float magY, float magZ)
{
  _accel_history[0][_history_index] = accelX;
  _accel_history[1][_history_index] = accelY;
  _accel_history[2][_history_index] = accelZ;
  _gyro_history[0][_history_index] = gyroX;
  _gyro_history[1][_history_index] = gyroY;
  _gyro_history[2][_history_index] = gyroZ;
  _mag_history[0][_history_index] = magX;
  _mag_history[1][_history_index] = magY;
  _mag_history[2][_history_index] = magZ;

  _history_index++;
  // Calculate covariance matrix
  if (_history_index >= CABLICATE_COUNT)
  {
    // Calculate mean
    float accel_mean[3] = {0};
    float gyro_mean[3] = {0};
    float mag_mean[3] = {0};
    for (int i = 0; i < CABLICATE_COUNT; i++)
    {
      for (int axis = 0; axis < 3; axis++)
      {
        accel_mean[axis] += _accel_history[axis][i];
        gyro_mean[axis] += _gyro_history[axis][i];
        mag_mean[axis] += _mag_history[axis][i];
      }
    }
    for (int axis = 0; axis < 3; axis++)
    {
      accel_mean[axis] /= CABLICATE_COUNT;
      gyro_mean[axis] /= CABLICATE_COUNT;
      mag_mean[axis] /= CABLICATE_COUNT;
    }

    // Calculate variance
    float accel_variance[3] = {0};
    float gyro_variance[3] = {0};
    float mag_variance[3] = {0};
    for (int i = 0; i < CABLICATE_COUNT; i++)
    {
      for (int axis = 0; axis < 3; axis++)
      {
        accel_variance[axis] += powf((_accel_history[axis][i] - accel_mean[axis]), 2);
        gyro_variance[axis] += powf((_gyro_history[axis][i] - gyro_mean[axis]), 2);
        mag_variance[axis] += powf((_mag_history[axis][i] - mag_mean[axis]), 2);
      }
    }
    for (int axis = 0; axis < 3; axis++)
    {
      accel_variance[axis] = sqrtf((accel_variance[axis] / CABLICATE_COUNT));
      gyro_variance[axis] = sqrtf((gyro_variance[axis] / CABLICATE_COUNT));
      mag_variance[axis] = sqrtf((mag_variance[axis] / CABLICATE_COUNT));
    }
    
    _imu_data.accelerometer_covariance.x = accel_variance[0];
    _imu_data.accelerometer_covariance.y = accel_variance[1];
    _imu_data.accelerometer_covariance.z = accel_variance[2];
    _imu_data.gyroscope_covariance.x = gyro_variance[0];
    _imu_data.gyroscope_covariance.y = gyro_variance[1];
    _imu_data.gyroscope_covariance.z = gyro_variance[2];
    _imu_data.magnetometer_covariance.x = mag_variance[0];
    _imu_data.magnetometer_covariance.y = mag_variance[1];
    _imu_data.magnetometer_covariance.z = mag_variance[2];
    _history_index = 0;
  }
}

void _cablicate()
{
  uint8_t addr = ACCEL_XOUT_H | 0x80;
  float acc_accel[3] = {0};
  float acc_gyro[3] = {0};

  _select_bank(0);
  for (int i = 0; i < CABLICATE_COUNT; i++) 
  {
    _select();
    HAL_SPI_Transmit(_hspi, &addr, 1, TIMEOUT_MS);
    HAL_SPI_Receive(_hspi, _buffer, IMU_READ_SIZE, TIMEOUT_MS);
    _unselect();
    float accel[3] = {0};
    float gyro[3] = {0};
    float mag[3] = {0};
    accel[0] = _get_accel(_buffer[0], _buffer[1], 0);
    accel[1] = _get_accel(_buffer[2], _buffer[3], 1);
    accel[2] = _get_accel(_buffer[4], _buffer[5], 2);
    gyro[0] = _get_gyro(_buffer[6], _buffer[7], 0);
    gyro[1] = _get_gyro(_buffer[8], _buffer[9], 1);
    gyro[2] = _get_gyro(_buffer[10], _buffer[11], 2);
    mag[0] = _get_mag(_buffer[16], _buffer[15], 0);
    mag[1] = _get_mag(_buffer[18], _buffer[17], 1);
    mag[2] = _get_mag(_buffer[20], _buffer[19], 2);
    acc_accel[0] += accel[0];
    acc_accel[1] += accel[1];
    acc_accel[2] += accel[2];
    acc_gyro[0] += gyro[0];
    acc_gyro[1] += gyro[1];
    acc_gyro[2] += gyro[2];
    _covariance_matrix(accel[0], accel[1], accel[2], 
      gyro[0], gyro[1], gyro[2], 
      mag[0], mag[1], mag[2]);
    HAL_Delay(10);
  }
  _accel_offset[0] = acc_accel[0] / CABLICATE_COUNT;
  _accel_offset[1] = acc_accel[1] / CABLICATE_COUNT;
  _accel_offset[2] = acc_accel[2] / CABLICATE_COUNT;
  _accel_offset[2] -= G_TO_MS2;
  _gyro_offset[0] = acc_gyro[0] / CABLICATE_COUNT;
  _gyro_offset[1] = acc_gyro[1] / CABLICATE_COUNT;
  _gyro_offset[2] = acc_gyro[2] / CABLICATE_COUNT;
}


void _init_mag()
{
  uint8_t buf = 0;

  // Reset I2C Master
  buf = _read(0, USER_CTRL);
  buf |= 0x02;
  _write(0, USER_CTRL, buf);
  HAL_Delay(100);

  // Enable I2C Master
  buf = _read(0, USER_CTRL);
  buf |= 0x20;
  _write(0, USER_CTRL, buf);
  HAL_Delay(10);

  // Set I2C Clock
  _write(3, I2C_MST_CTRL, 0x07);
  HAL_Delay(10);

  _write(0, LP_CONFIG, 0x40);
  HAL_Delay(10);

  _write(3, I2C_MST_ODR_CONFIG, 0x03);
  HAL_Delay(10);

  // Reset Mag
  _mag_write(MAG_CNTL3, 0x01);
  HAL_Delay(100);

  // Set Mag Mode
  _mag_write(MAG_CNTL2, 0x08);
  HAL_Delay(50);

  // Start Read
  _mag_read(MAG_ST1, 9);
  HAL_Delay(100);
}

void _imu_step_process()
{
  uint8_t value = 0;
  
  switch (_current_step) 
  {
    case get_accel_gyro_addr:
      value = ACCEL_XOUT_H | 0x80;
      _select();
      HAL_SPI_Transmit_IT(_hspi, &value, 1);
      break;
    case get_accel_gyro_read:
      HAL_SPI_Receive_IT(_hspi, _buffer, IMU_READ_SIZE);
      break;
    case read_power:
      POWER_Read_Start();
      _current_step++;
      break;
    case done:
      break;
  }
}

void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef * hspi)
{
  if (hspi->Instance == _hspi->Instance)
  {
    _current_step++;
    _imu_step_process();
  }
}

void HAL_SPI_RxCpltCallback(SPI_HandleTypeDef * hspi)
{
  if (hspi->Instance == _hspi->Instance)
  {
    _unselect();

    _imu_data.accelerometer.x = _get_accel(_buffer[0], _buffer[1], 0);
    _imu_data.accelerometer.y = _get_accel(_buffer[2], _buffer[3], 1);
    _imu_data.accelerometer.z = _get_accel(_buffer[4], _buffer[5], 2);
    _imu_data.gyroscope.x = _get_gyro(_buffer[6], _buffer[7], 0);
    _imu_data.gyroscope.y = _get_gyro(_buffer[8], _buffer[9], 1);
    _imu_data.gyroscope.z = _get_gyro(_buffer[10], _buffer[11], 2);
    _imu_data.magnetometer.x = _get_mag(_buffer[16], _buffer[15], 0);
    _imu_data.magnetometer.y = _get_mag(_buffer[18], _buffer[17], 1);
    _imu_data.magnetometer.z = _get_mag(_buffer[20], _buffer[19], 2);
    _covariance_matrix(_imu_data.accelerometer.x, _imu_data.accelerometer.y, _imu_data.accelerometer.z,
      _imu_data.gyroscope.x, _imu_data.gyroscope.y, _imu_data.gyroscope.z,
      _imu_data.magnetometer.x, _imu_data.magnetometer.y, _imu_data.magnetometer.z);

    _current_step++;
    _imu_step_process();
  }
}

/*
* Public Functions
*/

void IMU_Init(SPI_HandleTypeDef *hspi)
{
  _hspi = hspi;

  flash_data data = Flash_Get();
	if (data.modified == FLASH_DATA_MODIFIED) // If the data is not modified, use the default value
	{
    for(int i = 0; i < 3; i++)
    {
      _mag_hard_iron_max[i] = data.mag_hard_iron_max[i];
      _mag_hard_iron_min[i] = data.mag_hard_iron_min[i];
    }
    for(int i = 0; i < 9; i++)
    {
      _mag_soft_iron_matrix[i] = data.mag_soft_iron_matrix[i];
    }
  }

  // Reset
  _write(0, PWR_MGMT_1, 0x80);
  HAL_Delay(100);
  _write(0, PWR_MGMT_1, 0x01);

  // Set SPI Only
  _write(0, USER_CTRL, 0x10);
  HAL_Delay(10);

  // Start Sensors
  _write(2, ODR_ALIGN_EN, 0x01); 
  _write(2, GYRO_SMPLRT_DIV, 0x0A); // ~100Hz
  _write(2, GYRO_CONFIG_1, (6 << 3) | (_gyro_precision << 1) | 0x01);
  _write(2, ACCEL_SMPLRT_DIV_1, 0x00);
  _write(2, ACCEL_SMPLRT_DIV_2, 0x0A); // ~100Hz
  _write(2, ACCEL_CONFIG, (6 << 3) | (_accel_precision << 1) | 0x01);

  // Reduce Speed
  _init_mag();
  _cablicate();

  _select_bank(0);
}

void IMU_Read_Start()
{
 if (_current_step == done)
 {
    _current_step = get_accel_gyro_addr;
    _imu_step_process();
 }
 else if (_current_step == get_accel_gyro_read)
 {  
    // SPI read stuck, retry
    _unselect();
    _current_step = done;
 }
}

void IMU_Set_Mag_Calibration_Data(const McuSetMagCalibrationDataCommand *cmd)
{
  for(int i = 0; i < 3; i++)
  {
    _mag_hard_iron_max[i] = cmd->hard_iron_max[i];
    _mag_hard_iron_min[i] = cmd->hard_iron_min[i];
  }
  for(int i = 0; i < 9; i++)
  {
    _mag_soft_iron_matrix[i] = cmd->soft_iron_matrix[i];
  }
}

McuImuData IMU_Get_Data()
{

  return _imu_data;
}

float IMU_Get_Hard_Iron_Max(int axis)
{
  if (axis < 0 || axis > 2)
    return 0;

  return _mag_hard_iron_max[axis];
}

float IMU_Get_Hard_Iron_Min(int axis)
{
  if (axis < 0 || axis > 2)
    return 0;

  return _mag_hard_iron_min[axis];
}
float IMU_Get_Soft_Iron_Matrix(int index)
{
  if (index < 0 || index > 8)
    return 0;

  return _mag_soft_iron_matrix[index];
}