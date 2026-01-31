#include "main.h"
#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_conf.h"
#include "imu.h"
#include <stdint.h>

#define CABLICATE_COUNT 1000
#define IMU_READ_SIZE 23

const uint8_t _gyro_precision = GYRO_500_DPS;
const uint8_t _accel_precision = ACCEL_2G;

enum __imu_steps {
  select_bank_0_addr = 0,
  select_bank_0_value,
  get_accel_gyro_addr,
  get_accel_gyro_read,
  done
};
int _current_step = done;

SPI_HandleTypeDef *_hspi;
McuImuDof _imu_data = {0};
uint8_t _buffer[IMU_READ_SIZE] = {0};

float accel_offset[3] = {0};
float gyro_offset[3] = {0};

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

float _get_accel(uint8_t high, uint8_t low)
{
  int16_t value = (high << 8) | low;
  switch (_accel_precision) {
    case ACCEL_2G:
      return (float)value * (2 / 32768.0);
    case ACCEL_4G:
      return (float)value * (4 / 32768.0);
    case ACCEL_8G:
      return (float)value * (8 / 32768.0);
    case ACCEL_16G:
      return (float)value * (16 / 32768.0);
    default:
      return 0.0;
  }
}

float _get_gyro(uint8_t high, uint8_t low)
{
  int16_t value = (high << 8) | low;
  switch (_gyro_precision) {
    case GYRO_250_DPS:
      return (float)value * (250 / 32768.0);
    case GYRO_500_DPS:
      return (float)value * (500 / 32768.0);
    case GYRO_1000_DPS:
      return (float)value * (1000 / 32768.0);
    case GYRO_2000_DPS:
      return (float)value * (2000 / 32768.0);
    default:
      return 0.0;
  }
}

float _get_mag(uint8_t high, uint8_t low)
{
  int16_t value = (high << 8) | low;
  return (float)value * MAG_STEP;
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
    acc_accel[0] += _get_accel(_buffer[0], _buffer[1]);
    acc_accel[1] += _get_accel(_buffer[2], _buffer[3]);
    acc_accel[2] += _get_accel(_buffer[4], _buffer[5]);
    acc_gyro[0] += _get_gyro(_buffer[6], _buffer[7]);
    acc_gyro[1] += _get_gyro(_buffer[8], _buffer[9]);
    acc_gyro[2] += _get_gyro(_buffer[10], _buffer[11]);
    HAL_Delay(1);
  }
  accel_offset[0] = acc_accel[0] / CABLICATE_COUNT;
  accel_offset[1] = acc_accel[1] / CABLICATE_COUNT;
  accel_offset[2] = acc_accel[2] / CABLICATE_COUNT;
  accel_offset[2] -= 1;
  gyro_offset[0] = acc_gyro[0] / CABLICATE_COUNT;
  gyro_offset[1] = acc_gyro[1] / CABLICATE_COUNT;
  gyro_offset[2] = acc_gyro[2] / CABLICATE_COUNT;
}

void _imu_step_process()
{
  uint8_t value = 0;
  
  switch (_current_step) 
  {
    case select_bank_0_addr:
      value = REG_BANK_SEL;
      _select();
      HAL_SPI_Transmit_IT(_hspi, &value, 1);
      break;
    case select_bank_0_value:
      value = 0 << 4;
      HAL_SPI_Transmit_IT(_hspi, &value, 1);
      break;
    case get_accel_gyro_addr:
      value = ACCEL_XOUT_H | 0x80;
      _select();
      HAL_SPI_Transmit_IT(_hspi, &value, 1);
      break;
    case get_accel_gyro_read:
      HAL_SPI_Receive_IT(_hspi, _buffer, IMU_READ_SIZE);
      break;
    case done:
      break;
  }
}

void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef * hspi)
{
  if (hspi->Instance == _hspi->Instance)
  {
    if (_current_step == select_bank_0_value)
    {
      _unselect();
    }
    _current_step++;
    _imu_step_process();
  }
}

void HAL_SPI_RxCpltCallback(SPI_HandleTypeDef * hspi)
{
  if (hspi->Instance == _hspi->Instance)
  {
    _unselect();
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

  _init_mag();
  _cablicate();

  IMU_Read_Start();
}

void IMU_Read_Start()
{
 if (_current_step == done)
 {
    _current_step = select_bank_0_addr;
    _imu_step_process();
 }
}

McuImuDof IMU_Get_Data()
{
  _imu_data.accelerometer.x = (_get_accel(_buffer[0], _buffer[1]) - accel_offset[0]) * G_TO_M_S2;
  _imu_data.accelerometer.y = (_get_accel(_buffer[2], _buffer[3]) - accel_offset[1]) * G_TO_M_S2;
  _imu_data.accelerometer.z = (_get_accel(_buffer[4], _buffer[5]) - accel_offset[2]) * G_TO_M_S2;
  _imu_data.gyroscope.x = (_get_gyro(_buffer[6], _buffer[7]) - gyro_offset[0]) * DEG_TO_RAD;
  _imu_data.gyroscope.y = (_get_gyro(_buffer[8], _buffer[9]) - gyro_offset[1]) * DEG_TO_RAD;
  _imu_data.gyroscope.z = (_get_gyro(_buffer[10], _buffer[11]) - gyro_offset[2]) * DEG_TO_RAD;
  _imu_data.magnetometer.x = _get_mag(_buffer[16], _buffer[15]);
  _imu_data.magnetometer.y = _get_mag(_buffer[18], _buffer[17]);
  _imu_data.magnetometer.z = _get_mag(_buffer[20], _buffer[19]);
  return _imu_data;
}