#include "main.h"
#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_conf.h"
#include "imu.h"
#include <stdint.h>

#define CABLICATE_COUNT 100

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
uint8_t _buffer[12] = {0};

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

void _write(uint8_t addr, uint8_t value)
{
  _select();
  HAL_SPI_Transmit(_hspi, &addr, 1, TIMEOUT_MS);
  HAL_SPI_Transmit(_hspi, &value, 1, TIMEOUT_MS);
  _unselect();
}

uint8_t _read(uint8_t addr)
{
  uint8_t buf = 0;
  addr |= 0x80;
  _select();
  HAL_SPI_Transmit(_hspi, &addr, 1, TIMEOUT_MS);
  HAL_SPI_Receive(_hspi, &buf, 1, TIMEOUT_MS);
  _unselect();
  return buf;
}

float _get_accel(uint8_t high, uint8_t low)
{
  int16_t value = (high << 8) | low;
  switch (_accel_precision) {
    case ACCEL_2G:
      return (float)value * (2 / 32768.0) * G_TO_M_S2;
    case ACCEL_4G:
      return (float)value * (4 / 32768.0) * G_TO_M_S2;
    case ACCEL_8G:
      return (float)value * (8 / 32768.0) * G_TO_M_S2;
    case ACCEL_16G:
      return (float)value * (16 / 32768.0) * G_TO_M_S2;
    default:
      return 0.0;
  }
}

float _get_gyro(uint8_t high, uint8_t low)
{
  int16_t value = (high << 8) | low;
  switch (_gyro_precision) {
    case GYRO_250_DPS:
      return (float)value * (250 / 32768.0) * DEG_TO_RAD;
    case GYRO_500_DPS:
      return (float)value * (500 / 32768.0) * DEG_TO_RAD;
    case GYRO_1000_DPS:
      return (float)value * (1000 / 32768.0) * DEG_TO_RAD;
    case GYRO_2000_DPS:
      return (float)value * (2000 / 32768.0) * DEG_TO_RAD;
    default:
      return 0.0;
  }
}

void _cablicate()
{
  uint8_t addr = ACCEL_XOUT_H | 0x80;
  uint8_t buffer[12] = {0};
  int32_t acc_accel[3] = {0};
  int32_t acc_gyro[3] = {0};
  int16_t cab_accel[3] = {0};
  int16_t cab_gyro[3] = {0};

  _select_bank(0);
  for (int i = 0; i < CABLICATE_COUNT; i++) 
  {
    _select();
    HAL_SPI_Transmit(_hspi, &addr, 1, TIMEOUT_MS);
    HAL_SPI_Receive(_hspi, buffer, 12, TIMEOUT_MS);
    _unselect();
    acc_accel[0] += (int16_t)(buffer[0] << 8 | buffer[1]);
    acc_accel[1] += (int16_t)(buffer[2] << 8 | buffer[3]);
    acc_accel[2] += (int16_t)(buffer[4] << 8 | buffer[5]);
    acc_gyro[0] += (int16_t)(buffer[6] << 8 | buffer[7]);
    acc_gyro[1] += (int16_t)(buffer[8] << 8 | buffer[9]);
    acc_gyro[2] += (int16_t)(buffer[10] << 8 | buffer[11]);
    HAL_Delay(2);
  }
  cab_accel[0] = -(int16_t)(acc_accel[0] / CABLICATE_COUNT);
  cab_accel[1] = -(int16_t)(acc_accel[1] / CABLICATE_COUNT);
  cab_accel[2] = -(int16_t)(acc_accel[2] / CABLICATE_COUNT);
  cab_accel[0] = (cab_accel[0] << 1) & 0xFFFE;
  cab_accel[1] = (cab_accel[1] << 1) & 0xFFFE;
  cab_accel[2] = (cab_accel[2] << 1) & 0xFFFE;
  cab_gyro[0] = -(int16_t)(acc_gyro[0] / CABLICATE_COUNT);
  cab_gyro[1] = -(int16_t)(acc_gyro[1] / CABLICATE_COUNT);
  cab_gyro[2] = -(int16_t)(acc_gyro[2] / CABLICATE_COUNT);

  _select_bank(1);
  _write(XA_OFFS_H, (uint8_t)(cab_accel[0] >> 8));
  _write(XA_OFFS_L, (uint8_t)(cab_accel[0]));
  _write(YA_OFFS_H, (uint8_t)(cab_accel[1] >> 8));
  _write(YA_OFFS_L, (uint8_t)(cab_accel[1]));
  _write(ZA_OFFS_H, (uint8_t)(cab_accel[2] >> 8));
  _write(ZA_OFFS_L, (uint8_t)(cab_accel[2]));

  _select_bank(2);
  _write(XG_OFFS_USRH, (uint8_t)(cab_gyro[0] >> 8));
  _write(XG_OFFS_USRL, (uint8_t)(cab_gyro[0]));
  _write(YG_OFFS_USRH, (uint8_t)(cab_gyro[1] >> 8));
  _write(YG_OFFS_USRL, (uint8_t)(cab_gyro[1]));
  _write(ZG_OFFS_USRH, (uint8_t)(cab_gyro[2] >> 8));
  _write(ZG_OFFS_USRL, (uint8_t)(cab_gyro[2]));
}

void _imu_step_process()
{
  uint8_t value = 0;
  _select();
  switch (_current_step) 
  {
    case select_bank_0_addr:
      value = REG_BANK_SEL;
      HAL_SPI_Transmit_IT(_hspi, &value, 1);
      break;
    case select_bank_0_value:
      value = 0 << 4;
      HAL_SPI_Transmit_IT(_hspi, &value, 1);
      break;
    case get_accel_gyro_addr:
      value = ACCEL_XOUT_H | 0x80;
      HAL_SPI_Transmit_IT(_hspi, &value, 1);
      break;
    case get_accel_gyro_read:
      HAL_SPI_Receive_IT(_hspi, _buffer, 12);
      break;
    case done:
      break;
  }
}

void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef * hspi)
{
  if (hspi->Instance == _hspi->Instance)
  {
    _unselect();
    _current_step++;
    _imu_step_process();
  }
}

void HAL_SPI_RxCpltCallback(SPI_HandleTypeDef * hspi)
{
  if (hspi->Instance == _hspi->Instance)
  {
    _unselect();
    if (_current_step == get_accel_gyro_read)
    {
      _imu_data.accelerometer.x = _get_accel(_buffer[0], _buffer[1]);
      _imu_data.accelerometer.y = _get_accel(_buffer[2], _buffer[3]);
      _imu_data.accelerometer.z = _get_accel(_buffer[4], _buffer[5]);
      _imu_data.gyroscope.x = _get_gyro(_buffer[6], _buffer[7]);
      _imu_data.gyroscope.y = _get_gyro(_buffer[8], _buffer[9]);
      _imu_data.gyroscope.z = _get_gyro(_buffer[10], _buffer[11]);
    }
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

  // 1. Initialise
    // Reset
  _select_bank(0);
  _write(PWR_MGMT_1, 0xC1);
  HAL_Delay(100);
  _write(PWR_MGMT_1, 0x01);

    // Start sensors
  _select_bank(2);
  _write(ODR_ALIGN_EN, 0x01);

  _write(GYRO_SMPLRT_DIV, 0x01);
  _write(GYRO_CONFIG_1, (GYRO_1000_DPS << 1) | 0x01);


  _write(ACCEL_SMPLRT_DIV_1, 0x00);
  _write(ACCEL_SMPLRT_DIV_2, 0x00);
  _write(ACCEL_CONFIG, (ACCEL_16G << 1) | 0x01);

  _cablicate();
  _write(GYRO_CONFIG_1, (_gyro_precision << 1) | 0x01);
  _write(ACCEL_CONFIG, (_accel_precision << 1) | 0x01);

    // Enable I2C
  _select_bank(0);
  uint8_t config = _read(USER_CTRL);
  config |= 0x10; 
  _select_bank(2);
  _write(USER_CTRL, config);

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
  return _imu_data;
}