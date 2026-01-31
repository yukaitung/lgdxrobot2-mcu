#include "main.h"
#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_conf.h"

#include "imu.h"

#define CABLICATE_COUNT 1000
#define IMU_READ_SIZE 23

const uint8_t _gyro_precision = MCU_IMU_GYRO_500_DPS;
const uint8_t _accel_precision = MCU_IMU_ACCEL_2G;

enum __imu_steps {
  get_accel_gyro_addr = 0,
  get_accel_gyro_read,
  done
};
int _current_step = done;

SPI_HandleTypeDef *_hspi;
McuImuDof _imu_data = {0};
uint8_t _buffer[IMU_READ_SIZE] = {0};

int16_t accel_offset[3] = {0};
int16_t gyro_offset[3] = {0};

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

  _select_bank(0);

  IMU_Read_Start();
}

void IMU_Read_Start()
{
 if (_current_step == done)
 {
    _current_step = get_accel_gyro_addr;
    _imu_step_process();
 }
}

McuImuDof IMU_Get_Data()
{
  _imu_data.accelerometer.x = (int16_t)(_buffer[0] << 8 | _buffer[1]);
  _imu_data.accelerometer.y = (int16_t)(_buffer[2] << 8 | _buffer[3]);
  _imu_data.accelerometer.z = (int16_t)(_buffer[4] << 8 | _buffer[5]);
  _imu_data.gyroscope.x = (int16_t)(_buffer[6] << 8 | _buffer[7]);
  _imu_data.gyroscope.y = (int16_t)(_buffer[8] << 8 | _buffer[9]);
  _imu_data.gyroscope.z = (int16_t)(_buffer[10] << 8 | _buffer[11]);
  _imu_data.magnetometer.x = (int16_t)(_buffer[16] << 8 | _buffer[15]);
  _imu_data.magnetometer.y = (int16_t)(_buffer[18] << 8 | _buffer[17]);
  _imu_data.magnetometer.z = (int16_t)(_buffer[20] << 8 | _buffer[19]);
  _imu_data.accelerometer_precision = _accel_precision;
  _imu_data.gyroscope_precision = _gyro_precision;
  return _imu_data;
}