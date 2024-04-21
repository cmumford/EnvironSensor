#include <esp_log.h>
#include <esp_system.h>
#include <i2clib/operation.h>

#include "mybme280.h"

namespace {

constexpr char TAG[] = "BME280";
constexpr uint8_t kMaxSampleCount = 50;
constexpr i2c::Address kSlaveAddress = {.address = BME280_I2C_ADDR_PRIM,
                                        .addr_size = i2c::Address::Size::k7bit};

bool IsError(int8_t status_code) {
  return status_code < 0;
}

bool IsWarning(int8_t status_code) {
  return status_code > 0;
}

}  // namespace

// static
BME280_INTF_RET_TYPE BME280::ReadFunc(uint8_t reg_addr,
                                      uint8_t* reg_data,
                                      uint32_t len,
                                      void* intf_ptr) {
  BME280* instance = static_cast<BME280*>(intf_ptr);
  i2c::Operation op = instance->i2c_master_.CreateReadOp(
      kSlaveAddress, reg_addr, "bme-read-cb");
  i2c::Status s = op.Read(reg_data, len);
  if (!s.ok())
    return BME280_E_COMM_FAIL;
  s = op.Execute();
  return s.ok() ? BME280_INTF_RET_SUCCESS : BME280_E_COMM_FAIL;
}

// static
BME280_INTF_RET_TYPE BME280::WriteFunc(uint8_t reg_addr,
                                       const uint8_t* reg_data,
                                       uint32_t len,
                                       void* intf_ptr) {
  BME280* instance = static_cast<BME280*>(intf_ptr);
  i2c::Operation op = instance->i2c_master_.CreateWriteOp(
      kSlaveAddress, reg_addr, "bme-write-cb");
  i2c::Status s = op.Write(reg_data, len);
  if (!s.ok())
    return BME280_E_COMM_FAIL;
  s = op.Execute();
  return s.ok() ? BME280_INTF_RET_SUCCESS : BME280_E_COMM_FAIL;
}

// static
void BME280::DelayUsFunc(uint32_t period, void* intf_ptr) {
  esp_rom_delay_us(period);
}

BME280::BME280(i2c::Master& i2c_master) : i2c_master_(i2c_master) {
  dev_.read = ReadFunc;
  dev_.write = WriteFunc;
  dev_.delay_us = DelayUsFunc;
  dev_.intf = BME280_I2C_INTF;
  dev_.intf_ptr = this;
}

bool BME280::ReadSettings() {
  int8_t s = bme280_get_sensor_settings(&settings_, &dev_);
  return !IsError(s);
}

bool BME280::SetSensorMode() {
  int8_t s = bme280_set_sensor_mode(BME280_POWERMODE_NORMAL, &dev_);
  return !IsError(s);
}

bool BME280::SetSettings() {
  /* Configuring the over-sampling rate, filter coefficient and standby time */
  /* Overwrite the desired settings */
  settings_.filter = BME280_FILTER_COEFF_2;

  /* Over-sampling rate for humidity, temperature and pressure */
  settings_.osr_h = BME280_OVERSAMPLING_1X;
  settings_.osr_p = BME280_OVERSAMPLING_1X;
  settings_.osr_t = BME280_OVERSAMPLING_1X;

  /* Setting the standby time */
  settings_.standby_time = BME280_STANDBY_TIME_0_5_MS;

  int8_t s =
      bme280_set_sensor_settings(BME280_SEL_ALL_SETTINGS, &settings_, &dev_);
  return !IsError(s);
}

bool BME280::CalcMeasurementDelay() {
  int8_t s = bme280_cal_meas_delay(&period_, &settings_);
  if (IsError(s))
    return false;

  ESP_LOGD(TAG, "Measurement delay: %u usec",
           static_cast<unsigned int>(period_));
  return true;
}

std::expected<double, int8_t> BME280::GetTemperature() {
  for (int8_t i = 0; i < kMaxSampleCount; i++) {
    uint8_t status_reg;
    int8_t s = bme280_get_regs(BME280_REG_STATUS, &status_reg, 1, &dev_);
    if (IsError(s)) {
      return std::unexpected(s);
    }

    if (status_reg & BME280_STATUS_MEAS_DONE) {
      // Measurement time delay given to read the sample.
      dev_.delay_us(period_, dev_.intf_ptr);

      // Read the compensated data.
      bme280_data comp_data;
      s = bme280_get_sensor_data(BME280_TEMP, &comp_data, &dev_);
      if (IsError(s)) {
        return std::unexpected(s);
      }

#ifdef BME280_DOUBLE_ENABLE
      return comp_data.temperature;
#else
      return comp_data.temperature = comp_data.temperature / 100;
#endif
    }
  }
  return std::unexpected(BME280_E_COMM_FAIL);
}

bool BME280::Init() {
  int8_t s = bme280_init(&dev_);

  if (IsError(s)) {
    ESP_LOGE(TAG, "Failure initializing BME280: %d", s);
    return false;
  }
  if (IsWarning(s)) {
    ESP_LOGW(TAG, "Warning initializing BME280: %d", s);
  }
  // Always read the current settings before writing, especially when all the
  // configuration is not modified.
  if (!ReadSettings()) {
    return false;
  }
  if (!SetSettings()) {
    return false;
  }
  if (!SetSensorMode()) {
    return false;
  }
  if (!CalcMeasurementDelay()) {
    return false;
  }

  ESP_LOGI(TAG, "BME280 initialized");
  return true;
}