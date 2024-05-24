#include <esp_log.h>
#include <esp_system.h>
#include <i2clib/operation.h>

#include "mybme280.h"

namespace {

constexpr char TAG[] = "BME280";
constexpr uint8_t kMaxSampleCount = 50;
constexpr i2c::Address kSlaveAddress = {.address = BME280_I2C_ADDR_PRIM,
                                        .addr_size = i2c::Address::Size::k7bit};

constexpr bool IsError(int8_t status_code) {
  return status_code < 0;
}

constexpr bool IsWarning(int8_t status_code) {
  return status_code > 0;
}

esp_err_t BmeToEspErr(int8_t bme_err) {
  switch (bme_err) {
    [[unlikely]] case BME280_OK:
      return ESP_OK;
    case BME280_E_NULL_PTR:
      return ESP_ERR_INVALID_ARG;
    case BME280_E_INVALID_LEN:
      return ESP_ERR_INVALID_SIZE;
    case BME280_E_DEV_NOT_FOUND:
      return ESP_ERR_NOT_FOUND;
    case BME280_E_SLEEP_MODE_FAIL:
      [[fallthrough]];
    case BME280_E_NVM_COPY_FAILED:
      [[fallthrough]];
    case BME280_E_COMM_FAIL:
      [[fallthrough]];
    default:
      return ESP_FAIL;
  }
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
  if (!op.Read(reg_data, len).ok())
    return BME280_E_COMM_FAIL;
  return op.Execute().ok() ? BME280_INTF_RET_SUCCESS : BME280_E_COMM_FAIL;
}

// static
BME280_INTF_RET_TYPE BME280::WriteFunc(uint8_t reg_addr,
                                       const uint8_t* reg_data,
                                       uint32_t len,
                                       void* intf_ptr) {
  BME280* instance = static_cast<BME280*>(intf_ptr);
  i2c::Operation op = instance->i2c_master_.CreateWriteOp(
      kSlaveAddress, reg_addr, "bme-write-cb");
  if (!op.Write(reg_data, len).ok())
    return BME280_E_COMM_FAIL;
  return op.Execute().ok() ? BME280_INTF_RET_SUCCESS : BME280_E_COMM_FAIL;
}

// static
void BME280::DelayUsFunc(uint32_t period, void* intf_ptr) {
  esp_rom_delay_us(period);
}

// static
uint32_t BME280::MaxI2CClockSpeed() {
  return 1'000'000;  // Max BME280 I2C bus speed is 3.4 MHz.
}

BME280::BME280(i2c::Master& i2c_master)
    : dev_({
          .chip_id = 0,  // Will be retrieved from sensor.
          .intf = BME280_I2C_INTF,
          .intf_ptr = this,
          .intf_rslt = BME280_INTF_RET_SUCCESS,
          .read = ReadFunc,
          .write = WriteFunc,
          .delay_us = DelayUsFunc,
          .calib_data = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                         0},
      }),
      settings_({
          .osr_p = BME280_OVERSAMPLING_4X,
          .osr_t = BME280_OVERSAMPLING_8X,
          .osr_h = BME280_OVERSAMPLING_16X,
          .filter = BME280_FILTER_COEFF_2,
          .standby_time = BME280_STANDBY_TIME_0_5_MS,
      }),
      i2c_master_(i2c_master) {}

esp_err_t BME280::ReadSettings() {
  int8_t s = bme280_get_sensor_settings(&settings_, &dev_);
  if (IsWarning(s)) {
    ESP_LOGW(TAG, "bme280_get_sensor_settings returned %d", s);
    return ESP_OK;
  }
  return BmeToEspErr(s);
}

esp_err_t BME280::SetSensorMode() {
  int8_t s = bme280_set_sensor_mode(BME280_POWERMODE_NORMAL, &dev_);
  if (IsWarning(s)) {
    ESP_LOGW(TAG, "bme280_set_sensor_mode returned %d", s);
    return ESP_OK;
  }
  return BmeToEspErr(s);
}

esp_err_t BME280::WriteSettings() {
  int8_t s =
      bme280_set_sensor_settings(BME280_SEL_ALL_SETTINGS, &settings_, &dev_);
  if (IsWarning(s)) {
    ESP_LOGW(TAG, "bme280_set_sensor_settings returned %d", s);
    return ESP_OK;
  }
  return BmeToEspErr(s);
}

esp_err_t BME280::CalcMeasurementDelay() {
  int8_t s = bme280_cal_meas_delay(&period_, &settings_);
  if (IsError(s))
    return BmeToEspErr(s);

  if (IsWarning(s))
    ESP_LOGW(TAG, "bme280_cal_meas_delay returned %d", s);
  ESP_LOGD(TAG, "Measurement delay: %" PRIu32 " usec", period_);
  return ESP_OK;
}

std::expected<SensorData, esp_err_t> BME280::ReadData(uint8_t values) {
  for (int8_t i = 0; i < kMaxSampleCount; i++) {
    uint8_t status_reg;
    int8_t s = bme280_get_regs(BME280_REG_STATUS, &status_reg, 1, &dev_);
    if (IsError(s)) {
      return std::unexpected(BmeToEspErr(s));
    }

    if (status_reg & BME280_STATUS_MEAS_DONE) {
      // Measurement time delay given to read the sample.
      dev_.delay_us(period_, dev_.intf_ptr);

      // Read the compensated data.
      bme280_data comp_data;
      s = bme280_get_sensor_data(values, &comp_data, &dev_);
      if (IsError(s)) {
        return std::unexpected(BmeToEspErr(s));
      }

      SensorData data;

#ifdef BME280_DOUBLE_ENABLE
      if (values & kBME280Pressure)
        data.pressure = comp_data.pressure;
      if (values & kBME280Temperature)
        data.temperature = comp_data.temperature;
      if (values & kBME280Humidity)
        data.humidity = comp_data.humidity;
#else
      if (values & kBME280Pressure)
        data.pressure = comp_data.pressure / 100;
      if (values & kBME280Temperature)
        data.temperature = comp_data.temperature / 100;
      if (values & kBME280Humidity)
        data.humidity = comp_data.humidity / 100;
#endif
      return data;
    }
  }
  return std::unexpected(ESP_FAIL);
}

esp_err_t BME280::EnterSleep() {
  int ret = bme280_set_sensor_mode(BME280_POWERMODE_SLEEP, &dev_);
  if (ret == 0) {
    return ESP_OK;
  } else if (ret < 0) {
    ESP_LOGE(TAG, "Got error entering sleep");
    return ESP_FAIL;
  } else {
    ESP_LOGW(TAG, "Got warning entering sleep");
    return ESP_OK;
  }
}

esp_err_t BME280::Init() {
  int8_t s = bme280_init(&dev_);
  if (IsError(s)) {
    ESP_LOGE(TAG, "Failure initializing BME280: %d", s);
    return BmeToEspErr(s);
  }
  if (IsWarning(s)) {
    ESP_LOGW(TAG, "Warning initializing BME280: %d", s);
  }
  esp_err_t err;
  if (err = WriteSettings(); err != ESP_OK) {
    return err;
  }
  if (err = SetSensorMode(); err != ESP_OK) {
    return false;
  }
  if (err = CalcMeasurementDelay(); err != ESP_OK) {
    return false;
  }

  ESP_LOGI(TAG, "BME280 initialized");
  return ESP_OK;
}