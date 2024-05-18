#include "mybme68x.h"

#include <cassert>

#include <esp_log.h>
#include <esp_system.h>
#include <i2clib/operation.h>

namespace {

constexpr char TAG[] = "BME68X";
constexpr i2c::Address kSlaveAddress = {.address = BME68X_I2C_ADDR_LOW,
                                        .addr_size = i2c::Address::Size::k7bit};

constexpr bool IsError(int8_t status_code) {
  return status_code < 0;
}

constexpr bool IsWarning(int8_t status_code) {
  return status_code > 0;
}

}  // namespace

// static
BME68X_INTF_RET_TYPE BME68X::ReadFunc(uint8_t reg_addr,
                                      uint8_t* reg_data,
                                      uint32_t len,
                                      void* intf_ptr) {
  BME68X* instance = static_cast<BME68X*>(intf_ptr);
  i2c::Operation op = instance->i2c_master_.CreateReadOp(
      kSlaveAddress, reg_addr, "bme-read-cb");
  if (!op.Read(reg_data, len).ok())
    return BME68X_E_COM_FAIL;
  return op.Execute().ok() ? BME68X_OK : BME68X_E_COM_FAIL;
}

// static
BME68X_INTF_RET_TYPE BME68X::WriteFunc(uint8_t reg_addr,
                                       const uint8_t* reg_data,
                                       uint32_t len,
                                       void* intf_ptr) {
  BME68X* instance = static_cast<BME68X*>(intf_ptr);
  i2c::Operation op = instance->i2c_master_.CreateWriteOp(
      kSlaveAddress, reg_addr, "bme-write-cb");
  if (!op.Write(reg_data, len).ok())
    return BME68X_E_COM_FAIL;
  return op.Execute().ok() ? BME68X_OK : BME68X_E_COM_FAIL;
}

// static
void BME68X::DelayUsFunc(uint32_t period, void* intf_ptr) {
  esp_rom_delay_us(period);
}

BME68X::BME68X(i2c::Master& i2c_master)
    : dev_({
          .chip_id = 0,
          .intf_ptr = this,
          .variant_id = BME68X_VARIANT_GAS_LOW,
          .intf = BME68X_I2C_INTF,
          .mem_page = 0,
          .amb_temp = 25,
          .calib = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
          .read = ReadFunc,
          .write = WriteFunc,
          .delay_us = DelayUsFunc,
          .intf_rslt = BME68X_OK,
          .info_msg = BME68X_OK,
      }),
      conf_({
          .os_hum = BME68X_OS_16X,
          .os_temp = BME68X_OS_8X,
          .os_pres = BME68X_OS_4X,
          .filter = BME68X_FILTER_SIZE_3,
          .odr = BME68X_ODR_NONE,
      }),
      gas_heater_temp_profile_(
          {200, 240, 280, 320, 360, 360, 320, 280, 240, 200}),
      gas_heater_dur_profile_(
          {100, 100, 100, 100, 100, 100, 100, 100, 100, 100}),
      gas_heater_conf_({
          .enable = BME68X_ENABLE,
          .heatr_temp = 320,
          .heatr_dur = 150,
          .heatr_temp_prof = gas_heater_temp_profile_.data(),
          .heatr_dur_prof = gas_heater_dur_profile_.data(),
          .profile_len = static_cast<uint8_t>(gas_heater_temp_profile_.size()),
          .shared_heatr_dur = 0,
      }),
      i2c_master_(i2c_master) {}

std::expected<SensorData, BME68X_INTF_RET_TYPE> BME68X::ReadData() {
  BME68X_INTF_RET_TYPE rslt;
  rslt = bme68x_set_op_mode(BME68X_FORCED_MODE, &dev_);
  if (rslt != BME68X_OK)
    return std::unexpected(rslt);

  uint32_t delay_period_usec =
      bme68x_get_meas_dur(BME68X_FORCED_MODE, &conf_, &dev_) +
      (gas_heater_conf_.heatr_dur_prof[0] * 1000);
  dev_.delay_us(delay_period_usec, dev_.intf_ptr);

  bme68x_data data = {};
  uint8_t n_fields = 0;
  rslt = bme68x_get_data(BME68X_FORCED_MODE, &data, &n_fields, &dev_);
  if (rslt != BME68X_OK)
    return std::unexpected(rslt);
  assert(n_fields == 1);

  return SensorData{
      .pressure = data.pressure,
      .temperature = data.temperature,
      .humidity = data.humidity,
      .gas_resistance =
          data.status & (BME68X_HEAT_STAB_MSK | BME68X_GASM_VALID_MSK)
              ? std::optional<float>(data.gas_resistance)
              : std::nullopt};
}

BME68X_INTF_RET_TYPE BME68X::InternalInit() {
  BME68X_INTF_RET_TYPE rslt;

  rslt = bme68x_init(&dev_);
  if (rslt != BME68X_OK)
    return rslt;

  rslt = bme68x_set_conf(&conf_, &dev_);
  if (rslt != BME68X_OK)
    return rslt;

  rslt = bme68x_set_heatr_conf(BME68X_FORCED_MODE, &gas_heater_conf_, &dev_);
  if (rslt != BME68X_OK)
    return rslt;

  rslt = bme68x_set_op_mode(BME68X_FORCED_MODE, &dev_);
  if (rslt != BME68X_OK)
    return rslt;

  return BME68X_OK;
}

bool BME68X::Init() {
  BME68X_INTF_RET_TYPE err = InternalInit();
  if (err != BME68X_OK) {
    ESP_LOGE(TAG, "Error initializing BME68X: %d", err);
    return false;
  }
  return true;
}
