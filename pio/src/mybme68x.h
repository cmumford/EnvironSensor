#include <array>
#include <expected>
#include <optional>

#include <bme68x.h>
#include <i2clib/master.h>

#include "sensor.h"

#pragma once

class BME68X : public Sensor {
 public:
  static uint32_t MaxI2CClockSpeed();

  BME68X(i2c::Master& i2c_master);

  esp_err_t Init() override;
  std::expected<SensorData, esp_err_t> ReadData(uint8_t values) override;
  esp_err_t EnterSleep() override;

 private:
  // A read callback function used by the BME68X library.
  static BME68X_INTF_RET_TYPE ReadFunc(uint8_t reg_addr,
                                       uint8_t* reg_data,
                                       uint32_t len,
                                       void* intf_ptr);
  // A write callback function used by the BME68X library.
  static BME68X_INTF_RET_TYPE WriteFunc(uint8_t reg_addr,
                                        const uint8_t* reg_data,
                                        uint32_t len,
                                        void* intf_ptr);
  // A delay callback function used by the BME68X library.
  static void DelayUsFunc(uint32_t period, void* intf_ptr);

  BME68X_INTF_RET_TYPE InternalInit();

  bme68x_dev dev_;
  bme68x_conf conf_;
  std::array<uint16_t, 10> gas_heater_temp_profile_;
  std::array<uint16_t, 10> gas_heater_dur_profile_;
  bme68x_heatr_conf gas_heater_conf_;
  uint32_t period_ = 0;
  i2c::Master& i2c_master_;
};