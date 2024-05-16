#include <array>
#include <expected>
#include <optional>

#include <bme68x.h>
#include <i2clib/master.h>

#include "sensor_data.h"

#pragma once

// Bitmask values for GetData.
constexpr uint8_t kBME680Pressure = 1;
constexpr uint8_t kBME680Temperature = 1 << 1;
constexpr uint8_t kBME680Humidity = 1 << 2;
constexpr uint8_t kBME680All = 0x07;

class BME680 {
 public:
  BME680(i2c::Master& i2c_master);

  bool Init();
  // Retrieve the sensor data.
  // |values| is a bitmask identifying the desired values to read from the
  // sensor - See kBME680All above. If this function succeeds then all
  // requested values will be set.
  std::expected<SensorData, BME68X_INTF_RET_TYPE> ReadData();

 private:
  // A read callback function used by the BME680 library.
  static BME68X_INTF_RET_TYPE ReadFunc(uint8_t reg_addr,
                                       uint8_t* reg_data,
                                       uint32_t len,
                                       void* intf_ptr);
  // A write callback function used by the BME680 library.
  static BME68X_INTF_RET_TYPE WriteFunc(uint8_t reg_addr,
                                        const uint8_t* reg_data,
                                        uint32_t len,
                                        void* intf_ptr);
  // A delay callback function used by the BME680 library.
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