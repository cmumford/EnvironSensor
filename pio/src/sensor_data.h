#pragma once

#include <optional>

struct SensorData {
  std::optional<double> pressure;        // Pressure in Pascals (Pa).
  std::optional<double> temperature;     // Temperature in Celsius.
  std::optional<double> humidity;        // Percent relative humidity.
  std::optional<double> gas_resistance;  // Ohms.
};