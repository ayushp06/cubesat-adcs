#pragma once
#include <cstdint>

namespace adcs {
struct TimingContract {
  static constexpr std::uint32_t gyro_period_us = 10'000;
  static constexpr std::uint32_t estimator_period_us = 10'000;
  static constexpr std::uint32_t control_period_us = 50'000;
  static constexpr std::uint32_t magnetometer_period_us = 100'000;
  static constexpr std::uint32_t sun_period_us = 200'000;
  static constexpr std::uint32_t gps_period_us = 1'000'000;
  static constexpr std::uint32_t telemetry_period_us = 100'000;
};
inline bool due(std::uint32_t now, std::uint32_t last, std::uint32_t period) {
  return static_cast<std::uint32_t>(now - last) >= period;
}
}  // namespace adcs
