#include "adcs/control.hpp"
#include "adcs/protocol.hpp"
#include "adcs/timing.hpp"
#include <cmath>
#include <cstdint>
#include <iostream>

void require(bool condition) {
  if (!condition) throw "SIL check failed";
}

int main() {
  static_assert(sizeof(adcs::FrameHeader) == 12);
  static_assert(sizeof(adcs::ImuPacket) == 28);
  static_assert(sizeof(adcs::SunPacket) == 13);
  static_assert(sizeof(adcs::GpsPacket) == 37);
  static_assert(sizeof(adcs::ActuatorCommandPacket) == 28);
  const auto* check = reinterpret_cast<const std::uint8_t*>("123456789");
  require(adcs::crc32(check, 9) == 0xCBF43926u);
  require(adcs::due(5u, 0xFFFFFFF0u, 20u));

  const double half = std::sqrt(0.5);
  adcs::Quat identity{1, 0, 0, 0}, x90{half, half, 0, 0};
  adcs::Vec3 zero{0, 0, 0}, kp{1, 1, 1}, kd{1, 1, 1};
  const auto torque = adcs::quaternion_pd(x90, identity, zero, kp, kd);
  require(torque[0] > 0 && torque[1] == 0 && torque[2] == 0);
  const auto damping = adcs::quaternion_pd(identity, identity, {1, -2, 3}, kp, kd);
  require(damping[0] < 0 && damping[1] > 0 && damping[2] < 0);
  std::cout << "ADCS SIL TEST PASSED\n";
}
