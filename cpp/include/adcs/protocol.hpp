#pragma once
#include <array>
#include <cstddef>
#include <cstdint>

namespace adcs {
constexpr std::uint16_t kSync = 0xA55A;
constexpr std::uint8_t kProtocolVersion = 1;

enum class PacketType : std::uint8_t {
  imu = 1, sun = 2, gps = 3, attitude = 16, actuator_command = 32, heartbeat = 48
};

#pragma pack(push, 1)
struct FrameHeader {
  std::uint16_t sync;
  std::uint8_t version;
  PacketType type;
  std::uint16_t payload_bytes;
  std::uint16_t sequence;
  std::uint32_t timestamp_us;
};
struct ImuPacket { float gyro_rad_s[3]; float magnetic_tesla[3]; std::uint32_t status; };
struct SunPacket { float direction_body[3]; std::uint8_t valid; };
struct GpsPacket { double position_eci_m[3]; float velocity_eci_m_s[3]; std::uint8_t valid; };
struct AttitudePacket {
  float q_ib[4]; float gyro_bias_rad_s[3]; float covariance_diagonal[6];
  std::uint8_t mode; std::uint32_t status;
};
struct ActuatorCommandPacket {
  float wheel_torque_nm[3]; float magnetic_dipole_am2[3]; std::uint32_t command_valid_until_us;
};
#pragma pack(pop)

inline std::uint32_t crc32(const std::uint8_t* data, std::size_t size) {
  std::uint32_t crc = 0xFFFFFFFFu;
  for (std::size_t i = 0; i < size; ++i) {
    crc ^= data[i];
    for (int bit = 0; bit < 8; ++bit)
      crc = (crc >> 1) ^ (0xEDB88320u & (0u - (crc & 1u)));
  }
  return ~crc;
}
}  // namespace adcs
