#include "adcs/protocol.hpp"
#include "adcs/timing.hpp"
#include <gtest/gtest.h>

TEST(Protocol,PackedLayoutAndCrc){
  static_assert(sizeof(adcs::FrameHeader)==12);
  static_assert(sizeof(adcs::ImuPacket)==28);
  static_assert(sizeof(adcs::SunPacket)==13);
  static_assert(sizeof(adcs::GpsPacket)==37);
  static_assert(sizeof(adcs::ActuatorCommandPacket)==28);
  const auto* bytes=reinterpret_cast<const std::uint8_t*>("123456789");
  EXPECT_EQ(adcs::crc32(bytes,9),0xCBF43926u);
}
TEST(Timing,UnsignedRollover){
  EXPECT_TRUE(adcs::due(5u,0xFFFFFFF0u,20u));
  EXPECT_FALSE(adcs::due(5u,0u,10u));
}
