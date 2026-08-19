#pragma once
#include "adcs/control.hpp"
#include "adcs/estimation.hpp"
#include "adcs/mode.hpp"
namespace adcs {
struct FlightConfig{
  MekfConfig mekf{};
  ModeConfig modes{};
  Vec3 pd_kp{.0009792,.000864,.0002016};
  Vec3 pd_kd{.007344,.00648,.001512};
  Matrix3x6 lqr_gain=[](){Matrix3x6 k=Matrix3x6::Zero();k(0,0)=k(1,1)=k(2,2)=.0005;k(0,3)=.00583309523323595;k(1,4)=.00547950727711899;k(2,5)=.002650471656139714;return k;}();
  Mat3 wheel_axes=Mat3::Identity();
  Vec3 wheel_max_torque=Vec3::Constant(2e-4);
  double magnetometer_vector_noise_std=.008;
  double sun_vector_noise_std=.015;
};
}  // namespace adcs
