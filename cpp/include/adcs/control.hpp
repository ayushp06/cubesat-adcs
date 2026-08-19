#pragma once
#include <array>
#include <cmath>

namespace adcs {
using Vec3 = std::array<double, 3>;
using Quat = std::array<double, 4>;

inline Quat conjugate(const Quat& q) { return {q[0], -q[1], -q[2], -q[3]}; }
inline Quat multiply(const Quat& a, const Quat& b) {
  return {a[0]*b[0]-a[1]*b[1]-a[2]*b[2]-a[3]*b[3],
          a[0]*b[1]+a[1]*b[0]+a[2]*b[3]-a[3]*b[2],
          a[0]*b[2]-a[1]*b[3]+a[2]*b[0]+a[3]*b[1],
          a[0]*b[3]+a[1]*b[2]-a[2]*b[1]+a[3]*b[0]};
}
inline Vec3 quaternion_pd(const Quat& reference, const Quat& estimate,
                          const Vec3& rate, const Vec3& kp, const Vec3& kd) {
  auto error = multiply(conjugate(reference), estimate);
  if (error[0] < 0) for (auto& value : error) value = -value;
  return {-kp[0]*error[1]-kd[0]*rate[0],
          -kp[1]*error[2]-kd[1]*rate[1],
          -kp[2]*error[3]-kd[2]*rate[2]};
}
}  // namespace adcs
