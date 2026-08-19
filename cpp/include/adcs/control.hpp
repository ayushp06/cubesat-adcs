#pragma once
#include "adcs/math.hpp"
#include <Eigen/QR>
namespace adcs {
using Matrix3x6=Eigen::Matrix<double,3,6>;
inline Quat attitude_error(const Quat&r,const Quat&q){Quat e=multiply(conjugate(normalize(r)),normalize(q));if(e(0)<0)e=-e;return e;}
inline Vec3 quaternion_pd(const Quat&r,const Quat&q,const Vec3&w,const Vec3&kp,const Vec3&kd){return-kp.cwiseProduct(attitude_error(r,q).tail<3>())-kd.cwiseProduct(w);}
inline Vec3 quaternion_lqr(const Quat&r,const Quat&q,const Vec3&w,const Matrix3x6&k){Eigen::Matrix<double,6,1>x;x<<2*attitude_error(r,q).tail<3>(),w;return-k*x;}
template<int Wheels>inline Eigen::Matrix<double,Wheels,1> allocate_wheel_torque(const Vec3&t,const Eigen::Matrix<double,3,Wheels>&a){return a.completeOrthogonalDecomposition().solve(-t);}
}  // namespace adcs
