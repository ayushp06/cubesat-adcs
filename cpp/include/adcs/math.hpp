#pragma once
#include <Eigen/Core>
#include <Eigen/Eigenvalues>
#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace adcs {
using Vec3 = Eigen::Vector3d;
using Quat = Eigen::Vector4d;
using Mat3 = Eigen::Matrix3d;
inline Quat normalize(const Quat& q){const double n=q.norm();if(n<1e-15)throw std::invalid_argument("zero quaternion");return q/n;}
inline Quat conjugate(const Quat& q){return {q(0),-q(1),-q(2),-q(3)};}
inline Quat multiply(const Quat&a,const Quat&b){return {a(0)*b(0)-a(1)*b(1)-a(2)*b(2)-a(3)*b(3),a(0)*b(1)+a(1)*b(0)+a(2)*b(3)-a(3)*b(2),a(0)*b(2)-a(1)*b(3)+a(2)*b(0)+a(3)*b(1),a(0)*b(3)+a(1)*b(2)-a(2)*b(1)+a(3)*b(0)};}
inline Mat3 skew(const Vec3&v){Mat3 m;m<<0,-v(2),v(1),v(2),0,-v(0),-v(1),v(0),0;return m;}
inline Mat3 to_dcm(const Quat&input){const auto q=normalize(input);const double w=q(0),x=q(1),y=q(2),z=q(3);Mat3 c;c<<1-2*(y*y+z*z),2*(x*y-w*z),2*(x*z+w*y),2*(x*y+w*z),1-2*(x*x+z*z),2*(y*z-w*x),2*(x*z-w*y),2*(y*z+w*x),1-2*(x*x+y*y);return c;}
inline Quat from_dcm(const Mat3&c){Eigen::Matrix4d k;k<<c(0,0)-c(1,1)-c(2,2),c(1,0)+c(0,1),c(2,0)+c(0,2),c(1,2)-c(2,1),c(1,0)+c(0,1),c(1,1)-c(0,0)-c(2,2),c(2,1)+c(1,2),c(2,0)-c(0,2),c(2,0)+c(0,2),c(2,1)+c(1,2),c(2,2)-c(0,0)-c(1,1),c(0,1)-c(1,0),c(1,2)-c(2,1),c(2,0)-c(0,2),c(0,1)-c(1,0),c.trace();Eigen::SelfAdjointEigenSolver<Eigen::Matrix4d>s(k/3.0);const auto v=s.eigenvectors().col(3);Quat q{v(3),-v(0),-v(1),-v(2)};q=normalize(q);if(q(0)<0)q=-q;return q;}
inline double attitude_error_angle(const Quat&a,const Quat&b){return 2*std::acos(std::clamp(std::abs(normalize(a).dot(normalize(b))),0.0,1.0));}
}  // namespace adcs
