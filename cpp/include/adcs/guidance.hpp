#pragma once
#include "adcs/math.hpp"
namespace adcs {
struct SlewReference{Quat attitude;Vec3 body_rate_rad_s;};
inline Quat triad(const Eigen::Matrix<double,3,2>&body,const Eigen::Matrix<double,3,2>&inertial){const Vec3 b1=body.col(0).normalized(),r1=inertial.col(0).normalized();Vec3 b2=b1.cross(body.col(1)),r2=r1.cross(inertial.col(1));if(b2.norm()<1e-10||r2.norm()<1e-10)throw std::invalid_argument("collinear TRIAD vectors");b2.normalize();r2.normalize();Mat3 tb,ti;tb<<b1,b2,b1.cross(b2);ti<<r1,r2,r1.cross(r2);return from_dcm(ti*tb.transpose());}
inline Quat inertial_pointing(const Quat&q){return normalize(q);}
inline Quat vector_pointing(const Vec3&ba,const Vec3&t,const Vec3&bs,const Vec3&s){Eigen::Matrix<double,3,2>b,r;b<<ba,bs;r<<t,s;return triad(b,r);}
inline Quat sun_pointing(const Vec3&sun){Vec3 second=Vec3::UnitZ();if(sun.cross(second).norm()<1e-8)second=Vec3::UnitX();return vector_pointing(Vec3::UnitZ(),sun,Vec3::UnitX(),second);}
inline Quat safe_attitude(const Vec3&sun){return sun_pointing(sun);}
inline Quat nadir_pointing(const Vec3&r,const Vec3&v){const Vec3 z=-r.normalized(),x=(v-v.dot(z)*z).normalized();Mat3 c;c<<x,z.cross(x),z;return from_dcm(c);}
inline SlewReference slew_reference(Quat a,Quat b,double t,double duration){a=normalize(a);b=normalize(b);if(a.dot(b)<0)b=-b;const Quat rel=multiply(conjugate(a),b);const double angle=2*std::acos(std::clamp(rel(0),-1.0,1.0));if(angle<1e-12)return{a,Vec3::Zero()};const Vec3 axis=rel.tail<3>()/std::sin(angle/2);const double u=std::clamp(t/duration,0.0,1.0),s=3*u*u-2*u*u*u,rate=(6*u-6*u*u)/duration;Quat dq;dq<<std::cos(s*angle/2),axis*std::sin(s*angle/2);return{normalize(multiply(a,dq)),axis*angle*rate};}
}  // namespace adcs
