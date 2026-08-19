#include "adcs/flight.hpp"
#include <gtest/gtest.h>
#include <fstream>
#include <map>
#include <sstream>
#include <string>
#include <vector>

namespace {
using Table=std::map<std::string,std::vector<double>>;
Table load(){std::ifstream file(std::string(ADCS_SOURCE_DIR)+"/cpp/tests/data/matlab_reference.csv");if(!file)throw std::runtime_error("MATLAB reference vectors missing");Table table;std::string line;while(std::getline(file,line)){std::stringstream stream(line);std::string field;std::getline(stream,field,',');auto&values=table[field];while(std::getline(stream,field,','))values.push_back(std::stod(field));}return table;}
template<class Derived>void expect_vector(const Eigen::MatrixBase<Derived>&actual,const std::vector<double>&expected,double tolerance){ASSERT_EQ(static_cast<std::size_t>(actual.size()),expected.size());for(Eigen::Index i=0;i<actual.size();++i)EXPECT_NEAR(actual(i),expected[static_cast<std::size_t>(i)],tolerance);}
}  // namespace

TEST(CrossValidation,MathGuidanceControlAndModes){
  const auto ref=load();const adcs::Quat q1=adcs::normalize({.9,.1,-.2,.3}),q2=adcs::normalize({.8,-.3,.1,.2});
  expect_vector(adcs::multiply(q1,q2),ref.at("multiply"),1e-14);
  const adcs::Mat3 dcm=adcs::to_dcm(q1);expect_vector(Eigen::Map<const Eigen::VectorXd>(dcm.data(),9),ref.at("dcm"),1e-14);
  const adcs::Vec3 sun=adcs::Vec3(1,2,3).normalized();expect_vector(adcs::sun_pointing(sun),ref.at("sun"),1e-12);
  expect_vector(adcs::nadir_pointing({7e6,1e5,-2e5},{-100,7500,900}),ref.at("nadir"),1e-12);
  const auto slew=adcs::slew_reference({1,0,0,0},{std::sqrt(.5),0,0,std::sqrt(.5)},3,10);expect_vector(slew.attitude,ref.at("slew_q"),1e-13);expect_vector(slew.body_rate_rad_s,ref.at("slew_rate"),1e-13);
  adcs::FlightConfig config;const adcs::Quat qr{std::sqrt(.5),std::sqrt(.5),0,0},qe=adcs::normalize({.98,.1,-.05,.02});const adcs::Vec3 omega{.01,-.02,.03};
  expect_vector(adcs::quaternion_pd(qr,qe,omega,config.pd_kp,config.pd_kd),ref.at("pd"),1e-15);
  expect_vector(adcs::quaternion_lqr(qr,qe,omega,config.lqr_gain),ref.at("lqr"),2e-15);
  expect_vector(adcs::allocate_wheel_torque<3>({1e-4,-2e-4,5e-5},config.wheel_axes),ref.at("allocation"),1e-15);
  adcs::ModeState state;adcs::ModeStatus s;s.initialization_complete=true;s.nominal_requested=true;s.body_rate=.1;std::vector<double>m;adcs::update_mode(state,s);m.push_back(static_cast<int>(state.mode));s.body_rate=.001;adcs::update_mode(state,s);m.push_back(static_cast<int>(state.mode));adcs::update_mode(state,s);m.push_back(static_cast<int>(state.mode));s.wheel_speed_fraction=.9;adcs::update_mode(state,s);m.push_back(static_cast<int>(state.mode));s.fault=true;adcs::update_mode(state,s);m.push_back(static_cast<int>(state.mode));EXPECT_EQ(m,ref.at("modes"));
}

TEST(CrossValidation,Mekf){
  const auto ref=load();adcs::MekfConfig p;auto x=adcs::initialize_mekf({1,0,0,0},{.001,-.002,.0005},p);adcs::mekf_predict(x,{.01,-.02,.03},.01,p);
  expect_vector(x.q_ib,ref.at("mekf_predict_q"),1e-14);expect_vector(x.gyro_bias,ref.at("mekf_predict_bias"),1e-15);expect_vector(x.covariance.diagonal(),ref.at("mekf_predict_pdiag"),1e-13);
  adcs::mekf_update_vector(x,{.3,-.4,.866025403784},{.2,-.1,.974679434481},.008);
  expect_vector(x.q_ib,ref.at("mekf_update_q"),2e-13);expect_vector(x.gyro_bias,ref.at("mekf_update_bias"),1e-13);expect_vector(x.covariance.diagonal(),ref.at("mekf_update_pdiag"),2e-13);EXPECT_NEAR(x.last_nis,ref.at("mekf_update_nis").front(),2e-10);
}
