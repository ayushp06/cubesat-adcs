#include "adcs/estimation.hpp"
#include <gtest/gtest.h>

TEST(Estimation,MekfPredictionAndCorrection){
  adcs::MekfConfig p; p.gyro_noise_std=0; p.bias_random_walk_std=0;
  auto prediction=adcs::initialize_mekf({1,0,0,0},adcs::Vec3::Zero(),p);
  adcs::mekf_predict(prediction,{0,0,.1},1,p);
  EXPECT_NEAR(2*std::acos(prediction.q_ib(0)),.1,1e-12);
  auto x=adcs::initialize_mekf({1,0,0,0},adcs::Vec3::Zero(),p);
  EXPECT_GT(x.covariance(0,0),0);
  const adcs::Quat truth{std::cos(std::acos(-1.0)/18),0,0,std::sin(std::acos(-1.0)/18)};
  Eigen::Matrix<double,3,2> refs; refs<<1,0,0,1,0,0;
  const Eigen::Matrix<double,3,2> body=adcs::to_dcm(truth).transpose()*refs;
  adcs::mekf_update_vector(x,body.col(0),refs.col(0),1e-3);
  EXPECT_GT(std::abs(x.q_ib(3)),.01);
  for(int i=0;i<8;++i){
    adcs::mekf_update_vector(x,body.col(0),refs.col(0),1e-3);
    adcs::mekf_update_vector(x,body.col(1),refs.col(1),1e-3);
  }
  EXPECT_LT(adcs::attitude_error_angle(x.q_ib,truth),.002);
  EXPECT_GE(x.covariance.selfadjointView<Eigen::Lower>().eigenvalues().minCoeff(),-1e-12);
}
TEST(Estimation,QuestKnownAttitude){
  const adcs::Quat truth{std::cos(std::acos(-1.0)/6),0,0,std::sin(std::acos(-1.0)/6)};
  Eigen::Matrix<double,3,3> refs; refs<<1,0,1,0,1,1,0,0,1; refs.colwise().normalize();
  const Eigen::Matrix<double,3,3> body=adcs::to_dcm(truth).transpose()*refs;
  Eigen::Vector3d weights{1,2,.5};
  EXPECT_LT(adcs::attitude_error_angle(adcs::quest(body,refs,weights),truth),1e-12);
}
