#include "adcs/guidance.hpp"
#include <gtest/gtest.h>
TEST(Math,QuaternionConvention){const adcs::Quat q{std::sqrt(.5),0,0,std::sqrt(.5)};EXPECT_NEAR((adcs::to_dcm(q)*adcs::Vec3::UnitX()-adcs::Vec3::UnitY()).norm(),0,1e-14);EXPECT_LT(adcs::attitude_error_angle(adcs::from_dcm(adcs::to_dcm(q)),q),1e-12);}
TEST(Guidance,PointingAndSlew){const adcs::Vec3 sun=adcs::Vec3(1,2,3).normalized();EXPECT_LT((adcs::to_dcm(adcs::sun_pointing(sun))*adcs::Vec3::UnitZ()-sun).norm(),1e-12);const auto nadir=adcs::nadir_pointing({7e6,0,0},{0,7500,0});EXPECT_LT((adcs::to_dcm(nadir)*adcs::Vec3::UnitZ()+adcs::Vec3::UnitX()).norm(),1e-12);const auto slew=adcs::slew_reference({1,0,0,0},{std::sqrt(.5),0,0,std::sqrt(.5)},5,10);EXPECT_NEAR(slew.body_rate_rad_s(2),3*std::acos(-1.0)/40,1e-12);}
