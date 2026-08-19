#pragma once
#include "adcs/config.hpp"
#include "adcs/guidance.hpp"
namespace adcs {
struct VectorObservation{Vec3 measured_body;Vec3 reference_eci;double noise_std;bool valid;};
struct AttitudeReference{Quat q_ib;Vec3 body_rate_rad_s=Vec3::Zero();};
struct ActuatorOutput{Vec3 requested_body_torque_nm=Vec3::Zero();Vec3 wheel_motor_torque_nm=Vec3::Zero();Mode mode=Mode::initialization;};
enum class FeedbackLaw:std::uint8_t{pd,lqr};

class FlightComputer{
 public:
  explicit FlightComputer(FlightConfig config={},Quat initial_q={1,0,0,0},Vec3 initial_bias=Vec3::Zero()):config_(config),estimate_(initialize_mekf(initial_q,initial_bias,config.mekf)){}
  void predict_gyro(const Vec3&gyro_rad_s,double dt_s){mekf_predict(estimate_,gyro_rad_s,dt_s,config_.mekf);}
  void update_vector(const VectorObservation&o){if(o.valid)mekf_update_vector(estimate_,o.measured_body,o.reference_eci,o.noise_std);}
  void update_mode_state(const ModeStatus&s){update_mode(mode_,s,config_.modes);}
  ActuatorOutput control(const AttitudeReference&r,const Vec3&estimated_body_rate_rad_s,FeedbackLaw law)const{
    const Vec3 rate=estimated_body_rate_rad_s-r.body_rate_rad_s;
    Vec3 body=law==FeedbackLaw::pd?quaternion_pd(r.q_ib,estimate_.q_ib,rate,config_.pd_kp,config_.pd_kd):quaternion_lqr(r.q_ib,estimate_.q_ib,rate,config_.lqr_gain);
    Vec3 motors=allocate_wheel_torque<3>(body,config_.wheel_axes);
    motors=motors.cwiseMax(-config_.wheel_max_torque).cwiseMin(config_.wheel_max_torque);
    return{body,motors,mode_.mode};
  }
  const MekfState& estimate()const{return estimate_;}
  const ModeState& mode_state()const{return mode_;}
  const FlightConfig& config()const{return config_;}
 private:
  FlightConfig config_;MekfState estimate_;ModeState mode_{};
};
}  // namespace adcs
