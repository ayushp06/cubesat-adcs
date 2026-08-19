#pragma once
#include <cstdint>
namespace adcs {
enum class Mode:std::uint8_t{initialization,detumble,safe,nominal,slew,desaturation,fault};
enum class GuidanceMode:std::uint8_t{none,rate,safe,nominal,slew,hold};
struct ModeConfig{double detumble_entry=.08726646259971647,detumble_exit=.008726646259971648,desat_entry=.85,desat_exit=.50;};
struct ModeStatus{bool fault=false,fault_reset=false,initialization_complete=false,safe_requested=false,estimator_valid=true,slew_requested=false,slew_complete=false,nominal_requested=false;double body_rate=0,wheel_speed_fraction=0;};
struct ModeState{Mode mode=Mode::initialization,previous_operational_mode=Mode::safe;};
struct ModeCommand{bool reaction_wheels=false,magnetorquers=false;GuidanceMode guidance=GuidanceMode::none;};
inline void update_mode(ModeState&x,const ModeStatus&s,const ModeConfig&p={}){if(s.fault){x.mode=Mode::fault;return;}if(x.mode==Mode::fault){if(s.fault_reset)x.mode=Mode::initialization;return;}if(x.mode==Mode::initialization){if(!s.initialization_complete)return;x.mode=s.body_rate>p.detumble_entry?Mode::detumble:Mode::safe;return;}if(x.mode==Mode::detumble){if(s.body_rate<=p.detumble_exit)x.mode=Mode::safe;return;}if(s.safe_requested||!s.estimator_valid){x.mode=Mode::safe;return;}if(x.mode==Mode::desaturation){if(s.wheel_speed_fraction<=p.desat_exit)x.mode=x.previous_operational_mode;return;}if(s.wheel_speed_fraction>=p.desat_entry){x.previous_operational_mode=x.mode;x.mode=Mode::desaturation;return;}if(s.slew_requested){x.mode=Mode::slew;return;}if(x.mode==Mode::slew&&!s.slew_complete)return;x.mode=s.nominal_requested?Mode::nominal:Mode::safe;}
inline ModeCommand mode_command(Mode m){if(m==Mode::detumble)return{false,true,GuidanceMode::rate};if(m==Mode::safe)return{true,false,GuidanceMode::safe};if(m==Mode::nominal)return{true,false,GuidanceMode::nominal};if(m==Mode::slew)return{true,false,GuidanceMode::slew};if(m==Mode::desaturation)return{true,true,GuidanceMode::hold};return{};}
}  // namespace adcs
