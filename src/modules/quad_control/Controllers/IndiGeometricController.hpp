#pragma once

#include <lib/mathlib/math/filter/LowPassFilter2p.hpp>
#include <lib/mathlib/mathlib.h>
#include <lib/matrix/matrix/math.hpp>
// #include <uORB/topics/vehicle_angular_acceleration.h>
#include <uORB/topics/trajectory_setpoint.h>
#include <uORB/topics/vehicle_acceleration.h>
#include <uORB/topics/vehicle_angular_velocity.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_local_position.h>

using namespace matrix;

class IndiGeometricController {

public:
  IndiGeometricController();

  void set_gains();
  void update_state_pos(vehicle_local_position_s pos);
  void update_state_attitude(vehicle_attitude_s att);
  void update_state_Omega(vehicle_angular_velocity_s ang_vel);
  void update_state_acc(vehicle_acceleration_s acc);
  void update_setpoint(trajectory_setpoint_s sp);
  void run();

  float get_thrust_cmd();
  Vector3f get_torque_cmd();

private:
  // PARAMETERS
  float kx, kv, ka, kR, kOmega;
  float m, g;
  SquareMatrix<float, 3> J;

  // STATE
  Vector3f x, v, Omega;
  Vector3f a, a_filt, a_ext_filt;
  Vector3f tau_bz_filt;
  Dcmf R;

  // SETPOINT
  Vector3f x_ref, v_ref, a_ref, j_ref, s_ref, b1_ref, Omega_ref, alpha_ref;
  float yaw_ref, yaw_vel_ref, yaw_acc_ref;

  // results
  float thrust_cmd;
  Vector3f torque_cmd;

  // filters
  float sample_freq_acc, cutoff_freq_acc;
  math::LowPassFilter2p<Vector3f> lpf_accel;
  math::LowPassFilter2p<Vector3f> lpf_a_ext;
  math::LowPassFilter2p<Vector3f> lpf_tau_bz;
  math::LowPassFilter2p<float> lpf_thrust_cmd;
};
