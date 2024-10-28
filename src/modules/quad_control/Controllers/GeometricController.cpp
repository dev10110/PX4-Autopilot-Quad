
// Devansh Agrawal
// April 2023

#include "GeometricController.hpp"
#include "DiffFlat.hpp"

using namespace matrix;

GeometricController::GeometricController() {
  set_gains();
  reset_integral();
}

void GeometricController::set_gains(float _kx, float _kv, float _ki, float _kR,
                                    float _kOmega, float _m, float _Jxx,
                                    float _Jyy, float _Jzz) {

  g = 9.81f;

  kx = _kx;         // 1.0f;
  kv = _kv;         // 2.0f;
  ki = _ki;         // 0.05f;
  kR = _kR;         // 0.35f;
  kOmega = _kOmega; // 0.15f;

  m = _m; // 1.5f;

  J.setZero();
  J(0, 0) = _Jxx; // 0.005;
  J(1, 1) = _Jyy; // 0.005;
  J(2, 2) = _Jzz; // 0.009;
}

void GeometricController::enable_logging(bool enable)
{
  enable_logging_ = enable;
  return;
}

void GeometricController::reset_integral() { ei.setZero(); }

void GeometricController::update_state_pos(vehicle_local_position_s pos) {

  // grab pos (if valid)
  x(0) = pos.xy_valid ? pos.x : NAN;
  x(1) = pos.xy_valid ? pos.y : NAN;
  x(2) = pos.z_valid ? pos.z : NAN;

  // grab vel (if valid)
  v(0) = pos.v_xy_valid ? pos.vx : NAN;
  v(1) = pos.v_xy_valid ? pos.vy : NAN;
  v(2) = pos.v_z_valid ? pos.vz : NAN;
}

void GeometricController::update_state_attitude(vehicle_attitude_s att) {
  // grab rotation from body frame to earth frame
  R = Dcmf(Quatf(att.q));
}

void GeometricController::update_state_Omega(
    vehicle_angular_velocity_s ang_vel) {
  for (size_t i = 0; i < 3; i++) {
    Omega(i) = ang_vel.xyz[i];
  }
}

void GeometricController::update_setpoint(trajectory_setpoint_s sp) {

  // these are in FRD frame
  for (size_t i = 0; i < 3; i++) {
    x_ref(i) = sp.position[i];
    v_ref(i) = sp.velocity[i];
    a_ref(i) = sp.acceleration[i];
    j_ref(i) = sp.jerk[i];
    s_ref(i) = 0.0f;
  }

  yaw_ref = -sp.yaw;
  yaw_vel_ref = -sp.yawspeed;
  yaw_acc_ref = 0.0f;

  // do the flat state to trajectory setpoint conversion
  flat_state_FRD_to_quad_state_FRD(b1_ref, Omega_ref, alpha_ref, a_ref, j_ref,
                                   s_ref, yaw_ref, yaw_vel_ref, yaw_acc_ref);

}

// Run the controller
void GeometricController::run() {

  // assuming FRD frame for everything

  // define constants
  const Vector3f e3(0, 0, 1);

  // define the position loop
  Vector3f ex = (x - x_ref).elwise_zero_if_nan();
  Vector3f ev = (v - v_ref).elwise_zero_if_nan();

  // prevent too large ex:
  if (ex.norm() > 2) {
    ex = 2.0f * ex / ex.norm();
  }

  // prevent too large ev:
  if (ev.norm() > 5) {
    ev = 5.0f * ev / ev.norm();
  }

  // increment the integral error
  ei += ex;
  // prevent windup
  for (size_t i = 0; i < 3; i++) {
    ei(i) = (ei(i) > 2.0f / ki) ? 2.0f / ki : ei(i);
    ei(i) = (ei(i) < -2.0f / ki) ? -2.0f / ki : ei(i);
  }
  ei = ei.elwise_zero_if_nan();

  // calculate thrust
  Vector3f thrust = -kx * ex - kv * ev - ki * ei - m * g * e3 + m * (a_ref.zero_if_nan());

  // determine desired rotation matrix
  Vector3f b3d = -thrust.unit();
  Vector3f b2d = (b3d.cross(b1_ref)).unit();
  Vector3f b1d = (b2d.cross(b3d)).unit();

  // construct desired rotation matrix
  Dcmf Rd;
  for (size_t i = 0; i < 3; i++) {
    Rd(i, 0) = b1d(i);
    Rd(i, 1) = b2d(i);
    Rd(i, 2) = b3d(i);
  }

  Vector3f eR = 0.5f * (Dcmf(Rd.T() * R - R.T() * Rd)).vee();
  Vector3f eOmega = Omega - R.T() * Rd * Omega_ref;

  thrust_cmd = -thrust.dot(R * e3);

  torque_cmd =
      -kR * eR - kOmega * eOmega + Omega.cross(J * Omega) -
      J * (Omega.hat() * R.T() * Rd * Omega_ref - R.T() * Rd * alpha_ref);

  // LOG VARIABLES
  if (enable_logging_) {
    for (uint8_t i=0; i<3; ++i){
      log_msg.x_ref[i] = x_ref(i);
      log_msg.v_ref[i] = v_ref(i);
      log_msg.a_ref[i] = a_ref(i);
      log_msg.j_ref[i] = j_ref(i);
      log_msg.s_ref[i] = s_ref(i);
      log_msg.b1_ref[i] = b1_ref(i);
      log_msg.omega_ref[i] = Omega_ref(i);
      log_msg.alpha_ref[i] = alpha_ref(i);

      log_msg.x[i] = x(i);
      log_msg.v[i] = v(i);
      log_msg.omega[i] = Omega(i);
      log_msg.b1[i] = R(i, 0);
      log_msg.b2[i] = R(i, 1);
      log_msg.b3[i] = R(i, 2);
      log_msg.ex[i] = ex(i);
      log_msg.ev[i] = ev(i);
      log_msg.ei[i] = ei(i);
      log_msg.thrust[i] = thrust(i);
      log_msg.b1d[i] = b1d(i);
      log_msg.b2d[i] = b2d(i);
      log_msg.b3d[i] = b3d(i);
      log_msg.er[i] = eR(i);
      log_msg.eomega[i] = eOmega(i);
      log_msg.torque_cmd[i] = torque_cmd(i);
    }

    log_msg.yaw_ref = yaw_ref;
    log_msg.yaw_vel_ref = yaw_vel_ref;
    log_msg.yaw_acc_ref = yaw_acc_ref;
    log_msg.thrust_cmd = thrust_cmd;
  }

  return;
}

float GeometricController::get_thrust_cmd() { return thrust_cmd; }

Vector3f GeometricController::get_torque_cmd() { return torque_cmd; }

quad_control_log_s GeometricController::get_log_message() { return log_msg; }
