
// Devansh Agrawal
// April 2023

#include "GeometricController.hpp"
#include "DiffFlat.hpp"

using namespace matrix;

GeometricController::GeometricController() { set_gains(); }

void GeometricController::set_gains() {

  kx = 1.0f;
  kv = 2.0f;
  kR = 0.35f;
  kOmega = 0.15f;

  m = 0.9f; // THE QUAD ACTUALLY WEIGHS 0.8f; this should should up as a wrong
            // altitude, Geometric controller cant fix it, but the IndiGeometric
            // should;
  g = 9.81f;
  J.setZero();
  J(0, 0) = 0.005;
  J(1, 1) = 0.005;
  J(2, 2) = 0.009;
}

void GeometricController::update_state_pos(vehicle_local_position_s pos) {
  // grab pos and vel
  x(0) = pos.x;
  x(1) = pos.y;
  x(2) = pos.z;

  v(0) = pos.vx;
  v(1) = pos.vy;
  v(2) = pos.vz;
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

  yaw_ref = sp.yaw;
  yaw_vel_ref = sp.yawspeed;
  yaw_acc_ref = 0.0f;

  // do the flat state to trajectory setpoint conversion
  // flat_state_FRD_to_quad_state_FRD(b1_ref, Omega_ref, alpha_ref, a_ref,
  // j_ref, s_ref,  yaw_ref, yaw_vel_ref, yaw_acc_ref);

  // TODO(dev): update
  yaw_ref = sp.yaw; // desired yaw
  b1_ref(0) = std::cos(yaw_ref);
  b1_ref(1) = std::sin(yaw_ref);
  b1_ref(2) = 0.0;

  // TODO(dev): update
  Omega_ref.setZero();
  alpha_ref.setZero();
}

// Run the controller
// control input is stored in omega
void GeometricController::run() {

  // assuming FRD frame for everything

  const Vector3f e3(0, 0, 1);

  Vector3f ex = x - x_ref;
  Vector3f ev = v - v_ref;

  ex.print();

  Vector3f thrust = -kx * ex - kv * ev - m * g * e3 + m * a_ref;

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

  return;
}

float GeometricController::get_thrust_cmd() { return thrust_cmd; }

Vector3f GeometricController::get_torque_cmd() { return torque_cmd; }
