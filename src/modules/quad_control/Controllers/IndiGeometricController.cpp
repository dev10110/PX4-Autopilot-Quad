
// Devansh Agrawal 
// April 2023

#include "IndiGeometricController.hpp"
#include "DiffFlat.hpp"


using namespace matrix;


IndiGeometricController::IndiGeometricController(){

  set_gains();
  
}


void IndiGeometricController::set_gains(){

  kx = 1.0f;
  kv = 2.0f;
  ka = 0.5f;  
  kR = 0.35f;
  kOmega = 0.15f;

  m = 0.9f; // THE QUAD ACTUALLY WEIGHS 0.8f; this should should up as a wrong altitude, Geometric controller cant fix it, but the IndiGeometric should;
  g = 9.81f;
  J.setZero();
  J(0,0) = 0.005;
  J(1,1) = 0.005;
  J(2,2) = 0.009;
  

  // update filter objects
  sample_freq_acc = 250.0; // Hz
  cutoff_freq_acc = 5.0;  // Hz
  lpf_accel.set_cutoff_frequency(sample_freq_acc, cutoff_freq_acc);
  lpf_tau_bz.set_cutoff_frequency(sample_freq_acc, cutoff_freq_acc);
  lpf_a_ext.set_cutoff_frequency(sample_freq_acc, cutoff_freq_acc);
  lpf_thrust_cmd.set_cutoff_frequency(sample_freq_acc, cutoff_freq_acc);

  // initialize the a_ext to be gravity
  lpf_a_ext.reset(Vector3f(0,0,g));
  
}

void IndiGeometricController::update_state_acc(vehicle_acceleration_s acc){


  for (size_t i=0; i<3; i++){
    a(i) = acc.xyz[i];
  }
  a += Vector3f(0,0,g);
  
  //a_filt = lpf_accel.apply(a);

}


void IndiGeometricController::update_state_pos(vehicle_local_position_s pos)
{
  // grab pos and vel
  x(0) = pos.x;
  x(1) = pos.y;
  x(2) = pos.z;

  v(0) = pos.vx;
  v(1) = pos.vy;
  v(2) = pos.vz;

  //a(0) = pos.ax;
  //a(1) = pos.ay;
  //a(2) = pos.az;

  //// filter the acceleration
  //a_filt = lpf_accel.apply(a);

  //printf("a: ");
  //a.print();
  //printf("a_filt: ");
  //a_filt.print();
}


void IndiGeometricController::update_state_attitude(vehicle_attitude_s att)
{
  // grab rotation from body frame to earth frame
  R = Dcmf(Quatf(att.q));
}

void IndiGeometricController::update_state_Omega(vehicle_angular_velocity_s ang_vel)
{
  for (size_t i=0; i<3; i++){
    Omega(i) = ang_vel.xyz[i];
  }
}

void IndiGeometricController::update_setpoint(trajectory_setpoint_s sp) {


  // these are in FRD frame
  for (size_t i=0; i<3; i++){
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
  flat_state_FRD_to_quad_state_FRD(b1_ref, Omega_ref, alpha_ref, a_ref, j_ref, s_ref,  yaw_ref, yaw_vel_ref, yaw_acc_ref);



  // //TODO(dev): update
  // yaw_ref = sp.yaw; // desired yaw
  // b1_ref(0) = std::cosf(yaw_ref);
  // b1_ref(1) = std::sinf(yaw_ref);
  // b1_ref(2) = 0.0;
  // 
  // //TODO(dev): update
  // Omega_ref.setZero();
  // alpha_ref.setZero();

}


// Run the controller
// control input is stored in omega
void IndiGeometricController::run(){

  // assuming FRD frame for everything 

  const Vector3f e3 (0,0,1);

  Vector3f ex = x - x_ref;
  Vector3f ev = v - v_ref;
  ex.print();

  // compute accel command
  Vector3f a_cmd = (-kx * ex - kv * ev) / m + a_ref + 0*ka * (a_ref - a_filt);
  
  // compute thrust command
  //Vector3f tau_bz_cmd = tau_bz_filt + a_cmd  - a_filt;
  //Vector3f tau_bz_cmd = Vector3f(0,0,-g) + a_cmd;
  Vector3f tau_bz_cmd = a_cmd - a_ext_filt;
  //Vector3f tau_bz_cmd = a_cmd + 1.0f * (tau_bz_filt - a_filt);
  thrust_cmd = m * (tau_bz_cmd).dot(-R * e3);
  //thrust_cmd = (thrust_cmd < 0) ? 0 : thrust_cmd;
  printf("thrust_cmd: %f\n", (double)thrust_cmd);

  Vector3f b3d = -tau_bz_cmd.unit();
  Vector3f b2d = (b3d.cross(b1_ref)).unit();
  Vector3f b1d = (b2d.cross(b3d)).unit();

  // construct desired rotation matrix
  Dcmf Rd;
  for (size_t i=0; i < 3 ; i++){
    Rd(i, 0) = b1d(i);
    Rd(i, 1) = b2d(i);
    Rd(i, 2) = b3d(i);
  }

  Vector3f eR = 0.5f * (Dcmf(Rd.T() * R - R.T() * Rd)).vee();
  Vector3f eOmega = Omega - R.T() * Rd * Omega_ref;

  torque_cmd = -kR * eR - kOmega * eOmega + Omega.cross(J * Omega) - J * (Omega.hat() * R.T() * Rd * Omega_ref - R.T() * Rd * alpha_ref);


  // update the tau_bz_filter
  Vector3f tau_bz = (-thrust_cmd / m) * R * e3;
  tau_bz_filt = lpf_tau_bz.apply(tau_bz);


  // estimate external accel:
  Vector3f a_ext = a - (-thrust_cmd * R * e3);
  a_ext_filt = lpf_a_ext.apply(a_ext);

  //printf("tau_bz_cmd:  ");
  //tau_bz_cmd.print();
  //printf("tau_bz_filt: ");
  //tau_bz_filt.print();
  //printf("a:           ");
  //a.print();
  //printf("a_ext_filt:  ");
  //a_ext_filt.print();

  return;

}

float IndiGeometricController::get_thrust_cmd()
{
  return thrust_cmd;
}

Vector3f IndiGeometricController::get_torque_cmd()
{
  return torque_cmd;
}



