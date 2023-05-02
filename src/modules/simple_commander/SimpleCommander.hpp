#pragma once

#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>

// Publications
#include <uORB/Publication.hpp>
#include <uORB/topics/actuator_armed.h>
#include <uORB/topics/commander_status.h>
#include <uORB/topics/parameter_res.h>
#include <uORB/topics/trajectory_setpoint.h>
#include <uORB/topics/vehicle_control_mode.h>
#include <uORB/topics/vehicle_status.h>

// Subscriptions
#include <uORB/Subscription.hpp>
#include <uORB/SubscriptionInterval.hpp>
#include <uORB/topics/commander_set_state.h>
#include <uORB/topics/parameter_req.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/trajectory_setpoint.h>
#include <uORB/topics/vehicle_local_position.h>

using namespace time_literals;

class SimpleCommander : public ModuleBase<SimpleCommander>,
                        public ModuleParams {
public:
  SimpleCommander();
  ~SimpleCommander();

  static int task_spawn(int argc, char *argv[]);
  static SimpleCommander *instantiate(int argc, char *argv[]);
  static int custom_command(int argc, char *argv[]);
  static int print_usage(const char *reason = nullptr);

  void run() override;

  int print_status() override;

private:
  enum class VehicleState {
    DISARMED = 0, // disarmed state
    ARMED,    // armed state (i.e., received arming command and passed preflight
              // checks)
    OFFBOARD, // armed and receiving offboard messages
    LAND      // received land command or offboard timeout
  };

  bool check_has_landed();

  bool preflight_check_ekf();
  bool preflight_check();

  bool handle_command_arm();
  bool handle_command_disarm();
  bool handle_command_offboard();
  bool handle_command_land();
  void handle_parameter_req();

  bool set_state(VehicleState new_state);
  void run_state_machine();

  void publish_status();

  // Publishers
  uORB::Publication<trajectory_setpoint_s> _trajectory_setpoint_pub{
      ORB_ID(trajectory_setpoint)};
  uORB::Publication<vehicle_control_mode_s> _vehicle_control_mode_pub{
      ORB_ID(vehicle_control_mode)};
  uORB::Publication<actuator_armed_s> _actuator_armed_pub{
      ORB_ID(actuator_armed)};
  uORB::Publication<vehicle_status_s> _vehicle_status_pub{
      ORB_ID(vehicle_status)};
  uORB::Publication<commander_status_s> _commander_status_pub{
      ORB_ID(commander_status)};
  uORB::Publication<parameter_res_s> _parameter_res_pub{ORB_ID(parameter_res)};

  // Subscribers
  uORB::SubscriptionInterval _parameter_update_sub{ORB_ID(parameter_update),
                                                   1_s};
  uORB::Subscription _vehicle_local_position_sub{
      ORB_ID(vehicle_local_position)};
  uORB::Subscription _trajectory_setpoint_sub{ORB_ID(trajectory_setpoint)};
  uORB::Subscription _commander_set_state_sub{ORB_ID(commander_set_state)};
  uORB::Subscription _parameter_req_sub{ORB_ID(parameter_req)};

  VehicleState _state = VehicleState::DISARMED;
  hrt_abstime _boot_timestamp{0};
  hrt_abstime _last_preflight_check{0};
  hrt_abstime _last_arm_status_pub{0};
  hrt_abstime _last_timestamp_offboard{0};
  hrt_abstime _last_land_cmd_started{0};
};
