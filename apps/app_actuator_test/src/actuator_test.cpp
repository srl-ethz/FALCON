#include <chrono>
#include <filesystem>
#include <fstream>
#include <future>
#include <iostream>
#include <thread>
// MAVSDK
#include <mavsdk/mavsdk.h>
#include <mavsdk/plugins/action/action.h>
#include <mavsdk/plugins/gimbal/gimbal.h>
#include <mavsdk/plugins/manual_control/manual_control.h>
#include <mavsdk/plugins/mavlink_passthrough/mavlink_passthrough.h>
#include <mavsdk/plugins/offboard/offboard.h>
#include <mavsdk/plugins/telemetry/telemetry.h>

// helpers
#include "mavsdk_helper.h"

using namespace mavsdk;
using std::chrono::milliseconds;
using std::chrono::seconds;
using std::this_thread::sleep_for;

int main(int argc, char **argv) {
  /* INITIALIZE LOGGING */
  std::ofstream myLog;
  std::string Name = "temp";
  myLog.open("log/THRUST_STAND_" + Name + ".csv");
  std::cout << "Started logging to log/" << Name << ".csv\n";

  /* INITIALIZE MAVSDK */
  if (argc != 2) {
    usage(argv[0]);
    return 1;
  }

  Mavsdk mavsdk;
  ConnectionResult connection_result = mavsdk.add_any_connection(argv[1]);

  if (connection_result != ConnectionResult::Success) {
    std::cerr << "Connection failed: " << connection_result << '\n';
    return 1;
  }

  auto system = get_system(mavsdk);
  if (!system) {
    return 1;
  }

  auto action = Action{system};       // for arming / disarming etc
  auto offboard = Offboard{system};   // for offboard control
  auto telemetry = Telemetry{system}; // for telemetry services
  auto mavlink = MavlinkPassthrough{system};
  // auto gripper = Gimbal{system};

  std::cout << "System is ready\n";

  /* ARM PLANE */
  const auto arm_result = action.arm();

  /* STARTING OFFBOARD */
  // construct actuator control group message right to evade Segmenation faults.

  // std::this_thread::sleep_for(milliseconds(200));

  // for (float u = -1; u <= 1; u += 0.1) {
  //   gripper.set_pitch_and_yaw(u, u);
  //   // send thrust command
  //   std::cout << "now setting input to = " << u << std::endl;

  //   std::this_thread::sleep_for(milliseconds(500));
  // }
  MavlinkPassthrough::CommandLong cmd;
  cmd.command = MAV_CMD_DO_SET_SERVO;
  cmd.target_sysid = mavlink.get_target_sysid();
  cmd.target_compid = mavlink.get_target_compid();
  cmd.param1 = 7;

  for (int u = 1000; u <= 2000; u++) {

    cmd.param2 = u;
    mavlink.send_command_long(cmd);
    std::this_thread::sleep_for(milliseconds(500));
  }

  // std::vector<float> grp;
  // for (int i = 0; i < 8; i++) {
  //   grp.push_back(0.5);
  // }
  // Offboard::ActuatorControl act_cmd{};
  // Offboard::ActuatorControlGroup grp1{};
  // Offboard::ActuatorControlGroup grp2{};
  // grp1.controls = grp;
  // grp2.controls = grp;
  // act_cmd.groups.push_back(grp1);
  // act_cmd.groups.push_back(grp2);

  // std::this_thread::sleep_for(milliseconds(200));

  // offboard.set_actuator_control(act_cmd);
  // auto offboard_result = offboard.start();
  // std::cout << offboard_result << std::endl;

  // for (float u = -1; u <= 1; u += 0.1) {
  //   act_cmd.groups.at(0).controls.at(0) = u;
  //   // send thrust command
  //   std::cout << "now setting input to = " << u << std::endl;

  //   offboard.set_actuator_control(act_cmd);
  //   std::this_thread::sleep_for(milliseconds(500));
  // }

  // stop offboard
  // offboard_result = offboard.stop();

  /* DISARM PLANE */
  std::this_thread::sleep_for(seconds(2));
  const auto disarm_result = action.disarm();
  std::cout << "Disarming Result: " << disarm_result << '\n';
  myLog.close();
  return 0;
}
