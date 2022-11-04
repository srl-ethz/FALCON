#include <chrono>
#include <thread>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <future>
// MAVSDK
#include <mavsdk/mavsdk.h>
#include <mavsdk/plugins/action/action.h>
#include <mavsdk/plugins/mavlink_passthrough/mavlink_passthrough.h>
#include <mavsdk/plugins/offboard/offboard.h>
#include <mavsdk/plugins/telemetry/telemetry.h>

// helpers
#include "mavsdk_helper.h"

using namespace mavsdk;
using std::chrono::milliseconds;
using std::chrono::seconds;
using std::this_thread::sleep_for;

int main(int argc, char **argv)
{
  /* INITIALIZE LOGGING */
  std::ofstream myLog;
  std::string Name = "temp";
  myLog.open("log/THRUST_STAND_" + Name + ".csv");
  std::cout << "Started logging to log/" << Name << ".csv\n";

  /* INITIALIZE MAVSDK */
  if (argc != 2)
  {
    usage(argv[0]);
    return 1;
  }

  Mavsdk mavsdk;
  ConnectionResult connection_result = mavsdk.add_any_connection(argv[1]);

  if (connection_result != ConnectionResult::Success)
  {
    std::cerr << "Connection failed: " << connection_result << '\n';
    return 1;
  }

  auto system = get_system(mavsdk);
  if (!system)
  {
    return 1;
  }

  auto action = Action{system};              // for arming / disarming etc
  auto offboard = Offboard{system};          // for offboard control
  auto telemetry = Telemetry{system};        // for telemetry services
  auto mavlink = MavlinkPassthrough{system}; // for mavlink passtrough

  std::cout << "System is ready\n";

  /* ARM PLANE */
  const auto arm_result = action.arm();

  /* MAVLINK PASSTROUGH */
  mavsdk::MavlinkPassthrough::CommandLong cmd;
  cmd.command = MAV_CMD_DO_SET_ACTUATOR;
  cmd.target_sysid = mavlink.get_target_sysid();
  cmd.param1 = 0;
  cmd.param2 = 0.5;
  cmd.param3 = 0;
  cmd.param4 = 0;
  cmd.param5 = 0;
  cmd.param6 = 0;
  cmd.param7 = 0;

  const mavsdk::MavlinkPassthrough::CommandLong conscmd = cmd;

  for (int i = 0; i < 10; i++)
  {
    mavlink.send_command_long(conscmd);
    sleep_for(milliseconds(500));
  }
  // /* ATTITUDE OFFBOARD */
  // std::cout << "Starting Offboard attitude control\n";

  // // Send it once before starting offboard, otherwise it will be rejected.
  // Offboard::Attitude att_msg{};
  // att_msg.roll_deg = 0;

  // Offboard::Result offboard_result = offboard.start();
  // if (offboard_result != Offboard::Result::Success)
  // {
  //   std::cerr << "Offboard start failed: " << offboard_result << '\n';
  //   return false;
  // }
  // std::cout << "Offboard started\n";

  // std::cout << "Roll 30 degrees to the right\n";
  // offboard.set_attitude(att_msg);
  // sleep_for(seconds(10));

  // offboard_result = offboard.stop();
  // if (offboard_result != Offboard::Result::Success)
  // {
  //   std::cerr << "Offboard stop failed: " << offboard_result << '\n';
  //   return false;
  // }
  // std::cout << "Offboard stopped\n";

  /* STARTING OFFBOARD */
  // construct actuator control group message right to evade Segmenation faults.
  std::vector<float>
      grp;
  for (int i = 0; i < 8; i++)
  {
    grp.push_back(0);
  }
  Offboard::ActuatorControl act_cmd{};
  Offboard::ActuatorControlGroup grp1{};
  Offboard::ActuatorControlGroup grp2{};
  grp1.controls = grp;
  grp2.controls = grp;
  act_cmd.groups.push_back(grp1);
  act_cmd.groups.push_back(grp2);
  // Offboard::ActuatorControlGroup grp2{};
  std::cout << "starting offboard" << std::endl;
  offboard.set_actuator_control(act_cmd);
  std::cout << "started offboard" << std::endl;
  // start offboard
  Offboard::Result offboard_result = offboard.start();
  std::cout << offboard_result << std::endl;

  // act_cmd.groups.push_back(grp2);
  std::this_thread::sleep_for(milliseconds(500));

  // std::this_thread::sleep_for(milliseconds(2500));

  /* GO THROUGHT ALL ANGLES FROM -1 TO 1 */
  // for (float u = 0; u <= 1; u += 0.05)
  // {
  //   // send thrust command
  //   std::cout << "now setting input to = " << u << std::endl;
  //   act_cmd.groups.at(1).controls.at(0) = u;
  //   std::cout << act_cmd.groups.at(0).controls.at(1) << std::endl;

  //   offboard.set_actuator_control(act_cmd);

  //   // wait for thrust to settle
  //   std::this_thread::sleep_for(milliseconds(500));
  // }

  /* GO THROUGHT ALL THROTTLES FROM 0 TO 1 */
  // for (float throttle = 0.05; throttle <= 1.05; throttle += 0.05)
  // {
  //   // send thrust command
  //   std::cout << "now setting throttle = " << throttle << std::endl;
  //   act_cmd.groups.at(0).controls.at(3) = throttle;
  //   offboard.set_actuator_control(act_cmd);

  //   // wait for thrust to settle
  //   std::this_thread::sleep_for(milliseconds(10000));
  // }

  // stop offboard
  offboard_result = offboard.stop();

  /* DISARM PLANE */
  std::this_thread::sleep_for(seconds(2));
  const auto disarm_result = action.disarm();
  std::cout << "Disarming Result: " << disarm_result << '\n';
  myLog.close();
  return 0;
}
