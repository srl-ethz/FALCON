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

  /* STARTING OFFBOARD */
  // construct actuator control group message right to evade Segmenation faults.
  Offboard::ActuatorControlGroup grp;
  for (int i = 0; i < 8; i++)
  {
    grp.controls.push_back(0.0);
  }
  Offboard::ActuatorControl act_cmd{};
  act_cmd.groups.push_back(grp);
  act_cmd.groups.push_back(grp);
  offboard.set_actuator_control(act_cmd);
  // start offboard
  Offboard::Result offboard_result = offboard.start();

  /* GO THROUGHT ALL THROTTLES FROM 0 TO 1 */
  for (float throttle = 0.15; throttle <= 1.05; throttle += 0.05)
  {
    // send thrust command
    std::cout << "now setting throttle = " << throttle << std::endl;
    act_cmd.groups.at(0).controls.at(3) = throttle;
    offboard.set_actuator_control(act_cmd);

    // wait for thrust to settle
    std::this_thread::sleep_for(milliseconds(2500));
  }

  /* DISARM PLANE */
  std::this_thread::sleep_for(seconds(5));
  const auto disarm_result = action.disarm();
  std::cout << "Disarming Result: " << disarm_result << '\n';
  myLog.close();
  return 0;
}
