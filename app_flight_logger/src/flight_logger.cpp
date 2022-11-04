#include <chrono>
#include <thread>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <future>
#include <ctime>
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

/* PARAMETERS */
int freq = 100; // logging frequency

/* GLOBAL VARIABLES */
Telemetry::ActuatorControlTarget act_targ;
Telemetry::ActuatorOutputStatus act_out;
float airspeed;

/* CALLBACK FUNCTIONS */
std::function<void(Telemetry::ActuatorControlTarget)> actuatorControlTargetCallback =
    [](Telemetry::ActuatorControlTarget msg)
{
  act_targ = msg;
};

std::function<void(Telemetry::ActuatorOutputStatus)> actuatorOutputStatusCallback =
    [](Telemetry::ActuatorOutputStatus msg)
{
  act_out = msg;
};

std::function<void(const mavlink_message_t &)> airSpeedCallback =
    [](const mavlink_message_t &msg)
{
  airspeed = mavlink_msg_vfr_hud_get_airspeed(&msg);
};

// Commmand to start: /build/app_flight_logger/flight_logger serial:///dev/ttyUSB0:921600
int main(int argc, char **argv)
{
  /* INITIALIZE LOGGING */
  std::ofstream myLog;
  auto timenow = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
  std::string Date = ctime(&timenow);
  std::replace(Date.begin(), Date.end(), ' ', '_');
  std::replace(Date.begin(), Date.end(), ':', '_');
  Date.pop_back();
  myLog.open("app_flight_logger/log/Flight_" + Date + ".csv");
  std::cout << "Started logging to log/"
            << "Flight_" << Date << ".csv" << std::endl;
  std::cout << "closed log" << std::endl;
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

  auto telemetry = Telemetry{system}; // for telemetry services
  auto mavlink = MavlinkPassthrough{system};
  std::cout << "System is ready\n";

  std::this_thread::sleep_for(milliseconds(500));

  /* START LOGGING */
  float t_s = 1 / float(freq);

  /* SUBSCRIBER TO CALLBACK */
  telemetry.subscribe_actuator_control_target(actuatorControlTargetCallback);
  telemetry.subscribe_actuator_output_status(actuatorOutputStatusCallback);
  mavlink.subscribe_message_async(74, airSpeedCallback);

  std::vector<float> act_targs{0, 0, 0, 0, 0, 0, 0, 0};
  for (float t = 0;; t += t_s)
  {
    // std::cout << "TARGET: group: " << act_targ.group << " size: " << act_targ.controls.size() << std::endl;
    if (act_targ.controls.size() == 8)
    {
      for (int i = 0; i < 8; i++)
      {
        act_targs = act_targ.controls;
      }
      act_targs = {0, 0, 0, 0, 0, 0, 0, 0};
    }
    // std::cout << "airspeed: " << airspeed << std::endl;
    //<< act_targ.controls.at(1) << "\t" << act_targ.controls.at(2) << "\t" << act_targ.controls.at(3) << std::endl;
    //   write to log file
    myLog << t << ","
          << telemetry.position_velocity_ned().position.north_m << ","
          << telemetry.position_velocity_ned().position.east_m << ","
          << telemetry.position_velocity_ned().position.down_m << ","
          << telemetry.position_velocity_ned().velocity.north_m_s << ","
          << telemetry.position_velocity_ned().velocity.east_m_s << ","
          << telemetry.position_velocity_ned().velocity.down_m_s << ","
          << telemetry.attitude_euler().roll_deg << ","
          << telemetry.attitude_euler().pitch_deg << ","
          << telemetry.attitude_euler().yaw_deg << ","
          << telemetry.attitude_angular_velocity_body().roll_rad_s << ","
          << telemetry.attitude_angular_velocity_body().pitch_rad_s << ","
          << telemetry.attitude_angular_velocity_body().yaw_rad_s << ","
          << act_targs.at(0) << ","
          << act_targs.at(1) << ","
          << act_targs.at(2) << ","
          << act_targs.at(3) << ","
          << act_targs.at(4) << ","
          << act_targs.at(5) << ","
          << act_targs.at(6) << ","
          << act_targs.at(7) << ","
          << airspeed << "\n";

    //  ensure logging frequency
    std::this_thread::sleep_for(milliseconds(int(1000 / freq)));
  }

  /* CLOSE LOG */
  myLog.close();
  return 0;
}
