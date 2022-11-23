#include <chrono>
#include <ctime>
#include <filesystem>
#include <fstream>
#include <future>
#include <iostream>
#include <thread>
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

// Commmand to start: /build/app_flight_logger/flight_logger
// serial:///dev/ttyUSB0:921600
int main(int argc, char **argv) {
  /* INITIALIZE LOGGING */
  std::ofstream myLog;
  auto timenow =
      std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
  std::string Date = ctime(&timenow);
  std::replace(Date.begin(), Date.end(), ' ', '_');
  std::replace(Date.begin(), Date.end(), ':', '_');
  Date.pop_back();
  myLog.open("apps/app_flight_logger/log/Flight_" + Date + ".csv");
  std::cout << "Started logging to log/"
            << "Flight_" << Date << ".csv" << std::endl;

  myLog << "t,north,east,down,vel,roll,pitch,speed,throttle\n";

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

  auto telemetry = Telemetry{system}; // for telemetry services
  std::cout << "System is ready\n";
  double t_s = 1 / float(freq);

  for (double t = 0;; t += t_s) {
    std::this_thread::sleep_for(milliseconds(int(1000 * t_s)));

    /* START LOGGING */
    double velocity_abs =
        std::sqrt(telemetry.position_velocity_ned().velocity.north_m_s *
                      telemetry.position_velocity_ned().velocity.north_m_s +
                  telemetry.position_velocity_ned().velocity.east_m_s *
                      telemetry.position_velocity_ned().velocity.east_m_s);

    myLog << t << "," << telemetry.position_velocity_ned().position.north_m
          << "," << telemetry.position_velocity_ned().position.east_m << ","
          << telemetry.position_velocity_ned().position.down_m << ","
          << velocity_abs << "," << telemetry.attitude_euler().roll_deg << ","
          << telemetry.attitude_euler().pitch_deg << ","
          << telemetry.fixedwing_metrics().airspeed_m_s << ","
          << telemetry.fixedwing_metrics().throttle_percentage << "\n";

    //  ensure logging frequency
    std::this_thread::sleep_for(milliseconds(int(1000 / freq)));
    std::cout << "logging time: [" << t << "]\t throttle: ["
              << telemetry.fixedwing_metrics().throttle_percentage << "]"
              << std::endl;
  }

  /* CLOSE LOG */
  myLog.close();
  return 0;
}
