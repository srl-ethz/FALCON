#include <chrono>
#include <filesystem>
#include <fstream>
#include <future>
#include <iostream>
#include <thread>
// MAVSDK
#include "mavsdk/plugins/mission/mission.h"
#include <mavsdk/mavsdk.h>
#include <mavsdk/plugins/action/action.h>
#include <mavsdk/plugins/offboard/offboard.h>
#include <mavsdk/plugins/telemetry/telemetry.h>

using namespace mavsdk;
using std::chrono::milliseconds;
using std::chrono::seconds;
using std::this_thread::sleep_for;

// helpers
#include "mavsdk_helper.h"

// constants
const double Ts = 0.005;
const double R = 6378137; // earths Radius in [m]

//  way point before pickup (mission dependent)
const double p1_long = 8.5494224;
const double p1_lat = 47.3982380;

// way point after pickup (mission dependent)
const double p2_long = 8.5429150;
const double p2_lat = 47.3981082;

// obj coordinates (assuming here it is in the middle)
const double obj_long = (8.5429150 - 8.5494224) / 2 + 8.5494224;
const double obj_lat = (47.3981082 - 47.3982380) / 2 + 47.3982380;
const double obj_alt = 0;

// vector from p1 to p2
const double dir_long = obj_long - p1_long;
const double dir_lat = obj_lat - p1_lat;
const double dir_len = std::sqrt(dir_long * dir_long + dir_lat * dir_lat);

double deg_to_m(double deg) {
  double R = 6378000.137; // Radius of earth in m
  return M_PI * R * deg / 180;
}

double m_to_deg(double m) {
  double R = 6378000.137; // Radius of earth in m
  return (m * 180) / (M_PI * R);
}

void gps_to_xyz(double lon, double lat, double alt, double &x, double &y,
                double &z) {

  z = alt - obj_alt;
  x = (((lon - p1_long) * dir_long + (lat - p1_lat) * dir_lat) / dir_len -
       dir_len) *
      R * M_PI / 180;
  y = (((lon - p1_long) * dir_lat - (lat - p1_lat) * dir_long) / dir_len) * R *
      M_PI / 180;
  ;
}

int main(int argc, char **argv) {
  /* INITIALIZE LOGGING */
  std::ofstream myLog;
  auto timenow =
      std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());

  myLog.open("apps/app_mission/log/Flight.csv");
  std::cout << "Started logging to log/"
            << "Flight.csv" << std::endl;

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
  auto mission = Mission{system};

  std::cout << "System is ready\n";
  myLog << "x,y,z\n";

  // starting mission
  action.arm();
  std::this_thread::sleep_for(std::chrono::milliseconds(1500));
  mission.start_mission();

  std::vector<double> dir(2);
  dir.at(0) = deg_to_m(p2_long - p1_long);
  dir.at(1) = deg_to_m(p2_lat - p1_lat);

  double len_sq = dir.at(0) * dir.at(0) + dir.at(1) * dir.at(1);

  while (true) {
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    double x = 0, y = 0, z = 0;
    gps_to_xyz(telemetry.position().longitude_deg,
               telemetry.position().latitude_deg,
               telemetry.position().absolute_altitude_m, x, y, z);
    std::cout << x << "\t\t" << y << "\t\t" << z << std::endl;
  }

  while (true) {

    // delay
    std::this_thread::sleep_for(std::chrono::milliseconds(int(Ts * 1000)));

    // current position
    std::vector<double> pos(2);
    pos.at(0) = deg_to_m(telemetry.position().longitude_deg - p1_long);
    pos.at(1) = deg_to_m(telemetry.position().latitude_deg - p1_lat);

    // progress along mission route
    double prog = (pos.at(0) * dir.at(0) + pos.at(1) * dir.at(1)) / len_sq;

    // deviation
    std::vector<double> abw(2);
    abw.at(0) = pos.at(0) - prog * dir.at(0);
    abw.at(1) = pos.at(1) - prog * dir.at(1);
    double abw_len = std::sqrt(abw.at(0) * abw.at(0) + abw.at(1) * abw.at(1));

    // check if offboard should begin
    std::cout << prog << std::endl;
    if (abw_len < 1 && abw_len > -1 && prog < 0.8 && prog > 0.2) {
      break;
    }
  }

  double yaw = telemetry.attitude_euler().yaw_deg;
  std::cout << "starting offboard" << std::endl;

  // send msg before startring offbaord
  Offboard::Attitude msg;
  msg.pitch_deg = 0;
  msg.roll_deg = 0;
  msg.thrust_value = 0.75;
  msg.yaw_deg = yaw;
  offboard.set_attitude(msg);

  // starting offboard;
  offboard.start();

  // control parameters
  const double yaw_P = 10;
  const double yaw_I = 5;

  // I error
  double abw_error_sum = 0;

  while (true) {
    // delay
    std::this_thread::sleep_for(std::chrono::milliseconds(int(Ts * 1000)));
    // current position
    std::vector<double> pos(2);
    pos.at(0) = deg_to_m(telemetry.position().longitude_deg - p1_long);
    pos.at(1) = deg_to_m(telemetry.position().latitude_deg - p1_lat);

    // progress along mission route
    double prog = (pos.at(0) * dir.at(0) + pos.at(1) * dir.at(1)) / len_sq;

    // deviation
    std::vector<double> abw(2);
    abw.at(0) = pos.at(0) - prog * dir.at(0);
    abw.at(1) = pos.at(1) - prog * dir.at(1);
    double abw_len = std::sqrt(abw.at(0) * abw.at(0) + abw.at(1) * abw.at(1));

    // ctrl loop.
    abw_error_sum += abw_len * Ts;
    msg.yaw_deg = yaw_P * abw_len + yaw_I * abw_error_sum;

    // Send msg
    offboard.set_attitude(msg);
    std::cout << msg.yaw_deg << std::endl;
    std::cout << prog << "\t" << abw_len << std::endl;
    // check if offboard should begin
    if (!(abw_len < 10 && abw_len > -10 && prog < 0.8 && prog > 0.2)) {
      break;
    }
  }
  std::cout << "stopping offboard" << std::endl;
  offboard.stop();
  mission.start_mission();

  return 0;
}
