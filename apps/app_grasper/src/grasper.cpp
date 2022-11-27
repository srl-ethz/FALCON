#include <chrono>
#include <filesystem>
#include <fstream>
#include <future>
#include <iostream>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

// MAVSDK
#include "mavsdk/plugins/mission/mission.h"
#include "mavsdk/plugins/mission_raw/mission_raw.h"
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

// TODO Put into YAML file (dont care for now)
//   way point before pickup (mission dependent)
const double p1_long = 8.387438115684935;
const double p1_lat = 47.42465027591855;

// way point after pickup (mission dependent)
const double p2_long = 8.387276484315064;
const double p2_lat = 47.42472872408145;

// way point after pickup (mission dependent)
const double obj_long = 8.3873573;
const double obj_lat = 47.4246895;
const double obj_alt = 1.0;

const double grasp_length = 2; // length from beginning of MPC control to object
                               // (or from object to end of MPC control)

// normalized vector from p1 to p2
const double dir_len = std::sqrt((p2_long - p1_long) * (p2_long - p1_long) +
                                 (p2_lat - p1_lat) * (p2_lat - p1_lat));
const double dir_long = (p2_long - p1_long) / dir_len;
const double dir_lat = (p2_lat - p1_lat) / dir_len;

// control parameters
const double yaw_P = 1.0;
const double yaw_I = 0.2;

// Variables that have to be exactly the same as in traj_gen.cpp:
const double time_horizon = 0.7;
const double dt = 0.02;
const int iterations = int(time_horizon / dt);

// other constants
const double R = 6378137; // earths Radius in [m]
const int n_initial_conditions = 6;
const int n_ctrl_vars = 3;
const int n_z = 13;  // number of possible initial conditions for x
const int n_vx = 14; // number of possible initial conditions for vs
const std::vector<double> vx0 =
    std::vector<double>(n_vx); // values of possible initial conditions
const std::vector<double> x0 =
    std::vector<double>(n_z); // values of possible initial conditions

// values that are accessed by multiple threads and need to be guarded by mutex
std::vector<double> initial_condition(n_initial_conditions);

std::vector<std::vector<std::vector<std::vector<double>>>>
    u_opt(n_z, std::vector<std::vector<std::vector<double>>>(
                   n_vx, std::vector<std::vector<double>>(
                             n_ctrl_vars, std::vector<double>(iterations))));
std::vector<double> possible_z{0.7,  0.75, 0.8,  0.85, 0.9,  0.95, 1.0,
                               1.05, 1.1,  1.15, 1.2,  1.25, 1.3};
std::vector<double> possible_vx{9,     9.25, 9.5,   9.75, 10.0,  10.25, 10.5,
                                10.75, 11.0, 11.25, 11.5, 11.75, 12.0,  12.25};
// helper function from gps coordinates to meters
double deg_to_m(double deg) {
  double R = 6378000.137; // Radius of earth in m
  return M_PI * R * deg / 180;
}

void get_coords(Telemetry &telemetry, double &x, double &z, double &vx,
                double &xz) {}

int main(int argc, char **argv) {
  /* READ OPTIMAL CONTROLS FROM PRECALCULATED FILE*/
  std::ifstream reader;
  reader.open("apps/app_grasper/src/u_opt.csv");
  std::cout << "initialized reader" << std::endl;
  std::cout << "vector size: [" << u_opt.size() << ", " << u_opt.at(0).size()
            << ", " << u_opt.at(0).at(0).size() << ", "
            << u_opt.at(0).at(0).at(0).size() << "]" << std::endl;
  // Read string once to get rid of title line
  std::string line;
  std::getline(reader, line);

  // read in all precomputed solutions
  // (!) verify that the traj_gen and grasper constants align (!)
  for (int z = 0; z < n_z; z++) {
    for (int vx = 0; vx < n_vx; vx++) {
      for (int i = 0; i < iterations; i++) {
        // std::cout << "[" << z << "," << vx << "," << i << "]" << std::endl;

        std::cout << "[" << possible_z.at(z) << ", " << possible_vx.at(vx)
                  << ", " << i << "]" << std::endl;
        std::getline(reader, line);
        // std::cout << line << std::endl;

        std::vector<double> u;
        std::stringstream lineStream(line);
        std::string cell;
        while (std::getline(lineStream, cell, ',')) {
          u.push_back(stod(cell));
        }
        u_opt.at(z).at(vx).at(0).at(i) = u.at(0);
        u_opt.at(z).at(vx).at(1).at(i) = u.at(1);
        u_opt.at(z).at(vx).at(2).at(i) = u.at(2);
      }
    }
  }
  std::cout << "[SUCCESS] read in all values" << std::endl;

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

  Action action = Action{system};          // for arming / disarming etc
  Offboard offboard = Offboard{system};    // for offboard control
  Telemetry telemetry = Telemetry{system}; // for telemetry services
  Mission mission = Mission{system};
  MissionRaw missionRaw = MissionRaw{system};
  std::cout << "[SUCCESS] System is ready\n";

  // first initial condition = entry into offboard-zone
  initial_condition.at(0) = -10.0; // x-position
  initial_condition.at(1) = 2.0;   // z-position
  initial_condition.at(2) = 10.0;  // vx-velocity
  initial_condition.at(3) = 0.0;   // vz-velocity
  initial_condition.at(4) = 0.0; // pitch angle (evt need to change that value)
  initial_condition.at(5) = 0.0; // arm angle

  /* START MISSION AND WAIT FOR OFFBOARD ACTIVATION */
  action.arm();
  std::this_thread::sleep_for(
      std::chrono::milliseconds(1000)); // wait for arming to complete
  mission.start_mission();

  // wait until out first mission checkpoint is reached
  std::cout << "takoff not yet complete..." << std::endl;
  while (true) {
    std::this_thread::sleep_for(std::chrono::milliseconds(150));
    if (missionRaw.mission_progress().current == 2) {
      break;
    }
  }
  std::cout << "takeoff complete, Now watiting for MPC action!" << std::endl;

  std::vector<double> dir(2);
  dir.at(0) = deg_to_m(p2_long - p1_long);
  dir.at(1) = deg_to_m(p2_lat - p1_lat);

  double len_sq = dir.at(0) * dir.at(0) + dir.at(1) * dir.at(1);

  while (true) {

    // delay
    std::this_thread::sleep_for(std::chrono::milliseconds(20));

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
    double lat_err = std::sqrt(abw.at(0) * abw.at(0) + abw.at(1) * abw.at(1));

    // check if offboard should begin
    // std::cout << prog << std::endl;
    if (lat_err < 1 && lat_err > -1 && prog < 1 && prog > 0) {
      break;
    }
  }

  std::ofstream file;
  std::string path = "apps/app_mpc_plane/log/log.csv";
  file.open(path);
  file << "u0_opt,u1_opt,u2_opt,x,z,vx,vz,pitch\n";

  double yaw = telemetry.attitude_euler().yaw_deg;
  std::cout << "starting offboard" << std::endl;

  // send msg before startring offbaord
  Offboard::Attitude msg;
  msg.pitch_deg = 0;
  msg.roll_deg = 0;
  msg.thrust_value = 0.3;
  msg.yaw_deg = yaw;
  offboard.set_attitude(msg);

  // starting offboard;
  offboard.start();

  // TODO: determine initial conditions.

  int vx_idx = 3;
  int z_idx = 2;

  double gripper_angle;

  for (int i = 0; time_horizon; i++) {

    // CONTROL INPUTS HERE
    Offboard::Attitude msg;
    // set optimal control values
    msg.pitch_deg = u_opt.at(z_idx).at(vx_idx).at(1).at(i) * (180 / M_PI);
    msg.thrust_value = u_opt.at(z_idx).at(vx_idx).at(0).at(i);
    gripper_angle = u_opt.at(z_idx).at(vx_idx).at(2).at(i);
    // std::cout << "control: " << msg.pitch_deg << " , " << msg.thrust_value
    //           << " , " << gripper_angle << std::endl;

    offboard.set_attitude(msg); // send offboard msg

    /* SEND GRIPPER ANGLE TO ARDUINO */

    // // logging (u0,u1,u2,x,z,vx,vz)
    double pos_long = telemetry.position().longitude_deg - obj_long;
    double pos_lat = telemetry.position().latitude_deg - obj_lat;
    double x = deg_to_m(pos_long * dir_long + pos_lat * dir_lat);
    double vx = telemetry.velocity_ned().north_m_s * dir_lat +
                telemetry.velocity_ned().east_m_s * dir_long;

    // log << u_opt.at(z_idx).at(vx_idx).at(0).at(i);
    // << "," << u_opt.at(z_idx).at(vx_idx).at(1).at(i) << "," << gripper_angle
    // << "," << x << "," << telemetry.position().relative_altitude_m << "," <<
    // vx
    // << "," << -telemetry.velocity_ned().down_m_s << ","
    // << telemetry.attitude_euler().pitch_deg * M_PI / 180 << "\n";

    // after the grasp is over-> we leave the control loop and continue the
    // normal flight
    if (x > 2) {
      break;
    }
    // sleep
    std::this_thread::sleep_for(std::chrono::milliseconds(int(1000 * dt)));
  }

  std::cout << "Finished MPC -> continuing mission" << std::endl;
  offboard.stop();
  mission.start_mission();
  std::cout << "continued mission -> MPC Program will now stop! "
               "Good Luck Landing that aircraft :)"
            << std::endl;
  file.close();
  return 0;
}