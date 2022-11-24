#include <chrono>
#include <filesystem>
#include <fstream>
#include <future>
#include <iostream>
#include <mutex>
#include <thread>
#include <vector>

#include "casadi/casadi.hpp"
#include "mpc.h"

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

using namespace casadi;

//  way point before pickup (mission dependent)
const double p1_long = 8.5456071;
const double p1_lat = 47.3978404;

// way point after pickup (mission dependent)
const double p2_long = 8.5456071;
const double p2_lat = 47.3976608;

// way point after pickup (mission dependent)
const double obj_long = 8.5456071;
const double obj_lat = 47.3976608;
const double obj_alt = 1.0;

const double grasp_length =
    10.0; // length from beginning of MPC control to object (or
          // from object to end of MPC control)

// normalized vector from p1 to p2
const double dir_len = std::sqrt((p2_long - p1_long) * (p2_long - p1_long) +
                                 (p2_lat - p1_lat) * (p2_lat - p1_lat));
const double dir_long = (p2_long - p1_long) / dir_len;
const double dir_lat = (p2_lat - p1_lat) / dir_len;

// global unprotected variables
int n_opt = 0;
int n_ctrl = 0;

// control parameters
const double yaw_P = 1.0;
const double yaw_I = 0.2;

// global read variables
const double time_horizon = 1.0;
const double R = 6378137; // earths Radius in [m]
const double dt = 0.1;
const int iterations = int(time_horizon / dt);
const int n_initial_conditions = 6;
const int n_inputs = 3;
const int debug_level = 0;

// a global instance of std::mutex to protect global variable
std::mutex myMutex;

// values that are accessed by multiple threads and need to be guarded by mutex
std::vector<double> initial_condition(n_initial_conditions);
std::vector<std::vector<double>>
    optimal_controls(n_inputs, std::vector<double>(iterations));
bool new_opt = false;
bool mpc_finished = false;

// generate reference
MX pos_ref = MX(2, 1);
MX obj_ref = MX(2, 1);
MX vel_ref = MX(2, 1);

double deg_to_m(double deg) {
  double R = 6378000.137; // Radius of earth in m
  return M_PI * R * deg / 180;
}

double m_to_deg(double m) {
  double R = 6378000.137; // Radius of earth in m
  return (m * 180) / (M_PI * R);
}

// void gps_to_xyz(double lon, double lat, double alt, double &x, double &y,
//                 double &z) {

//   z = alt - obj_alt;
//   x = (((lon - p1_long) * dir_long + (lat - p1_lat) * dir_lat) / dir_len -
//        dir_len) *
//       R * M_PI / 180;
//   y = (((lon - p1_long) * dir_lat - (lat - p1_lat) * dir_long) / dir_len) * R
//   *
//       M_PI / 180;
//   ;
// }

void controller(Offboard &offboard, Telemetry &telemetry);
void optimizer();

int main(int argc, char **argv) {
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
  std::cout << "System is ready\n";

  /* INITIALIZE OPTIMIZER */
  // set reference position (from mission)
  // 10 meters in deg: 0.0000898246531
  pos_ref(0, 0) = 10;
  pos_ref(1, 0) = 2;
  vel_ref(0, 0) = 8;
  vel_ref(1, 0) = 0;
  obj_ref(0, 0) = 0;
  obj_ref(1, 0) = 1;

  // first initial condition = entry into offboard-zone
  initial_condition.at(0) = -10.0; // x-position
  initial_condition.at(1) = 2.0;   // z-position
  initial_condition.at(2) = 8.0;   // vx-velocity
  initial_condition.at(3) = 0.0;   // vz-velocity
  initial_condition.at(4) = 0.0; // pitch angle (evt need to change that value)
  initial_condition.at(5) = 0.0; // arm angle

  // run first optimization
  optimizer();

  /* START MISSION AND WAIT FOR OFFBOARD ACTIVATION */
  action.arm();
  std::this_thread::sleep_for(std::chrono::milliseconds(1500));
  mission.start_mission();

  // wait until out first mission checkpoint is reached
  while (true) {
    std::cout << "takoff not yet complete" << std::endl;
    std::this_thread::sleep_for(std::chrono::milliseconds(150));
    if (missionRaw.mission_progress().current == 2) {
      break;
    }
  }
  std::cout << "Now watiting for MPC action!" << std::endl;

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
  // I error
  double lat_err_sum = 0;
  int test = 4;

  // start MPC Controller
  while (mpc_finished == false) {
    new_opt = false; // set new optimization available to false
    n_ctrl = 0;      // Set count value to zero

    // Spawn threads

    std::thread optimization_thread(optimizer);
    std::thread control_thread =
        std::thread{controller, std::ref(offboard),
                    std::ref(telemetry)}; //(controller, offboard);
    // wait for optimization to finish
    optimization_thread.join();
    // update new_opt to tell control thread that new optimal control inputs are
    // available
    {
      std::lock_guard<std::mutex> guard(myMutex);
      new_opt = true;
    }
    std::cout << "NEW OPTIMAL CONTROL STRATEGIES AVAILABLE" << std::endl;
    // this results in control thread to end.
    control_thread.join();
    // TODO implement finishing criterion
  }

  std::cout << "Finished MPC -> continuing mission" << std::endl;
  offboard.stop();
  mission.start_mission();
  std::cout << "continued mission -> MPC Program will now stop! "
               "Good Luck Landing that aircraft :)"
            << std::endl;
  // return 0;
}

void controller(Offboard &offboard, Telemetry &telemetry) {
  // read optimal control inputs (Mutex)
  std::vector<std::vector<double>> u(n_inputs, std::vector<double>(iterations));
  {
    std::lock_guard<std::mutex> guard(myMutex);
    u = optimal_controls;
  }

  // do controls
  bool loop = true;
  for (int i = 0; loop == true; i++) {
    std::this_thread::sleep_for(std::chrono::milliseconds(int(1000 * dt)));
    /*

    PUT CONTROL INPUTS HERE

    */
    Offboard::Attitude msg;
    msg.pitch_deg = u.at(1).at(i);
    msg.thrust_value = u.at(0).at(i);
    offboard.set_attitude(msg);

    std::cout << "control: " << u.at(0).at(i) << " , " << u.at(1).at(i) << " , "
              << u.at(2).at(i) << std::endl;

    n_ctrl++; // counting variable for debugging

    // check if new optimal control values available
    {
      std::lock_guard<std::mutex> guard(myMutex);
      if (new_opt == true) {
        loop = false;
      }
    }
  }

  // determine coordinates in xz-plane //TODO compress
  //  current position
  double pos_long = telemetry.position().longitude_deg - obj_long;
  double pos_lat = telemetry.position().latitude_deg - obj_lat;

  // progress along mission route
  double x = deg_to_m(pos_long * dir_long + pos_lat * dir_lat);
  double vx = telemetry.velocity_ned().north_m_s * dir_lat +
              telemetry.velocity_ned().east_m_s * dir_long;
  // write current values into inital condition vector for
  // next optimization problem
  std::vector<double> x0(n_initial_conditions);
  x0.at(0) = x;                                        // x-position
  x0.at(1) = telemetry.position().relative_altitude_m; // z-position
  x0.at(2) = vx;                                       // vx-velocity
  x0.at(3) = -telemetry.velocity_ned().down_m_s;       // vz-velocity
  x0.at(4) = telemetry.attitude_euler().pitch_deg * M_PI /
             180; // pitch angle (evt need to change that value)
  x0.at(5) = 0.0; // arm angle

  // check if MPC Control part is finished
  if (x > grasp_length) {
    mpc_finished = true;
  }
  // write initial conditions (Mutex)
  {
    std::lock_guard<std::mutex> guard(myMutex);
    initial_condition = x0;
  }
}

void optimizer() {

  // read initial conditions (Mutex)
  std::vector<double> x0;
  {
    std::lock_guard<std::mutex> guard(myMutex);
    x0 = initial_condition;
  }

  // prepare values for optimizer
  std::vector<std::vector<double>> u_opt(n_inputs,
                                         std::vector<double>(iterations));

  // set initial state
  DM pos_0 = DM::zeros(2, 1);
  DM vel_0 = DM::zeros(2, 1);
  DM alpha_0 = DM::zeros(1, 1);
  DM beta_0 = DM::zeros(1, 1);

  pos_0(0, 0) = x0.at(0);   //-5;
  pos_0(1, 0) = x0.at(1);   // 6;
  vel_0(0, 0) = x0.at(2);   // 8;
  vel_0(1, 0) = x0.at(3);   // 0;
  alpha_0(0, 0) = x0.at(4); // 0.1;
  beta_0(0, 0) = x0.at(5);

  // do optimization
  AttitudeMPC::doControlStep(u_opt, pos_ref, obj_ref, vel_ref, pos_0, vel_0,
                             alpha_0, beta_0, time_horizon, dt, debug_level,
                             "real");

  n_opt++;

  // write optimal control inputs (Mutex)
  {
    std::lock_guard<std::mutex> guard(myMutex);
    optimal_controls = u_opt;
  }
}
