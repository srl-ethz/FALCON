#include <boost/asio.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>
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

// ROS 2
#include "geometry_msgs/msg/point.hpp"
#include "raptor_interface/srv/set_servo.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/trigger.hpp"

using namespace mavsdk;
using std::chrono::milliseconds;
using std::chrono::seconds;
using std::this_thread::sleep_for;

// helpers
#include "mavsdk_helper.h"

namespace params {
const double K_alt = 100.0;
const double K_spd = 0.3;
}; // namespace params

// TODO Put into YAML file (dont care for now)
//   way point before pickup (mission dependent)
const double p1_long = 8.3874380;
const double p1_lat = 47.4246895;

// way point after pickup (mission dependent)
const double p2_long = 8.3872764;
const double p2_lat = 47.4246895;

// way point after pickup (mission dependent)
const double obj_long = 8.3873573;
const double obj_lat = 47.4246895;
const double ground_alt = 389.2;

const double grasp_length = 5; // length from beginning of MPC control to object
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
const double time_horizon = 1.0;
const double dt = 0.02;
const int iterations = int(time_horizon / dt);

// other constants
const double R = 6378137; // earths Radius in [m]
const int n_initial_conditions = 6;
const int n_ctrl_vars = 6; // u0, u1, u2, x, z, vel
const int n_z = 10;        // number of possible initial conditions for x
const int n_vx = 9;        // number of possible initial conditions for vs
const std::vector<double> vx0 =
    std::vector<double>(n_vx); // values of possible initial conditions
const std::vector<double> x0 =
    std::vector<double>(n_z); // values of possible initial conditions

std::vector<std::vector<std::vector<std::vector<double>>>>
    u_opt(n_z, std::vector<std::vector<std::vector<double>>>(
                   n_vx, std::vector<std::vector<double>>(
                             n_ctrl_vars, std::vector<double>(iterations))));

std::vector<double> possible_z{9.75,  9.80,  9.85,  9.90,  9.95,
                               10.00, 10.05, 10.10, 10.15, 10.20};
std::vector<double> possible_vx{8.5,  8.75, 9.0,   9.25, 9.5,
                                9.75, 10.0, 10.25, 10.5};

/// @brief struct for cooridnates in the pick-up plane
/// @param x x coordinate
/// @param z z coordinate
/// @param vx x-velocity
/// @param vz z-velocity
struct coordinates {
  double x;
  double z;
  double vx;
  double vz;
};

/// @brief calculates a distance in meters from gps degrees
/// @param deg gps latitude or longitude
/// @return meters
double deg_to_m(double deg) {
  double R = 6378000.137; // Radius of earth in m
  return M_PI * R * deg / 180;
}

/// @brief Finds index of possible_vx vector whose value closest corresponds
/// to the input argument
/// @param speed speed of the plane
/// @return index in possible_vx vector
int find_vx_id(double speed) {
  double min_vx = 1000;
  int min_vx_id = 0;
  for (int i = 0; i < n_vx; i++) {
    if (std::abs(speed - possible_vx.at(i)) < min_vx) {
      min_vx = std::abs(speed - possible_vx.at(i));
      min_vx_id = i;
    }
  }
  return min_vx_id;
}

/// @brief Finds index of possible_z vector whose value closest corresponds
/// to the input argument
/// @param speed altitude of the plane
/// @return index in possible_z vector
int find_z_id(double alt) {
  double min_z = 1000;
  int min_z_id = 0;
  for (int i = 0; i < n_z; i++) {
    if (std::abs(alt - possible_z.at(i)) < min_z) {
      min_z = std::abs(alt - possible_z.at(i));
      min_z_id = i;
    }
  }
  return min_z_id;
}

int find_closest_id(std::vector<double> vec, double val) {
  double val_min = 1000;
  int val_min_id = 0;
  for (int i = 0; i < vec.size(); i++) {
    if (std::abs(val - vec.at(i)) < val_min) {
      val_min = std::abs(val - vec.at(i));
      val_min_id = i;
    }
  }
  return val_min_id;
}

/// @brief This functions transforms global gps coordinates (provided by
/// mavsdk telemetry plugin) to coordinates realtive to the object in the
/// pickup plane
/// @param telemetry mavsdk telemetry plugin
/// @return a coords struct with the values x,z,vx and vz
coordinates get_coords(Telemetry &telemetry) {
  double pos_long = telemetry.position().longitude_deg - obj_long;
  double pos_lat = telemetry.position().latitude_deg - obj_lat;
  coordinates coords;
  coords.x = deg_to_m(pos_long * dir_long + pos_lat * dir_lat);
  coords.z = telemetry.position().absolute_altitude_m - ground_alt;
  coords.vx = telemetry.velocity_ned().north_m_s * dir_lat +
              telemetry.velocity_ned().east_m_s * dir_long;
  coords.vz = -telemetry.velocity_ned().down_m_s;
  return coords;
}

int main(int argc, char **argv) {
  /* INITIALIZE ROS2 */
  rclcpp::init(argc, argv);
  std::shared_ptr<rclcpp::Node> node =
      rclcpp::Node::make_shared("grasper_sim_node");

  /* CREATE ROS2 service clients */
  rclcpp::Client<raptor_interface::srv::SetServo>::SharedPtr
      client_set_gripper = node->create_client<raptor_interface::srv::SetServo>(
          "gripperFinger_deg");
  rclcpp::Client<raptor_interface::srv::SetServo>::SharedPtr client_set_arm =
      node->create_client<raptor_interface::srv::SetServo>("gripperArm_deg");
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr client_video =
      node->create_client<std_srvs::srv::Trigger>("videoTrigger");

  /* READ OPTIMAL CONTROLS FROM PRECALCULATED FILE*/
  std::ifstream reader;
  reader.open("apps/app_grasper_sim/src/u_opt.csv");
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
        // std::cout << "[" << z << "," << vx << "," << i << "]" <<
        // std::endl;

        // std::cout << "[" << possible_z.at(z) << ", " <<
        // possible_vx.at(vx)
        //           << ", " << i << "]" << std::endl;
        std::getline(reader, line);
        // std::cout << line << std::endl;

        std::vector<double> u;
        std::stringstream lineStream(line);
        std::string cell;
        while (std::getline(lineStream, cell, ',')) {
          u.push_back(stod(cell));
        }
        // std::cout << u.size() << std::endl;
        u_opt.at(z).at(vx).at(0).at(i) = u.at(0); // u0_opt
        u_opt.at(z).at(vx).at(1).at(i) = u.at(1); // u1_opt
        u_opt.at(z).at(vx).at(2).at(i) = u.at(2); // u2_opt
        u_opt.at(z).at(vx).at(3).at(i) = u.at(3); // x_opt
        u_opt.at(z).at(vx).at(4).at(i) = u.at(4); // z_opt
        u_opt.at(z).at(vx).at(5).at(i) = u.at(5); // vel_opt
      }
    }
  }
  std::cout << "[SUCCESS] read in all values" << std::endl;

  /* INITIALIZE LOGGING */
  std::ofstream file;
  std::string path = "apps/app_grasper_sim/log/grasp1.csv";
  file.open(path);
  file << "u0_opt,u1_opt,u2_opt,x,z,vel,xr,zr,"
          "vxr,vzr,pitchr\n";

  /* INITIALIZE MAVSDK */
  if (argc != 2) {
    usage(argv[0]);
    return 1;
  }

  Mavsdk mavsdk;
  ConnectionResult connection_result =
      mavsdk.add_any_connection("udp://:14540");

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

  std::cout << "setting gripper to inital position" << std::endl;
  auto armRequest =
      std::make_shared<raptor_interface::srv::SetServo::Request>();
  armRequest->angle = u_opt.at(0).at(0).at(2).at(0) * 180.0 / M_PI;
  auto armResponse = client_set_arm->async_send_request(armRequest);

  auto gripperRequest =
      std::make_shared<raptor_interface::srv::SetServo::Request>();
  gripperRequest->angle = 90.0;
  auto gripperResponse = client_set_gripper->async_send_request(gripperRequest);

  // wait until we are sufficiently close to the object
  while (get_coords(telemetry).x < -grasp_length * 5) {
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  // start video recording
  auto videoRequest = std::make_shared<std_srvs::srv::Trigger::Request>();
  auto videoResponse = client_video->async_send_request(videoRequest);
  std::cout << "started video recording" << std::endl;

  while (get_coords(telemetry).x < -grasp_length) {
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
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

  // TODO: determine initial conditions.
  coordinates coords = get_coords(telemetry);
  int vx_idx = find_vx_id(coords.vx);
  int z_idx = find_z_id(coords.z);

  std::cout << "Actual Coords: [" << coords.z << ", " << coords.vx << "]"
            << "\n x0 coords: [" << possible_z.at(z_idx) << ", "
            << possible_vx.at(vx_idx) << "]" << std::endl;

  double gripper_angle;

  boost::asio::io_service io;
  int idx = 0;
  for (int i = 0; idx < iterations; i++) {
    boost::asio::deadline_timer t(io, boost::posix_time::milliseconds(5));

    // FIND CONTROLS (TODO: CHECK FOR out_of_range)
    coordinates coords = get_coords(telemetry);
    while (coords.x > u_opt.at(z_idx).at(vx_idx).at(3).at(idx)) {
      idx++;
    }
    std::cout << "x: " << coords.x
              << " x_vec:" << u_opt.at(z_idx).at(vx_idx).at(3).at(idx)
              << " idx:" << idx << std::endl;
    // CONTROL INPUTS HERE
    Offboard::Attitude msg;
    // set optimal control values
    double alt_error = coords.z - u_opt.at(z_idx).at(vx_idx).at(4).at(idx);
    msg.pitch_deg = u_opt.at(z_idx).at(vx_idx).at(1).at(idx) * (180.0 / M_PI) -
                    params::K_alt * alt_error;

    double speed_error = coords.vx * coords.vx + coords.vz * coords.vz -
                         u_opt.at(z_idx).at(vx_idx).at(5).at(idx) *
                             u_opt.at(z_idx).at(vx_idx).at(5).at(idx);
    msg.thrust_value =
        u_opt.at(z_idx).at(vx_idx).at(0).at(idx) - params::K_spd * speed_error;

    // msg.pitch_deg = u_opt.at(z_idx).at(vx_idx).at(1).at(idx) * (180.0 /
    // M_PI); msg.thrust_value = u_opt.at(z_idx).at(vx_idx).at(0).at(idx);
    gripper_angle = u_opt.at(z_idx).at(vx_idx).at(2).at(idx) * (180.0 / M_PI);
    // std::cout << "control: " << msg.pitch_deg << " , " << msg.thrust_value
    //           << " , " << gripper_angle << std::endl;

    /* SET ATTITUDE */
    offboard.set_attitude(msg); // send offboard msg

    /* SEND GRIPPER ANGLE TO SIMULATOR */
    auto request = std::make_shared<raptor_interface::srv::SetServo::Request>();
    request->angle = u_opt.at(z_idx).at(vx_idx).at(2).at(idx) * (180.0 / M_PI);
    auto response = client_set_arm->async_send_request(request);

    // check if gripper should be closed
    if (coords.x > 0) {
      gripperRequest =
          std::make_shared<raptor_interface::srv::SetServo::Request>();
      gripperRequest->angle = 10.0;
      gripperResponse = client_set_gripper->async_send_request(gripperRequest);
    }

    // logging (u0,u1,u2,x,z,vx,vz)
    // file << u_opt.at(z_idx).at(vx_idx).at(0).at(idx) << ","
    //      << u_opt.at(z_idx).at(vx_idx).at(1).at(idx) << "," << gripper_angle
    //      << "," << coords.x << "," << coords.z << "," << coords.vx << ","
    //      << coords.vz << ","
    //      << telemetry.attitude_euler().pitch_deg * M_PI / 180 << "\n";
    file << u_opt.at(z_idx).at(vx_idx).at(0).at(idx) << ","
         << u_opt.at(z_idx).at(vx_idx).at(1).at(idx) << ","
         << u_opt.at(z_idx).at(vx_idx).at(2).at(idx) << ","
         << u_opt.at(z_idx).at(vx_idx).at(3).at(idx) << ","
         << u_opt.at(z_idx).at(vx_idx).at(4).at(idx) << ","
         << u_opt.at(z_idx).at(vx_idx).at(5).at(idx) << "," << coords.x << ","
         << coords.z << "," << coords.vx << "," << coords.vz << ","
         << telemetry.attitude_euler().pitch_deg * M_PI / 180 << "\n";

    // after the grasp is over-> we leave the control loop and continue the
    // normal flight
    if (coords.x > 3.0) {
      break;
    }
    // sleep for dt
    t.wait();
    // std::this_thread::sleep_for(std::chrono::milliseconds(int(1000 * dt)));
  }

  std::cout << "Finished MPC -> continuing mission" << std::endl;
  offboard.stop();
  mission.start_mission();
  std::this_thread::sleep_for(std::chrono::seconds(1));
  std::cout << "setting gripper to landing position..." << std::endl;
  armRequest = std::make_shared<raptor_interface::srv::SetServo::Request>();
  armRequest->angle = 180.0;
  armResponse = client_set_arm->async_send_request(armRequest);

  std::cout << "continued mission -> Offboard controller will now stop! "
               "Good Luck Landing that aircraft :)"
            << std::endl;

  // stop video recording
  std::this_thread::sleep_for(std::chrono::milliseconds(1500));
  videoRequest = std::make_shared<std_srvs::srv::Trigger::Request>();
  videoResponse = client_video->async_send_request(videoRequest);
  std::cout << "stopped video recording" << std::endl;
  file.close();
  return 0;
}