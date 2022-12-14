#include <filesystem>
#include <fstream>
#include <iostream>

#include "casadi/casadi.hpp"
#include "mpc.h"

using namespace casadi;

// constants
const double time_horizon = 1.0;
const double dt = 0.02;
const int iterations = int(time_horizon / dt);
const int n_initial_conditions = 6;
const int n_reference_vars = 6; // number of reference variables
const int n_ctrl_vars = 3;
const std::string file_name = "opt"; // filename for trajectories

int main(int argc, char *argv[]) {
  // prepare function output variable
  std::vector<std::vector<double>> u_opt(n_ctrl_vars,
                                         std::vector<double>(iterations));

  // set initial state
  DM x0 = DM::zeros(n_initial_conditions, 1);
  x0(0, 0) = -5;                   // x
  x0(1, 0) = 10;                   // z
  x0(2, 0) = 10;                   // vx
  x0(3, 0) = 0;                    // vz
  x0(4, 0) = 0.026212279457713025; // initial pitch
  x0(5, 0) = 30;                   // beta

  // set reference
  MX ref = MX(n_reference_vars, 1);
  ref(0, 0) = 5;  // x_ref
  ref(1, 0) = 10; // z_ref
  ref(2, 0) = 10; // vx_ref
  ref(3, 0) = 0;  // vz_ref
  ref(4, 0) = 0;  // obj_x
  ref(5, 0) = 10; // obj_z

  std::string path_full_log =
      "apps/app_traj_gen/out/" + file_name + "_full_log.csv";
  std::string path_ctrl_log =
      "apps/app_traj_gen/out/" + file_name + "_ctrl_log.csv";
  // initialize full logger (that logs all variables)
  std::ofstream full_log;
  full_log.open(path_full_log);
  full_log << "z0,vx0,i,u0_opt,u1_opt,u2_opt,x,z,v_abs,v_arg,vx,vz,gripx,gripz,"
              "aoa\n";
  // initialize ctrl log that only logs the inputs to the system
  std::ofstream ctrl_log;
  ctrl_log.open(path_ctrl_log);
  ctrl_log << "u0_opt,u1_opt,u2_opt,x,z,vel\n";

  for (double z = 9.75; z < 10.25; z += 0.05) {
    for (double vx = 8.5; vx < 10.6; vx += 0.25) {
      // double z = 10.2;
      // double vx = 10;
      std::cout << "NEW OPITIMZATION STARTED [z,vx]: [" << z << ", " << vx
                << "]" << std::endl;
      x0(1, 0) = z;
      x0(2, 0) = vx;
      // do control step
      AttitudeMPC::doControlStep(u_opt, x0, ref, time_horizon, dt, 1, full_log,
                                 ctrl_log);
    }
  }

  full_log.close();
  ctrl_log.close();
  return 0;
}