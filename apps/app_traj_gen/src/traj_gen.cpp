#include <filesystem>
#include <fstream>
#include <iostream>

#include "casadi/casadi.hpp"
#include "mpc.h"

using namespace casadi;

// constants
const double time_horizon = 0.7;
const double dt = 0.02;
const int iterations = int(time_horizon / dt);
const int n_initial_conditions = 6;
const int n_reference_vars = 6; // number of reference variables
const int n_ctrl_vars = 3;
const int n_z = 20;  // number of possible initial conditions for x
const int n_vx = 10; // number of possible initial conditions for vs
const std::string file_name = "test"; // filename for trajectories

int main(int argc, char *argv[]) {
  // prepare function output variable
  std::vector<std::vector<double>> u_opt(n_ctrl_vars,
                                         std::vector<double>(iterations));

  // set initial state
  DM x0 = DM::zeros(n_initial_conditions, 1);
  x0(0, 0) = -2;  // x
  x0(1, 0) = 1;   // z
  x0(2, 0) = 10;  // vx
  x0(3, 0) = 0;   // vz
  x0(4, 0) = 0.1; // alpha
  x0(5, 0) = 0.0; // beta

  // set reference
  MX ref = MX(n_reference_vars, 1);
  ref(0, 0) = 2; // x_ref
  ref(1, 0) = 2; // z_ref
  ref(2, 0) = 2; // vx_ref
  ref(3, 0) = 0; // vz_ref
  ref(4, 0) = 0; // obj_x
  ref(5, 0) = 1; // obj_z

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
  ctrl_log << "u0_opt,u1_opt,u2_opt\n";

  for (double z = 0.7; z < 1.35; z += 0.05) {
    for (double vx = 9; vx < 12.5; vx += 0.25) {

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