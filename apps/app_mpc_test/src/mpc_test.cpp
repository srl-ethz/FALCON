#include <iostream>

#include "casadi/casadi.hpp"
#include "mpc.h"

using namespace casadi;

int main(int argc, char *argv[])
{

  // prepare function output variable
  std::vector<std::vector<double>> u_opt(2, std::vector<double>(int(3 / 0.08)));

  // set initial state
  DM pos_0 = DM::zeros(2, 1);
  DM vel_0 = DM::zeros(2, 1);
  DM alpha_0 = DM::zeros(1, 1);

  pos_0(0, 0) = 0;
  pos_0(1, 0) = 0;
  vel_0(0, 0) = 8;
  vel_0(1, 0) = 0;
  alpha_0(0, 0) = 0.1;

  // generate reference
  MX pos_ref = MX(2, 1);
  MX vel_ref = MX(2, 1);

  pos_ref(0, 0) = 50;
  pos_ref(1, 0) = 4;
  vel_ref(0, 0) = 8;
  vel_ref(1, 0) = 0;

  // do control step
  AttitudeMPC::doControlStep(u_opt, pos_ref, vel_ref, pos_0, vel_0, alpha_0, 3.0, 0, "test");

  // print solution
  // std::cout << "OPTIMAL SOLUTION" << std::endl;
  // for (int i = 0; i < 25; i++)
  // {
  //   std::cout << "timestep: " << i << " \t " << u_opt.at(0).at(i) << "\t" << u_opt.at(1).at(i) << std::endl;
  // }
  return 0;
}