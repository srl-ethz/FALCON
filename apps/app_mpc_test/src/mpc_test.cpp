#include <iostream>

#include "casadi/casadi.hpp"
#include "mpc.h"

using namespace casadi;

int main(int argc, char *argv[])
{

  // prepare function output variable
  std::vector<double> u(2);

  // set initial state
  DM current_state = DM::zeros(4, 1);
  for (int i = 0; i < 4; ++i)
  {
    current_state(i, 0) = 2;
  }

  // generate reference
  MX pos_ref = MX(2, 1);
  MX vel_ref = MX(2, 1);
  for (int i = 0; i < 2; ++i)
  {
    pos_ref(i, 0) = 1;
    vel_ref(i, 0) = 0;
  }

  // do control step
  AttitudeMPC::doControlStep(u, pos_ref, vel_ref, current_state, 2., 1);

  return 0;
}