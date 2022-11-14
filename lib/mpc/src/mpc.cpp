#include <chrono>
#include <thread>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <future>

// CASADI
#include <casadi/casadi.hpp>

// header file
#include "mpc.h"

using namespace casadi;
using std::chrono::milliseconds;
using std::chrono::seconds;
using std::this_thread::sleep_for;

MX Model::drag(MX aoA_rad, MX airspeed_squared)
{
  return 0.5 * Model::A_tot * Model::rho * airspeed_squared * (1.97950651 * pow(aoA_rad, 2) + 0.01655889);
}
MX Model::lift(MX aoA_rad, MX airspeed_squared)
{
  return 0.5 * Model::A_tot * Model::rho * airspeed_squared * (0.44482359 * aoA_rad + 0.00245455);
}
MX Model::thrust(MX u_throttle)
{
  return 14.456 * pow(u_throttle, 2);
}

void AttitudeMPC::doControlStep(std::vector<double> &optimal_input,
                                casadi::MX &pos_ref,
                                casadi::MX &vel_ref,
                                casadi::DM &initial_state,
                                double time,
                                int debug_level)
{
  /////////////////////////////////////////////////////////////////// parameters
  // input constraints
  const double u0_MAX = 1;                  // [ - ] max throttle cmd
  const double u0_MIN = 0;                  // [ - ] min throttle cmd
  const double u1_MAX = 30 * 3.1415 / 180;  // [ rad ] max pitch
  const double u1_MIN = -30 * 3.1415 / 180; // [ rad ] min pitch

  const double aoA_MAX = 20 * 3.1415 / 180; // [ rad ] max angle of attack
  const double aoA_MIN = 0;                 // [ rad ] min angle of attack

  const double V_MAX = pow(15, 2); // [ m/s ] max speed
  const double V_MIN = pow(6, 2);  // [ m/s ] min speed

  // timstep (for now constant)
  double dt = 0.08;
  // determine horizon
  int horizon = time / dt;
  // gravity
  MX g = MX::zeros(2, 1);
  g(1) = -9.81;

  ///////////////////////////////////////////////////////// initialize optimizer
  Opti opti = Opti();
  MX pos = opti.variable(2, horizon + 1); // x and z
  MX vel = opti.variable(2, horizon + 1); // x_dot and z_dot
  MX u = opti.variable(2, horizon);       // thurst and alpha

  // cost function
  MX J = 0;
  double weight = 100 / horizon;
  for (int i = 0; i < horizon; ++i)
  {
    std::cout << i << std::endl;
    weight = (i + 1) * 100 / horizon;
    J += weight * sumsqr(pos(Slice(), i + 1) - pos_ref) + weight / 5 * sumsqr(vel(Slice(), i + 1) - vel_ref);
  }
  opti.minimize(J);

  // system dynamics (highly nonlinear)
  for (int i = 1; i <= horizon; ++i)
  {
    // x vector of plane coords (local)
    MX x_loc = MX::zeros(2, 1);
    x_loc(0) = cos(u(1, i - 1));
    x_loc(1) = sin(u(1, i - 1));

    // z vector of plane coords (local)
    MX z_loc = MX::zeros(2, 1);
    z_loc(0) = -sin(u(1, i - 1));
    z_loc(1) = cos(u(1, i - 1));

    // calculate absolute value of speed and angel of Attack
    MX speed_squared = vel(0, i - 1) * vel(0, i - 1) + vel(1, i - 1) * vel(1, i - 1);
    MX aoA = atan2(vel(0, i - 1), vel(1, i - 1)) - u(1, i - 1);

    // calculate lift, drag and thrust
    MX lift = Model::lift(aoA, speed_squared);
    MX drag = Model::drag(aoA, speed_squared);
    MX thrust = Model::thrust(u(0, i - 1));

    // calculate acceleration on plane
    MX a = (thrust * x_loc + lift * z_loc - drag * x_loc) / Model::mass;

    //  position update
    opti.subject_to(
        pos(Slice(), i) ==
        pos(Slice(), i - 1) + dt * vel(Slice(), i - 1));
    // velocity update
    opti.subject_to(
        vel(Slice(), i) ==
        vel(Slice(), i - 1) + dt * a + dt * g);
  }

  // initial state
  opti.subject_to(pos(Slice(), 0) == initial_state(Slice(0, 2), 0));
  opti.subject_to(vel(Slice(), 0) == initial_state(Slice(2, 4), 0));

  opti.set_initial(pos(Slice(), 0), initial_state(Slice(0, 2), 0));
  opti.set_initial(vel(Slice(), 0), initial_state(Slice(2, 4), 0));

  // constraints
  for (int i = 0; i < horizon; ++i)
  {
    // inputs
    opti.subject_to(opti.bounded(u0_MIN, u(0, i), u0_MAX));
    opti.subject_to(opti.bounded(u1_MIN, u(1, i), u1_MAX));

    // angle of attack
    opti.subject_to(opti.bounded(aoA_MIN, atan2(vel(0, i), vel(1, i)) - u(1, i - 1), aoA_MAX));

    // velocities
    opti.subject_to(opti.bounded(V_MIN, vel(0, i + 1) * vel(0, i + 1) + vel(1, i + 1) * vel(1, i + 1), V_MAX));
  }

  ////////////////////////////////////////////////////////////////////// solving
  // suppress ipopt console output?
  Dict opts_dict = Dict();
  if (debug_level < 3)
  {
    opts_dict["ipopt.print_level"] = 0;
    opts_dict["ipopt.sb"] = "yes";
    opts_dict["print_time"] = 0;
    opti.solver("ipopt", opts_dict);
  }
  else
  {
    opti.solver("ipopt");
  }

  // solve
  auto sol = opti.solve();

  // debug
  if (debug_level > 1)
  {
    std::cout << sol.value(pos) << '\n';
    std::cout << sol.value(u) << '\n';
  }

  // get result and cast to double vector
  // https://groups.google.com/g/casadi-users/c/npPcKItdLN8
  // for reverse casting (irrelevant here):
  // https://github.com/casadi/casadi/issues/2563
  DM solution = sol.value(u)(Slice(), 0);
  optimal_input = DM::densify(solution).nonzeros();

  // debug
  if (debug_level > 0)
  {
    std::cout << optimal_input << '\n';
  }
}