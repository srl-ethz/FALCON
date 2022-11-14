#pragma once

namespace AttitudeMPC
{
  // compute optimal control input
  /**
   * Find optimal control input.
   *
   * @param[out] optimal_input Computed optimal control input.
   * @param[in] pos_ref Reference position
   * @param[in] vel_ref Reference velocity
   * @param[in] acc_ref Reference acceleration
   * @param[in] initial_state Current position
   * @param[in] time time horizon after which reference state should be reached
   * @param[in] debug_level Determines what is printed to the console:
   * 0 = nothing, 1 = final result only, 2 = full result, 3 = all
   */
  void doControlStep(std::vector<double> &optimal_input,
                     casadi::MX &pos_ref,
                     casadi::MX &vel_ref,
                     casadi::DM &initial_state,
                     double time,
                     int debug_level);
}

namespace Model
{
  // constants
  const double A_tot = 1;  // m2
  const double rho = 1.28; // kg_m3
  const double mass = 3.5; // kg
  // functions
  casadi::MX drag(casadi::MX aoA, casadi::MX speed_squared);
  casadi::MX lift(casadi::MX aoA, casadi::MX speed_squared);
  casadi::MX thrust(casadi::MX u_throttle);
}