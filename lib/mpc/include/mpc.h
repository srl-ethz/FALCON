#pragma once
#include <string>
#include <chrono>
#include <thread>
#include <filesystem>
#include <fstream>
#include <iostream>

// CASADI
#include <casadi/casadi.hpp>

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
  void doControlStep(std::vector<std::vector<double>> &u_opt,
                     casadi::MX &pos_ref,
                     casadi::MX &obj_pos,
                     casadi::MX &vel_ref,
                     casadi::DM &pos_0,
                     casadi::DM &vel_0,
                     casadi::DM &alpha_0,
                     casadi::DM &beta_0,
                     double time,
                     double dt,
                     int debug_level,
                     std::string file_name);

}

namespace Model
{
  // constants
  const double A_tot = 1;        // m2
  const double rho = 1.28;       // kg_m3
  const double mass = 1;         // kg
  const double arm_length = 0.4; // kg
  // functions
  casadi::MX drag(casadi::MX aoA, casadi::MX speed_squared);
  casadi::MX lift(casadi::MX aoA, casadi::MX speed_squared);
  casadi::MX thrust(casadi::MX u_throttle);
}