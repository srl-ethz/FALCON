#pragma once
#include <chrono>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <string>
#include <thread>

// CASADI
#include <casadi/casadi.hpp>

namespace AttitudeMPC {
// compute optimal control input
/**
 * Find optimal control input.
 *
 * @param[out] u_opt optimal control input: [throttle, pitch,grip_angle]
 * @param[in] x0 Initial Contitions:[x,z,vx,vz,alpha,beta]
 * @param[in] x_ref Reference: [x_ref,z_ref, vx_ref, vz_ref, obj_x, obj_z]
 * @param[in] time time horizon after which reference state should be reached
 * @param[in] dt time steps
 * @param[in] debug_level Determines what is printed to the console:
 * 0 = nothing, 1 = final result only, 2 = full result, 3 = all
 * @param[in] full_log Writes all variables into a csv file
 * @param[in] full_log Writes only the opitmal controls into a file
 */

void doControlStep(std::vector<std::vector<double>> &u_opt, casadi::DM &x0,
                   casadi::MX &x_ref, double time, double dt, int debug_level,
                   std::ofstream &full_log, std::ofstream &ctrl_log);

// void doControlStep(std::vector<std::vector<double>> &u_opt, casadi::MX
// &pos_ref,
//                    casadi::MX &obj_pos, casadi::MX &vel_ref, casadi::DM
//                    &pos_0, casadi::DM &vel_0, casadi::DM &alpha_0, casadi::DM
//                    &beta_0, double time, double dt, int debug_level,
//                    std::string file_name);

} // namespace AttitudeMPC

namespace Model {
// constants
const double A_tot = 1;        // m2
const double rho = 1.28;       // kg_m3
const double mass = 1;         // kg
const double arm_length = 0.4; // kg
// functions
casadi::MX drag(casadi::MX aoA, casadi::MX speed_squared);
casadi::MX lift(casadi::MX aoA, casadi::MX speed_squared);
casadi::MX thrust(casadi::MX u_throttle);
} // namespace Model