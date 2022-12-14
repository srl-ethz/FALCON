// header full_log
#include "mpc.h"
#include <math.h> /* atan2 */

using namespace casadi;
using std::chrono::milliseconds;
using std::chrono::seconds;
using std::this_thread::sleep_for;

/* REAL PARAMETERS */
// MX Model::drag(MX aoA_rad, MX airspeed_squared) {
//   return if_else(aoA_rad < 0, 0,
//                  0.5 * Model::A_tot * Model::rho * airspeed_squared *
//                      (1.97950651 * pow(aoA_rad, 2) + 0.01655889));
// }
// MX Model::lift(MX aoA_rad, MX airspeed_squared) {
//   return if_else(aoA_rad < 0, 0,
//                  0.5 * Model::A_tot * Model::rho * airspeed_squared *
//                      (0.44482359 * aoA_rad + 0.00245455));
// }
// MX Model::thrust(MX u_throttle) { return 14.456 * pow(u_throttle, 2); }

/* SIM PARAMETERS */
MX Model::drag(MX aoA_rad, MX airspeed_squared) {
  return if_else(aoA_rad < 0,
                 0.5 * Model::A_tot * Model::rho * airspeed_squared *
                     0.04909224,
                 0.5 * Model::A_tot * Model::rho * airspeed_squared *
                     (12.10066432 * aoA_rad * aoA_rad + 0.02392831));
}
MX Model::lift(MX aoA_rad, MX airspeed_squared) {
  return if_else(aoA_rad < 0, 0,
                 0.5 * Model::A_tot * Model::rho * airspeed_squared *
                     (5.63145487 * aoA_rad + 0.0714831));
}
MX Model::thrust(MX u_throttle, MX airspeed) {
  return 104.72 * u_throttle * u_throttle * (1 - airspeed / 25);
}

void AttitudeMPC::doControlStep(std::vector<std::vector<double>> &u_opt,
                                casadi::DM &x0, casadi::MX &x_ref, double time,
                                double dt, int debug_level,
                                std::ofstream &full_log,
                                std::ofstream &ctrl_log) {
  // restructure variables for more readable code.
  casadi::DM pos_0 = x0(Slice(0, 2), 0);
  casadi::DM vel_0 = x0(Slice(2, 4), 0);
  casadi::DM alpha_0 = x0(Slice(4), 0);
  casadi::DM beta_0 = x0(Slice(5), 0);
  casadi::MX pos_ref = x_ref(Slice(0, 2), 0);
  casadi::MX vel_ref = x_ref(Slice(2, 4), 0);
  casadi::MX obj_pos = x_ref(Slice(4, 6), 0);

  // input constraints
  const double u0_MAX = 0.5;              // [ - ] max throttle cmd
  const double u0_MIN = 0;                // [ - ] min throttle cmd
  const double u1_MAX = 30 * M_PI / 180;  // [ rad ] max pitch
  const double u1_MIN = -30 * M_PI / 180; // [ rad ] min pitch
  const double delta_u1 =
      0.05 * M_PI / 180; // max pitch change per timestep (was 10 before)
  const double u2_MIN = 20 * M_PI / 180;
  const double u2_MAX = 180 * M_PI / 180;
  const double delta_u2 = 25 * M_PI / 180; // max arm angle change per timestep

  const double aoA_MAX = 6 * M_PI / 180;  // [ rad ] max angle of attack
  const double aoA_MIN = -4 * M_PI / 180; // [ rad ] min angle of attack

  const double V_ABS_MAX = 20;               // [ m/s ] max speed
  const double V_ABS_MIN = 6;                // [ m/s ] min speed
  const double V_ARG_MAX = 30 * M_PI / 180;  // [ m/s ] max speed
  const double V_ARG_MIN = -30 * M_PI / 180; // [ m/s ] min speed

  // determine horizon
  int horizon = time / dt;

  std::cout << horizon << std::endl;
  // gravity

  double g = 9.81;

  ///////////////////////////////////////////////////////// initialize optimizer
  Opti opti = Opti();
  MX pos = opti.variable(2, horizon + 1);      // x and z
  MX vel_pol = opti.variable(2, horizon + 1);  // polar: v_abs and v_arg
  MX vel_cart = opti.variable(2, horizon + 1); // cartesian: v_x and v_z
  MX u = opti.variable(3, horizon);            // thurst, alpha and grip angle
  MX angleOfAttack = opti.variable(1, horizon + 1); // angle of attack
  MX gripper_pos = opti.variable(2, horizon + 1);   // gripper position
  MX lift = opti.variable(1, horizon + 1);
  MX drag = opti.variable(1, horizon + 1);
  MX thrust = opti.variable(1, horizon + 1);

  // cost function (TODO does not alway converge -> correct tomorrow)
  MX J = 0;
  gripper_pos(0, 0) = pos(0, 0) + Model::arm_length * cos(u(1, 0) - u(2, 0));
  gripper_pos(1, 0) = pos(1, 0) + Model::arm_length * sin(u(1, 0) - u(2, 0));
  for (int i = 1; i < horizon; ++i) {
    //  J += weight * sumsqr(pos(Slice(), i + 1) - pos_ref) + weight / 5 *
    //  sumsqr(vel_pol(Slice(), i + 1) - vel_ref);
    // J += sumsqr(gripper_pos(Slice(), i + 1) - obj_pos) + sumsqr(pos(Slice(),
    // i + 1) - pos_ref); J += sumsqr(pos(Slice(), i + 1) - pos_ref);
    gripper_pos(0, i) = pos(0, i) + Model::arm_length * cos(u(1, i) - u(2, i));
    gripper_pos(1, i) = pos(1, i) + Model::arm_length * sin(u(1, i) - u(2, i));

    // WORKS:
    // J += (sumsqr(gripper_pos(Slice(), i) - obj_pos)) *
    //      (25 - pos(0, i) * pos(0, i));

    // try:
    MX J_grip = (sumsqr(gripper_pos(Slice(), i) - obj_pos)) *
                (25 - pos(0, i) * pos(0, i));
    MX J_post = sumsqr(pos(Slice(), i) - pos_ref) +
                20 * sumsqr(u(2, i) - 140.0 * M_PI / 180);

    J += if_else(pos(0, i) < obj_pos(0) + Model::arm_length, J_grip, J_post);

    // MX J_grip = i * i *
    //             ((200 * (gripper_pos(1, i) - obj_pos(1)) *
    //               (gripper_pos(1, i) - obj_pos(1))) +
    //              sumsqr(gripper_speed));
    // NOT BAD
    // MX J_grip = i * i *
    //             ((60 * (gripper_pos(0, i) - obj_pos(0)) *
    //               (gripper_pos(0, i) - obj_pos(0))) +
    //              (600 * (gripper_pos(1, i) - obj_pos(1)) *
    //               (gripper_pos(1, i) - obj_pos(1))) +
    //              sumsqr(gripper_speed));

    // MX J_grip = i * i * sumsqr(gripper_pos(Slice(), i) - obj_pos);

    // MX J_post = sumsqr(pos(Slice(), i) - pos_ref);
    //  J += if_else(gripper_pos(0, i) < obj_pos(0) - 1, J_pre,
    //               if_else(gripper_pos(0, i) < obj_pos(0) + 0.2, J_grip,
    //               J_post));
    //  J += //if_else(gripper_pos(0, i) < obj_pos(0) + Model::arm_length,
    //  J_grip,
    //               J_post);
  }

  opti.minimize(J);

  // system dynamics
  for (int i = 1; i <= horizon; ++i) {
    // calculate angle of attack
    opti.subject_to(angleOfAttack(0, i - 1) == u(1, i - 1) - vel_pol(1, i - 1));

    //  calculate lift, drag and thrust
    opti.subject_to(lift(0, i - 1) ==
                    Model::lift(angleOfAttack(0, i - 1),
                                vel_pol(0, i - 1) * vel_pol(0, i - 1)));

    opti.subject_to(drag(0, i - 1) ==
                    Model::drag(angleOfAttack(0, i - 1),
                                vel_pol(0, i - 1) * vel_pol(0, i - 1)));

    opti.subject_to(thrust(0, i - 1) ==
                    Model::thrust(u(0, i - 1), vel_pol(0, i - 1)));

    // polar cartesian
    opti.subject_to(vel_cart(0, i - 1) ==
                    vel_pol(0, i - 1) * cos(vel_pol(1, i - 1)));
    opti.subject_to(vel_cart(1, i - 1) ==
                    vel_pol(0, i - 1) * sin(vel_pol(1, i - 1)));

    //  position update
    opti.subject_to(pos(0, i) == pos(0, i - 1) + dt * vel_cart(0, i - 1));
    opti.subject_to(pos(1, i) == pos(1, i - 1) + dt * vel_cart(1, i - 1));
    // velocity update
    opti.subject_to(
        vel_cart(0, i) ==
        vel_cart(0, i - 1) -
            dt * (thrust(0, i - 1) * cos(u(1, i - 1)) - drag(0, i - 1)) /
                Model::mass);
    opti.subject_to(
        vel_cart(1, i) ==
        vel_cart(1, i - 1) +
            dt * ((thrust(0, i - 1) * sin(u(1, i - 1)) + lift(0, i - 1)) /
                      Model::mass -
                  g));
  }
  // constraints
  for (int i = 0; i < horizon; ++i) {
    // inputs
    opti.subject_to(u(0, i) < u0_MAX);
    opti.subject_to(u(0, i) > u0_MIN);

    opti.subject_to(u(1, i) < u1_MAX);
    opti.subject_to(u(1, i) > u1_MIN);

    opti.subject_to(u(2, i) < u2_MAX);
    opti.subject_to(u(2, i) > u2_MIN);

    // constrain input change
    if (i > 0) {
      opti.subject_to(opti.bounded(-delta_u1, u(1, i) - u(1, i - 1), delta_u1));
      opti.subject_to(opti.bounded(-delta_u2, u(2, i) - u(2, i - 1), delta_u2));
    }
    // angle of attack
    opti.subject_to(angleOfAttack(0, i + 1) > aoA_MIN);
    opti.subject_to(angleOfAttack(0, i + 1) < aoA_MAX);

    // velocities
    opti.subject_to(opti.bounded(V_ABS_MIN, vel_pol(0, i + 1), V_ABS_MAX));
    opti.subject_to(opti.bounded(V_ARG_MIN, vel_pol(1, i + 1), V_ARG_MAX));
  }

  // initial state
  opti.subject_to(pos(Slice(), 0) == pos_0);
  opti.subject_to(vel_pol(Slice(), 0) == vel_0);

  opti.set_initial(pos(Slice(), 0), pos_0);
  opti.set_initial(vel_pol(Slice(), 0), vel_0);

  ////////////////////////////////////////////////////////////////////// solving
  // suppress ipopt console output?
  Dict opts_dict = Dict();
  opts_dict["ipopt.max_iter"] = 100000;
  if (debug_level < 3) {
    opts_dict["ipopt.print_level"] = 0;
    opts_dict["ipopt.sb"] = "yes";
    opts_dict["print_time"] = 0;
    opti.solver("ipopt", opts_dict);
  } else {
    opti.solver("ipopt", opts_dict);
  }

  // solve
  auto sol = opti.solve();

  // debug
  if (debug_level > 1) {
    std::cout << sol.value(pos) << '\n';
    std::cout << sol.value(u) << '\n';
  }

  for (int i = 0; i < horizon; i++) {
    DM solution = sol.value(u)(Slice(), i);
    std::vector<double> sol = DM::densify(solution).nonzeros();
    u_opt.at(0).at(i) = sol.at(0);
    u_opt.at(1).at(i) = sol.at(1);
    u_opt.at(2).at(i) = sol.at(2);
  }

  // std::cout << path << std::endl;
  for (int i = 0; i < horizon; i++) {
    std::vector<double> x0_vec = x0(Slice(), 0).nonzeros();
    full_log << x0_vec.at(1) << "," << x0_vec.at(2) << "," << i << ",";
    // optimal inputs
    std::vector<double> sol_vec =
        DM::densify(sol.value(u)(Slice(), i)).nonzeros();
    full_log << sol_vec.at(0) << "," << sol_vec.at(1) << "," << sol_vec.at(2)
             << ",";

    // states (position)
    std::vector<double> pos_vec =
        DM::densify(sol.value(pos)(Slice(), i)).nonzeros();
    full_log << pos_vec.at(0) << "," << pos_vec.at(1) << ",";

    // states (velocity)
    std::vector<double> vel_pol_vec =
        DM::densify(sol.value(vel_pol)(Slice(), i)).nonzeros();
    full_log << vel_pol_vec.at(0) << "," << vel_pol_vec.at(1) << ",";

    // states (velocity)
    std::vector<double> vel_cart_vec =
        DM::densify(sol.value(vel_cart)(Slice(), i)).nonzeros();
    full_log << vel_cart_vec.at(0) << "," << vel_cart_vec.at(1) << ",";
    // states (gripper_pos)
    std::vector<double> gripper_pos_vec =
        DM::densify(sol.value(gripper_pos)(Slice(), i)).nonzeros();
    full_log << gripper_pos_vec.at(0) << "," << gripper_pos_vec.at(1) << ",";
    // angle of attack
    std::vector<double> aoa_vec =
        DM::densify(sol.value(angleOfAttack)(Slice(), i)).nonzeros();
    // double aoa = (180 / M_PI) * std::atan2(vel_vec.at(1), vel_vec.at(0));
    full_log << aoa_vec.at(0) << "\n";

    // LOG FOR THE CONTROLLER
    ctrl_log << sol_vec.at(0) << "," << sol_vec.at(1) << "," << sol_vec.at(2)
             << "," << pos_vec.at(0) << "," << pos_vec.at(1) << ","
             << vel_pol_vec.at(0) << "\n";
  }
}