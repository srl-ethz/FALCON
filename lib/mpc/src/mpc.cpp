// header file
#include "mpc.h"
#include <math.h> /* atan2 */

using namespace casadi;
using std::chrono::milliseconds;
using std::chrono::seconds;
using std::this_thread::sleep_for;

MX Model::drag(MX aoA_rad, MX airspeed_squared)
{
  return if_else(aoA_rad < 0, 0, 0.5 * Model::A_tot * Model::rho * airspeed_squared * (1.97950651 * pow(aoA_rad, 2) + 0.01655889));
}
MX Model::lift(MX aoA_rad, MX airspeed_squared)
{
  return if_else(aoA_rad < 0, 0, 0.5 * Model::A_tot * Model::rho * airspeed_squared * (0.44482359 * aoA_rad + 0.00245455));
}
MX Model::thrust(MX u_throttle)
{
  return 14.456 * pow(u_throttle, 2);
}

void AttitudeMPC::doControlStep(std::vector<std::vector<double>> &u_opt,
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
                                std::string file_name)
{
  /////////////////////////////////////////////////////////////////// parameters
  // input constraints
  const double u0_MAX = 1;                 // [ - ] max throttle cmd
  const double u0_MIN = 0;                 // [ - ] min throttle cmd
  const double u1_MAX = 30 * M_PI / 180;   // [ rad ] max pitch
  const double u1_MIN = -30 * M_PI / 180;  // [ rad ] min pitch
  const double delta_u1 = 10 * M_PI / 180; // max pitch change per timestep
  const double u2_MIN = 20 * M_PI / 180;
  const double u2_MAX = 140 * M_PI / 180;
  const double delta_u2 = 30 * M_PI / 180; // max arm angle change per timestep

  const double aoA_MAX = 20 * M_PI / 180; // [ rad ] max angle of attack
  const double aoA_MIN = 0;               // [ rad ] min angle of attack

  const double V_ABS_MAX = 10;               // [ m/s ] max speed
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
  MX pos = opti.variable(2, horizon + 1);           // x and z
  MX vel_pol = opti.variable(2, horizon + 1);       // polar: v_abs and v_arg
  MX vel_cart = opti.variable(2, horizon + 1);      // cartesian: v_x and v_z
  MX u = opti.variable(3, horizon);                 // thurst, alpha and grip angle
  MX angleOfAttack = opti.variable(1, horizon + 1); // angle of attack
  MX gripper_pos = opti.variable(2, horizon + 1);   // gripper position
  //    cost function
  MX J = 0;
  gripper_pos(0, 0) = pos(0, 0) + Model::arm_length * cos(u(1, 0) - u(2, 0));
  gripper_pos(1, 0) = pos(1, 0) + Model::arm_length * sin(u(1, 0) - u(2, 0));
  for (int i = 1; i < horizon; ++i)
  {
    //  J += weight * sumsqr(pos(Slice(), i + 1) - pos_ref) + weight / 5 * sumsqr(vel_pol(Slice(), i + 1) - vel_ref);
    // J += sumsqr(gripper_pos(Slice(), i + 1) - obj_pos) + sumsqr(pos(Slice(), i + 1) - pos_ref);
    // J += sumsqr(pos(Slice(), i + 1) - pos_ref);
    gripper_pos(0, i) = pos(0, i) + Model::arm_length * cos(u(1, i) - u(2, i));
    gripper_pos(1, i) = pos(1, i) + Model::arm_length * sin(u(1, i) - u(2, i));
    MX gripper_speed = (gripper_pos(Slice(), i) - gripper_pos(Slice(), i - 1)) / dt;
    MX J_pre = sumsqr(pos(Slice(), i) - obj_pos);
    MX J_grip = i * i * (200 * sumsqr(gripper_pos(Slice(), i) - obj_pos) + sumsqr(gripper_speed));
    MX J_post = sumsqr(pos(Slice(), i) - pos_ref);
    J += if_else(gripper_pos(0, i) < obj_pos(0) - 1, J_pre, if_else(gripper_pos(0, i) < obj_pos(0) + 0.2, J_grip, J_post));
  }

  opti.minimize(J);

  // system dynamics
  for (int i = 1; i <= horizon; ++i)
  {
    // calculate angle of attack
    angleOfAttack(0, i - 1) = u(1, i - 1) - vel_pol(1, i - 1);

    //  calculate lift, drag and thrust
    MX lift = Model::lift(angleOfAttack(0, i - 1), vel_pol(0, i - 1) * vel_pol(0, i - 1));
    MX drag = Model::drag(angleOfAttack(0, i - 1), vel_pol(0, i - 1) * vel_pol(0, i - 1));
    MX thrust = Model::thrust(u(0, i - 1));

    // calculate acceleration on plane
    MX a = MX::zeros(2, 1);
    a(0) = (thrust * cos(u(1, i - 1)) - drag) / Model::mass;
    a(1) = (thrust * sin(u(1, i - 1)) + lift - g) / Model::mass;

    vel_cart(0, i - 1) = vel_pol(0, i - 1) * cos(vel_pol(1, i - 1));
    vel_cart(1, i - 1) = vel_pol(0, i - 1) * sin(vel_pol(1, i - 1));

    //  position update
    opti.subject_to(
        pos(Slice(), i) ==
        pos(Slice(), i - 1) + dt * vel_cart(Slice(), i - 1));
    // velocity update
    opti.subject_to(
        vel_cart(Slice(), i) ==
        vel_cart(Slice(), i - 1) + dt * a);
  }
  // constraints
  for (int i = 0; i < horizon; ++i)
  {
    // inputs
    opti.subject_to(opti.bounded(u0_MIN, u(0, i), u0_MAX));
    opti.subject_to(opti.bounded(u1_MIN, u(1, i), u1_MAX));
    // opti.subject_to(opti.bounded(u2_MIN, u(2, i), u2_MAX));
    opti.subject_to(u(2, i) < u2_MAX);
    opti.subject_to(u(2, i) > u2_MIN);

    // constrain input change
    if (i > 0)
    {
      opti.subject_to(opti.bounded(-delta_u1, u(1, i) - u(1, i - 1), delta_u1));
      opti.subject_to(opti.bounded(-delta_u2, u(2, i) - u(2, i - 1), delta_u2));
    }

    // no negative position
    opti.subject_to(gripper_pos(1, i + 1) > obj_pos(1));

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

  for (int i = 0; i < horizon; i++)
  {
    DM solution = sol.value(u)(Slice(), i);
    std::vector<double> sol = DM::densify(solution).nonzeros();
    u_opt.at(0).at(i) = sol.at(0);
    u_opt.at(1).at(i) = sol.at(1);
    u_opt.at(2).at(i) = sol.at(2);
  }

  std::ofstream file;
  std::string path = "lib/mpc/out/" + file_name + ".csv";
  file.open(path);
  file << "i,u0_opt,u1_opt,u2_opt,x,z,v_abs,v_arg,vx,vz,gripx,gripz,aoa,aoa_true\n";
  std::cout << path << std::endl;
  for (int i = 0; i < horizon; i++)
  {
    file << i << ",";
    // optimal inputs
    std::vector<double> sol_vec = DM::densify(sol.value(u)(Slice(), i)).nonzeros();
    file << sol_vec.at(0) << "," << sol_vec.at(1) << "," << sol_vec.at(2) << ",";

    // states (position)
    std::vector<double> pos_vec = DM::densify(sol.value(pos)(Slice(), i)).nonzeros();
    file << pos_vec.at(0) << "," << pos_vec.at(1) << ",";

    // states (velocity)
    std::vector<double> vel_pol_vec = DM::densify(sol.value(vel_pol)(Slice(), i)).nonzeros();
    file << vel_pol_vec.at(0) << "," << vel_pol_vec.at(1) << ",";

    // states (velocity)
    std::vector<double> vel_cart_vec = DM::densify(sol.value(vel_cart)(Slice(), i)).nonzeros();
    file << vel_cart_vec.at(0) << "," << vel_cart_vec.at(1) << ",";
    // states (gripper_pos)
    std::vector<double> gripper_pos_vec = DM::densify(sol.value(gripper_pos)(Slice(), i)).nonzeros();
    file << gripper_pos_vec.at(0) << "," << gripper_pos_vec.at(1) << ",";
    // angle of attack
    std::vector<double> aoa_vec = DM::densify(sol.value(angleOfAttack)(Slice(), i)).nonzeros();
    // double aoa = (180 / M_PI) * std::atan2(vel_vec.at(1), vel_vec.at(0));
    file << aoa_vec.at(0) << "," << sol_vec.at(1) - std::atan2(vel_cart_vec.at(1), vel_cart_vec.at(0)) << "\n";
  }
  file.close();

  // u1_opt = DM::densify(sol.value(u)(1, Slice())).nonzeros();
  //  std::cout << solution << std::endl;
  //  optimal_input = DM::densify(solution).nonzeros();
  //  debug
  //  if (debug_level > 0)
  //  {
  //    std::cout << optimal_input << '\n';
  //  }
}