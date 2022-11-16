// header file
#include "mpc.h"
#include <math.h> /* atan2 */

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

void AttitudeMPC::doControlStep(std::vector<std::vector<double>> &u_opt,
                                casadi::MX &pos_ref,
                                casadi::MX &vel_ref,
                                casadi::DM &pos_0,
                                casadi::DM &vel_0,
                                casadi::DM &alpha_0,
                                double time,
                                int debug_level,
                                std::string file_name)
{
  /////////////////////////////////////////////////////////////////// parameters
  // input constraints
  const double u0_MAX = 1;                // [ - ] max throttle cmd
  const double u0_MIN = 0;                // [ - ] min throttle cmd
  const double u1_MAX = 30 * M_PI / 180;  // [ rad ] max pitch
  const double u1_MIN = -30 * M_PI / 180; // [ rad ] min pitch

  const double aoA_MAX = 20 * M_PI / 180; // [ rad ] max angle of attack
  const double aoA_MIN = 0;               // [ rad ] min angle of attack

  const double V_MAX = 15; // [ m/s ] max speed
  const double V_MIN = 6;  // [ m/s ] min speed

  // timstep (for now constant)
  double dt = 0.08;
  // determine horizon
  int horizon = time / dt;

  std::cout << horizon << std::endl;
  // gravity
  MX g = MX::zeros(2, 1);
  g(1) = -9.81;

  ///////////////////////////////////////////////////////// initialize optimizer
  Opti opti = Opti();
  MX pos = opti.variable(2, horizon + 1);           // x and z
  MX vel = opti.variable(2, horizon + 1);           // x_dot and z_dot
  MX u = opti.variable(2, horizon);                 // thurst and alpha
  MX angleOfAttack = opti.variable(1, horizon + 1); // angle of attack
  //  cost function
  MX J = 0;
  double weight = 100 / horizon;
  for (int i = 0; i < horizon; ++i)
  {
    weight = (i + 1) * 100 / horizon;
    J += weight * sumsqr(pos(Slice(), i + 1) - pos_ref) + weight / 5 * sumsqr(vel(Slice(), i + 1) - vel_ref);
  }
  opti.minimize(J);

  // constraints
  for (int i = 0; i < horizon; ++i)
  {
    // inputs
    opti.subject_to(opti.bounded(u0_MIN, u(0, i), u0_MAX));
    opti.subject_to(opti.bounded(u1_MIN, u(1, i), u1_MAX));

    // angle of attack
    opti.subject_to(opti.bounded(aoA_MIN, angleOfAttack(0, i), aoA_MAX));

    // velocities
    opti.subject_to(opti.bounded(V_MIN, vel(0, i + 1), V_MAX));
  }

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

    // PROBLEM : Calculate angle arctan( vz / vx )

    float epsilon = 0.0001;
    // angleOfAttack(i - 1) = 0.0 ;
    // MX angle_vel = 0;
    // if_else(fabs(vel(1, i - 1)) < epsilon, angle_vel = 0.01, angle_vel = atan2(vel(1, i - 1), vel(0, i - 1)));

    angleOfAttack(0, i - 1) = u(1, i - 1) - atan2(vel(1, i - 1), vel(0, i - 1));

    //  calculate lift, drag and thrust
    //  MX lift = Model::lift(aoA, speed_squared);
    //  MX drag = Model::drag(aoA, speed_squared);
    MX lift = Model::lift(angleOfAttack(0, i - 1), speed_squared);
    MX drag = Model::drag(angleOfAttack(0, i - 1), speed_squared);

    MX thrust = Model::thrust(u(0, i - 1));

    // calculate acceleration on plane
    // MX a = (thrust * x_loc + lift * z_loc - drag * x_loc) / Model::mass;
    MX a = (thrust * x_loc + 3 * lift * z_loc - drag * x_loc) / Model::mass;
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
  opti.subject_to(pos(Slice(), 0) == pos_0);
  opti.subject_to(vel(Slice(), 0) == vel_0);

  opti.set_initial(pos(Slice(), 0), pos_0);
  opti.set_initial(vel(Slice(), 0), vel_0);

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
  }

  std::ofstream file;
  std::string path = "lib/mpc/out/" + file_name + ".csv";
  file.open(path);
  file << "i,u0_opt,u1_opt,x,z,vx,vz,aoa,aoa_true\n";
  std::cout << path << std::endl;
  for (int i = 0; i < horizon; i++)
  {
    file << i << ",";
    // optimal inputs
    std::vector<double> sol_vec = DM::densify(sol.value(u)(Slice(), i)).nonzeros();
    file << sol_vec.at(0) << "," << sol_vec.at(1) << ",";

    // states (position)
    std::vector<double> pos_vec = DM::densify(sol.value(pos)(Slice(), i)).nonzeros();
    file << pos_vec.at(0) << "," << pos_vec.at(1) << ",";

    // states (velocity)
    std::vector<double> vel_vec = DM::densify(sol.value(vel)(Slice(), i)).nonzeros();
    file << vel_vec.at(0) << "," << vel_vec.at(1) << ",";

    // angle of attack
    std::vector<double> aoa_vec = DM::densify(sol.value(angleOfAttack)(Slice(), i)).nonzeros();
    // double aoa = (180 / M_PI) * std::atan2(vel_vec.at(1), vel_vec.at(0));
    file << (180 / M_PI) * aoa_vec.at(0) << "," << (180 / M_PI) * std::atan2(vel_vec.at(1), vel_vec.at(0)) << "\n";
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