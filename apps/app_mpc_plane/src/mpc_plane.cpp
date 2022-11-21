#include <iostream>
#include <thread>
#include <vector>
#include <chrono>
#include <mutex>

#include "casadi/casadi.hpp"
#include "mpc.h"

using namespace casadi;

// global unprotected variables
int n_opt = 0;
int n_ctrl = 0;

// global read variables
const double time_horizon = 2.0;
const double dt = 0.10;
const int iterations = int(time_horizon / dt);
const int n_initial_conditions = 8;
const int n_inputs = 3;
const int debug_level = 0;

// a global instance of std::mutex to protect global variable
std::mutex myMutex;

// values that are accessed by multiple threads and need to be guarded by mutex
std::vector<double> initial_condition;
std::vector<std::vector<double>> optimal_controls;
bool new_opt = false;

// generate reference
MX pos_ref = MX(2, 1);
MX obj_ref = MX(2, 1);
MX vel_ref = MX(2, 1);

void optimizer()
{
    // read initial conditions (Mutex)
    std::vector<double> x0;
    {
        std::lock_guard<std::mutex> guard(myMutex);
        x0 = initial_condition;
    }

    // prepare values for optimizer
    std::vector<std::vector<double>> u_opt(n_inputs, std::vector<double>(iterations));

    // set initial state
    DM pos_0 = DM::zeros(2, 1);
    DM vel_0 = DM::zeros(2, 1);
    DM alpha_0 = DM::zeros(1, 1);
    DM beta_0 = DM::zeros(1, 1);

    pos_0(0, 0) = x0.at(0);   //-5;
    pos_0(1, 0) = x0.at(1);   // 6;
    vel_0(0, 0) = x0.at(2);   // 8;
    vel_0(1, 0) = x0.at(3);   // 0;
    alpha_0(0, 0) = x0.at(4); // 0.1;
    beta_0(0, 0) = x0.at(5);
    // do optimization
    AttitudeMPC::doControlStep(u_opt, pos_ref, obj_ref, vel_ref, pos_0, vel_0, alpha_0, beta_0, time_horizon, dt, debug_level, "real");

    n_opt++;

    // write optimal control inputs (Mutex)
    {
        std::lock_guard<std::mutex> guard(myMutex);
        optimal_controls = u_opt;
    }
}

void controller()
{
    // read optimal control inputs (Mutex)
    std::vector<std::vector<double>> u(n_inputs, std::vector<double>(iterations));
    {
        std::lock_guard<std::mutex> guard(myMutex);
        u = optimal_controls;
    }

    // do controls
    bool loop = true;
    for (int i = 0; loop == true; i++)
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(int(1000 * dt)));
        /*

        PUT CONTROL INPUTS HERE

        */
        std::cout << "control: " << u.at(0).at(i) << " , " << u.at(1).at(i) << " , " << u.at(2).at(i) << std::endl;

        n_ctrl++; // counting variable for debugging

        // check if new optimal control values available
        {
            std::lock_guard<std::mutex> guard(myMutex);
            if (new_opt == true)
            {
                loop = false;
            }
        }
    }

    // write current values into inital condition vector for next optimization problem
    std::vector<double> x0(n_initial_conditions);
    for (int i = 0; i < n_initial_conditions; i++)
    {
        x0.at(i) = n_ctrl;
    }

    // write initial conditions (Mutex)
    {
        std::lock_guard<std::mutex> guard(myMutex);
        initial_condition = x0;
    }
}

int main()
{
    // set reference position

    pos_ref(0, 0) = 5;
    pos_ref(1, 0) = 9;
    vel_ref(0, 0) = 8;
    vel_ref(1, 0) = 0;
    obj_ref(0, 0) = 0;
    obj_ref(1, 0) = 4;

    // get first initial condition
    std::vector<double> initial_condition(n_initial_conditions);
    for (int i = 0; i < n_initial_conditions; i++)
    {
        initial_condition.at(i) = 12;
    }
    // run first optimization
    optimizer();

    // start multithreating
    while (true)
    {
        new_opt = false; // set new optimization available to false
        n_ctrl = 0;      // Set count value to zero

        // Spawn threads
        std::thread optimization_thread(optimizer);
        std::thread control_thread(controller);
        // wait for optimization to finish
        optimization_thread.join();
        // update new_opt to tell control thread that new optimal control inputs are available
        {
            std::lock_guard<std::mutex> guard(myMutex);
            new_opt = true;
        }
        // this results in control thread to end.
        control_thread.join();
        std::cout << "control steps: " << n_ctrl << std::endl;
    }

    return 0;
}
