#include <iostream>
#include <thread>
#include <vector>
#include <chrono>
#include <mutex>

// global unprotected variables
int n_opt = 0;
int n_ctrl = 0;

// global read variables
const double horizon = 3.0;
const double dt = 0.05;
const int iterations = int(horizon / dt);
const int n_initial_conditions = 8;
const int n_inputs = 2;

std::mutex myMutex;
// a global instance of std::mutex to protect global variable
std::vector<double> initial_condition;
std::vector<std::vector<double>> optimal_controls;
bool new_opt = false;

void dummy_optimizer()
{
    // read initial conditions (Mutex)
    std::vector<double> x0;
    {
        std::lock_guard<std::mutex> guard(myMutex);
        x0 = initial_condition;
    }

    // do optimization
    std::this_thread::sleep_for(std::chrono::milliseconds(rand() % (2001 - 500) + 500));
    /*

    PUT OPTIMIZER HERE

    */

    // write optimal control inputs into vector
    n_opt++;
    std::vector<std::vector<double>> u(n_inputs, std::vector<double>(iterations));
    for (int x = 0; x < n_inputs; x++)
    {
        for (int y = 0; y < iterations; y++)
        {
            u.at(x).at(y) = n_opt * 10000 + 100 * x + y;
        }
    }

    // write optimal control inputs (Mutex)
    {
        std::lock_guard<std::mutex> guard(myMutex);
        optimal_controls = u;
    }
}

void dummy_controller()
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
        std::cout << "control: " << u.at(0).at(i) << " , " << u.at(1).at(i) << std::endl;

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

    // get first initial condition
    std::vector<double> initial_condition(n_initial_conditions);
    for (int i = 0; i < n_initial_conditions; i++)
    {
        initial_condition.at(i) = 12;
    }
    // run first optimization
    dummy_optimizer();

    // start multithreating
    while (true)
    {
        new_opt = false; // set new optimization available to false
        n_ctrl = 0;      // Set count value to zero

        // Spawn threads
        std::thread optimization_thread(dummy_optimizer);
        std::thread control_thread(dummy_controller);
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