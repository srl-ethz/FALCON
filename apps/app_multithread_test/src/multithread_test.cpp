#include <iostream>
#include <thread>
#include <list>
#include <vector>
#include <chrono>
#include <algorithm>
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
    // read protected
    std::vector<double> x0;
    {
        std::lock_guard<std::mutex> guard(myMutex);
        x0 = initial_condition;
    }

    // do optimization
    std::this_thread::sleep_for(std::chrono::milliseconds(rand() % (2001 - 500) + 500));
    n_opt++;
    std::vector<std::vector<double>> u(n_inputs, std::vector<double>(iterations));
    for (int x = 0; x < n_inputs; x++)
    {
        for (int y = 0; y < iterations; y++)
        {
            u.at(x).at(y) = n_opt * 10000 + 100 * x + y;
        }
    }

    // write protected
    {
        std::lock_guard<std::mutex> guard(myMutex);
        optimal_controls = u;
    }
}

void dummy_controller()
{
    // read protected
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
        std::cout << "control: " << u.at(0).at(i) << " , " << u.at(1).at(i) << std::endl;
        n_ctrl++;
        {
            std::lock_guard<std::mutex> guard(myMutex);
            if (new_opt == true)
            {
                loop = false;
            }
        }
    }

    std::vector<double> x0(n_initial_conditions);
    for (int i = 0; i < n_initial_conditions; i++)
    {
        x0.at(i) = n_ctrl;
    }

    // write protected
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
        new_opt = false;
        n_ctrl = 0;
        std::thread t1(dummy_optimizer);
        std::thread t2(dummy_controller);
        std::cout << "Done spawning threads. Now waiting for them to join:\n";
        t1.join();
        {
            std::lock_guard<std::mutex> guard(myMutex);
            new_opt = true;
        }
        t2.join();
        std::cout << "control steps: " << n_ctrl << std::endl;
    }

    // t1.join();
    // t2.join();
    // t3.join();

    return 0;
}