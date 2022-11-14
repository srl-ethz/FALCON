cmake_minimum_required(VERSION 3.16)
project(quad_control VERSION 0.1)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find requirements
find_package(MAVSDK REQUIRED)
find_package(casadi REQUIRED)
#find_package (Eigen3 3.3 REQUIRED NO_MODULE)

add_subdirectory(apps)
#add_subdirectory(external)
add_subdirectory(lib)

