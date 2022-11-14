cmake_minimum_required(VERSION 3.0.0)
project(mpc VERSION 0.1.0)
set(CMAKE_CXX_STANDARD 17)


# create library
add_library(mpc src/mpc.cpp)
target_link_libraries(mpc PRIVATE casadi)

# include header file directory
target_include_directories(mpc PUBLIC include)
