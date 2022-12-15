cmake_minimum_required(VERSION 3.16)
project(falcon VERSION 0.1)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

############ PARAMETERS
#only turn this ON if you have ROS2 installed and build with colcon
set(ROS2_BUILD ON)

#only turn this ON if you are using openCV
set(OPENCV_BUILD OFF)

#turn this on if you are using casadi (traj gen)
set(CASADI_BUILD OFF)

#turn this on if you are using vicon (gripper_mocap_test)
set(VICON_BUILD OFF)

############ FIND DEPENDENCIES
find_package(MAVSDK REQUIRED)
if(CASASADI_BUILD)
    find_package(casadi REQUIRED)
endif()

#ROS dependencies
# DEPENDENCIES
if(ROS2_BUILD)
    find_package(ament_cmake REQUIRED)
    find_package(rclcpp REQUIRED)
    find_package(rclcpp_action REQUIRED) # TODO
    find_package(std_msgs REQUIRED)
    find_package(geometry_msgs REQUIRED)
    find_package(std_srvs REQUIRED)
    find_package(raptor_interface REQUIRED)
endif()

if(OPENCV_BUILD)
    find_package( OpenCV REQUIRED )
endif()

add_subdirectory(apps)
#add_subdirectory(external)
add_subdirectory(lib)

if(ROS2_BUILD)
    ament_package()
endif()

