add_executable(gripper_test src/gripper_test.cpp serialib/serialib.cpp)
target_include_directories(gripper_test PUBLIC serialib)