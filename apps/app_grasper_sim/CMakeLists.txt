add_executable(grasper_sim src/grasper_sim.cpp)
#ROS dependencies
ament_target_dependencies(grasper_sim
  rclcpp 
  std_srvs 
  geometry_msgs
  raptor_interface)

target_include_directories(grasper_sim PUBLIC include)

#other dependencies
target_link_libraries(grasper_sim MAVSDK::mavsdk)

install(TARGETS
  grasper_sim
  DESTINATION lib/${PROJECT_NAME})
