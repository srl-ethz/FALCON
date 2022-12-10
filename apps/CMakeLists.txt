add_subdirectory(app_actuator_test)
add_subdirectory(app_flight_logger)
add_subdirectory(app_mission)
add_subdirectory(app_gripper_test)
add_subdirectory(app_multithread_test)
add_subdirectory(app_grasper)

if(OPENCV_BUILD)
    add_subdirectory(app_video_writer_test)
endif()

if(ROS2_BUILD)
    add_subdirectory(app_grasper_sim)
endif()

if(VICON_BUILD)
    add_subdirectory(app_gripper_mocap_test)
endif()

if(CASADI_BUILD)
    add_subdirectory(app_traj_gen)
    #add_subdirectory(app_mpc_plane)
endif()