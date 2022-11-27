add_executable(traj_gen src/traj_gen.cpp)

target_link_libraries(traj_gen PRIVATE 
  casadi
  mpc  
)