add_executable(mpc_plane src/mpc_plane.cpp)

target_include_directories(mpc_plane PUBLIC include)

target_link_libraries(mpc_plane PRIVATE 
  casadi
  mpc  
)