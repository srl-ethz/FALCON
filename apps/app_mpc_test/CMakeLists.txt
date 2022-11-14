add_executable(mpc_test src/mpc_test.cpp)

target_link_libraries(mpc_test PRIVATE 
  casadi
  mpc  
)