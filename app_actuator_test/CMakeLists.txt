add_executable(actuator_test src/actuator_test.cpp )



target_include_directories(actuator_test PUBLIC include)


target_link_libraries(actuator_test PRIVATE
MAVSDK::mavsdk
)
