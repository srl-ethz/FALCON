add_executable(grasper src/grasper.cpp)

target_include_directories(grasper PUBLIC include)

target_link_libraries(grasper PRIVATE 
  MAVSDK::mavsdk
)