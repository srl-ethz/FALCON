add_executable(mission src/mission.cpp )



target_include_directories(mission PUBLIC include)


target_link_libraries(mission PRIVATE
MAVSDK::mavsdk
)
