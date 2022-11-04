add_executable(flight_logger src/flight_logger.cpp )

target_include_directories(flight_logger PUBLIC include)

target_link_libraries(flight_logger PRIVATE
MAVSDK::mavsdk
)
