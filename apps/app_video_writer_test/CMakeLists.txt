add_executable(video_writer_test src/video_writer_test.cpp)


include_directories( ${OpenCV_INCLUDE_DIRS} )
target_link_libraries( video_writer_test ${OpenCV_LIBS} )
