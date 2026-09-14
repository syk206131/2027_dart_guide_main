cmake_minimum_required(VERSION 3.16)

add_library(stereo STATIC stereo_range.cpp)
target_link_libraries(stereo ${OpenCV_LIBS} yaml-cpp)
