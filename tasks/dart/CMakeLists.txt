cmake_minimum_required(VERSION 3.16)

add_library(dart STATIC dart_calculator.cpp)
target_include_directories(dart PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(dart stereo ${OpenCV_LIBS} yaml-cpp)
