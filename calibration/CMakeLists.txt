cmake_minimum_required(VERSION 3.16)

add_library(calibration STATIC calibration.cpp)
target_link_libraries(calibration PUBLIC ${OpenCV_LIBS})
target_include_directories(calibration PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})

add_executable(calibration_tool main.cpp)
target_link_libraries(calibration_tool PRIVATE calibration ${OpenCV_LIBS})