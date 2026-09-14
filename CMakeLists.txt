cmake_minimum_required(VERSION 3.16.3)

project(dart_guide)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_BUILD_TYPE Debug)
message(STATUS "--------------------CMAKE_BUILD_TYPE: ${CMAKE_BUILD_TYPE}--------------------")

find_package(OpenCV REQUIRED)
find_package(fmt REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(spdlog REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(nlohmann_json REQUIRED)
set(OpenVINO_DIR "/opt/intel/openvino_2024.6.0/runtime/cmake/")
find_package(OpenVINO REQUIRED)

include_directories(${EIGEN3_INCLUDE_DIR})
include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(${PROJECT_SOURCE_DIR})


add_subdirectory(tools)
add_subdirectory(io)
add_subdirectory(tasks/light_detect)
add_subdirectory(tasks/stereo)
add_subdirectory(calibration)


####################src###################
add_executable(standard src/main.cpp)


target_link_libraries(standard ${OpenCV_LIBS} fmt::fmt yaml-cpp tools io light_detect stereo)

add_executable(hsv_tuner tools/hsv_tuner.cpp)
# hsv_tuner needs IO, tools, and their dependencies (fmt, spdlog)
target_link_libraries(hsv_tuner ${OpenCV_LIBS} fmt::fmt spdlog::spdlog yaml-cpp tools io)
