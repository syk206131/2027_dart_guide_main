cmake_minimum_required(VERSION 3.16)

add_library(calibration STATIC
  calibration.cpp
  stereo_calibration.cpp
)
target_link_libraries(calibration PUBLIC ${OpenCV_LIBS})
target_include_directories(calibration PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})

add_executable(calibration_tool main.cpp)
target_link_libraries(calibration_tool PRIVATE calibration ${OpenCV_LIBS})

# Stereo pair capture: needs the live camera stack and the frame synchronizer.
add_executable(stereo_capture stereo_capture.cpp)
target_link_libraries(
  stereo_capture PRIVATE ${OpenCV_LIBS} fmt::fmt spdlog::spdlog yaml-cpp tools io)

# Stereo calibration from saved synchronized pairs.
add_executable(stereo_calibration_tool stereo_main.cpp)
target_link_libraries(
  stereo_calibration_tool PRIVATE calibration ${OpenCV_LIBS} yaml-cpp)

# Optical axis alignment aid: live cameras, frame sync, and the light detector.
add_executable(align_tool align_main.cpp)
target_link_libraries(
  align_tool PRIVATE ${OpenCV_LIBS} fmt::fmt spdlog::spdlog yaml-cpp tools io light_detect)
