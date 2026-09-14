cmake_minimum_required(VERSION 3.16)

set(OpenVINO_DIR "/opt/intel/openvino_2024.6.0/runtime/cmake")

find_package(OpenVINO REQUIRED COMPONENTS Runtime)

add_library(light_detect OBJECT 
    Detect.cpp
    openvino/openvinoInfer.cpp
)

target_link_libraries(light_detect openvino::runtime tools)