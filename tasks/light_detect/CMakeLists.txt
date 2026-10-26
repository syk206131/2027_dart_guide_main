cmake_minimum_required(VERSION 3.16)

add_library(light_detect OBJECT 
    Detect.cpp
)

target_link_libraries(light_detect ${OpenCV_LIBS} tools)
