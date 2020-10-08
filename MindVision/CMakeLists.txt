cmake_minimum_required(VERSION 3.5)
project(mindvision)

set(CMAKE_CXX_STANDARD 17)

find_package(OpenCV 4 REQUIRED)

add_library(mindvision MindVision.cpp)
target_link_libraries(mindvision MVSDK ${OpenCV_LIBS})
