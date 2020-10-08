cmake_minimum_required(VERSION 3.17)
project(myslam)

set(CMAKE_CXX_STANDARD 17)


# 如果有ccache则使用ccache加速编译
FIND_PROGRAM(CCACHE_FOUND ccache)
IF (CCACHE_FOUND)
    SET_PROPERTY(GLOBAL PROPERTY RULE_LAUNCH_COMPILE ccache)
    SET_PROPERTY(GLOBAL PROPERTY RULE_LAUNCH_LINK ccache)
    MESSAGE("-- Use ccache for compiler")
ENDIF ()

list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake)

find_package(OpenCV 4 REQUIRED)
include_directories(${OpenCV_INCLUDE_DIR})

find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})

find_package(Sophus REQUIRED)
include_directories(${Sophus_INCLUDE_DIRS})

find_package(G2O REQUIRED)
include_directories(${G2O_INCLUDE_DIRS})

find_package(Cholmod)
include_directories(${CHOLMOD_INCLUDE_DIR})

find_package(Boost REQUIRED system timer)

add_subdirectory(MindVision)

add_executable(myslam main.cpp)
target_link_libraries(myslam ${OpenCV_LIBS} ${CHOLMOD_LIBRARIES} mindvision Boost::system Boost::timer)
target_link_libraries(myslam ${G2O_CORE_LIBRARY} ${G2O_STUFF_LIBRARY} ${G2O_TYPES_SBA})
