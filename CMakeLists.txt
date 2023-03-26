cmake_minimum_required(VERSION 3.20)
project(ATS_23_Vision)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR})
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} --std=c++17 -Os -pthread -g")

find_package(OpenCV 4.5.0)
find_package(Eigen3 REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(PythonLibs REQUIRED)
find_package(matplotlib_cpp REQUIRED)
find_package(Ceres REQUIRED COMPONENTS EigenSparse)
find_package(fmt REQUIRED)
find_package(glog REQUIRED)

include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(${EIGEN3_INCLUDE_DIRS})

add_executable(ATS_23_Vision main.cpp
        ${PROJECT_SOURCE_DIR}/autoaim/autoaim.h
        autoaim/autoaim.cpp
        autoaim/autoaim.cpp
        autoaim/predictor/predictor.cpp
        autoaim/predictor/predictor.h
        autoaim/autoaim.h
        thread/thread.cpp
        thread/thread.h
        serial/CRC_Check.cpp
        serial/CRC_Check.h
        serial/serial.cpp
        serial/serial.h
        camera/DaHengCamera.cpp
        camera/DaHengCamera.h
        camera/DxImageProc.h
        camera/GxIAPI.h
        buff/buff.cpp buff/buff.h buff/fan_tracker.h buff/fan_tracker.cpp buff/predictor/predictor.cpp buff/predictor/predictor.h coordsolver/coordsolver.cpp coordsolver/coordsolver.h general/general.cpp general/general.h)

target_link_libraries(ATS_23_Vision ${OpenCV_LIBRARIES} ${InferenceEngine_LIBRARIES} ${Eigen3_LIBRARIES}
        yaml-cpp gxiapi Ceres::ceres fmt::fmt glog matplotlib_cpp::matplotlib_cpp)

