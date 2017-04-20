project(UnscentedKF)

cmake_minimum_required (VERSION 3.5)

add_definitions(-std=c++0x)

set(sources
   src/Sensor/LidarSensor.cpp
   src/Sensor/RadarSensor.cpp
   src/ukf.cpp
   src/main.cpp
   src/tools.cpp)

add_executable(UnscentedKF ${sources})
