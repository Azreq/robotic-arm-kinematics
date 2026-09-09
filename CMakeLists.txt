cmake_minimum_required(VERSION 3.16)
project(RoboticArmKinematics VERSION 1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

include_directories(include)

file(GLOB SOURCES "src/*.cpp")

add_executable(robotic_arm ${SOURCES})

find_package(raylib REQUIRED)
target_link_libraries(robotic_arm PRIVATE raylib)
