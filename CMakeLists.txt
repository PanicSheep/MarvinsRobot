cmake_minimum_required(VERSION 3.10)
project(MarvinsRobot)

set(CMAKE_CXX_STANDARD 20)

include_directories(MarvinsRobot)

add_subdirectory(MarvinsRobot)
add_subdirectory(Tests)
add_subdirectory(googletest)
