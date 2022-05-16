cmake_minimum_required(VERSION 3.21)
project(robot)

set(CMAKE_CXX_STANDARD 17)

add_executable(robot main.cpp robot_arm_params.h watchdog.cpp watchdog.h controll.cpp controll.h logger.cpp logger.h)
