cmake_minimum_required(VERSION 3.16)
project(robot)

set(CMAKE_CXX_STANDARD 17)

add_executable(robot main.cpp robot_arm_params.h controll_interface.cpp controll_interface.h logger.cpp logger.h calculate_robot_position.cpp calculate_robot_position.h supervisor.cpp supervisor.h console_interface.cpp console_interface.h trajectory_generator.cpp trajectory_generator.h)
