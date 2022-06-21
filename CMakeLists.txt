cmake_minimum_required(VERSION 3.16)
project(robot)

set(CMAKE_CXX_STANDARD 17)

add_executable(robot src/main.cpp src/robot_arm_params.h src/controll_interface.cpp src/controll_interface.h src/logger.cpp src/logger.h src/calculate_robot_position.cpp src/calculate_robot_position.h src/supervisor.cpp src/supervisor.h src/console_interface.cpp src/console_interface.h src/trajectory_generator.cpp src/trajectory_generator.h src/instruction_processor.cpp src/instruction_processor.h src/fine_trajectory.cpp src/fine_trajectory.h src/read_instruction_file.cpp src/read_instruction_file.h src/instruction_set.h)
target_link_libraries(robot pthread rt)