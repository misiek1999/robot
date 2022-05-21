//
// Created by Ja on 16.05.2022.
//
// File include function to communicate with robot or simulator via Internet
// UDP protocol is used in this implementation
#include "controll_interface.h"

// Define posix variables used to communicate with robot




//TODO: delete below variables
// Robot position & coresponding mutex's
//extern robot_joint_position_t setpoint_robot_position;
//extern robot_joint_position_t current_robot_position;
/*
 * Global variable to check is manipulator reach setpoint position
 * False -when position is not rached, true - position reached
*/
//extern bool is_manipulator_reach_setpoint_position;

//// mutex's
//std::mutex setpoint_robot_position_mutex;
//std::mutex current_robot_position_mutex;
//std::mutex is_manipulator_reach_setpoint_position_mutex;

// Initialize communication with robot or simulator
void initialize_robot_communication(){



}

// Read robot joint position and controll signals
void read_robot_position(){




}

// Write robot position to reach
void write_robot_position(){




}

// Write and read robot position and controll signal;
void communicate_with_robot() {
    // Initialize communication with robot
    initialize_robot_communication();
    // Enter to infinite loop
    //TODO: implement this function


}