//
// Created by Ja on 16.05.2022.
//
/*
 * File include interface to communicate with raspberry and visualisation,
 * Global variables with robot position and setpoint position, alarms etc.
 */
#ifndef ROBOT_CONTROLL_INTERFACE_H
#define ROBOT_CONTROLL_INTERFACE_H

// Include liblaries
#include "mutex"
#include "thread"
#include "atomic"
#include "robot_arm_params.h"
#include "stdlib.h"
#include "unistd.h"
#include "sys/types.h"
#include "sys/socket.h"


// Global controll singals


// Robot position & coresponding mutex's
extern robot_joint_position_t setpoint_robot_position;
extern std::mutex setpoint_robot_position_mutex;

extern robot_joint_position_t current_robot_position;
extern std::mutex current_robot_position_mutex;

// Binary robot output and input
extern robot_binary_interface_t output_binary;
extern robot_binary_interface_t input_binary;

/*
 * Global variable to check is manipulator reach setpoint position
 * False -when position is not rached, true - position reached
*/
// TODO: Zmienić ten mechanizm na ten z zajęć 8
extern std::atomic<bool> is_manipulator_reach_setpoint_position;




// TODO: change below implmentation to diffrent method,
// Check if you are compiling on a raspberry pi or pc
#ifdef __arm__


#endif

// Define global structure to communicate with robot
// Data which is send to robot
struct packet_to_send{
    robot_joint_position_t setpoint_position;           // Current robot position in each joint
    robot_binary_interface_t send_digital_signals;      // Sended robot digital signals
};
// Data which is received from robot
struct packet_to_receive{
    robot_joint_position_t received_position;           // Setpoint robot position in each joint
    robot_binary_interface_t received_digital_signals;  // Received robot digital signals
};

// Use ethernet to communicate with robot
// Initialize communication with robot or simulator
void initialize_robot_communication();
// Read robot joint position and controll signals
void read_robot_position();
// Write robot position to reach
void write_robot_position();
// Write and read robot position and controll signal;
void communicate_with_robot();

#endif //ROBOT_CONTROLL_INTERFACE_H
