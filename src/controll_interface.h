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
#include "stdio.h"
#include "unistd.h"
#include "sys/types.h"
#include "sys/socket.h"
#include "memory.h"
#include "supervisor.h"
#include "iostream"
#include "netinet/in.h"
#include "netinet/udp.h"
#include "arpa/inet.h"
#include "logger.h"
#include "console_interface.h"
#include "signal.h"

// period of time between communication with the robot [ms]
#define CONTROL_TIME_PERIOD 20

/*
 * Global variable to check is manipulator reach setpoint position
 * False -when position is not rached, true - position reached
*/

// Define global structure to communicate with robot
// Data which is send to robot
struct PacketToSend{
    robot_joint_position_t setpoint_position;           // Current robot position in each joint
    robot_binary_interface_t send_digital_signals;      // Sended robot digital signals
};
// Data which is received from robot
struct PacketToReceive{
    robot_joint_position_t received_position;           // Setpoint robot position in each joint
    robot_binary_interface_t received_digital_signals;  // Received robot digital signals
};

/*
 * Run this function in a separate thread
 * it is required to wake up this thread using a timer and signals every 20ms
 * Use posix signal to wake up this thread
 * function use UDP protocol to communicate with robot/simulator
 */
void* communicate_with_robot(void* _arg_input);
// Write digital output to robot
void set_digital_output(const robot_digital_data_type _input);
// get digital output
robot_digital_data_type get_digital_output();
// read robot current joint position
void get_current_robot_position(robot_joint_position_t _current_position);
// write robot setpoint joint position
void write_setpoint_robot_position(const robot_joint_position_t _setpoint_position);
// get robot setpoint position
void get_setpoint_robot_position(robot_joint_position_t _setpoint_position);

#endif //ROBOT_CONTROLL_INTERFACE_H
