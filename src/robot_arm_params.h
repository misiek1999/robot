//
// Created by Ja on 16.05.2022.
//
/*
 * This file include all constance varaibles of robot arm like. lenght, motor params etc.
 * Robot arm structure is RRR
 */
#ifndef ROBOT_ROBOT_ARM_PARAMS_H
#define ROBOT_ROBOT_ARM_PARAMS_H
// Include libraries
#include "atomic"

// Manipulator position in cartesian system
struct Manipulator_position{
    float x;    // x position in cartesian system [cm]
    float y;    // y position in cartesian system [cm]
    float z;    // z position in cartesian system [cm]
};

// Robot joint definition
#define NUMBER_OF_ROBOT_JOINT 3 // Number of robot joints
typedef float robot_joint_position_t [NUMBER_OF_ROBOT_JOINT]; //Joint position array, unit in deg or cm, depends on joint type
// Array with min and max value for each joint
const float ROBOT_JOINT_LIMIT_ARRAY[NUMBER_OF_ROBOT_JOINT][2] =
        {{-90, 90}, {-90, 90}, {-90, 90}};
typedef std::uint8_t robot_digital_data_t;                    // Robot universal input/output digital data type
typedef std::atomic <robot_digital_data_t> robot_binary_interface_t; // Atomic robot digital interface
// First joint
#define ARM_1_LENGTH 12.0f          // Length of first arm of robot [cm]
#define MOTOR_SPEED_DEG 2.83f     // Speed of first motor [ms/deg]

// Second joint
#define ARM_2_LENGTH 10.0f          // Length of second arm of robot [cm]

// Third joint
#define ARM_3_LENGTH 10.0f          // Length of third arm of robot [cm]

// Tolerance of error robot joint positions
#define ROBOT_JOINT_POSITION_TOLERANCE 0.5f
// Tolerance of error robot manipulator positions
#define ROBOT_MANIPULATOR_POSITION_TOLERANCE 0.1f

#endif //ROBOT_ROBOT_ARM_PARAMS_H
