//
// Created by Ja on 16.05.2022.
//
/*
 * This file include all constance varaibles of robot arm like. lenght, motor params etc.
 * Robot arm structure is RRR
 */
#ifndef ROBOT_ROBOT_ARM_PARAMS_H
#define ROBOT_ROBOT_ARM_PARAMS_H
// Include liblaries
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
typedef std::uint8_t robot_digital_data_type;                    // Robot universal input/output digital data type
typedef std::atomic <robot_digital_data_type> robot_binary_interface_t; // Atomic robot digital interface
// First joint
#define ARM_1_LENGTH 12.0f          // Length of first arm of robot [cm]
#define MOTOR_1_SPEED_DEG 2.83f     // Speed of first motor [ms/deg]
#define MOTOR_1_MIN_DEG -90.0f      // Minimal angle of motor 1
#define MOTOR_1_MAX_DEG 90.0f       // Maximum angle of motor 1
#define MOTOR_1_OFFSET 90.0f        // Offset of motor 1

// Second joint
#define ARM_2_LENGTH 10.0f          // Length of second arm of robot [cm]

// Third joint
#define ARM_3_LENGTH 8.0f          // Length of third arm of robot [cm]

#endif //ROBOT_ROBOT_ARM_PARAMS_H
