//
// Created by Ja on 16.05.2022.
//

#ifndef ROBOT_CALCULATE_ROBOT_POSITION_H
#define ROBOT_CALCULATE_ROBOT_POSITION_H
// Include liblares
#include "math.h"
#include "mutex"
#include "robot_arm_params.h"   // Constance of robot arm
#include "controll_interface.h"           // Load global variables
/*
 * Function to calculate inverse robot kinematic of given robot
 * Depending on the robot configuration used, change the function code
 */
int calculate_inverse_robot_kinematics(Manipulator_position cartesian_position,
                                        robot_joint_position_t calculated_joint_position);

/*
 * Function to calculate simple robot kinematic of given robot
 * Depending on the robot configuration used, change the function code
 */
void calculate_simple_robot_kinematics(Manipulator_position &cartesian_position,
                                       robot_joint_position_t joint_position);

#endif //ROBOT_CALCULATE_ROBOT_POSITION_H
