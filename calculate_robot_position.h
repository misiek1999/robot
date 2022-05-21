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

void calculate_inverse_robot_kinematics(Manipulator_position setpoint_manipulator_position,
                                        robot_joint_position_t designated_control);
//TODO: Implement function below
void interpolate_robot_trajectory();

#endif //ROBOT_CALCULATE_ROBOT_POSITION_H
