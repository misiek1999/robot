//
// Created by Ja on 15.06.2022.
//

#ifndef ROBOT_FINE_TRAJECTORY_H
#define ROBOT_FINE_TRAJECTORY_H
#include "math.h"
#include "calculate_robot_position.h"
#include "robot_arm_params.h"
#include "controll_interface.h"

/*
 * Interpol linear trajectory between current robot position and setpoint robot position
 * This method uses inverse robot kinematic
 * Return 0 if trajectory was created successful
 * If returned value is higher than 0 check error code table in fine_trajectory.cpp
 */
int generate_fine_trajectory(robot_joint_position_t _next_position, robot_joint_position_t _setpoint, float _linear_speed);

/*
 * Check speed limit in each joint
 * Return true if joint speed is below limit
 */
bool check_joint_speed_limit(robot_joint_position_t _curr, robot_joint_position_t _req);

#endif //ROBOT_FINE_TRAJECTORY_H
