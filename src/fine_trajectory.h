//
// Created by Ja on 15.06.2022.
//

#ifndef ROBOT_FINE_TRAJECTORY_H
#define ROBOT_FINE_TRAJECTORY_H
#include "math.h"
#include "calculate_robot_position.h"
#include "robot_arm_params.h"

/*
 * Interpol linear trajectory between current robot position and setpoint robot position
 * This method uses inverse robot kinematic
 */
void generate_fine_trajectory(robot_joint_position_t _setpoint, robot_joint_position_t * _next_position);






#endif //ROBOT_FINE_TRAJECTORY_H
