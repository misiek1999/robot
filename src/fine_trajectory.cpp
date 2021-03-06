//
// Created by Ja on 15.06.2022.
//

#include "fine_trajectory.h"

// Returned values from function generate_fine_trajectory
#define TRAJECTORY_GENERATED_SUCCESSFUL 0
#define OVERRUN_JOINT_SPEED 1
#define ARM_POSITION_UNREACHABLE 2
#define UNDEFINED_ERROR -1

/*
 * Interpol linear trajectory between current robot position and setpoint robot position
 * This method uses inverse robot kinematic
 * Return 0 if trajectory was created successful
 * If returned value is higher than 0 check error code table in fine_trajectory.cpp
 */
int generate_fine_trajectory(robot_joint_position_t _next_position, Manipulator_position _setpoint, float _linear_speed){
    // read current robot joint position
    robot_joint_position_t curr_pos;
    get_current_robot_position(curr_pos);
    // Manipulator position variable
    Manipulator_position curr_manipulator_cartesian_pos;
    Manipulator_position new_manipulator_cartesian_pos;

    // calculate manipulator position in cartesian system from joint position
    calculate_simple_robot_kinematics(curr_manipulator_cartesian_pos, curr_pos);

    // find point between set point and current manipulator position in cartesian system with given speed
    robot_joint_position_t fine_pos;    // joint position calculated from inverted robot kinematic

    // linear distance between setpoint and current point [cm]
    float dist = 0;
    dist += pow(_setpoint.x - curr_manipulator_cartesian_pos.x, 2);
    dist += pow(_setpoint.y - curr_manipulator_cartesian_pos.y, 2);
    dist += pow(_setpoint.z - curr_manipulator_cartesian_pos.z, 2);
    dist = sqrt(dist);
    // calculate the displacement factor
    float u = _linear_speed *(CONTROL_TIME_PERIOD / 1000.0f) / dist;
    // if u factor is higher than 1 then move manipulator directly to setpoint position
    if(u < 1){
        // find next point to reach to generate fine trajectory
        new_manipulator_cartesian_pos.x = (1-u) * curr_manipulator_cartesian_pos.x + u * _setpoint.x;
        new_manipulator_cartesian_pos.y = (1-u) * curr_manipulator_cartesian_pos.y + u * _setpoint.y;
        new_manipulator_cartesian_pos.z = (1-u) * curr_manipulator_cartesian_pos.z + u * _setpoint.z;
        // solve inverse robot kinematic
        if (calculate_inverse_robot_kinematics(new_manipulator_cartesian_pos, fine_pos) != 0)
            return ARM_POSITION_UNREACHABLE;
    }else{
        // solve inverse robot kinematic
        if (calculate_inverse_robot_kinematics(_setpoint, fine_pos) != 0)
            return ARM_POSITION_UNREACHABLE;
    }

    // check limits and overrun in joints
    if(!check_joint_limit(fine_pos))    // check joint position limit
        return ARM_POSITION_UNREACHABLE;
    if(!check_joint_speed_limit(curr_pos, fine_pos))// check joint speed limit
        return OVERRUN_JOINT_SPEED;

    // copy selected solution to input variable
    memcpy(_next_position, fine_pos, sizeof(fine_pos));
    // If trajectory was generated successful
    return TRAJECTORY_GENERATED_SUCCESSFUL;
}

bool check_joint_speed_limit(robot_joint_position_t _curr, robot_joint_position_t _req){
    // calculate max speed for joint
    float max_joint_move = MOTOR_SPEED_DEG * CONTROL_TIME_PERIOD;
    // check speed limit for each joint
    for (size_t itr = 0; itr < NUMBER_OF_ROBOT_JOINT; ++itr)
        if(max_joint_move < abs(_req[itr] - _curr[itr]))
            return false;   // if joint speed is overrun return false
    // return true if joint speed limits are correct
    return true;
}