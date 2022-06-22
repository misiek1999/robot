//
// Created by Ja on 15.06.2022.
//

#include "fine_trajectory.h"

// returned values from function generate_fine_trajectory
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
int generate_fine_trajectory(robot_joint_position_t _next_position, robot_joint_position_t _setpoint, float _linear_speed){
    // read current robot joint position
    robot_joint_position_t curr_pos;
    get_current_robot_position(curr_pos);
    // Manipulator position
    Manipulator_position curr_manipulator_cartesian_pos;
    Manipulator_position sp_manipulator_cartesian_pos;
    Manipulator_position new_manipulator_cartesian_pos;
    // change joint position to manipulator position in cartesian system
    calculate_simple_robot_kinematics(curr_manipulator_cartesian_pos, curr_pos);
    calculate_simple_robot_kinematics(sp_manipulator_cartesian_pos, _setpoint);

    // find next point between setpoint and current manipulator position in cartesian system with given speed
    robot_joint_position_t fine_pos;

    // linear distance between setpoint and current point [cm]
    float dist = 0;
    dist += pow(sp_manipulator_cartesian_pos.x - curr_manipulator_cartesian_pos.x, 2);
    dist += pow(sp_manipulator_cartesian_pos.y - curr_manipulator_cartesian_pos.y, 2);
    dist += pow(sp_manipulator_cartesian_pos.z - curr_manipulator_cartesian_pos.z, 2);
    dist = sqrt(dist);
    // calculate the displacement factor
    float u = _linear_speed *CONTROL_TIME_PERIOD / 1000 / dist;
    // if u factor is higher than 1 then move manipulator directly to setpoint position
    if(u < 1){
        // find next point to reach to generate fine trajectory
        new_manipulator_cartesian_pos.x = (1-u) * curr_manipulator_cartesian_pos.x + u * sp_manipulator_cartesian_pos.x;
        new_manipulator_cartesian_pos.y = (1-u) * curr_manipulator_cartesian_pos.y + u * sp_manipulator_cartesian_pos.y;
        new_manipulator_cartesian_pos.z = (1-u) * curr_manipulator_cartesian_pos.z + u * sp_manipulator_cartesian_pos.z;
        // solve inverse robot kinematic
        if (calculate_inverse_robot_kinematics(new_manipulator_cartesian_pos, fine_pos) != 0)
            return ARM_POSITION_UNREACHABLE;
    }else{
        memcpy(fine_pos, _setpoint, sizeof(fine_pos));
    }

    // check limits and overrun in calculated trajectory
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
            return false;

    // return true if joint speed limits are correct
    return true;
}