//
// Created by Ja on 16.05.2022.
//

#include "calculate_robot_position.h"

/*
 * Function to calculate inverse robot kinematics
 * Input two arguments:
 * setpoint_manipulator_position - requested position to reach
 * current_position - current position of manipulator
 * designated_control - setpoint to each joint
 * Replace this function for the kinematics of your robot
 */
void calculate_inverse_robot_kinematics(Manipulator_position setpoint_manipulator_position,
                                        robot_joint_position_t designated_control){
    // extract the setpoint manipulator position in cartesian system
    float setpoint_x =  setpoint_manipulator_position.x;
    float setpoint_y =  setpoint_manipulator_position.y;
    float setpoint_z =  setpoint_manipulator_position.z;

    // Calculate position to first joint
    float angle_1 = atan2(setpoint_y, setpoint_x);

    // Calculate position to last joint
    float L = pow(setpoint_z - ARM_1_LENGTH,2) + pow(setpoint_x,2) +  pow(setpoint_y,2) - pow(ARM_2_LENGTH,2) -  pow(ARM_3_LENGTH,2);
    L = L /(2 * ARM_2_LENGTH * ARM_3_LENGTH);
    float temp_numinator = sqrt(1 - L*L);
    float angle_3 = atan2(temp_numinator, L);

    // Calculate position to second joint
    float beta = atan2(setpoint_z - ARM_1_LENGTH, sqrt(setpoint_y * setpoint_y + setpoint_x * setpoint_x));
    float theta = atan2(ARM_3_LENGTH*sin(angle_3),ARM_2_LENGTH + ARM_3_LENGTH*cos(angle_3));
    float angle_2 = beta + theta;

    // save the calculated values to output array
    designated_control[0] = angle_1;
    designated_control[1] = angle_2;
    designated_control[2] = angle_3;
}