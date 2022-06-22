//
// Created by Ja on 16.05.2022.
//

#include "calculate_robot_position.h"

/*
 * Function to calculate inverse robot kinematics
 * Input two arguments:
 * cartesian_position - requested position to reach in cartesian position
 * current_position - current position of manipulator in joint position
 * calculated_joint_position - requested of manipulator position in joint position
 * Replace this function for the kinematics of your robot
 */
int calculate_inverse_robot_kinematics(Manipulator_position cartesian_position,
                                        robot_joint_position_t calculated_joint_position){
    // extract the setpoint manipulator position in cartesian system
    float setpoint_x =  cartesian_position.x;
    float setpoint_y =  cartesian_position.y;
    float setpoint_z =  cartesian_position.z;

    // Calculate position to first joint
    float angle_1 = atan2(setpoint_y, setpoint_x);

    float ex = setpoint_x / cos(angle_1);
    float ez = setpoint_z - ARM_1_LENGTH;

    float angle_3 = acos((ex*ex + ez*ez - ARM_1_LENGTH*ARM_1_LENGTH - ARM_2_LENGTH*ARM_2_LENGTH)
            /(2*ARM_1_LENGTH * ARM_2_LENGTH));
    float angle_2 = atan2(ez, ex) - atan2(ARM_3_LENGTH * sin(angle_3), ARM_1_LENGTH + ARM_2_LENGTH
    * cos(angle_3));

    // convert rad to deg
    angle_1 = angle_1 * 180 / M_PI;
    angle_2 = angle_2 * 180 / M_PI;
    angle_3 = angle_3 * 180 / M_PI;

    // Check error in calculated values
    if (angle_1 == NAN or angle_1 == INFINITY)
        return -1;
    if (angle_2 == NAN or angle_2 == INFINITY)
        return -1;
    if (angle_3 == NAN or angle_3 == INFINITY)
        return -1;

    // save the calculated values to output array
    calculated_joint_position[0] = angle_1;
    calculated_joint_position[1] = angle_2;
    calculated_joint_position[2] = angle_3;
    return 0;
}

/*
 * Function to calculate simple robot kinematic of given robot
 * Depending on the robot configuration used, change the function code
 */
void calculate_simple_robot_kinematics(Manipulator_position &cartesian_position,
                                       robot_joint_position_t joint_position){
    // read robot joint position
    float joint_1_angle = joint_position[0];
    float joint_2_angle = joint_position[1];
    float joint_3_angle = joint_position[2];

    // convert deg to rads
    joint_1_angle = joint_1_angle * M_PI / 180;
    joint_2_angle = joint_2_angle * M_PI / 180;
    joint_3_angle = joint_3_angle * M_PI / 180;

    // find manipulator position
    float dx; // joint 2 and 3 2D position x
    float dy; // joint 2 and 3 2D position y
    dx =  cos(joint_2_angle)* ARM_2_LENGTH + cos(joint_2_angle + joint_3_angle) * ARM_3_LENGTH;
    dy =  sin(joint_2_angle)* ARM_2_LENGTH + sin(joint_2_angle + joint_3_angle) * ARM_3_LENGTH;

    // write to cartesian position
    cartesian_position.x = cos(joint_1_angle) * dx;
    cartesian_position.y = sin(joint_1_angle) * dx;
    cartesian_position.z = ARM_1_LENGTH + dy;
}