//
// Created by Ja on 16.05.2022.
//

#include "calculate_robot_position.h"

/*
 *  Check difference between two manipulator position. Return true if difference is smaller than tolerance
 */
bool check_manipulator_position_tolerance(Manipulator_position first_pos, Manipulator_position second_pos){
    float diff_pow = pow(first_pos.x - second_pos.x, 2);
    diff_pow += pow(first_pos.y - second_pos.y, 2);
    diff_pow += pow(first_pos.z - second_pos.z, 2);
    diff_pow = sqrt(diff_pow);
    if (diff_pow> ROBOT_MANIPULATOR_POSITION_TOLERANCE)
        return false;
    return true;
}

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

    // Calculate position to last joint
    float L = pow(setpoint_z - ARM_1_LENGTH,2) + pow(setpoint_x,2) +  pow(setpoint_y,2) - pow(ARM_2_LENGTH,2) -  pow(ARM_3_LENGTH,2);
    L = L /(2 * ARM_2_LENGTH * ARM_3_LENGTH);
    float sin_angle_3_pos = sqrt(1 - L*L);
    float sin_angle_3_neg = -sqrt(1 - L*L);
    float angle_3_pos = atan2(sin_angle_3_pos, L);
    float angle_3_neg = atan2(sin_angle_3_neg, L);

    // Calculate position to second joint
    float beta = atan2(setpoint_z - ARM_1_LENGTH, sqrt(setpoint_y * setpoint_y + setpoint_x * setpoint_x));
    float theta_pos = atan2(ARM_3_LENGTH*sin(angle_3_pos),ARM_2_LENGTH + ARM_3_LENGTH*cos(angle_3_pos));
    float angle_2_pos = beta + theta_pos;

    float theta_neg = atan2(ARM_3_LENGTH*sin(angle_3_neg),ARM_2_LENGTH + ARM_3_LENGTH*cos(angle_3_neg));
    float angle_2_neg = beta + theta_neg;

    // convert rad to deg
    angle_1 = angle_1 * 180 / M_PI;
    angle_2_pos = angle_2_pos * 180 / M_PI;
    angle_2_neg = angle_2_neg * 180 / M_PI;
    angle_3_pos = angle_3_pos * 180 / M_PI;
    angle_3_neg = angle_3_neg * 180 / M_PI;

    // find manipulator position of each solution
    Manipulator_position calc_sol[4];
    robot_joint_position_t solutions[4] = {{angle_1, angle_2_pos, angle_3_pos},
                                           {angle_1, angle_2_neg, angle_3_pos},
                                           {angle_1, angle_2_pos, angle_3_neg},
                                           {angle_1, angle_2_neg, angle_3_neg}};
    for (size_t itr = 0; itr < 4; ++itr)
        calculate_simple_robot_kinematics(calc_sol[itr], solutions[itr]);

    // select correct solution
    float angle_2 = NAN, angle_3 = NAN; // Uninitialized variable
    for (size_t itr = 0; itr < 4; ++itr)
        if (check_manipulator_position_tolerance(calc_sol[itr], cartesian_position)){
            angle_2 = solutions[itr][1];
            angle_3 = solutions[itr][2];
        }

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