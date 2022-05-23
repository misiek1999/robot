//
// Created by Ja on 21.05.2022.
//
/*
 * This file has function to generate robot trajectory
 * We define two type of robot trajectory:
 * 1. User enter control signal using console
 * 2. Load predefine file with robot trajectory
 *
 * We can choose type of robot trajectory after starting application by entering valid number to console
 */

#ifndef ROBOT_TRAJECTORY_GENERATOR_H
#define ROBOT_TRAJECTORY_GENERATOR_H
//Include libs
#include "atomic"

/*
 * Enum include two state of robot trajectory generator
 * 0 - Undefined
 * 1 - Manual change robot position using console
 * 2 - Automatic robot trajectory generator using predefined file
 */
enum Trajectory_control_type{
    UNDEFINED = 0,
    MANUAL = 1,
    AUTO = 2
};
// global trajectory control type variable
extern std::atomic<Trajectory_control_type> robot_trajectory_mode;



#endif //ROBOT_TRAJECTORY_GENERATOR_H
