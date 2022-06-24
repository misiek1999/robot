//
// Created by Ja on 21.05.2022.
//
/*
 * This file has function to generate robot trajectory
 * We define two type of robot trajectory:
 * 1. Manual - User enter control signal using console
 * 2. Auto - Load predefine file with robot trajectory
 *
 * We can choose type of robot trajectory after starting application by entering valid number to console
 */

#ifndef ROBOT_TRAJECTORY_GENERATOR_H
#define ROBOT_TRAJECTORY_GENERATOR_H
//Include libs
#include "atomic"
#include "mutex"
#include "mqueue.h"
#include "iostream"
#include "errno.h"
#include "string.h"
#include "signal.h"
#include "controll_interface.h"
#include "instruction_processor.h"

// barrier to lock trajectory barrier
extern pthread_barrier_t trajectory_barrier;

// global state of loading trajectory from file
extern std::atomic<bool> is_file_trajectory_load;

/*
 * Enum include two state of robot trajectory generator
 * 0 - Undefined
 * 1 - Manual change robot position using console
 * 2 - Automatic robot trajectory generator using predefined file
 */
enum class Trajectory_control_type{ // enum class to avoid name conflict with other enum's
    UNDEFINED = 0,
    MANUAL = 1,
    AUTO = 2
};

// global trajectory control type variable
extern std::atomic<Trajectory_control_type> robot_trajectory_mode;

// enum for instruction from console in manual mode
enum ManualModeControlInstruction{
    joint_1_right,
    joint_1_left,
    joint_2_right,
    joint_2_left,
    joint_3_right,
    joint_3_left
};

// Message queue from console to trajectory generator in manual mode
extern mqd_t mes_to_trajectory_queue;
extern struct mq_attr mes_to_trajectory_queue_attr;
typedef ManualModeControlInstruction mq_traj_manual_data_t;

// Function to send message with instruction to trajectory generator in manual mode
void send_manual_control(ManualModeControlInstruction _instruction);

// Function to run trajectory generator. Run this function in new thread
void* generate_trajectory(void *pVoid);

/*
 * Check is position reached
 * Compare global set-point and current robot joint position, if difference is smaller than tolerance then function
 * return true else function return false.
*/
bool is_position_reached();

/*
 * Function to check joint limits
 * Return true if position is reachable
 */
bool check_joint_limit(const robot_joint_position_t _position);

#endif //ROBOT_TRAJECTORY_GENERATOR_H
