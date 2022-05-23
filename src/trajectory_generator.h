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
#include "mutex"
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

/*
 * enum containing the instructions set of the trajectory generator
 * contains the following instructions:
 *  GO_PTP  // moves the robot arm in joint coordinates [Joint1, Joint2, Joint3] [rad]
 *  FINE,   // moves the robot arm in Cartesian coordinates at a given speed    [X, Y, Z, SPEED][cm, cm, cm, cm/s]
 *  IF,     // compare the digital outputs with the given condition, in case of true jump to the given address [digital_read_value, condition, instruction]
 *  JUMP,   //Jump to the given instruction [instruction_number]
 *  WAIT,   //Program will wait certain time in [us]
 *  STOP,   //stop program
 *  EXIT    //exit program
 */
enum Trajectory_instruction_set{
    GO_PTP, // moves the robot arm in joint coordinates
    FINE,   // moves the robot arm in Cartesian coordinates at a given speed
    IF,     // compare the digital outputs with the given condition, in case of true jump to the given address
    JUMP,   //Jump to the given instruction
    WAIT,   //Program will wait certain time in us
    STOP,   //stop program
    EXIT    //exit program
};
// Max instruction data size in byte
#define INSTRUCTION_DATA_MAX_SIZE 16
/*
 * Instruction structure
 * Include single instruction and data for each instruction
 */
struct TrajectoryInstruction
{
    Trajectory_instruction_set instruction; // Predefined instruction
    uint8_t data[INSTRUCTION_DATA_MAX_SIZE];// Data for each instruction
};

//Limit for instructions in buffer
#define MAX_INSTRUCTION_PER_TRAJECTORY 128
// Global instruction buffer
extern TrajectoryInstruction trajectory_instruction_buffer[MAX_INSTRUCTION_PER_TRAJECTORY];
// mutex for trajectory instruction buffer
extern std::mutex trajectory_instruction_buffer_mutex;

#endif //ROBOT_TRAJECTORY_GENERATOR_H
