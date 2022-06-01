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
#include "mqueue.h"
#include "iostream"
#include "errno.h"
#include "string.h"
#include "controll_interface.h"


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

/*
 * enum containing the instructions set of the trajectory generator
 * contains the following instructions:
 *  UNDEFINED,// non specified instruction, means data error
 *  GO_PTP  // moves the robot arm in joint coordinates [Joint1, Joint2, Joint3] [rad]
 *  FINE,   // moves the robot arm in Cartesian coordinates at a given speed    [X, Y, Z, SPEED][cm, cm, cm, cm/s]
 *  WRITE_DIGITAL,// Write digital pin to robot [digital_output]
 *  IF,     // compare the digital outputs with the given condition, in case of true jump to the given address [digital_read_value, condition, instruction]
 *  JUMP,   //Jump to the given instruction [instruction_number]
 *  WAIT,   //Program will wait certain time in [ms]
 *  STOP,   //stop program
 *  EXIT    //exit program
 */

enum class Trajectory_instruction_set{
    UNDEFINED = 0,  // Undefined instruction
    GO_PTP, // moves the robot arm in joint coordinates
    FINE,   // moves the robot arm in Cartesian coordinates at a given speed
    WRITE_DIGITAL,// Write digital pin to robot
    IF,     // compare the digital outputs with the given condition, in case of true jump to the given address
    JUMP,   //Jump to the given instruction
    WAIT,   //Program will wait certain time in ms
    STOP,   //stop program
    EXIT    //exit program
};
/*
 * Instruction data for each instruction
 */
union InstructionData{
    float fine_data[4];     //Data for FINE instruction: X, Y, Z in cardesian pos and required speed
    float go_ptp_data[3];   //Data used for ptp move, include joint position
    uint8_t digital_data;   // Digital output
    unsigned int if_data[2];// Data for if statement
    unsigned int jump_data; // Address in jump instruction
    unsigned int wait_data; // Time to wait in ms
    uint8_t bytes[16];      // bytes of this union
};

/*
 * Instruction structure
 * Include single instruction and data for each instruction
 */
struct TrajectoryInstruction
{
    Trajectory_instruction_set instruction; // Predefined instruction
    InstructionData data;// Data for each instruction
};

//Limit for instructions in buffer
#define MAX_INSTRUCTION_PER_TRAJECTORY 128
// Global instruction buffer
extern TrajectoryInstruction trajectory_instruction_buffer[MAX_INSTRUCTION_PER_TRAJECTORY];
// mutex for trajectory instruction buffer
extern std::mutex trajectory_instruction_buffer_mutex;

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
extern struct	mq_attr mes_to_trajectory_queue_attr;
typedef ManualModeControlInstruction mq_traj_manual_data_t;

// Function to send message with instruction to trajectory generator in manual mode
void send_manual_control(ManualModeControlInstruction _instruction);

#endif //ROBOT_TRAJECTORY_GENERATOR_H
