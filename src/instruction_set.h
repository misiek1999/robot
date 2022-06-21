//
// Created by Ja on 21.06.2022.
//

#ifndef ROBOT_INSTRUCTION_SET_H
#define ROBOT_INSTRUCTION_SET_H
#include "stdlib.h"
#include "mutex"
#include "robot_arm_params.h"

// Instruction number iterator type
typedef size_t InstructionIteratorType;

/*
 * enum containing the instructions set of the trajectory generator
 * contains the following instructions:
 *  UNDEFINED,--> non specified instruction, means data error
 *  GO_PTP  --> moves the robot arm in joint coordinates [Joint1, Joint2, Joint3] [rad]
 *  FINE,   --> moves the robot arm in Cartesian coordinates at a given speed    [X, Y, Z, SPEED][cm, cm, cm, cm/s]
 *  WRITE_DIGITAL,--> Write digital pin to robot [digital_output]
 *  IF,     --> compare the digital outputs with the given condition, in case of true jump to the given address [digital_read_value, condition, instruction]
 *  JUMP,   -->Jump to the given instruction [instruction_number_iterator]
 *  WAIT,   -->Program will wait certain time in [ms]
 *  STOP,   -->stop program
 *  EXIT    -->exit program
 *  RESUME --> resume stop program
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
    EXIT,    //exit program
    RESUME  // resume stop program
};
/*
 * Struct for if instruction data
 */
struct IfInstructionDataType{
    robot_digital_data_type data_to_compare;
    InstructionIteratorType address_to_jump;
};
/*
 * Instruction data for each instruction
 */
union InstructionData{
    float fine_data[4];     //Data for FINE instruction: X, Y, Z in cardesian pos and required speed
    float go_ptp_data[3];   //Data used for ptp move, include joint position
    uint8_t digital_data;   // Digital output
    IfInstructionDataType if_data;// Data for if statement
    InstructionIteratorType jump_data; // Address in jump instruction
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
#define MAX_INSTRUCTION_PER_TRAJECTORY 256
// Global instruction buffer
extern TrajectoryInstruction trajectory_instruction_buffer[MAX_INSTRUCTION_PER_TRAJECTORY];
// mutex for trajectory instruction buffer
extern std::mutex trajectory_instruction_buffer_mutex;
#endif //ROBOT_INSTRUCTION_SET_H
