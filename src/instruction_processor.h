/*
 * This file includes description of instruction type and function which implement each instruction
 */

#ifndef ROBOT_INSTRUCTION_PROCESSOR_H
#define ROBOT_INSTRUCTION_PROCESSOR_H

#include "supervisor.h"
#include "controll_interface.h"
#include "supervisor.h"

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
struct __attribute__ ((packed)) IfInstructionDataType{
    robot_digital_data_type data_to_compare;
    InstructionIteratorType address_to_jump;
};

/*
 * Instruction data for each instruction
 */
union __attribute__ ((packed)) InstructionData{
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
struct __attribute__ ((packed)) TrajectoryInstruction
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

/*
 * Each function call execute next instruction from global instruction buffer
 * Return false if executing failed
*/
void execute_instructions();

#endif //ROBOT_INSTRUCTION_PROCESSOR_H
