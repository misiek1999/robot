/*
 * This file includes description of instruction type and function which implement each instruction
 */

#ifndef ROBOT_INSTRUCTION_PROCESSOR_H
#define ROBOT_INSTRUCTION_PROCESSOR_H

#include "instruction_set.h"
#include "supervisor.h"
#include "controll_interface.h"
#include "supervisor.h"
#include "fine_trajectory.h"
#include "console_interface.h"
/*
 * Each function call execute next instruction from global instruction buffer
 * Return false if executing failed
*/
void execute_instructions();

#endif //ROBOT_INSTRUCTION_PROCESSOR_H
