//
// Created by Ja on 20.06.2022.
//

#ifndef READ_TRAJECTORY_INSTRUCTION_READ_INSTRUCTION_FILE_H
#define READ_TRAJECTORY_INSTRUCTION_READ_INSTRUCTION_FILE_H

#include <iostream>
#include "fstream"
#include "string"
#include "memory.h"
#include "algorithm"
#include "sstream"
#include "instruction_set.h"
/*
 * Read instructions from given file and save resolute to binary instruction buffer
 * If instruction reading is successful function return 0 otherwise return error code
 */
int  read_instruction_from_file(std::string file_name, TrajectoryInstruction *_input_buffer);

#endif //READ_TRAJECTORY_INSTRUCTION_READ_INSTRUCTION_FILE_H
