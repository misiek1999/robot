//
// Created by Ja on 20.06.2022.
//

#include "read_instruction_file.h"

/*
 * Change string to Instruction type in enum
 */
Trajectory_instruction_set change_string_to_instruction(std::string word){
    // uppercase word
    std::transform(word.begin(), word.end(),word.begin(), ::toupper);
    Trajectory_instruction_set instruction = Trajectory_instruction_set::UNDEFINED;
    // select instruction type
    if (word == "UNDEFINED")
        instruction = Trajectory_instruction_set::UNDEFINED;
    if (word == "GO_PTP")
        instruction = Trajectory_instruction_set::GO_PTP;
    if (word == "FINE")
        instruction = Trajectory_instruction_set::FINE;
    if (word == "WRITE_DIGITAL")
        instruction = Trajectory_instruction_set::WRITE_DIGITAL;
    if (word == "IF")
        instruction = Trajectory_instruction_set::IF;
    if (word == "JUMP")
        instruction = Trajectory_instruction_set::JUMP;
    if (word == "WAIT")
        instruction = Trajectory_instruction_set::WAIT;
    if (word == "STOP")
        instruction = Trajectory_instruction_set::STOP;
    if (word == "EXIT")
        instruction = Trajectory_instruction_set::EXIT;
    if (word == "RESUME")
        instruction = Trajectory_instruction_set::RESUME;
    return instruction;
}

/*
 * Return true if instruction number is valid.
 */
bool check_instruction_count(instruction_iterator_t _instr){
    if(_instr > 0 && _instr <= MAX_INSTRUCTION_PER_TRAJECTORY)
        return true;
    return false;
}

/*
 * Read instruction data from given string
 */
int read_instruction_data(std::istringstream &_iss, TrajectoryInstruction &_instr, size_t _instr_line) {
    char buff[64]; // cstring buffer
    char invalid_args_message[64]; // invalid arguments message to console
    sprintf(invalid_args_message, "Invalid instruction arguments at: %ld", _instr_line);
    std::string words[3];   // buffer for string data
    InstructionData data;
    // try to read instruction
    switch (_instr.instruction) {
        case Trajectory_instruction_set::GO_PTP:
            if (!(_iss >> data.go_ptp_data[0] >> data.go_ptp_data[2] >> data.go_ptp_data[2])) {
                printf(invalid_args_message);
                return -1;
            }
            break;
        case Trajectory_instruction_set::FINE:
            if (!(_iss >> data.fine_data[0] >> data.fine_data[1] >> data.fine_data[2] >> data.fine_data[3])) {
                printf(invalid_args_message);
                return -1;
            }
            break;
        case Trajectory_instruction_set::WRITE_DIGITAL:
            if (!(_iss >> words[0])){
                printf(invalid_args_message);
                return -1;
            }else
                data.digital_data = std::stoi(words[0]);
            break;
        case Trajectory_instruction_set::IF:
            if (!(_iss >> words[0] >> words[1])) {
                printf(invalid_args_message);
                return -1;
            }else{
                data.if_data.data_to_compare = std::stoi(words[0]);
                data.if_data.address_to_jump = std::stoi(words[1]);
                if (!check_instruction_count(data.if_data.address_to_jump))
                    return -2;
            }
            break;
        case Trajectory_instruction_set::JUMP:
            if (!(_iss >> words[0])) {
                printf(invalid_args_message);
                return -1;
            }else
                data.jump_data = std::stoi(words[0]);
            if (!check_instruction_count(data.jump_data))
                return -2;
            break;
        case Trajectory_instruction_set::WAIT:
            if (!(_iss >> data.wait_data)) {
                printf(invalid_args_message);
                return -1;
            }
            break;
        case Trajectory_instruction_set::STOP:
            break;
        case Trajectory_instruction_set::EXIT:
            break;
        case Trajectory_instruction_set::RESUME:
            break;
        default:
            sprintf(buff, "Undefined instruction on: %d", _instr_line);
            printf(buff);
            return -1;
            break;
    }
    std::string word;
    // check that you have not given too many arguments
    if (_iss >> word){
        sprintf(buff, "Too many instruction argument on: %d", _instr_line);
        printf(buff);
        return -1;
    }
    //copy temp data to input instruction data
    memcpy(&_instr.data, &data, sizeof(data));

    // if load successful
    return 0;
}

int  read_instruction_from_file(std::string _file_name, TrajectoryInstruction *_input_buffer) {
    // try to open file
    std::ifstream file;
    file.open(_file_name, std::ios::in);
    if (!file.is_open()) {    // if file is not open
        // Write to console
        printf("Cannot open file");
        return -1;
    }
    // Init instruction buffer to default instructions
    TrajectoryInstruction buffer[MAX_INSTRUCTION_PER_TRAJECTORY];
    for (auto & i : buffer)
        i.instruction = Trajectory_instruction_set::UNDEFINED;
    // Create variable used to read instruction from file
    std::string line;
    size_t line_itr = 0;
    size_t instr_itr = 0;
    TrajectoryInstruction instruction_with_data;
    // read every file line
    while (std::getline(file, line) && instr_itr < MAX_INSTRUCTION_PER_TRAJECTORY)
    {
        if (!line.empty()) {
            // delete \r char from end of line
            if (line[line.size() - 1] == '\r')
                line.erase(line.size() - 1);
            std::istringstream iss(line);
            std::string word;
            // set instruction type to undefined
            instruction_with_data.instruction = Trajectory_instruction_set::UNDEFINED;
            // select right instruction type
            if (iss >> word) {
                instruction_with_data.instruction = change_string_to_instruction(word);
            } else {
                // write invalid instruction line to file
                char buff[40];
                sprintf(buff, "Invalid instruction in line: %d", line_itr);
                printf(buff);
                return -1;
            }
            // try to read instruction data
            if (read_instruction_data(iss, instruction_with_data, line_itr) == 0){
                buffer[instr_itr] = instruction_with_data;
                instr_itr += 1; // increase instruction counter
            }else{  // if reading fail
                return -1;
            }
        }
        line_itr += 1;
    }

    // check instruction buffer overflow
    if (instr_itr >= MAX_INSTRUCTION_PER_TRAJECTORY){
        printf("Instruction limit overflow");
        return -1;
    }

    // copy temp buffer to input buffer
    memcpy(_input_buffer, buffer, sizeof(buffer));

    // If everything is complete without errors
    return 0;
}

