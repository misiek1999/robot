//
// Created by Ja on 13.06.2022.
//
#include "instruction_processor.h"

// Global instruction buffer
TrajectoryInstruction trajectory_instruction_buffer[MAX_INSTRUCTION_PER_TRAJECTORY];
// mutex for trajectory instruction buffer
std::mutex trajectory_instruction_buffer_mutex;

// program instruction number iterator
static InstructionIteratorType instruction_number_iterator;   // default value is 0

// Alarm function if position is unrechable
void position_unreachable_alarm(){
    // send communicate to console and logger
    write_to_console("Position unreachable error");
    char buff[40];
    sprintf(buff, "Position unreachable error at: %d !", (int)instruction_number_iterator);
    write_to_log(buff);
    // change program state to emergency stop
    set_program_state(ProgramState::EMERGENCY_STOP);
}

/*
 * The following functions below execute the command of a given instruction
 */

void undefined_instruction_stop(){
    // Display and save communicate to console and log
    write_to_console("Undefined instruction! Emergency stop.");
    char buff[40];
    sprintf(buff, "Undefined instruction at %d !", (int)instruction_number_iterator);
    write_to_log(buff);
    write_to_log("Emergency stop.");
    // Change program mode to emergency stop
    set_program_state(ProgramState::EMERGENCY_STOP);
}

void go_ptp_instruction(InstructionData _data){
    robot_joint_position_t new_position;
    // set new setpoint value for each joint
    for (size_t itr = 0; itr < NUMBER_OF_ROBOT_JOINT; ++itr){
        new_position[itr] = _data.go_ptp_data[itr];
    }
    // check limits
    if (check_joint_limit(new_position)) {
        // Set new position to reach
        write_setpoint_robot_position(new_position);
    }else{
        position_unreachable_alarm();
    }
}

void fine_instruction(InstructionData _data) {
    robot_joint_position_t new_position;
    //TODO: finish this function

//    // set new setpoint value for each joint
//    for (size_t itr = 0; itr < NUMBER_OF_ROBOT_JOINT; ++itr){
//        new_position[itr] = _data.go_ptp_data[itr];
//    }
//    // check limits
//    if (check_joint_limit(new_position)) {
//        // Set new position to reach
//        write_setpoint_robot_position(new_position);
//    }else{
//        position_unreachable_alarm();
//    }
}

void write_digital_instruction(InstructionData _data){
    robot_digital_data_type digital_out;
    // get digital out from data
    digital_out = _data.digital_data;
    // write digital output
    write_digital_output(digital_out);
}

void if_instruction(InstructionData _data, InstructionIteratorType &instr_numb, bool &jump_flag){
    robot_digital_data_type digital_input;
    // get digital input from robot
    digital_input = get_digital_output();
    // read data to compare
    robot_digital_data_type data_to_compare;
    data_to_compare = _data.if_data.data_to_compare;
    // Compare data
    if (digital_input == data_to_compare){  // if data are correct then jump
        instr_numb = _data.if_data.address_to_jump;
        jump_flag = true;
    }
}

void jump_instruction(InstructionData _data, InstructionIteratorType &instr_numb, bool &jump_flag){
    instr_numb = _data.if_data.address_to_jump;
    jump_flag = true;
}

void wait_instruction(InstructionData _data){
    unsigned int time_to_sleep;
    time_to_sleep = _data.wait_data;
    // Sleep this thread for given time
    std::this_thread::sleep_for(std::chrono::milliseconds(time_to_sleep));
}

void stop_instruction(){
    // Change program state to resume
    set_program_state(ProgramState::STOP);
}

void resume_instruction(){
    // Change program state to resume
    set_program_state(ProgramState::RUNNING);
}

void exit_instruction(){
    // Change program state to resume
    set_program_state(ProgramState::CLOSE_PROGRAM);
}

/*
 * Each function call execute next instruction from global instruction buffer
 * Return false if executing failed
*/
void execute_instructions(){
    // Check the status of the program, in case of a shutdown or an emergency stop, skip the execution of the instructions
    ProgramState current_state = get_program_state();
    if (current_state == ProgramState::RUNNING || current_state == ProgramState::STOP) {
        // Continue only if setpoint position is reached
        if (is_position_reached()) {
            /* bool variable to detect jump */
            bool jump_instruction_detected = false;

            // Read current instruction and data
            trajectory_instruction_buffer_mutex.lock();
            TrajectoryInstruction curr_instr_set = trajectory_instruction_buffer[instruction_number_iterator];
            trajectory_instruction_buffer_mutex.unlock();
            Trajectory_instruction_set instruction = curr_instr_set.instruction;
            InstructionData data = curr_instr_set.data;

            // Select and execute instruction
            switch (instruction) {
                case Trajectory_instruction_set::UNDEFINED:
                    undefined_instruction_stop();
                    break;
                case Trajectory_instruction_set::GO_PTP:
                    go_ptp_instruction(data);
                    break;
                case Trajectory_instruction_set::FINE:
                    fine_instruction(data);
                    break;
                case Trajectory_instruction_set::WRITE_DIGITAL:
                    write_digital_instruction(data);
                    break;
                case Trajectory_instruction_set::IF:
                    if_instruction(data, instruction_number_iterator, jump_instruction_detected);
                    break;
                case Trajectory_instruction_set::JUMP:
                    jump_instruction(data, instruction_number_iterator, jump_instruction_detected);
                    break;
                case Trajectory_instruction_set::WAIT:
                    wait_instruction(data);
                    break;
                case Trajectory_instruction_set::STOP:
                    stop_instruction();
                    break;
                case Trajectory_instruction_set::EXIT:
                    exit_instruction();
                    break;
                case Trajectory_instruction_set::RESUME:
                    resume_instruction();
                    break;
                default:
                    undefined_instruction_stop();
                    break;
            }

            // If jump instruction was not executed then increase instruction number by 1
            if (!jump_instruction_detected)
                ++instruction_number_iterator;
        }
    }
}