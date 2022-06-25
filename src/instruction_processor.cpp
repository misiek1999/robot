//
// Created by Ja on 13.06.2022.
//
#include "instruction_processor.h"

// Global static bool variable with new instruction information
std::atomic<bool> new_position_selected;

// Global instruction buffer
TrajectoryInstruction trajectory_instruction_buffer[MAX_INSTRUCTION_PER_TRAJECTORY];
// mutex for trajectory instruction buffer
std::mutex trajectory_instruction_buffer_mutex;

// program instruction number iterator
static instruction_iterator_t instruction_number_iterator;   // default value is 0

// Alarm function if position is unreachable
void position_unreachable_alarm(){
    // send communicate to console and logger
    write_to_console("Position unreachable error");
    char buff[40];
    sprintf(buff, "Position unreachable error at: %d !", (int)instruction_number_iterator);
    write_to_log(buff);
    // Lock robot movement
    lock_robot_movement();
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
        set_setpoint_robot_position(new_position);
    }else{
        position_unreachable_alarm();
    }
    char buff [40];
    sprintf(buff, "Go ptp:  j1:%.2f, j2:%.2f, j3:%.2f", new_position[0], new_position[1], new_position[2]);
    write_to_console(buff);
    write_to_log(buff);
}

/*
 * If final position is reached then return true, otherwise return false
 */
bool fine_instruction(InstructionData _data) {
    robot_joint_position_t new_position;
    Manipulator_position requied_position;
    // set new set point value for each joint
    requied_position.x = _data.fine_data[0];
    requied_position.y = _data.fine_data[1];
    requied_position.z = _data.fine_data[2];
    float arm_linear_speed = _data.fine_data[3];

    // Find next point to reach in fine trajectory generator mode
    int status = generate_fine_trajectory(new_position, requied_position, arm_linear_speed);

    // check fine generation status 
    if (status == 0) {
        // Set new position to reach
        set_setpoint_robot_position(new_position);
    }else{
        position_unreachable_alarm();
    }

    // calculate manipulator position
    Manipulator_position man_pos;
    calculate_simple_robot_kinematics(man_pos, new_position);

    // Check position reach status
    int position_reach_status = check_manipulator_position_tolerance(man_pos, requied_position);
    // Write to console and log message if position is reached
    if (position_reach_status == true){
        char buff [40];
        sprintf(buff, "Fine trajectory reach: x:%.2f, y:%.2f, z:%.2f", requied_position.x, requied_position.y, requied_position.z);
        write_to_console(buff);
        write_to_log(buff);
    }
    // return status of manipulator position with tolerance
    return position_reach_status;
}

void write_digital_instruction(InstructionData _data){
    robot_digital_data_t digital_out;
    // get digital out from data
    digital_out = _data.digital_data;
    // write digital output
    set_digital_output(digital_out);
    char buff [40];
    sprintf(buff, "Write output: %d", digital_out);
    write_to_console(buff);
    write_to_log(buff);
}

void if_instruction(InstructionData _data, instruction_iterator_t &instr_numb, bool &jump_flag){
    robot_digital_data_t digital_input;
    // get digital input from robot
    digital_input = get_digital_output();
    // read data to compare
    robot_digital_data_t data_to_compare;
    data_to_compare = _data.if_data.data_to_compare;
    // Compare data
    if (digital_input == data_to_compare){  // if data are correct then jump
        instr_numb = _data.if_data.address_to_jump + 1;
        jump_flag = true;
        char buff [40];
        sprintf(buff, "If condition true. Jump to: %d", instr_numb);
        write_to_console(buff);
        write_to_log(buff);
    }else
    {
        write_to_console("If condition false");
        write_to_log("If condition false");
    }
}

void jump_instruction(InstructionData _data, instruction_iterator_t &instr_numb, bool &jump_flag){
    instr_numb = _data.if_data.address_to_jump + 1;
    jump_flag = true;
    char buff [40];
    sprintf(buff, "Jump to: %d", instr_numb);
    write_to_console(buff);
    write_to_log(buff);
}

void wait_instruction(InstructionData _data){
    unsigned int time_to_sleep;
    time_to_sleep = _data.wait_data;
    char buff [40];
    sprintf(buff, "Sleep for: %d", time_to_sleep);
    write_to_console(buff);
    write_to_log(buff);
    // Sleep this thread for given time
    std::this_thread::sleep_for(std::chrono::milliseconds(time_to_sleep));
}

void stop_instruction(){
    write_to_console("Instruction stop");
    write_to_log("Instruction stop");
    // stop robot
    stop_robot();
}

void resume_instruction(){
    write_to_console("Instruction resume");
    write_to_log("Instruction resume");
    // Change program state to resume
    resume_robot_movement();
}

void exit_instruction(){
    write_to_console("Program end successful");
    write_to_log("Program end successful");
    // Lock robot movement
    lock_robot_movement();
    // Change program state to close program
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
        /* bool variable to detect jump */
        bool stop_auto_instruction_iterator = false;

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
                if (get_program_state() == ProgramState::RUNNING)// stop movement in different program state as RUNNING
                    go_ptp_instruction(data);
                else
                    stop_auto_instruction_iterator = true;
                break;
            case Trajectory_instruction_set::FINE:
                stop_auto_instruction_iterator = true;
                if (get_program_state() == ProgramState::RUNNING)// stop movement in different program state as RUNNING
                    if(fine_instruction(data))
                        stop_auto_instruction_iterator = false; // If final position is reached then increase instruction
                break;
            case Trajectory_instruction_set::WRITE_DIGITAL:
                write_digital_instruction(data);
                break;
            case Trajectory_instruction_set::IF:
                if_instruction(data, instruction_number_iterator, stop_auto_instruction_iterator);
                break;
            case Trajectory_instruction_set::JUMP:
                jump_instruction(data, instruction_number_iterator, stop_auto_instruction_iterator);
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
        if (!stop_auto_instruction_iterator)
            ++instruction_number_iterator;

        // If selected instruction is go_ptp, block thread until reach set point position
        if (instruction == Trajectory_instruction_set::GO_PTP){
            // enable flag with new position selected
            new_position_selected = true;
            // Block this thread until reach set point position
            pthread_barrier_wait(&trajectory_barrier);
            // disable flag with new position selected
            new_position_selected = false;
        }
    }
}