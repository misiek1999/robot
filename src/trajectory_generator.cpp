//
// Created by Ja on 21.05.2022.
//

#include "trajectory_generator.h"
// Manual mode position change value
#define MANUAL_MODE_CONTROL_CHANGE_VALUE 1.0f;

std::atomic<Trajectory_control_type>  robot_trajectory_mode;

// Global instruction buffer
TrajectoryInstruction trajectory_instruction_buffer[MAX_INSTRUCTION_PER_TRAJECTORY];
// mutex for trajectory instruction buffer
std::mutex trajectory_instruction_buffer_mutex;

// Message queue for data from console to trajectory generator
mqd_t	mes_to_trajectory_queue;
struct	mq_attr mes_to_trajectory_queue_attr;

/*
 * Check joint limit
 * return false if restrictions are violated
 * return true if limits are correct
*/
 bool check_joint_limit(const robot_joint_position_t _position){
    // check each joint if limits are correct
    for (size_t itr = 0; itr < NUMBER_OF_ROBOT_JOINT; ++itr){
        // If minimum or maximum value is not correct then return false
        if (ROBOT_JOINT_LIMIT_ARRAY[itr][0] < _position[itr] || ROBOT_JOINT_LIMIT_ARRAY[itr][1] > _position[itr])
            return false;
    }
    // if limits are correct return true
    return true;
}

// Function to send message with instruction to trajectory generator in manual mode
void send_manual_control(ManualModeControlInstruction _instruction){
    int status = mq_send(mes_to_trajectory_queue, (const char *)&_instruction, sizeof(mq_traj_manual_data_t), 0);
    // Catch error code
    if (status < 0 )
        std::cerr<<"MQ SEND ERROR: "<<status<<" -> "<< strerror(errno) <<std::endl;
}

// Read message from message queue every 10ms
void raed_manual_control(){
    ManualModeControlInstruction instruction;
    // Timeout for receive data
    struct timespec rec_timeout = {0, 10000000}; //10ms
    // read data from queue
    int status = mq_timedreceive(mes_to_trajectory_queue, (char *)&instruction, sizeof(mq_traj_manual_data_t), NULL, &rec_timeout);
    if (status>=0){ // if received message is correct and not timeouted
        // Create local variable for current robot position
        robot_joint_position_t curr_pos;
        // read current robot position
        read_current_robot_position(curr_pos);
        // set new value for joint position
        switch (instruction) {
            case ManualModeControlInstruction::joint_1_left:
                curr_pos[0] = MANUAL_MODE_CONTROL_CHANGE_VALUE;
                break;
            case ManualModeControlInstruction::joint_1_right:
                curr_pos[0] = -MANUAL_MODE_CONTROL_CHANGE_VALUE;
                break;
            case ManualModeControlInstruction::joint_2_left:
                curr_pos[1] = MANUAL_MODE_CONTROL_CHANGE_VALUE;
                break;
            case ManualModeControlInstruction::joint_2_right:
                curr_pos[1] = -MANUAL_MODE_CONTROL_CHANGE_VALUE;
                break;
            case ManualModeControlInstruction::joint_3_left:
                curr_pos[2] = MANUAL_MODE_CONTROL_CHANGE_VALUE;
                break;
            case ManualModeControlInstruction::joint_3_right:
                curr_pos[2] = -MANUAL_MODE_CONTROL_CHANGE_VALUE;
                break;
        }
        // Check joint limit
        if (check_joint_limit(curr_pos)) {
            //if new position is correct then write new robot position
            write_setpoint_robot_position(curr_pos);
        }
    }
}