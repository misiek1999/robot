//
// Created by Ja on 21.05.2022.
//

#include "trajectory_generator.h"
// Manual mode position change value
#define MANUAL_MODE_CONTROL_CHANGE_VALUE 1.0f;
// Tolerance of robot joint position [deg]
#define ROBOT_POSITION_TOLERANCE 1.0f

// global state of loading trajectory from file
std::atomic<bool> is_file_trajectory_load;

// global atomic enum with trajectory generator mode
std::atomic<Trajectory_control_type>  robot_trajectory_mode;

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
        std::cerr<<"MQ TRJ SEND ERROR: "<<status<<" -> "<< strerror(errno) <<std::endl;
}

// Read message from message queue every 10ms
void manual_control(){
    ManualModeControlInstruction instruction;
    // Timeout for receive data
    struct timespec rec_timeout = {0, 10000000}; //10ms
    // read data from queue
    int status = mq_timedreceive(mes_to_trajectory_queue, (char *)&instruction, sizeof(mq_traj_manual_data_t), NULL, &rec_timeout);
    if (status>=0){ // if received message is correct and not timeout
        // Create local variable for current robot position
        robot_joint_position_t curr_pos;
        // read current robot position
        get_current_robot_position(curr_pos);
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

//Automatic robot control
void auto_control(){
     // if file with trajectory instruction is loaded then continue
     if (is_file_trajectory_load){
       execute_instructions();// execute next instructions
     }
 }

// Function to run trajectory generator in new thread
void* generate_trajectory(void *pVoid){
     // init variables
    is_file_trajectory_load = false;    // set no selected trajectory bool

    // Init thread priority
    int policy;     //Scheduling policy: FIFO or RR
    struct sched_param param;   //Structure of other thread parameters

    /* Read modify and set new thread priority */
    pthread_getschedparam( pthread_self(), &policy, &param);
    param.sched_priority = sched_get_priority_max(policy);  // Read minimum value for thread priority
    pthread_setschedparam( pthread_self(), policy+3, &param);   //set thread priority for this thread to max + 3

    // Enter to loop until program close
    while(get_program_state() != ProgramState::CLOSE_PROGRAM){
        // select right option to right generation mode
        switch (robot_trajectory_mode) {
            case Trajectory_control_type::UNDEFINED:    // If mode is unspecified then wait 100ms
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
                break;
            case Trajectory_control_type::AUTO:         // Enter to auto mode control
                auto_control();     // Read instruction and control robot
                break;
            case Trajectory_control_type::MANUAL:       // Enter to manual mode control
                manual_control();  // read manual control from console
                break;
            default:
                // When mode is not defined in enum then throw except
                throw std::runtime_error("Undefined trajectory generator mode");
        }
    }
    return nullptr;
 }

bool is_position_reached(){
     // variable for position array
    robot_joint_position_t setpoint_pos;
    robot_joint_position_t current_pos;
    // read current and setpoint position
    get_current_robot_position(current_pos);
    get_setpoint_robot_position(setpoint_pos);
    // Check every joint in robot
    for (size_t itr = 0; itr <  NUMBER_OF_ROBOT_JOINT; ++itr){
        float diff = setpoint_pos[itr] - current_pos[itr];
        // If one of joint is not correct then return false
        if (abs(diff ) > ROBOT_JOINT_POSITION_TOLERANCE)
            return false;
    }
    // if all joint positions are correct then return true
     return true;
 }