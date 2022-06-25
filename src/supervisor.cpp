//
// Created by Ja on 17.05.2022.
//

#include "supervisor.h"

// Program state
std::atomic <ProgramState> program_state;

// Robot set-point position in stop mode
robot_joint_position_t set_point_position_in_stop_mode;

// supervisor thread handler
pthread_t supevisor_thread_id;
// change set-point position to current position and save set-point position
void lock_robot_movement(){
    // read set-point robot position
    get_setpoint_robot_position(set_point_position_in_stop_mode);
    // read current robot position
    robot_joint_position_t curr_pos;
    get_current_robot_position(curr_pos);
    // lock robot at current position
    set_setpoint_robot_position(curr_pos);
}

//  Stop robot movement and change program state to STOP
void stop_robot(){
    // change program state to STOP
    set_program_state(ProgramState::STOP);
    // stop robot movement
    lock_robot_movement();
}

// resume robot movement and change program state to RUNNING
void resume_robot_movement(){
    // reasume program only in stop mode 
    if(get_program_state() == ProgramState::STOP){
        // load previous robot set-point position
        set_setpoint_robot_position(set_point_position_in_stop_mode);
        // set program state to running
        set_program_state(ProgramState::RUNNING);
    }
}

// Function to close program when closing signal was detected
static void exit_handler(int input_signal){
    // Send message to log and console
    write_to_console("External close signal detected!");
    write_to_log("External close signal detected!");
    // set close program state
    set_program_state(ProgramState::CLOSE_PROGRAM);
}

// Empty function to close program when interprocess signal was detected
static void interprocess_exit_handler(int input_signal){
    // Function to stop sigsuspend in supervisor thread
}

// Function for emergency stop signal
static void emergency_stop_handler(int input_signal){
    // change program state to EMERGENCY STOP
    set_program_state(ProgramState::EMERGENCY_STOP);
    // lock robot movement
    lock_robot_movement();
    // Send message to log and console
    write_to_console("External emergency stop detected!");
    write_to_log("External emergency stop detected!");
}

// Get program state
const ProgramState get_program_state(){
    return program_state;
}

// Set program state
void set_program_state(const ProgramState _state_to_set){
    program_state = _state_to_set;
    // Send interprocess signal to process threads
    switch (program_state) {
        case ProgramState::CLOSE_PROGRAM:
            pthread_kill(supevisor_thread_id, INTERPOCESS_CLOSE_PROGRAM_SIGNAL);// send interprocess signal to stop supervisor thread
            break;
        case ProgramState::EMERGENCY_STOP:
            kill(getpid(), SIGNAL_EMERGENCY_STOP_CONSOLE);      // send emergency stop signal to console thread
            break;
        case ProgramState::STOP:
            kill(getpid(), SIGNAL_STOP_CONSOLE);                // send stop signal to rad console thead
            break;
        default:
            break;
    }
}

// Supervisor thread function
void * program_supervisor(void *pVoid){
    // get supervisor pthread id
    supevisor_thread_id = pthread_self();
    // Init thread priority
    int policy;     //Scheduling policy: FIFO or RR
    struct sched_param param;   //Structure of other thread parameters

    /* Read modify and set new thread priority */
    pthread_getschedparam( pthread_self(), &policy, &param);
    param.sched_priority = sched_get_priority_max(policy);  // Read max value for thread priority
    pthread_setschedparam( pthread_self(), policy, &param);   //set maximum thread priority for this thread

    // Accept all signals in this thread
    sigset_t supervisor_mask;
    sigemptyset(&supervisor_mask);
    sigaddset(&supervisor_mask, SIGNAL_STOP_CONSOLE); // Block stop signal to console
    sigaddset(&supervisor_mask, SIGNAL_EMERGENCY_STOP_CONSOLE); // Block emergency stop signal to console
    pthread_sigmask(SIG_SETMASK, &supervisor_mask, NULL); // Add signals to supervisor_mask

    // Prepare signal action struct for function handler
    struct sigaction emergency_stop_action;
    emergency_stop_action.sa_handler = emergency_stop_handler;
    sigemptyset(&emergency_stop_action.sa_mask);
    emergency_stop_action.sa_flags = 0;
    // Register signal handler for EMERGENCY_STOP_SIGNAL and INTERPOCESS_CLOSE_PROGRAM_SIGNAL
    if (sigaction(EMERGENCY_STOP_SIGNAL, &emergency_stop_action, NULL) < 0) {
        std::cerr <<  "Cannot register EMERGENCY STOP handler.\n";
        return 0;
    }

    struct sigaction interprocess_close_action;
    interprocess_close_action.sa_handler = interprocess_exit_handler;
    sigemptyset(&interprocess_close_action.sa_mask);
    interprocess_close_action.sa_flags = 0;
    if (sigaction(INTERPOCESS_CLOSE_PROGRAM_SIGNAL, &interprocess_close_action, NULL) < 0) {
        std::cerr <<  "Cannot register CLOSE PROGRAM handler.\n";
        return 0;
    }

    // setup signal SIGINT handler and quit function
    struct sigaction close_act;
    close_act.sa_handler = exit_handler;    // set close function handle
    sigemptyset(&close_act.sa_mask);        // init signal struct with zeros
    close_act.sa_flags = 0;
    // Register signal handler for SIGINT
    if (sigaction(EXTERNAL_CLOSE_PROGRAM, &close_act, NULL) < 0) {
        std::cerr <<  "Cannot register SIGINT handler.\n";
        return 0;
    }

    // Enter to loop and wake up only receive any signal
    while(get_program_state() != ProgramState::CLOSE_PROGRAM){
        sigsuspend(&supervisor_mask);
    }

    // Stop this thread
    return 0;
}