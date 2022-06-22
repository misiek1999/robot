//
// Created by Ja on 17.05.2022.
//

#include "supervisor.h"

// Program state variable
std::atomic <ProgramState> program_state;

// change setpoint position to current position to stop robot movement
void stop_robot_movement(){
    // Change program state to stop
    set_program_state(ProgramState::STOP);
    // read current robot position
    robot_joint_position_t curr_pos;
    get_current_robot_position(curr_pos);
    // lock robot at current position
    write_setpoint_robot_position(curr_pos);
    // send stop signal to rad console thead
    kill(getpid(), SIGNAL_STOP_CONSOLE);
}

// Function to close program when closing signal was detected
static void exit_handler(int input_signal){
    // Send message to log and console
    write_to_console("External close signal detected!");
    write_to_log("External close signal detected!");
    // set close state in program
    set_program_state(ProgramState::CLOSE_PROGRAM);
}

// Function to close program when interprocess signal was detected
static void interprocess_exit_handler(int input_signal){
    // Function to stop sigsuspend loop in supervisor thread
}

// Function for signal handler
static void emergency_stop_handler(int input_signal){
    // stop robot movement
    stop_robot_movement();
    set_program_state(ProgramState::EMERGENCY_STOP);
    // Send message to log and console
    write_to_console("Emergency stop detected!");
    write_to_log("Emergency stop detected!");
    // send emergency stop signal to console read thread
    kill(getpid(), SIGNAL_EMERGENCY_STOP_CONSOLE);
}

// Get program state
const ProgramState get_program_state(){
    return program_state;
}

// Set program state
void set_program_state(const ProgramState _state_to_set){
    program_state = _state_to_set;
    // Check if selected state is CLOSE_PROGRAM
    if (program_state == ProgramState::CLOSE_PROGRAM)
        kill(getpid(), INTERPOCESS_CLOSE_PROGRAM_SIGNAL);    // send interprocess signal to stop supervisor thread
}

// Supervisor thread function
void * program_supervisor(void *pVoid){
    // Init thread priority
    int policy;     //Scheduling policy: FIFO or RR
    struct sched_param param;   //Structure of other thread parameters

    /* Read modify and set new thread priority */
    pthread_getschedparam( pthread_self(), &policy, &param);
    param.sched_priority = sched_get_priority_max(policy);  // Read minimum value for thread priority
    pthread_setschedparam( pthread_self(), policy, &param);   //set maximum thread priority for this thread

    // Accept all signals in this thread
    sigset_t supervisor_mask;
    sigemptyset(&supervisor_mask);
    sigaddset(&supervisor_mask, SIGNAL_STOP_CONSOLE); // Block stop signal to console
    sigaddset(&supervisor_mask, SIGNAL_EMERGENCY_STOP_CONSOLE); // Block emergency stop signal to console
    sigaddset(&supervisor_mask, SIGNAL_WAKE_UP_TRAJECTORY_THREAD); // Block woke up signal to trajectory generator thread
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