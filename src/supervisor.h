//
// Created by Ja on 17.05.2022.
//

#ifndef ROBOT_SUPERVISOR_H
#define ROBOT_SUPERVISOR_H
// Include libraries
#include "signal.h"
#include "thread"
#include "atomic"
#include "iostream"
#include "logger.h"
#include "console_interface.h"
#include "trajectory_generator.h"

// External close signal
#define EXTERNAL_CLOSE_PROGRAM SIGINT
// Interprocess close signal
#define INTERPOCESS_CLOSE_PROGRAM_SIGNAL SIGUSR1
//Interprocess emergency stop signal
#define EMERGENCY_STOP_SIGNAL SIGRTMIN

/*
 * Define global enum of program state
 * We define 4 state:
 * 0 - RUNNING          - normal work of robot (DEFAULT)
 * 1 - STOP             - stop the movement of the robot with the possibility to continue the movement
 * 2 - EMERGENCY STOP   - stop the robot's movement, no continuation possible
 * 3 - CLOSE_PROGRAM    - close program, stop all thread
*/
enum class ProgramState{
    RUNNING = 0,
    STOP = 1,
    EMERGENCY_STOP = 2,
    CLOSE_PROGRAM = 3
};

// lock robot movement
void stop_robot_movement();

// Get program state
const ProgramState get_program_state();

// Set program state
void set_program_state(const ProgramState _state_to_set);

/*
 * Supervisor thread function
 * Run this function in separately thread with the highest priority
 * Function catch signals with other process
*/
void * program_supervisor(void *pVoid);

#endif //ROBOT_SUPERVISOR_H
