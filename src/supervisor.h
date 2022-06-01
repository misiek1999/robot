//
// Created by Ja on 17.05.2022.
//

#ifndef ROBOT_SUPERVISOR_H
#define ROBOT_SUPERVISOR_H
// Include liblaries
#include "thread"
#include "atomic"
#include "iostream"
/*
 * Define global enum of controller state
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

// Get program state
const ProgramState get_program_state();
// Set program state
void set_program_state(const ProgramState _state_to_set);

//Supervisor thread function
void * program_supervisor(void *pVoid);

#endif //ROBOT_SUPERVISOR_H
