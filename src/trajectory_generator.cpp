//
// Created by Ja on 21.05.2022.
//

#include "trajectory_generator.h"

std::atomic<Trajectory_control_type>  robot_trajectory_mode;

// Global instruction buffer
TrajectoryInstruction trajectory_instruction_buffer[MAX_INSTRUCTION_PER_TRAJECTORY];
// mutex for trajectory instruction buffer
std::mutex trajectory_instruction_buffer_mutex;

// Message queue for data from console to trajectory generator
mqd_t	mes_to_trajectory_queue;
struct	mq_attr mes_to_trajectory_queue_attr;
