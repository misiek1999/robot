//
// Created by Ja on 16.05.2022.
//
// File include function to communicate with robot or simulator via Internet
// UDP protocol is used in this implementation
#include "controll_interface.h"
// Define posix variables used to communicate with robot using UDP
// UDP socket
struct sockaddr_in socket_udp_control_addr; //udp communication params
int control_socket; //socket for udp communication
int addr_length;
#define UDP_CONTROL_PORT 22222
#define UDP_CONTROL_ADDRESS 0x7fffffff

// Robot position & coresponding mutex's
robot_joint_position_t setpoint_robot_position;
robot_joint_position_t current_robot_position;

// Binary robot output and input global atomic variable
robot_binary_interface_t robot_output_binary;
robot_binary_interface_t robot_input_binary;
/*
 * Global variable to check is manipulator reach setpoint position
 * False -when position is not rached, true - position reached
*/
//extern bool is_manipulator_reach_setpoint_position;

// mutex's
std::mutex setpoint_robot_position_mutex;
std::mutex current_robot_position_mutex;
std::mutex is_manipulator_reach_setpoint_position_mutex;

// Initialize communication with robot or simulator
void initialize_robot_communication(){
    // Create socket using UDP
    control_socket = socket(PF_INET, SOCK_DGRAM, 0);
    // Check is socket create property
    if (control_socket == -1) {

        write_to_log("Cannot create socket UDP");
        throw std::runtime_error("Error during create socket");
    }
    // Initialize socket address to 0
    memset(&socket_udp_control_addr, 0, sizeof(socket_udp_control_addr));
    // Set socket address parameters
    socket_udp_control_addr.sin_family = AF_INET; //set IPV4 protocol
    socket_udp_control_addr.sin_port = htons(UDP_CONTROL_PORT); // set udp port
    socket_udp_control_addr.sin_addr.s_addr = INADDR_ANY;   // bind to all address
}

// close robot connection
void close_robot_communication(){
    close(control_socket);
}

// Read robot joint position and controll signals
void read_robot_position(){
    PacketToReceive received_packet;
    // receive feedback message from robot
    ssize_t status= recvfrom(control_socket,  &received_packet, sizeof(PacketToReceive), MSG_WAITALL,
                     (struct sockaddr *) &socket_udp_control_addr, &addr_length);
    // check is message is not received property
    if (status < 0){
        write_to_log("Cannot received packet");
    }else{  //if received packet is correct
        // create variable for current position and digital input
        robot_digital_data_type digit_in;
        digit_in = received_packet.received_digital_signals;
        robot_output_binary = digit_in; //atomic variable
        // lock robot current joint position mutex
        current_robot_position_mutex.lock();
        // read current robot position
        memcpy(current_robot_position, received_packet.received_position, sizeof(received_packet.received_position));
        // unlock robot current joint position mutex
        current_robot_position_mutex.unlock();
    }
}

// Write robot position to reach
void write_robot_position(){
    // Packet to send
    PacketToSend packet_to_send;
    // create variable for setpoint position and digital output
    robot_joint_position_t set_pos_out;
    robot_digital_data_type digit_out;
    // read joint setpoint position and digital output
    get_setpoint_robot_position(packet_to_send.setpoint_position);
    digit_out = robot_input_binary;
    packet_to_send.send_digital_signals = digit_out;
    // send packet to robot or simulator using UDP
    ssize_t status = sendto(control_socket, &packet_to_send, sizeof(PacketToSend), 0x800	,// MSG_CONFIRM value
                            (const struct sockaddr *) &socket_udp_control_addr, sizeof(socket_udp_control_addr));
    // check is message is not sent property
    if (status < 0){
        write_to_log("Cannot send packet");
    }
}

// Write and read robot position and controll signal;
void* communicate_with_robot(void* _arg_input) {
    // Change thread priority
    // Init thread priority
    int policy;     //Scheduling policy: FIFO or RR
    struct sched_param param;   //Structure of other thread parameters
    /* Read modify and set new thread priority */
    pthread_getschedparam( pthread_self(), &policy, &param);
    param.sched_priority = sched_get_priority_max(policy);  // Read minimum value for thread priority
    pthread_setschedparam( pthread_self(), policy+1, &param);   //set max+1 thread priority for this thread
    // Initialize communication with robot
    initialize_robot_communication();
    // Enter to infinite loop until close program
    //TODO: implement this function
    while(get_program_state() != ProgramState::CLOSE_PROGRAM){


    }

    return 0;
}

// Function to write digital output in robot
void write_digital_output(const robot_digital_data_type _input){
    // Change digital output with xor current robot digital output with input
    robot_input_binary = (_input ^ robot_input_binary);
}


// read robot current joint position
void read_current_robot_position(robot_joint_position_t _current_position){
    // lock robot current joint position mutex
    current_robot_position_mutex.lock();
    // read current robot position
    memcpy(_current_position, current_robot_position, sizeof(robot_joint_position_t));
    // unlock robot current joint position mutex
    current_robot_position_mutex.unlock();
}

// write robot setpoint joint position
void write_setpoint_robot_position(const robot_joint_position_t _setpoint_position){
    // lock rocot setpoint position mutex
    setpoint_robot_position_mutex.lock();
    // write new robot position
    memcpy(setpoint_robot_position, _setpoint_position, sizeof(robot_joint_position_t));
    // unlock rocot setpoint position mutex
    setpoint_robot_position_mutex.unlock();
}

// get robot setpoint position
void get_setpoint_robot_position(robot_joint_position_t _setpoint_position){
    // lock rocot setpoint position mutex
    setpoint_robot_position_mutex.lock();
    // copy setpoint robot position
    memcpy(_setpoint_position, setpoint_robot_position, sizeof(robot_joint_position_t));
    // unlock rocot setpoint position mutex
    setpoint_robot_position_mutex.unlock();
}