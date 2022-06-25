//
// Created by Ja on 16.05.2022.
//

// File include function to communicate with robot or simulator via Internet
// UDP protocol is used in this implementation
#include "controll_interface.h"
// Define port numbers to  communicate with robot using UDP
#define UDP_CONTROL_PORT_RECEIVE 22222
#define UDP_CONTROL_PORT_SEND 22223

// UDP socket send
struct sockaddr_in socket_udp_control_send_addr; //udp communication params to send packet
int control_socket_send; //socket for udp communication

// UDP socket receive
struct sockaddr_in socket_udp_control_receive_addr; //udp communication params to send packet
struct sockaddr server_addr;
int control_socket_receive; //socket for udp communication
socklen_t addr_length_receive;

// Variables to timer
// Structure with time values
struct itimerspec timer_control_spec;
// Timer variable
timer_t	timer_to_control;
// Signal variable
struct sigevent timer_control_signal;

// Robot position & corresponding mutex's
robot_joint_position_t setpoint_robot_position;
robot_joint_position_t current_robot_position;

// Binary robot output and input global atomic variable
robot_binary_interface_t robot_output_binary;
robot_binary_interface_t robot_input_binary;
// mutex's
std::mutex setpoint_robot_position_mutex;
std::mutex current_robot_position_mutex;

// Initialize communication with robot or simulator
void initialize_robot_communication(){
    // Create socket using UDP to send packet
    control_socket_send = socket(PF_INET, SOCK_DGRAM, 0);
    // Check is socket create property
    if (control_socket_send == -1) {
        write_to_log("Cannot create send socket UDP");
        throw std::runtime_error("Error during create send socket");    // throw error and end program
    }
    // Initialize socket address to 0
    memset(&socket_udp_control_send_addr, 0, sizeof(socket_udp_control_send_addr));
    // Set socket address parameters
    socket_udp_control_send_addr.sin_family = AF_INET; //set IPV4 protocol
    socket_udp_control_send_addr.sin_port = htons(UDP_CONTROL_PORT_SEND); // set udp port
    socket_udp_control_send_addr.sin_addr.s_addr = INADDR_ANY; // bind to all address

    // Create socket using UDP to receive packet
    control_socket_receive = socket(AF_INET, SOCK_DGRAM, 0);
    // Check is socket create property
    if (control_socket_receive == -1) {
        write_to_log("Cannot create received socket UDP ");
        throw std::runtime_error("Error during create received socket");
    }
    // Initialize socket address to 0
    memset(&socket_udp_control_receive_addr, 0, sizeof(socket_udp_control_receive_addr));
    // Set socket address parameters
    socket_udp_control_receive_addr.sin_family = AF_INET; //set IPV4 protocol
    socket_udp_control_receive_addr.sin_port = htons(UDP_CONTROL_PORT_RECEIVE); // set udp port
    socket_udp_control_receive_addr.sin_addr.s_addr = INADDR_ANY;   // bind to all address

    // set 1sec timeout for receive
    struct timeval timeout;
    timeout.tv_sec = 1;
    timeout.tv_usec = 0;
    setsockopt(control_socket_receive, SOL_SOCKET, SO_RCVTIMEO,  (const char*)&timeout, sizeof(timeout));
    addr_length_receive = sizeof(server_addr);

    // Bind socket to socket address struct
    if(bind(control_socket_receive, (struct sockaddr *)&socket_udp_control_receive_addr,
            sizeof(socket_udp_control_receive_addr)) == -1) {
        close(control_socket_receive);
        write_to_log("Cannot bind UDP receive socket");
        throw std::runtime_error("cannot bind socket");
    }
}

// close robot connection sockets
void close_robot_communication(){
    close(control_socket_send);
    close(control_socket_receive);
}

// Read robot joint position and controll signals
void receive_robot_position_packet(){
    PacketToReceive received_packet;
    // receive feedback message from robot
    ssize_t status= recvfrom(control_socket_receive, &received_packet, sizeof(PacketToReceive),
                             MSG_WAITALL,&server_addr, &addr_length_receive);
    // check is message is received property
    if (status >= 0){
        // create variable for current position and digital input
        robot_digital_data_t digit_in;
        digit_in = received_packet.received_digital_signals;
        robot_output_binary = digit_in; //atomic variable
        // lock robot current joint position mutex
        current_robot_position_mutex.lock();
        // read current robot position
        memcpy(current_robot_position, received_packet.received_position, sizeof(received_packet.received_position));
        // unlock robot current joint position mutex
        current_robot_position_mutex.unlock();
        // unblock trajectory thread in AUTO mode
        if(robot_trajectory_mode == Trajectory_control_type::AUTO){
            // check is set point position is reached
            bool current_position_status = is_position_reached();
            // if set point position is reached then unlock barrier
            if(current_position_status == true and new_position_selected == true)
                pthread_barrier_wait(&trajectory_barrier);
        }
    }
}

// Write robot position to reach
void send_robot_position_packet(){
    // Init thread priority
    int policy;     //Scheduling policy: FIFO or RR
    struct sched_param param;   //Structure of other thread parameters
    /* Read modify and set new thread priority */
    pthread_getschedparam( pthread_self(), &policy, &param);
    param.sched_priority = sched_get_priority_max(policy) - 1;  // Read max value for thread priority
    pthread_setschedparam( pthread_self(), policy, &param);   //set max -1 thread priority for this thread
    // Packet to send
    PacketToSend packet_to_send;
    // create variable for setpoint position and digital output
    robot_digital_data_t digit_out;
    // read setpoint joint position and digital output
    get_setpoint_robot_position(packet_to_send.setpoint_position);
    digit_out = robot_input_binary;
    packet_to_send.send_digital_signals = digit_out;
    // send packet to robot or simulator using UDP
    ssize_t status = sendto(control_socket_send, &packet_to_send, sizeof(packet_to_send), 0x800	,// MSG_CONFIRM value
                            (const struct sockaddr *) &socket_udp_control_send_addr, sizeof(socket_udp_control_send_addr));
    // check is message is not sent property
    if (status < 0){
        write_to_console("Cannot send UDP packet");
        write_to_log("Cannot send packet");
    }
}

// Write and read robot position and control signal;
void* communicate_with_robot(void* _arg_input) {
    // Change thread priority
    // Init thread priority
    int policy;     //Scheduling policy: FIFO or RR
    struct sched_param param;   //Structure of other thread parameters

    /* Read modify and set new thread priority */
    pthread_getschedparam( pthread_self(), &policy, &param);
    param.sched_priority = sched_get_priority_max(policy) - 2;  // Read max value for thread priority
    pthread_setschedparam( pthread_self(), policy, &param);   //set max -2 thread priority for this thread

    // Init posix thread barrier
    pthread_barrier_init(&trajectory_barrier, NULL, 2);

    // Initialize communication with robot
    initialize_robot_communication();

    //Init thread to send packet every 20ms
    pthread_attr_t send_packet_thread_attr;
    pthread_attr_init(&send_packet_thread_attr);
    pthread_attr_setschedpolicy(&send_packet_thread_attr, SCHED_FIFO);

    // Init timer to send udp packet every 20ms
    // Try to launch control thread with timer interrupt
    /* Initialize event to send signal SIGRTMAX */
    timer_control_signal.sigev_notify = SIGEV_THREAD;
    timer_control_signal.sigev_notify_function = reinterpret_cast<void (*)(sigval_t)>(send_robot_position_packet);
    timer_control_signal.sigev_notify_attributes = &send_packet_thread_attr;

    int status; // status for timer create
    /* Create timer */
    if ((status = timer_create(CLOCK_REALTIME, &timer_control_signal, &timer_to_control))) {
        std::cerr <<  "Error creating timer : "<<  status <<std::endl;
        throw std::runtime_error("Cannot crate timer");
    }

    /* Set up timer structure with time parameters */
    timer_control_spec.it_value.tv_sec = 0;
    timer_control_spec.it_value.tv_nsec = CONTROL_TIME_PERIOD * 1000000;//20ms timer expiration
    timer_control_spec.it_interval.tv_sec = 0;
    timer_control_spec.it_interval.tv_nsec = CONTROL_TIME_PERIOD * 1000000;//20ms time to next interrupt

    /* Change timer parameters and run */
    timer_settime( timer_to_control, 0, &timer_control_spec, NULL);

    // Enter to infinite loop until close program to receive packet with timeout 1s
    while(get_program_state() != ProgramState::CLOSE_PROGRAM){  //stop if program is shutdown
        receive_robot_position_packet();    // Try to receive udp packet with 1s timeout
      }

    //stop timer
    timer_delete(timer_to_control);

    //close socket and other communication with robot
    close_robot_communication();
    return 0;
}

// Function to write digital output in robot
void set_digital_output(const robot_digital_data_t _input){
    // Change digital output with xor current robot digital output with input
    robot_input_binary = (_input ^ robot_input_binary);
}

// get digital output
robot_digital_data_t get_digital_output(){
    return robot_output_binary;
}

// read robot current joint position
void get_current_robot_position(robot_joint_position_t _current_position){
    // lock robot current joint position mutex
    current_robot_position_mutex.lock();
    // read current robot position
    memcpy(_current_position, current_robot_position, sizeof(robot_joint_position_t));
    // unlock robot current joint position mutex
    current_robot_position_mutex.unlock();
}

// write robot setpoint joint position
void set_setpoint_robot_position(const robot_joint_position_t _setpoint_position){
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