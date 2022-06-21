//
// Created by Ja on 18.05.2022.
//


#include "console_interface.h"

/*
 * Define message queue to receive messages from other thread's
 * Read data from other thread and display message in console
 * This structure is default in posix message queue
 */
mqd_t	mes_to_console_queue;
struct	mq_attr mes_to_console_queue_attr;
mq_consol_data_t  mq_console_read_data;

/*
 * Function to select trajectory generator mode
 * Read console input and set the global variable: robot_trajectory_mode from trajectory_generator.h
 */
void select_trajectory_mode(){
    uint8_t mode = 0;   // Robot trajectory generator mode
    bool correct_mode = false;  // Bool variable for loop terminator
    // Wait for the correct value
    while(!correct_mode)
    {
        // Display message in console
        write_to_console("Enter number to choose robot trajectory mode: ");
        write_to_console("[1] - Manual mode \r\n[2] - Automatic mode");
        // read single char from input and reset errors
        if( ! (std::cin >> mode) ) {
            std::cin.clear();
            std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\r\n');
            write_to_console("Invalid input. Try again");
        }
        // display a message in case of an incorrect value
        if (std::isdigit(mode) && mode - 48 > 0 && mode - 48 < 3)
            correct_mode = true; //
        else
            write_to_console("Invalid input. Try again");
        if (get_program_state() == ProgramState::CLOSE_PROGRAM)
            return;
    }
    // Change char number to enum number
    robot_trajectory_mode = static_cast<Trajectory_control_type> (mode - 48);
    // Display correct value
    if (robot_trajectory_mode == Trajectory_control_type::AUTO) {
        write_to_console("Selected  mode: Auto");
        write_to_log("Selected mode : Auto");
    }
    else{
        write_to_console("Selected  mode: Manual");
        write_to_log("Selected mode : Manual");
    }
}

/*
 * Send csting to console from other threads
 */
void write_to_console(const  char* str_to_console){
    int status = mq_send(mes_to_console_queue, (const char *)&str_to_console[0], sizeof(mq_consol_data_t), 0);
    // Catch error code
    if (status < 0 )
        std::cerr<<"MQ CONSOLE SEND ERROR: "<<status<<" -> "<< strerror(errno) <<std::endl;
}

/*
 * Function read data from queue and display it on console
 * Inline function to reduce unnecessary calls
 */
inline void display_queue_messages(){
    // Timeout for receive data 10ms
    struct timespec rec_timeout = {0, 10000000};
    // read data from queue
    int status = mq_timedreceive(mes_to_console_queue, (char *)&mq_console_read_data[0], sizeof(mq_consol_data_t), NULL, &rec_timeout);
    if (status >= 0 ) { // If message is receive successful then save it to file
        // Print read data in console
        std::cout<< mq_console_read_data << std::endl;
    }
}

#define ECHOFLAGS (ECHO | ECHOE | ECHOK | ECHONL)   // Define flags for linux terminal
/*
 * Display control instruction in manual mode in console
 */
void display_manual_instruction(){
    // Display control instruction
    write_to_console("-------------------------------------- \r\nTo move 1 joint press:  < [q] - [w] >");
    write_to_console("To move 2 joint press:  < [a] - [s] >\r\nTo move 3 joint press:  < [z] - [x] >");
    write_to_console("To write binary output press number from 1 to 8\r\nPress 'c' to exit\t\tPress 't' to stop");
    // Disable input console display
    struct termios termios;
    tcgetattr(STDIN_FILENO, &termios);
    termios.c_lflag &= ~ECHO;
    tcsetattr(STDIN_FILENO, TCSAFLUSH, &termios);
    termios .c_lflag &=~ECHOFLAGS;
}

/*
 * Read control from console
 */
void read_control_from_console(){
    // Create temp variable for income char
    char read_char;
    // Set the terminal to raw mode, works only on linux
    system("stty raw");
    read_char = getchar();    // Read char from console
    fflush(stdin);  // clear console input buffer
    // Select right option
    switch (read_char) {
        case 'c':
            set_program_state(ProgramState::CLOSE_PROGRAM);  // Close program after press 'q'
            break;
        case 't':
            stop_robot_movement(); // Stop program after press 't'
            break;
        case 'q':
            send_manual_control(ManualModeControlInstruction::joint_1_left);  // Turn 1 joint
            break;
        case 'w':
            send_manual_control(ManualModeControlInstruction::joint_1_right);  // Turn 1 joint
            break;
        case 'a':
            send_manual_control(ManualModeControlInstruction::joint_2_left);  // Turn 2 joint
            break;
        case 's':
            send_manual_control(ManualModeControlInstruction::joint_2_right);   // Turn 2 joint
            break;
        case 'z':
            send_manual_control(ManualModeControlInstruction::joint_3_left);   // Turn 3 joint
            break;
        case 'x':
            send_manual_control(ManualModeControlInstruction::joint_3_right);   // Turn 3 joint
            break;
        // If input is number from 1 to 8 then change robot digital output
        case 49 ... 56:
            // change number from 1 ... 8 to powers of 2
            set_digital_output((uint8_t) pow(read_char - 48, 2));
            break;
        default:
            break;  // Do nothing after selecting undefined key;
    }
}

/*
 * File to read predefined trajectory from file
 * also makes the read data available to the trajectory generator
 */
bool read_trajectory_from_file(std::string _path_to_file){
    // Open file
    std::ifstream file;
    file.open(_path_to_file);
    if (file.is_open()) // Check is file opened
    {
        // Lock trajectory instruction mutex
        trajectory_instruction_buffer_mutex.lock();
        // Try to read all structures from file
        while(file.good())
        {
            // Read instructions from binary file
            file.read((char*)&trajectory_instruction_buffer, sizeof(trajectory_instruction_buffer));
        }
        //Unlock the mutex when it has finished reading data from the file
        trajectory_instruction_buffer_mutex.unlock();
        is_file_trajectory_load = true;
        // Display message after successful read data from file
        write_to_console("File loaded successful! ");
        char str[32];
        sprintf(str,"Trajectory loaded from file: %s", _path_to_file.c_str());
        write_to_log(str);
    }
    else
    {
        write_to_console("Cannot open file");
        return false;
    }
    file.close();   //Close file after completed read
    return true;
}

/*
 * Function to read correct file path from console
 */
void read_file_from_console_path(){
    std::string path_to_file;
    write_to_console( "Enter the full path of the file");   // Display path request message
    bool path_is_correct = false;   // bool variable to stop reading data from console
    int res;    // variable to check file is exist
    while(!path_is_correct){
        std::cin >> path_to_file; // read file path from console
        // check file exist
        res = access(path_to_file.c_str(), R_OK);   //check file is exist
        if (res == 0) {  // If res value is non-negative then path is correct
            // Check extension of file
            if(path_to_file.substr(path_to_file.find_last_of(".") + 1) == "bin") {  // binary file
                // try to open file and read trajectory from file
                if(read_trajectory_from_file(path_to_file))
                    path_is_correct = true;
            } else {    // text file
                trajectory_instruction_buffer_mutex.lock();
                if(read_instruction_from_file(path_to_file, trajectory_instruction_buffer) == 0)
                    path_is_correct = true;
                trajectory_instruction_buffer_mutex.unlock();
            }
        }else
            write_to_console("Invalid path! Please specified valid path." );
        // If program is shutdown, then return
        if (get_program_state() == ProgramState::CLOSE_PROGRAM)
            return;
    }
    // Works only on unix system
    char buff [100];
    sprintf(buff, "Specified path is correct. Path: %s ", path_to_file.c_str());
    write_to_console(buff);
    is_file_trajectory_load = true; // set global atomic flag to true
}

/*
 * Function to communicate with console in automatic mode
 * Displays messages from threads with higher priority
 */
void console_communication_auto_mode(){
    // Read path to predefined trajectory from console
    read_file_from_console_path();
    // If program is shutdown before final setup, close function
    if (get_program_state() == ProgramState::CLOSE_PROGRAM)
        return;
    // Enter to infinity loop until the program is terminated
    while(get_program_state() != ProgramState::CLOSE_PROGRAM){
        // display queue data from other thread's
        display_queue_messages();
        // Wait 10ms to next loop iteration
        std::this_thread::sleep_for(std::chrono::milliseconds (10));
    }
}

/*
 * Function to communicate with console in manual mode
 * Displays messages from threads with higher priority, read console input
 */
void console_communication_manual_mode(){
    // Display console instruction in manual mode
    display_manual_instruction();
    // Enter to infinity loop until the program is terminated
    while(get_program_state() != ProgramState::CLOSE_PROGRAM){
        // read control from console
        read_control_from_console();
        // display queue data from other thread's
        display_queue_messages();
        // Wait 10ms to next loop iteration
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
}

/*
 * Function below display welcome message and is waiting for the appropriate mode to be specified
 */
void console_read_initialization(){
    // Select trajectory generation mode
    select_trajectory_mode();
}

/*
 * Wait for console input in emergency stop mode
 */
static void console_emergency_stop_handler(int input_signal){
    write_to_console("Press 'c' to close program.");
    char c;
    bool symbol_detected = false;
    while(!symbol_detected){
        std::cin >> c; // read char from console
        if (c == 'c')
            symbol_detected = true; // if char 'c' is detected then close application
    }
    set_program_state(ProgramState::CLOSE_PROGRAM);
}

/*
 * Wait for console input in stop mode
 */
static void console_stop_handler(int input_signal){
    write_to_console("Program stopped!\r\nPress 'c' to close program or 'r' to resume");
    char c;
    bool symbol_detected = false;
    while(!symbol_detected) {
        std::cin >> c; // read char from console
        if (c == 'c') {
            symbol_detected = true; // if char 'c' is detected then close application
            set_program_state(ProgramState::CLOSE_PROGRAM);
        } else if (c == 'r') {
            symbol_detected = true; // if char 'c' is detected then close application
            set_program_state(ProgramState::RUNNING);
            write_to_console("\rProgram resumed");
        }
    }
}

/*
 * Read console input in separate thread
 */
void * read_console_input(void *pVoid){
    // Init thread priority
    int policy;     //Scheduling policy: FIFO or RR
    struct sched_param param;   //Structure of other thread parameters
    /* Read modify and set new thread priority */
    pthread_getschedparam( pthread_self(), &policy, &param);
    param.sched_priority = sched_get_priority_min(policy)+1;  // Read minimum value for thread priority
    pthread_setschedparam( pthread_self(), policy, &param);   //set almost minimum thread priority for this thread
    // set cancel mode in this thread
    pthread_setcanceltype(PTHREAD_CANCEL_ASYNCHRONOUS,nullptr);

    // Accept only stop and emergency stop signal in this thread
    sigset_t read_mask;
    sigfillset(&read_mask);
    sigdelset(&read_mask, SIGNAL_STOP_CONSOLE); // Block stop signal to console
    sigdelset(&read_mask, SIGNAL_EMERGENCY_STOP_CONSOLE); // Block emergency stop signal to console
    pthread_sigmask(SIG_SETMASK, &read_mask, nullptr); // Add signals to supervisor_mask

    // set handler for upper signals
    struct sigaction emergency_stop_action;
    emergency_stop_action.sa_handler = console_emergency_stop_handler;
    sigemptyset(&emergency_stop_action.sa_mask);
    emergency_stop_action.sa_flags = 0;
    // Register signal handler for EMERGENCY_STOP_SIGNAL and INTERPOCESS_CLOSE_PROGRAM_SIGNAL
    if (sigaction(SIGNAL_EMERGENCY_STOP_CONSOLE, &emergency_stop_action, nullptr) < 0) {
        std::cerr <<  "Cannot register EMERGENCY STOP CONSOLE handler.\r\n";
        throw std::runtime_error("Cannot register EMERGENCY STOP CONSOLE handler");
    }

    struct sigaction interprocess_close_action;
    interprocess_close_action.sa_handler = console_stop_handler;
    sigemptyset(&interprocess_close_action.sa_mask);
    interprocess_close_action.sa_flags = 0;
    if (sigaction(SIGNAL_STOP_CONSOLE, &interprocess_close_action, nullptr) < 0) {
        std::cerr <<  "Cannot register STOP CONSOLE handler.\r\n";
        throw std::runtime_error("Cannot register EMERGENCY STOP CONSOLE handler");
    }

    //Initialize console input
    console_read_initialization();

    // Select right program mode
    if (get_program_state() != ProgramState::CLOSE_PROGRAM)

        // Select mode of communication
        switch(robot_trajectory_mode)
        {
            case Trajectory_control_type::AUTO:     // Automatic mode case
                console_communication_auto_mode();  // Select function to communication in auto mode
                break;
            case Trajectory_control_type::MANUAL:   // Manual mode case
                console_communication_manual_mode();// Select function to communication in manual mode
                break;
            default: //Undefined case
                write_to_console("Undefined mode! Stop application."); // Display error message
                throw std::runtime_error("Undefined mode error!"); //Stop program
        }
    return 0;
}

/*
 * Function to communication with console
 * Initialise the data exchange between threads and set the necessary parameters
 */
void * console_interface(void *pVoid){
    // Display welcome message
    std::cout <<"Welcome to robot trajectory generator!" <<std::endl;

    // Init thread priority
    int policy;     //Scheduling policy: FIFO or RR
    struct sched_param param;   //Structure of other thread parameters
    /* Read modify and set new thread priority */
    pthread_getschedparam( pthread_self(), &policy, &param);
    param.sched_priority = sched_get_priority_min(policy) +2;  // Read minimum value for thread priority
    pthread_setschedparam( pthread_self(), policy, &param);   //set almost minimum +2  thread priority for this thread

    // Create thread to read console input
    pthread_t console_read_thread;
    pthread_attr_t console_read_thread_attr;
    pthread_attr_init(&console_read_thread_attr);
    pthread_attr_setschedpolicy(&console_read_thread_attr, SCHED_FIFO);
    if (pthread_create( &console_read_thread, &console_read_thread_attr, read_console_input, nullptr)) {
        std::cerr <<  "Cannot create console read thread.\r\n";
        throw std::runtime_error("Cannot create console raed thread");
    }

    // display messages from other threads until program end
    while(get_program_state() != ProgramState::CLOSE_PROGRAM){
        // display queue data from other thread's
        display_queue_messages();
    }

    // terminate console input thread
    pthread_cancel(console_read_thread);

    // Display exit message in console
    std::cout<<"Exit program.\r\n";
    return 0;
}



