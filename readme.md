# Robot trajectory controller 

Program that allows you to control robot movement using Internet communication. Application allows you to control robot movement in two modes:
 - Automatic - after loading a defined trajectory
 - Manual - control read from user console


The selection of the operation mode take place after starting the program in the console.
## Autoamtic mode 
Allows you to control the robot's movement using a simple scripting language. It consists of the following instructions:
 *  UNDEFINED     --> non specified instruction, means data error
 *  GO_PTP        --> moves the robot arm in joint coordinates [Joint1, Joint2, Joint3] [deg]
 *  FINE          --> moves the robot arm in Cartesian coordinates at a given speed    [X, Y, Z, SPEED][cm, cm, cm, cm/s]
 *  WRITE_DIGITAL --> Write digital pin to robot [digital_output]
 *  IF     --> compare the digital outputs with the given condition, in case of true jump to the given address [digital_read_value, condition, instruction]
 *  JUMP          -->Jump to the given instruction [instruction_number_iterator]
 *  WAIT          -->Program will wait certain time in [ms]
 *  STOP          -->stop program
 *  EXIT          -->exit program
 *  RESUME        --> resume stop program
---
The instruction script must be loaded into the program from the text file or binary file. 

## Manual mode 
Use console input to contole robot movement. 

### Information 
In order to work properly, the application must be connected to a simulator or real robot using the internet.

The program was compiled on Ubuntu 20.04 using Clang 10. 

Example scripts are placed in the directory *sample*

GitHub: https://github.com/misiek1999/robot
