/* ************************************************************************** */
/** 
  @Project
     ECE 4534 Embedded Design
     Team 4 - 11:00 TR

  @File Name
    proj_definitions.h

  @Summary
     Location of key constants and values used throughout system
 */
/* ************************************************************************** */

#ifndef _PROJ_DEFINITIONS_H    /* Guard against multiple inclusion */
#define _PROJ_DEFINITIONS_H


/* ************************************************************************** */
/* ************************************************************************** */
/* Section: Included Files                                                    */
/* ************************************************************************** */
/* ************************************************************************** */

/* Provide C++ Compatibility */
#ifdef __cplusplus
extern "C" {
#endif


    /* ************************************************************************** */
    /* ************************************************************************** */
    /* Section: Constants                                                         */
    /* ************************************************************************** */
    /* ************************************************************************** */

// Message format. Constant through system. 
#define MSG_START 0x7E   // '~'  126
#define MSG_STOP 0x2E    // '.'  46
#define MSG_LENGTH 10    // START - TYPE - COUNT - DATA x6 - STOP
    
    
//Boolean operators
#define FALSE 0 
#define TRUE 1
    
// MESSAGE TYPES
// Coordinator
#define TRACKED_OBJECT_AMOUNT 20 //can track 20 objects, these are obstacles & rovers
#define REFRESH_RATE 5 //Will issue 5 move commands before refreshing rover location with map data
    
#define TYPEC_LR_SENSOR_TO_FR 0x8F  //143 - Token found message to send to Follower Rover
#define TYPEC_LR_HANDSHAKE 0x4f     //79  - Confirm 'Token found' message receive to Lead Rover
#define TYPEC_MOTOR_CONTROL 0x5E   //94 - Rover motor control
#define TYPEC_TOKEN_NUMBER 0x5D //93 - RPI communicating number of tokens
#define TYPEC_ACK_TOKEN 0x50 //80 - Coordinator acknoledgment 
#define TYPEC_CLEAR_MAP 0x51 //81 - Tell RPI or Coordinator to clear its map cache
#define TYPEC_UPDATE_MAP 0x52 //82 - Tell RPI to display the map
#define TYPEC_MAP_DATA 0x74 //116 - Tell the RPI or coordinator about a map object
#define TYPEC_END_MAP_DATA 0x5A //90 - The RPI tells the coordinator thats the last of the map data    
#define TYPEC_MAKE_MOVE 0x54 //84 - Tell the Coordinator to make the LR move 
#define TYPEC_TOKEN_FOUND 0x56 //86 - Tell the Coordinator that a token has been found    
#define TYPEC_LR_MOVE_COMPLETE 0x57 //87 - LR tells the Coordinator it finished moving. Could include an AMT? 
#define TYPEC_CONTINUE 0x58 //88 Tell the coordinator to continue to the next state     
#define TYPEC_SKIP 0x59 //89 Tell the coordinator to skip the state its waiting in
    
#define TYPEC_ACK_SENSOR_DATA 0x71 //113 Tell the SENSOR you've received the map data it sent. 
#define ACK_FROM_SENSOR 0x71 //113 decimal 
    
#define TYPEC_TOKEN_FOUND_ACK 0x60 //96 Tell the FR that we have ACKNOWLEDGED it has picked up a token
       
#define TYPE_SENSOR_MULTIPLEREQUESTED 0x77 //119 Tell the Sensor to do multiple pans 
#define TYPE_SENSOR_SINGLEREQUESTED 0x76 //118 Tell the sensor to do one pan and report back
   
#define TYPEC_MOVE_FORWARD 0x30 //48 Tell the rover to move forward
#define TYPEC_TURN_LEFT 0x31 //49 Tell the rover spin to the left
#define TYPEC_TURN_RIGHT 0x32 //50 Tell the rover to spin to the right
#define TYPEC_TURN_UP 0x34 //52 Tell the rover to spin to up
#define TYPEC_TURN_DOWN 0x35  //53 Tell the rover to spin to down
#define TYPEC_ROTATE 0x36 //54 //Tell the rover to rotate to the left or right by x degrees
    
//#define TYPEC_UPDATE_COORD_LIST 0x54 //84 - Updates the coordinate list on the RPI    
    
// Lead Rover
#define TYPE_LR_SENSOR 0xF7     //247 - Lead Rover line sensor reading
#define TYPE_LR_ENCODER 0x7F    //127 - Lead Rover encoder report data
// Follower Rover
#define TYPE_FR_ENCODER 0x6F    //111 - Motor encoder report data
#define TYPE_FR_DIST 0x5F       //95  - Follower forward distance data
#define TYPE_FR_IR 0x9F         //159 - Follower IR receiver data
#define TYPEC_GO 0x99 //153 - Give the follower the "GO" signal
#define TYPE_FR_ACK 0xCD //205 - ACK from the FR
#define TYPE_FR_FOUND_TOKEN 0xF1 //241 - Token found message + data from the FR  
  
    
// Sensors
#define TYPE_SENSOR_DIST 0xE7        //231 - Field IR distance sensor data
#define TYPE_SENSOR_IR_REC 0xE8      //232 - Field IR receiver data
// Shared in all components
#define TYPE_MESSAGE_RECEIVE_DATA 0xFE    //254 - Reports UART received message data 
    
// Message send disablers
#define CUT_SENSOR 0
#define CUT_MOTOR 0
#define CUT_RECEIVE_DATA 0
    
// SIMULATED ERROR CONSTANTS
// Add duplicate byte inside message once every x bytes. NORMAL OPERATION = 1
#define ADD_MESSAGE_DIV 1      
// Remove byte inside message once every x bytes. NORMAL OPERATION = 1.
#define BREAK_MESSAGE_DIV 1 
// Message send rate multiplication. NORMAL OPERATION = 2.
#define MESSAGE_RATE_DIV 2     
// Skip entire message (including count) once every x messages. NORMAL OPERATION = 1
#define MESSAGE_SKIP_DIV 1
    
#define PI 3.14159265
    
//ROVER DATA
#define ROVER_WIDTH 4 //width of the rover in centimeters     
#define BUFFER_ZONE 6 //Amount of buffer to give objects in cm.     
#define MAX_MOVE 30 //Max amount of centimeters per one move
#define PATH_WIDTH 5 //width of the path, lower number means more passes 
    
//OBSTACLE DATA
#define OBSTACLE_DIAMETER 10 //diameter of the obstacles 
    
//SENSOR DATA
#define ERROR_AMT 4 //amount of expected error
#define TOLERANCE 3 //Amount of tolerance to give to positions rover is trying to navigate to
    
//CURRENT DIRECTION OF THE LEAD ROVER
#define LR_UNKNOWN 0
#define LR_UP 1
#define LR_DOWN 2
#define LR_RIGHT 3
#define LR_LEFT 4    
    
// SYSTEM MESSAGE RATES (message every 2x milliseconds)
#define LR_MOTOR_TIMER_RATE 50      // Motor message send rate (ms)
#define LR_SENSOR_TIMER_RATE 50      // Sensor message send rate (ms)
    
#define FR_MOTOR_TIMER_RATE 50  // Follower rover motor message send rate
#define FR_IR_TIMER_RATE 20     // Follower rover IR receive rate
#define FR_DIST_TIMER_RATE 50   // Follower rover distance sensor rate
#define PROC_TIMER_RATE 50      // Process task timer rate
    
#define SEND_TIMER_RATE 200   //in ms  
#define PROCESS_TIMER_RATE 100
#define DIST_TIMER_RATE 100     // Receive code send to motor rate (for MS#2)
#define MOTOR_CTRL_TIMER_RATE 50  // Coordinator -> Lead Rover instruction timer rate
#define RECEIVE_TIMER_RATE 5000    // Message received data report timer
    
    
// DEBUG CODE - POTENTIAL VITAL ERRORS
#define SEND_RECEIVEFROMQ 0x03
#define SEND_SENDTOTRANSMITQ 0x23
#define SEND_SENDTOTRANSMITQ_FAIL 0x73
#define SEND_TIMERINIT_FAIL 0x63
#define SEND_ENTERED_DEFAULT 0xE3
#define SEND_FULLQUEUE 0xB3
#define SEND_QUEUE_FAIL 0x23
    
#define RECEIVE_RECEIVEFROMUARTQ 0x04
#define RECEIVE_SENDTOMOTORQ 0x14
#define RECEIVE_SENDTOMOTORQ_FAIL 0x74
#define RECEIVE_TIMERINIT_FAIL 0x64
#define RECEIVE_ENTERED_DEFAULT 0xE4
#define RECEIVE_FULLQUEUE 0xB4
#define RECEIVE_QUEUE_FAIL 0x24
    
#define MOTOR_RECEIVEFROMQ 0x05
#define MOTOR_SENDTOSENDQ 0x15
#define MOTOR_SENDTOSENDQ_FAIL 0x75
#define MOTOR_TIMERINIT_FAIL 0x65
#define MOTOR_ENTERED_DEFAULT 0xE5
#define MOTOR_FULLQUEUE 0xB5
#define MOTOR_QUEUE_FAIL 0x25
    
#define SENSOR_SENDTOSENDQ 0x06
#define SENSOR_RECEIVEFROMSENSOR 0x16
#define SENSOR_SENDTOSENDQ_FAIL 0x76
#define SENSOR_TIMERINIT_FAIL 0x66
#define SENSOR_ENTERED_DEFAULT 0xE6
#define SENSOR_SENDTOSENSORQ_FAIL 0xC6
#define SENSOR_FULLQUEUE 0xB6
#define SENSOR_QUEUE_FAIL 0x26

#define PROCESS_ENTERED_DEFAULT 0xA1
    
    // *****************************************************************************
    // *****************************************************************************
    // Section: Data Types
    // *****************************************************************************
    // *****************************************************************************

    typedef struct
    {
        char start;
        char type;
        char count;
        char data[6];
        char stop;
    } MESSAGE;

    // *****************************************************************************
    // *****************************************************************************
    // Section: Interface Functions
    // *****************************************************************************
    // *****************************************************************************

    int ExampleFunction(int param1, int param2);


    /* Provide C++ Compatibility */
#ifdef __cplusplus
}
#endif

#endif /* _EXAMPLE_FILE_NAME_H */

/* *****************************************************************************
 End of File
 */
