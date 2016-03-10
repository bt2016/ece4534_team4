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
    
// MESSAGE TYPES
// Coordinator
#define TYPEC_LR_SENSOR_TO_FR 0x8F  //143 - Token found message to send ot Follower Rover
#define TYPEC_LR_HANDSHAKE 0x4f     //79  - Confirm 'Token found' message receive to Lead Rover
#define TYPEC_MOTOR_CONTROL 0x5E   //94 - Rover motor control
// Lead Rover
#define TYPE_LR_SENSOR 0xF7     //247 - Lead Rover line sensor reading
#define TYPE_LR_ENCODER 0x7F    //127 - Lead Rover encoder report data
// Follower Rover
#define TYPE_FR_ENCODER 0x6F    //111 - Motor encoder report data
#define TYPE_FR_DIST 0x5F       //95  - Follower forward distance data
#define TYPE_FR_IR 0x9F         //159 - Follower IR receiver data
// Sensors
#define TYPE_SENSOR_DIST 0xE7        //231 - Field IR distance sensor data
#define TYPE_SENSOR_IR_REC 0xE8      //232 - Field IR receiver data
#define TYPE_SENSORARRAY_PROCESSED 0xE9   //233 - Processed Field data from sensor array
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
    
    
// SYSTEM MESSAGE RATES (message every 2x milliseconds)
// 55 on Motor gives error every 10 messages - Debug
#define LR_MOTOR_TIMER_RATE 50      // Motor message send rate (ms)
#define LR_SENSOR_TIMER_RATE 50      // Sensor message send rate (ms)
    
#define FR_MOTOR_TIMER_RATE 50  // Follower rover motor message send rate
#define FR_IR_TIMER_RATE 20     // Follower rover IR receive rate
#define FR_DIST_TIMER_RATE 50   // Follower rover distance sensor rate
#define PROC_TIMER_RATE 50      // Process task timer rate
    
#define SEND_TIMER_RATE 60       
#define DIST_TIMER_RATE 100     // Receive code send to motor rate (for MS#2)
#define MOTOR_CTRL_TIMER_RATE 50  // Coordinator -> Lead Rover instruction timer rate
#define RECEIVE_TIMER_RATE 10000    // Message received data report timer
    
//#define SA_DIST_TIMER_RATE 20
#define SA_DIST_TIMER_RATE 100
#define SA_IR_TIMER_RATE 50
#define SA_PROC_TIMER_RATE 210
    
    
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
#define SENSOR_SENDTOPROCESSQ_FAIL 0xA6
#define SENSOR_TIMERINIT_FAIL 0x66
#define SENSOR_ENTERED_DEFAULT 0xE6
#define SENSOR_SENDTOSENSORQ_FAIL 0xC6
#define SENSOR_FULLQUEUE 0xB6
#define SENSOR_QUEUE_FAIL 0x26
    
#define TYPE_BROOKE_DISPLAY 97
#define TYPE_BROOKE_APPENDPOLAR 100
#define TYPE_BROOKE_CLEAR 99
#define TYPE_BROOKE_ECHO 101
    
#define PROCESS_TIMERINIT_FAIL 0x67
#define PROCESS_QUEUE_FAIL 0x27

   
    
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
    
    typedef struct
    {
        int start_radius;  //first degree that the sensor picked up the obstacle
        int end_radius;    //last degree that the sensor picked up the obstacle
        int length_of_arc; //difference between start_radius and end_radius in cm
        unsigned int midpoint_r; //midpoint of the obstacle in polar coordinates in cm
        int midpoint_theta;      //midpoint of the obstacle in polar coordinates in degrees
        int midpoint_x;    //midpoint of the obstacle in rectangular coordinates in cm
        int midpoint_y;    //midpoint of the obstacle in rectangular coordinates in cm
        double slope;      //slope of the line in polar coordinates
    } Obstacle;

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
