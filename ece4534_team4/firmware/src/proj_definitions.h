/* ************************************************************************** */
/** Descriptive File Name

  @Company
    Company Name

  @File Name
    proj_definitions.h

  @Summary
     Location of key constants and values used throughout system

  @Description
    Describe the purpose of this file.
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
#define MSG_START 0x7E   // '~'
#define MSG_STOP '.'
#define MSG_LENGTH 10
    
// Message types
#define TYPE_LR_SENSOR 0xF7
#define TYPE_LR_ENCODER 0x7F
#define TYPE_SENSOR 0xE7
#define TYPE_COORDINATOR_MOTOR_CONTROL 0x5E
#define TYPE_MESSAGE_RECEIVE_DATA 0xFE
    
    
    
// Add duplicate byte inside message once every x bytes. Normal operation = 1
#define ADD_MESSAGE_DIV 1      
// Remove byte inside message once every x bytes. Normal operation = 1.
#define BREAK_MESSAGE_DIV 1       
// Message send rate multiplication. Normal operation = 2.
#define MESSAGE_RATE_DIV 2       
// Skip byte count every x messages. Normal operation = 0.
#define MESSAGE_COUNT_SKIP_DIV 0 
    
#define MOTOR_TIMER_RATE 55       // Motor message send rate (ms)
#define SENSOR_TIMER_RATE 40      // Sensor message send rate (ms)
#define SEND_TIMER_RATE 100       
#define RECEIVE_TIMER_RATE 125    // Receive code send to motor rate (for MS#2)
    
    
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
