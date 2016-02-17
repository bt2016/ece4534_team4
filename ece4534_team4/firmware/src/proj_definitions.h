/* ************************************************************************** */
/** Descriptive File Name

  @Company
    Company Name

  @File Name
    filename.h

  @Summary
    Brief description of the file.

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

#define MSG_START '~'
#define MSG_STOP '.'
#define MSG_LENGTH 10
    
#define SEND_RECEIVEFROMQ 0x03
#define SEND_SENDTOTRANSMITQ 0x23
#define SEND_SENDTOTRANSMITQ_FAIL 0x73
#define SEND_TIMERINIT_FAIL 0x63
#define SEND_ENTERED_DEFAULT 0xE3
    
#define RECEIVE_RECEIVEFROMUARTQ 0x04
#define RECEIVE_SENDTOMOTORQ 0x14
#define RECEIVE_SENDTOMOTORQ_FAIL 0x74
#define RECEIVE_TIMERINIT_FAIL 0x64
#define RECEIVE_ENTERED_DEFAULT 0xE4
    
#define MOTOR_RECEIVEFROMQ 0x05
#define MOTOR_SENDTOSENDQ 0x15
#define MOTOR_SENDTOSENDQ_FAIL 0x75
#define MOTOR_TIMERINIT_FAIL 0x65
#define MOTOR_ENTERED_DEFAULT 0xE5
    
#define SENSOR_SENDTOSENDQ 0x06
#define SENSOR_RECEIVEFROMSENSOR 0x16
#define SENSOR_SENDTOSENDQ_FAIL 0x76
#define SENSOR_TIMERINIT_FAIL 0x66
#define SENSOR_ENTERED_DEFAULT 0xE6
#define SENSOR_SENDTOSENSORQ_FAIL 0xC6

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
