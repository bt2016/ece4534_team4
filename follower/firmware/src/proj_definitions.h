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
#define TYPEC_GO 0x99 // 153
#define TYPEC_STOP 0x91 // 145
#define TYPEC_LR_SENSOR_TO_FR 0x8F  //143 - Token found message to send ot Follower Rover
#define TYPEC_LR_HANDSHAKE 0x4f     //79  - Confirm 'Token found' message receive to Lead Rover
#define TYPEC_MOTOR_CONTROL 0x5E   //94 - Rover motor control
// Lead Rover
#define TYPE_LR_SENSOR 0xF7     //247 - Lead Rover line sensor reading
#define TYPE_LR_ENCODER 0x7F    //127 - Lead Rover encoder report data
// Sensors
#define TYPE_SENSOR_DIST 0xE7        //231 - Field IR distance sensor data
#define TYPE_SENSOR_IR_REC 0xE8      //232 - Field IR receiver data
// Shared in all components
#define TYPE_MESSAGE_RECEIVE_DATA 0xFE    //254 - Reports UART received message data 
    
////////////////////////////// Follower Rover //////////////////////////////////
    
#define TYPE_FR_MOTOR 0xAF      //175 - FR Motor instruction data
#define TYPE_FR_ENCODER 0x6F    //111 - Motor encoder report data
#define TYPE_FR_DIST 0x5F       //95  - Follower forward distance data
#define TYPE_FR_IR 0x9F         //159 - Follower IR receiver data
#define TYPE_FR_ACK 0xCD       // 205 - Token received acknowledgement
#define TYPE_PID 0x44          // Directional PID values
#define TYPE_DISTPID 0xA5      // Distance PID values
#define TYPE_FR_TARGETDATA 0xAB   // Distance to token/target values
#define TYPE_FR_FOUNDTOKEN 0xF1   // 241 - FOUND TOKEN
    
#define TYPE_ADC 0x18
#define TYPE_HISTORY 0x65
#define TYPE_FR_STATUS 0x66
    
#define TYPEC_TOKEN_FOUND_ACK 0x60   // 96
    
    
#define TOKEN_VISUAL_CONTACT 0x3
#define TOKEN_TIME_TRACK 0x5
#define FF_ENABLE 0x0
    
//////////////////// MOTOR DIRECTION CONTROL //////////////////////////////////
#define MOTOR_FORWARD 0x11
#define MOTOR_REVERSE 0x55
#define MOTOR_STOP 0x04
   
/////////////////// IR DIRECTIONAL SIGNAL ENUMERATION /////////////////////////
#define LEAD_FRONT 0
#define LEAD_REAR 1
#define LEAD_RIGHT 2
#define LEAD_LEFT 3
    
/////////////////// CONTROL SIGNAL ENABLE ////////////////////////////////////
#define SET_HIGH 1
#define SET_LOW 0
    
#define TARG_DIST 8
#define TARG_DIST_SEC 9
#define DIST_HISTORY 20
    
#define TICK_CM 76
    
    
    
    
    
///////////////////////////// MILESTONE 4 /////////////////////////////////////
    
// 1 -> When follower hits token, stopAll message sent and motors stopped
// 0 -> Normal operation
#define STOP_ON_TOKEN 0x0
    
// 1 -> On Test Timer rollover, send a 'Token Found' signal to follower to acknowledge
#define SEND_FAKE_TOKEN 0x0
    
// 1 -> Have rover move to last known location whenever the front signal is lost
// 0 -> Have rover rotate to track current location   
#define MODE_LAST_LOC 0x1

#define MODE_NUANCE 0x0
#define MODE_ROUNDED 0x0
#define MODE_STATUS_UPDATE 0x1
    
// Message send enablers (0 - Normal operation)
// Message send message MOD dividers (1 - Normal operation)
// 1 -> Send distance report data via WiFly 
#define SEND_IRDATA 0x0
#define DIST_MOD 250    // 50
// 1 -> Send encoder reads for left/right wheel along with ideal speeds via WiFly
#define SEND_ENCODERS 0x0
#define ENCODER_MOD 10
// 1 -> Send Motor encoder PID calculated values via WiFly
#define SEND_PID 0x0
#define PID_DIV 10
// 1 -> Send current distance to token/target location
#define SEND_TARGETDATA 0x0
#define TARGETDATA_MOD 10
    
#define SEND_ADC 0x0
#define MS4_SET 0x52
    
    
// 1 -> Send IR signal report data via WiFly 
#define SEND_IR 0x0
#define IR_MOD 6      // 12
// 1 -> Send processed motor control information via WiFly 
#define SEND_MOTOR 0x0
#define MOTOR_MOD 1   // 24
// 1 -> Send Distance PID calculated values via WiFly
#define SEND_DISTPID 0x0
#define DISTPID_MOD 20
    
// I/O Function disablers
// 1 - Do not allow motor operation by setting PWM
#define CUT_MOTOR 0
// 1 - Do not send received data information to WiFly
#define CUT_RECEIVE_DATA 0
// 1 - Do not read IR distance input
#define CUT_IR_DIST 0x0
    
// At 1, manually define the IR directional data.
#define HC_SIGNALS 0
// If HC_SIGNALS == 1, define which IR directions will be made to read 'HIT'
#define TEST_FRONT 1
#define TEST_RIGHT 0
#define TEST_LEFT 0
#define TEST_REAR 0
    
#define IROFFSET 10
#define IRMIN 20
#define IRMAX 256

//////////////////////////// MOTOR PWM & PID ////////////////////////////////
#define MAX_MOTOR 6249   // MAX PWM output value for motors
#define MAX_CHANGE 255   // MAX PWM output value change per evaluation for motors
#define SP_DIST 9        // Set point for ideal distance
    
    
    

    
    
    
    
    
    
///////////////////// SIMULATED ERROR CONSTANTS (MS2) /////////////////////////////////
// Add duplicate byte inside message once every x bytes. NORMAL OPERATION = 1
#define ADD_MESSAGE_DIV 1      
// Remove byte inside message once every x bytes. NORMAL OPERATION = 1.
#define BREAK_MESSAGE_DIV 1 
// Message send rate multiplication. NORMAL OPERATION = 2.
#define MESSAGE_RATE_DIV 2     
// Skip entire message (including count) once every x messages. NORMAL OPERATION = 1
#define MESSAGE_SKIP_DIV 1
    
    
////////////////////// SYSTEM MESSAGE RATES /////////////////////////////////////
////////////////////// (message every 2x milliseconds) //////////////////////////
#define LR_MOTOR_TIMER_RATE 50      // Motor message send rate (ms)
#define LR_SENSOR_TIMER_RATE 50      // Sensor message send rate (ms)
    
#define TEST_TIMER_RATE 2000
#define FR_MOTOR_TIMER_RATE 200  // Follower rover motor message send rate
#define FR_ENCODER_TIMER_RATE 100
#define FR_IR_TIMER_RATE 100     // 20 - Follower rover IR receive rate
#define FR_DIST_TIMER_RATE 5   // Follower rover distance sensor rate
#define PROC_TIMER_RATE 100      // Process task timer rate
    
#define SEND_TIMER_RATE 60       
#define DIST_TIMER_RATE 100     // Receive code send to motor rate (for MS#2)
#define MOTOR_CTRL_TIMER_RATE 50  // Coordinator -> Lead Rover instruction timer rate
#define RECEIVE_TIMER_RATE 10000    // Message received data report timer
    
    
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
