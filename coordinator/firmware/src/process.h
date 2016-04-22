/*******************************************************************************
  MPLAB Harmony Application Header File

  Company:
    Microchip Technology Inc.

  File Name:
    process.h

  Summary:
    This header file provides prototypes and definitions for the application.

  Description:
    This header file provides function prototypes and data type definitions for
    the application.  Some of these are required by the system (such as the
    "APP_Initialize" and "APP_Tasks" prototypes) and some of them are only used
    internally by the application (such as the "APP_STATES" definition).  Both
    are defined here for convenience.
*******************************************************************************/

//DOM-IGNORE-BEGIN
/*******************************************************************************
Copyright (c) 2013-2014 released Microchip Technology Inc.  All rights reserved.

Microchip licenses to you the right to use, modify, copy and distribute
Software only when embedded on a Microchip microcontroller or digital signal
controller that is integrated into your product or third party product
(pursuant to the sublicense terms in the accompanying license agreement).

You should refer to the license agreement accompanying this Software for
additional information regarding your rights and obligations.

SOFTWARE AND DOCUMENTATION ARE PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
EITHER EXPRESS OR IMPLIED, INCLUDING WITHOUT LIMITATION, ANY WARRANTY OF
MERCHANTABILITY, TITLE, NON-INFRINGEMENT AND FITNESS FOR A PARTICULAR PURPOSE.
IN NO EVENT SHALL MICROCHIP OR ITS LICENSORS BE LIABLE OR OBLIGATED UNDER
CONTRACT, NEGLIGENCE, STRICT LIABILITY, CONTRIBUTION, BREACH OF WARRANTY, OR
OTHER LEGAL EQUITABLE THEORY ANY DIRECT OR INDIRECT DAMAGES OR EXPENSES
INCLUDING BUT NOT LIMITED TO ANY INCIDENTAL, SPECIAL, INDIRECT, PUNITIVE OR
CONSEQUENTIAL DAMAGES, LOST PROFITS OR LOST DATA, COST OF PROCUREMENT OF
SUBSTITUTE GOODS, TECHNOLOGY, SERVICES, OR ANY CLAIMS BY THIRD PARTIES
(INCLUDING BUT NOT LIMITED TO ANY DEFENSE THEREOF), OR OTHER SIMILAR COSTS.
 *******************************************************************************/
//DOM-IGNORE-END

#ifndef _PROCESS_H
#define _PROCESS_H

// *****************************************************************************
// *****************************************************************************
// Section: Included Files
// *****************************************************************************
// *****************************************************************************

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdlib.h>
#include "system_config.h"
#include "system_definitions.h"
#include <queue.h>
#include <timers.h>
#include "debug.h"       
#include "timerCallback.h"
#include "proj_definitions.h"
#include "math.h"

// DOM-IGNORE-BEGIN
#ifdef __cplusplus  // Provide C++ Compatibility

extern "C" {

#endif
// DOM-IGNORE-END 

// *****************************************************************************
// *****************************************************************************
// Section: Type Definitions
// *****************************************************************************
// *****************************************************************************

// *****************************************************************************
/* Application states

  Summary:
    Application states enumeration

  Description:
    This enumeration defines the valid application states.  These states
    determine the behavior of the application at various times.
*/

typedef enum
{
	/* Application's state machine's initial state. */
	PROCESS_STATE_INIT=0,
    PROCESS_STATE_SET_TOKENS=1,
    PROCESS_STATE_MESSAGES=2,
    PROCESS_STATE_SEND_MAP=3,
    PROCESS_STATE_MOVE_ORIGIN=4, //move the rover towards starting point
    PROCESS_STATE_MOVE=5, //issue next move command for search path
    PROCESS_STATE_LOCATE_ROVER_INIT=6, //locate the rover and its direction
    PROCESS_STATE_UPDATE_LOCATIONS=7, //update object locations
    PROCESS_STATE_LOCATE_ROVER_MOVE=8, //locate the rover and its direction
    PROCESS_STATE_LOCATE_ROVER_ANALYZE=9, //locate the rover and its direction
    PROCESS_STATE_DIVERT=10, //Handles diverting around an obstacle
    PROCESS_STATE_WAIT_FOR_CALIBRATION_START=11,
            

	/* TODO: Define states used by the application state machine. */

} PROCESS_STATES;


// *****************************************************************************
/* Application Data

  Summary:
    Holds application data

  Description:
    This structure holds the application's data.

  Remarks:
    Application strings and buffers are be defined outside this structure.
 */

typedef struct
{
    /* The application's current state */
    PROCESS_STATES state;
    PROCESS_STATES next_state;
    
    QueueHandle_t processQ_CD;
    
    int number_of_tokens;
    
    //DEBUG
    double last_distance_calculated;
    int max_move_amount_last_calculated;
    int obs_x;
    int obs_y;
    int rover_location_in_q; 
    
        
    int debug;
    int turn_rover_called;
    
    int number_of_sensor_pans; 
    
    int last_move_amount;
    
    int divert_direction; //direction you WANT the rover to go
    int divert_moves; //number of steps diverted
    int lr_last_direction; //last known direction of the lead rover
    double direction_degrees; //last known exact orientation
    int hit_origin; 
    int rover_located; 
    int finished;
    int expecting_new_locations; //set to true to set object_locations_am
    
    int refresh_rate; //Number of move commands to issue before map refresh
    int need_divert; //TRUE if need to divert for obstacle, false else
    
    int next_move_amt; //cm's you want to move forward by
    
    //Farthest we want to go in either direction 
    int max_x;
    int max_y; 
    
    int min_x;
    int min_y;
    
    int rover_x;
    int rover_y;
    
    char new_message[MSG_LENGTH];
    
    //A 2D array capable of holding N messages
    //Where N = TRACKED_OBJECT_AMOUNT
    char object_locations[TRACKED_OBJECT_AMOUNT][MSG_LENGTH];
    char object_locations_am[TRACKED_OBJECT_AMOUNT][MSG_LENGTH]; //locations AFTER a move 
    char obstacle_averages[TRACKED_OBJECT_AMOUNT][MSG_LENGTH]; //locations AFTER a move 
    
    
    int new_object_location;
    int new_object_location_am;
    int number_of_obstacle_averages;
    /* TODO: Define any additional data used by the application. */


} PROCESS_DATA;


// *****************************************************************************
// *****************************************************************************
// Section: Application Callback Routines
// *****************************************************************************
// *****************************************************************************
/* These routines are called by drivers when certain events occur.
*/

	
// *****************************************************************************
// *****************************************************************************
// Section: Application Initialization and State Machine Functions
// *****************************************************************************
// *****************************************************************************

/*******************************************************************************
  Function:
    void PROCESS_Initialize ( void )

  Summary:
     MPLAB Harmony application initialization routine.

  Description:
    This function initializes the Harmony application.  It places the 
    application in its initial state and prepares it to run so that its 
    APP_Tasks function can be called.

  Precondition:
    All other system initialization routines should be called before calling
    this routine (in "SYS_Initialize").

  Parameters:
    None.

  Returns:
    None.

  Example:
    <code>
    PROCESS_Initialize();
    </code>

  Remarks:
    This routine must be called from the SYS_Initialize function.
*/

void PROCESS_Initialize ( void );


/*******************************************************************************
  Function:
    void PROCESS_Tasks ( void )

  Summary:
    MPLAB Harmony Demo application tasks function

  Description:
    This routine is the Harmony Demo application's tasks function.  It
    defines the application's state machine and core logic.

  Precondition:
    The system and application initialization ("SYS_Initialize") should be
    called before calling this.

  Parameters:
    None.

  Returns:
    None.

  Example:
    <code>
    PROCESS_Tasks();
    </code>

  Remarks:
    This routine must be called from SYS_Tasks() routine.
 */
void processTokenMsg();
void PROCESS_Tasks( void );


#endif /* _PROCESS_H */

//DOM-IGNORE-BEGIN
#ifdef __cplusplus
}
#endif
//DOM-IGNORE-END

/*******************************************************************************
 End of File
 */

