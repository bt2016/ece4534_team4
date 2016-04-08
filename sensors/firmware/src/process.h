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
#include <timers.h>
#include <queue.h>

#include "proj_definitions.h"

// DOM-IGNORE-BEGIN
#ifdef __cplusplus  // Provide C++ Compatibility

extern "C" {

#endif
// DOM-IGNORE-END 



typedef enum
{
	/* Define states used by the application state machine. */
	PROCESS_STATE_INIT=0,
    PROCESS_STATE_GATHERDATA=1,
    PROCESS_STATE_PROCESS=2
} PROCESS_STATES;


typedef struct
{
    /* The application's current state */
    PROCESS_STATES state;
    
    /* Define any additional data used by the application. */
    unsigned short int clearCount;
    unsigned short int displayCount;
    unsigned short int displayFieldCount;
    unsigned short int displayAveragesCount;
    unsigned short int appendCount;
    unsigned short int echoCount;
    unsigned short int mapCount;
    unsigned short int linesCount;
    unsigned short int targetsCount;
    QueueHandle_t processQ_SA;
	TimerHandle_t process_Timer_SA;
    
    Obstacle map[SERVO_DEGREES+1];
    Obstacle obstacles_from_sensors[50]; //raw lines data from sensors
    Obstacle processed_obstacles[50];    //targets data for this iteration
    Obstacle averaged_obstacles[50];     //average targets data for lifetime of the program
    
    Obstacle lr_obstacles_from_sensors[50]; //raw lines data from lr sensors
    Obstacle lr_processed_obstacles[50]; //targets data from lr sensors
    
    int map_index;
    int obstacles_from_sensors_index;
    int processed_obstacles_index;
    int averaged_obstacles_index;
    int lr_obstacles_from_sensors_index;
    int lr_processed_obstacles_index;
    
    int iterations;

} PROCESS_DATA;


void PROCESS_Initialize ( void );
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

