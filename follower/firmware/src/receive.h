/*******************************************************************************
  MPLAB Harmony Application Header File
  Company:
    Microchip Technology Inc.
  File Name:
    receive.h
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

#ifndef _RECEIVE_H
#define _RECEIVE_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdlib.h>
#include "system_config.h"
#include "system_definitions.h"
#include <queue.h>
#include <timers.h>
#include "timerCallback.h"


// DOM-IGNORE-BEGIN
#ifdef __cplusplus  // Provide C++ Compatibility
extern "C" {
#endif
// DOM-IGNORE-END 


typedef enum
{
	RECEIVE_STATE_INIT=0,

} RECEIVE_STATES;

typedef struct
{
    
    RECEIVE_STATES state;
    QueueHandle_t receiveIntQ_FR;
    
    TimerHandle_t receiveTimer_FR;
    TimerHandle_t receiveDistTimer_FR;
    char sendCount;
    unsigned short int goodMsg;
    unsigned short int badMsg;
    
} RECEIVE_DATA;


typedef struct
{
    /* TODO: Define any additional data used by the application. */
   
    int nextByteAt; 
    
    char start; // "~"
    char sendCount; //number of messages the sender has sent
    char types[10]; //you can add more types in if you'd like
    
    char stop; //this is our stop value "."
    
    char buffer[10]; // the actual buffer that holds what we've got
    
    
    
    
} M_BUFFER;


void RECEIVE_Initialize ( void );
void RECEIVE_Tasks( void );
void clearBuffer();


#endif /* _RECEIVE_H */

//DOM-IGNORE-BEGIN
#ifdef __cplusplus
}
#endif
//DOM-IGNORE-END

/*******************************************************************************
 End of File
 */
