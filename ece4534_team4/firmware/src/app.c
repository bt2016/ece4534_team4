
/*******************************************************************************
  MPLAB Harmony Application Source File
  
  Company:
    Microchip Technology Inc.
  
  File Name:
    app.c

  Summary:
    This file contains the source code for the MPLAB Harmony application.

  Description:
    This file contains the source code for the MPLAB Harmony application.  It 
    implements the logic of the application's state machine and it may call 
    API routines of other MPLAB Harmony modules in the system, such as drivers,
    system services, and middleware.  However, it does not call any of the
    system interfaces (such as the "Initialize" and "Tasks" functions) of any of
    the modules in the system or make any assumptions about when those functions
    are called.  That is the responsibility of the configuration-specific system
    files.
 *******************************************************************************/

// DOM-IGNORE-BEGIN
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
// DOM-IGNORE-END


#include "app.h"

APP_DATA appData;

char msg_type = 'q';
char msg_data1 = 0x30;
char msg_data2 = 0x39;
unsigned short int msg_datashort = 0x3639;
unsigned short int count = 0;

void app1WriteMessage() {
    // Use transmit interrupt for UART 
    // Same interrupt as receiver - account for both (clear both), handle the one that was raised
    
    //writeMsgChar(msg_type, msg_data1, msg_data2);
    writeMsgStr("roo");
    count++;
    dbgOutputVal(count >> 8);
    //writeMsgShortInt(msg_type, msg_datashort);
}


void APP_Initialize ( void )
{
    appData.state = APP_STATE_INIT;
    appData.letterPosition = 0;
    
    //Create a queue capable of holding 25 unsigned long numbers
    appData.xTimerIntQ = xQueueCreate( 250, sizeof( unsigned int ) ); 
    if( appData.xTimerIntQ == 0 ) stopAll();
    
    //Create a timer
    appData.xTimer100ms = xTimerCreate(  
                     "Timer100ms", //Just a text name
                     ( 100 / portTICK_PERIOD_MS ), //period is 100ms
                     pdTRUE, //auto-reload when expires
                     (void *) 23, //a unique id
                     vTimerCallback ); //pointer to callback function
    
    //Start the timer
    if( appData.xTimer100ms == NULL ) stopAll();
    else
    {
         if( xTimerStart( appData.xTimer100ms, 0 ) != pdPASS ) stopAll();
    }
   
}


void APP_Tasks ( void )
{
    while (1)
    {
        unsigned int *qData;

        switch ( appData.state )
        {
            case APP_STATE_INIT:
            {
                writeString("START");
                appData.state = APP_STATE_TX;
                break;
            }

            case APP_STATE_TX:
            {
                msg_data1++;
                msg_data2 += 2;
                if (msg_data1 > 57) msg_data1 = 48;
                if (msg_data2 > 57) msg_data2 = 48;
                
                msg_datashort += 3;
               
               break;
            }

            default: /* The default state should never be executed. */
            {
                writeString("DEFAULT_ERROR");
                appData.state = APP_STATE_TX;
                break;
            }

        }//end switch
    }//end while
}//end APP_Tasks

void app1SendTimerValToMsgQ(unsigned int* millisecondsElapsed)
{    
    if( xQueueSend( appData.xTimerIntQ,
                             (void*) millisecondsElapsed,
                             portMAX_DELAY) != pdPASS )
    {
        stopAll(); //failed to send to queue
    }
}

void app1SendValFromISR(unsigned int* message)
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    if (xQueueSendFromISR( appData.xTimerIntQ,
                            (void*) message,
                            &xHigherPriorityTaskWoken) != pdPASS)//errQUEUE_FULL)
    {
        stopAll(); //failed to send to queue
    }
}

/*******************************************************************************
 End of File
 */
