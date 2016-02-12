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


// *****************************************************************************
// *****************************************************************************
// Section: Included Files 
// *****************************************************************************
// *****************************************************************************

#include "app.h"

//Include files required for milestone1
#include "queue.h" //FreeRTOS file
#include "timers.h" //FreeRTOS file
#include "app1_public.h" //Created by me file
//#include "timerCallback.h" //Created by me ** included in app.h
#include "debug.h" //Created by me
#include "sender.h" //Created my me

// *****************************************************************************
// *****************************************************************************
// Section: Global Data Definitions
// *****************************************************************************
// *****************************************************************************





//The handle for the messaging queue which receives messages from the timer interrupt
QueueHandle_t xTimerIntQ;
TimerHandle_t xTimer100ms;




// *****************************************************************************
/* Application Data

  Summary:
    Holds application data

  Description:
    This structure holds the application's data.

  Remarks:
    This structure should be initialized by the APP_Initialize function.
    
    Application strings and buffers are be defined outside this structure.
*/

APP_DATA appData;

// *****************************************************************************
// *****************************************************************************
// Section: Application Callback Functions
// *****************************************************************************
// *****************************************************************************

/* TODO:  Add any necessary callback funtions.
*/

// *****************************************************************************
// *****************************************************************************
// Section: Application Local Functions
// *****************************************************************************
// *****************************************************************************

/* TODO:  Add any necessary local functions.
*/

/*******************************************************************************
  Function:
     int app1SendTimerValToMsgQ(unsigned int millisecondsElapsed)

  Remarks:
    millisecondsElapsed is sent to the back of the xTimerIntQ. 
 */
void app1SendTimerValToMsgQ(unsigned int millisecondsElapsed){    
        
    if( xQueueSendToBack( xTimerIntQ,
                             ( void * ) &millisecondsElapsed,
                             ( TickType_t ) 0 ) != pdPASS ) //change to portMAX_DELAY
        {
            // Failed to post the message.
            //stopAll();            
        }  
    
}

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


// *****************************************************************************
// *****************************************************************************
// Section: Application Initialization and State Machine Functions
// *****************************************************************************
// *****************************************************************************

/*******************************************************************************
  Function:
    void APP_Initialize ( void )

  Remarks:
    See prototype in app.h.
 */


void APP_Initialize ( void )
{
    /* Place the App state machine in its initial state. */
    appData.state = APP_STATE_INIT;
    
    
    appData.letterPosition = 0;
    
    //Initialize FreeRTOS task Queue (& and later, timer)
       
    //Create a queue capable of holding 25 unsigned long numbers
    xTimerIntQ = xQueueCreate( 25, sizeof( unsigned int ) ); 
    if( xTimerIntQ == 0 )
    {
        // TODO: Queue was not created and must not be used. Call debug stop function
    }
    
    //Create a timer
    
    xTimer100ms = xTimerCreate(  
                     // Just a text name, not used by the RTOS kernel. 
                     "Timer100ms",
                     // The timer period in ms (100), must be greater than 0. 
                     ( 100 / portTICK_PERIOD_MS ),
                     // The timers will auto-reload themselves when they
                     //expire. 
                     pdTRUE,
                     // Assign a unique id  
                     (void *) 23,
                     // TimerCallback function 
                     vTimerCallback );
    
    //Start the timer
    if( xTimer100ms == NULL )
         {
             /* The timer was not created. Stop and throw error */
            stopAll();
         }
         else
         {
             /* Start the timer.  No block time is specified, and even if one
             was it would be ignored because the RTOS scheduler has not yet
             been started. */
             if( xTimerStart( xTimer100ms, 0 ) != pdPASS )
             {
                 /* The timer could not be set into the Active state. */
             }
         }
}


/******************************************************************************
  Function:
    void APP_Tasks ( void )

  Remarks:
    See prototype in app.h.
 */

void APP_Tasks ( void )
{
    //You can put code here, it should get executed like forever.
    //stopAll(); //Uncomment to demo the HALT method thats in debug. This will flash your LED 
    
    unsigned int *qData;
/*
    //DRV_USART0_WriteByte('.');
    if( xTimerIntQ != 0 )
    {
        // Receive a message on the created queue.  Block for ever if a
        // message is not immediately available.
        if( xQueueReceive( xTimerIntQ, &( qData ), portMAX_DELAY ) )
        {       
            
            if(appData.letterPosition == 0){
                dbgOutputVal('T');
            }
            else if(appData.letterPosition == 1){
                dbgOutputVal('E');
            }
            else if(appData.letterPosition == 2){
                dbgOutputVal('A');
            }
            else if(appData.letterPosition == 3){
                dbgOutputVal('M');
            }
            else if(appData.letterPosition == 4){
                dbgOutputVal('4');
            }
            
            appData.letterPosition += 1;
            if(appData.letterPosition == 5){
                appData.letterPosition = 0;
            }
             
        }
    }
 */
    
    /* Check the application's current state. */
    switch ( appData.state )
    {
        /* Application's initial state. */
        case APP_STATE_INIT:                        // application state after system and application are initialized
        {
            writeString("START");
            appData.state = APP_STATE_TX;           // change state to receive after initializing application
            break;
        }

        case APP_STATE_TX:                              // USART transmit state
        {
            msg_data1++;
            msg_data2 += 2;
            if (msg_data1 > 57) msg_data1 = 48;
            if (msg_data2 > 57) msg_data2 = 48;
            
            msg_datashort += 3;
           
           break;
        }

        default:    /* The default state should never be executed. */
        {
            writeString("DEFAULT_ERROR");
            appData.state = APP_STATE_TX;
            break;  /* TODO: Handle error in application's state machine. */
        }
    }
}


/*******************************************************************************
 End of File
 */
