/*******************************************************************************
  MPLAB Harmony Application Source File
  
  Company:
    Microchip Technology Inc.
  
  File Name:
    process.c

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

#include "process.h"

PROCESS_DATA processData;

// Pack and transfer message to data queue in Send task
void processSensorData(char* sensorData) {
    
    // PROCESSING ALGORITHMS HERE
    
}

void sendProcessedData() {
    // Convert sensor data to message format character array
    char data[MSG_LENGTH];
    data[0] = MSG_START;            // Start byte
    data[1] = TYPE_SENSORARRAY_PROCESSED;      // Type byte
    data[2] = processData.sendCount;  // Count byte
    // Dummy values (data bytes x6)
    data[3] = 0x22;
    data[4] = 0x33;
    data[5] = 0x44;
    data[6] = 0x55; 
    data[7] = 0x66; 
    data[8] = 0x77;
    data[9] = MSG_STOP;             // Stop byte
          
    putMsgOnSendQueue(data);  // Transfer message to Send task queue
            
    processData.sendCount++;
}

// Remove oldest data on full local process queue
// Returns 1 if successful, 0 otherwise
uint8_t removeProcessQueueData() {
    char readdata[MSG_LENGTH];

    if (xQueueReceive(processData.processQ_SA, &readdata, portMAX_DELAY))
    {
        return 0x1;
    } 
    
    return 0x0;
}

// Check local process queue for vacancies. If none, remove oldest data.
// After, add the parameter char* message to local process queue. 
void putDataOnProcessQ(char* data) {
    if (processData.processQ_SA != 0) {
        
        // Check for full queue. If no spaces available, call to remove oldest data.
        if (uxQueueSpacesAvailable( processData.processQ_SA ) == 0) {
            // If message is not removed from queue, return and signal error
            if (removeProcessQueueData() == 0) {
                dbgOutputVal(SENSOR_FULLQUEUE);
                stopAll();
                return;
            }
        }
        
        // Send to queue, with at least one vacancy guaranteed for this data
        if( xQueueSend( processData.processQ_SA, (void*) data, portMAX_DELAY) != pdPASS )
        {
            dbgOutputVal(SENSOR_SENDTOPROCESSQ_FAIL);
        }
    }
}


void PROCESS_Initialize ( void )
{
    /* Place the App state machine in its initial state. */
    processData.state = PROCESS_STATE_INIT;
    processData.sendCount = 0;
    
        //Create a queue capable of holding 1000 characters (bytes))
    processData.processQ_SA = xQueueCreate(1000, MSG_LENGTH+1 ); 
    if( processData.processQ_SA == 0 ) {
        dbgOutputVal(PROCESS_QUEUE_FAIL);
        stopAll(); //ERROR
    }
    
    //Timer to send to internal task queue
    processData.process_Timer_SA = xTimerCreate(  
                     "ProcessTimer", //Just a text name
                     ( SA_PROC_TIMER_RATE / portTICK_PERIOD_MS ), //period in ms
                     pdTRUE, //auto-reload when expires
                     (void *) 31, //a unique id
                     processTimerCallback ); //pointer to callback function
    
    //Start the timer
    if( processData.process_Timer_SA == NULL ) {
        dbgOutputVal(PROCESS_TIMERINIT_FAIL);
        stopAll();
    }
    else if( xTimerStart( processData.process_Timer_SA, 0 ) != pdPASS ) {
        dbgOutputVal(PROCESS_TIMERINIT_FAIL);
        stopAll();
    }
    
}



void PROCESS_Tasks ( void )
{
    /* Check the application's current state. */
    switch ( processData.state )
    {
        char qData[10];
        
        /* Application's initial state. */
        case PROCESS_STATE_INIT:
        {
            processData.state = PROCESS_STATE_PROCESS;
            break;
        }
        
        case PROCESS_STATE_PROCESS:
        {
           // Receive from sensor queue
		    if (xQueueReceive(processData.processQ_SA, &qData, portMAX_DELAY))
			{
                processSensorData(qData);
			}
			break;
        }
        
        /* The default state should never be executed. */
        default:
        {
            /* TODO: Handle error in application's state machine. */
            break;
        }
    }
}
 

/*******************************************************************************
 End of File
 */
