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

void sendClearMessage(){
    //Convert sensor data to message format character array
    char data[4];
    data[0] = MSG_START;
    data[1] = 'c'; //TYPE_BROOKE_CLEAR;
    data[2] = processData.clearCount;
    data[3] = MSG_STOP;
    putMsgOnSendQueue(data);  // Transfer message to Send task queue]
    processData.clearCount++;
}

void sendDisplayMessage(){
    //Convert sensor data to message format character array
    char data[4];
    data[0] = MSG_START;
    data[1] = 'd';
    data[2] = processData.displayCount;
    data[3] = MSG_STOP;
    putMsgOnSendQueue(data);  // Transfer message to Send task queue]
    processData.displayCount++;
}

void sendEchoMessage(Obstacle o){
    char data[10];
    data[0] = MSG_START;             //Start byte
    data[1] = 'e';                   //TYPE_BROOKE_ECHO
    data[2] = processData.echoCount; //Count byte
    data[3] = o.midpoint_r;                 //data1;
    data[4] = o.midpoint_theta;             //data2;
    data[5] = o.midpoint_x;                 //data3;
    data[6] = o.midpoint_y;                 //data4;
    data[7] = o.slope;                      //data5;
    data[8] = o.length_of_arc;              //data6;
    data[9] = MSG_STOP;                     // Stop byte
    putMsgOnSendQueue(data);
    processData.echoCount++;
}

// Convert sensor data to message format character array
void sendProcessedData(Obstacle o) {
    //char data[MSG_LENGTH];
    char data[10];
    data[0] = MSG_START;                    // Start byte
    //data[1] = TYPE_SENSORARRAY_PROCESSED; // Type byte
    data[1] = 'a'; //TYPE_BROOKE_APPENDPOLAR;
    data[2] = processData.appendCount;      // Count byte
    data[3] = o.midpoint_r;                 //data1;
    data[4] = o.midpoint_theta;             //data2;
    data[5] = o.midpoint_x;                 //data3;
    data[6] = o.midpoint_y;                 //data4;
    data[7] = o.slope;                      //data5;
    data[8] = o.length_of_arc;              //data6;
    data[9] = MSG_STOP;                     // Stop byte
    putMsgOnSendQueue(data);  // Transfer message to Send task queue
    processData.appendCount++;
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
    processData.clearCount = 0;
    processData.displayCount = 0;
    processData.appendCount = 0;
    processData.echoCount = 0;
    
    //Create a queue capable of holding 1000 characters (bytes))
    //processData.processQ_SA = xQueueCreate(1000, MSG_LENGTH+1 ); 
    processData.processQ_SA = xQueueCreate(1000, sizeof(Obstacle) ); 
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
        Obstacle qData;
        
        /* Application's initial state. */
        case PROCESS_STATE_INIT:
        {
            //send a message to clear coordinates.txt
            sendClearMessage();
            processData.state = PROCESS_STATE_PROCESS;
            break;
        }
        
        case PROCESS_STATE_PROCESS:
        {            
            // Receive from sensor queue
		    if (xQueueReceive(processData.processQ_SA, &qData, portMAX_DELAY))
			{
                //If we are in ISOLATESENSOR debug mode, just echo the coordinate
                #ifdef SENSOR_DEBUG_ISOLATESENSOR
                    sendEchoMessage(qData);
                    break;
                #endif

                //If we are in FULLMAP debug mode, immediately forward the obstacle to the PI
                //Display the map and reset the coordinates.txt file after we've sent a full 90 degrees
                #ifdef SENSOR_DEBUG_FULLMAP
                    sendProcessedData(qData);
                    //if we have sent an entire panorama, theta == 90
                    if (qData.midpoint_theta == 90){
                        sendDisplayMessage();
                        processData.state = PROCESS_STATE_INIT;
                    }
                #endif
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
