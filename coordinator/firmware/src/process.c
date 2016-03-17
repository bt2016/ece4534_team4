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
#include "proj_definitions.h"

// *****************************************************************************
// *****************************************************************************
// Section: Global Data Definitions
// *****************************************************************************
// *****************************************************************************

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

PROCESS_DATA processData;

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


// *****************************************************************************
// *****************************************************************************
// Section: Application Initialization and State Machine Functions
// *****************************************************************************
// *****************************************************************************


// Remove oldest data on full local data queue
// Returns 1 if successful, 0 otherwise
uint8_t removeProcessQueueData() {
    char readdata[MSG_LENGTH];
 
    if (xQueueReceive(processData.processQ_CD, &readdata, portMAX_DELAY))
    {
        return 0x1;
    } 
    
    return 0x0;
}


// Check queue holding the data from other component tasks
// THIS IS BLOCKING
void receiveFromProcessQ()
{    
   char read[MSG_LENGTH]; 
   char newData = 0;

   // Read the top message of the queue
    if (xQueueReceive(processData.processQ_CD, &read, portMAX_DELAY))
    {
        newData = 1;
    } 
   
   
   char* k = read; 
   int i = 0;
   for(i=0; i<MSG_LENGTH; i++){
       
      processData.new_message[i] = *k;
      ++k;
   }
   

  
}



// Place data passed from other tasks into send queue
void putMsgOnProcessQueue(char* data) {
        
    if (processData.processQ_CD != 0) {
        
        // Check for full queue. If no spaces available, call to remove oldest data.
        if (uxQueueSpacesAvailable( processData.processQ_CD ) == 0) {
            // If message is not removed from queue, return and signal error
            if (removeProcessQueueData() == 0) {                
                stopAll();
                return;
            }
        }
        
        // Send to queue, with at least one vacancy guaranteed for this data
        if( xQueueSend( processData.processQ_CD, (void*) data, portMAX_DELAY) != pdPASS )
        {
            dbgOutputVal(SEND_QUEUE_FAIL);
        }
    }
}

//Call this whenever we get a message to set the tokens
void processTokenMsg(){
    char msgToSend[MSG_LENGTH];
    msgToSend[0] = MSG_START;
    msgToSend[MSG_LENGTH-1] = MSG_STOP;
    
    //Success! Set the number of tokens
    processData.number_of_tokens = processData.new_message[8];
    //Send ACK
    msgToSend[1] = TYPEC_ACK_TOKEN;
    putMsgOnSendQueue(msgToSend);

    processData.state = PROCESS_STATE_MESSAGES;

}

//Call this if you want to adjust map data
void processMapDataMsg(){
    
    if(processData.new_object_location != TRACKED_OBJECT_AMOUNT+1){        
        strncpy(processData.object_locations[processData.new_object_location], processData.new_message, MSG_LENGTH);
        processData.new_object_location++;
    }
    else{
        processData.new_object_location = 0;
        strncpy(processData.object_locations[processData.new_object_location], processData.new_message, MSG_LENGTH);
        processData.new_object_location++;        
    }   
            
}

void processUpdateMap(){
    
    char msgToClear[MSG_LENGTH];
    
    msgToClear[0] = MSG_START;
    msgToClear[MSG_LENGTH-1] = MSG_STOP;
    msgToClear[1] = TYPEC_CLEAR_MAP;
    
    //Clear the map
    putMsgOnSendQueue(msgToClear);
    
    
    //Update the coordinate list
    int msg = 0;
    
    char mapMessages[MSG_LENGTH];
    /*
    mapMessages[0] = MSG_START;
    mapMessages[MSG_LENGTH-1] = MSG_STOP;
    mapMessages[1] = TYPEC_MAP_DATA;
    mapMessages[3] = 21;
    mapMessages[4] = 13;
    */
    
    for(msg=0; msg < processData.new_object_location; msg++){        
        //memcpy(mapMessages, processData.object_locations[msg], MSG_LENGTH);
        processData.object_locations[msg][2] = msg; //This protects against repeat count bytes
        putMsgOnSendQueue(processData.object_locations[msg]);
    }
    
    //Tell the pi to display the map
    char msgToSend[MSG_LENGTH];    
    msgToSend[0] = MSG_START;
    msgToSend[MSG_LENGTH-1] = MSG_STOP;
    msgToSend[1] = TYPEC_UPDATE_MAP;
    putMsgOnSendQueue(msgToSend);
    
}

/*******************************************************************************
  Function:
    void PROCESS_Initialize ( void )

  Remarks:
    See prototype in process.h.
 */

void PROCESS_Initialize ( void )
{
    /* Place the App state machine in its initial state. */
    processData.state = PROCESS_STATE_INIT;
    processData.number_of_tokens = -1;
    processData.new_object_location = 0;
    
    /* TODO: Initialize your application's state machine and other
     * parameters.
     */
    
    //Create a Q capable of holding 250 messages
    processData.processQ_CD = xQueueCreate(250, MSG_LENGTH+1);
    if (processData.processQ_CD == 0) {
        stopAll();
    }
    
    
    //For testing:
    char msgToSend[MSG_LENGTH];
    msgToSend[0] = MSG_START;
    msgToSend[MSG_LENGTH-1] = MSG_STOP;
    msgToSend[1] = TYPEC_MAP_DATA;
    
    msgToSend[3] = 10;
    msgToSend[4] = 45;
    
    memcpy(processData.object_locations[0], msgToSend,  MSG_LENGTH);
    
    
    msgToSend[3] = 27;
    msgToSend[4] = 21;
    
    memcpy(processData.object_locations[1], msgToSend,  MSG_LENGTH);
    
    msgToSend[3] = 34;
    msgToSend[4] = 32;
    
    memcpy(processData.object_locations[2], msgToSend,  MSG_LENGTH);
    
    processData.new_object_location = 3;
    
    
}


/******************************************************************************
  Function:
    void PROCESS_Tasks ( void )

  Remarks:
    See prototype in process.h.
 */
// Create a queue capable of holding 250 messages

    
void PROCESS_Tasks ( void )
{
    
    /* Check the application's current state. */
    switch ( processData.state )
    {
        /* Application's initial state. */
        case PROCESS_STATE_INIT:
        {
            
            processData.state = PROCESS_STATE_SET_TOKENS;
            break;
        }
        /* Request the number of tokens after connecting to RPI*/
        case PROCESS_STATE_SET_TOKENS:
        {
            char msgToSend[MSG_LENGTH];
            msgToSend[0] = MSG_START;
            msgToSend[MSG_LENGTH-1] = MSG_STOP;
            
            //Tell the Pi we want the token number
            msgToSend[1] = TYPEC_TOKEN_NUMBER;
            putMsgOnSendQueue(msgToSend);            
            
            //Wait for something to appear on the receive queue.
            receiveFromProcessQ();            
            
            //Check that the message is for setting tokens
            if(processData.new_message[1] == TYPEC_TOKEN_NUMBER){
                processTokenMsg();
            }
            
            //If it isn't, request from the Pi that we want token numbers
            else{
                msgToSend[1] = TYPEC_TOKEN_NUMBER;
                putMsgOnSendQueue(msgToSend); 
            }

            /*           
            if(processData.new_message[4] == 'o'){
                //Turn a light on
                LATASET = 1 << 3;
            }
            else if(processData.new_message[4] == 'f'){
               //Turn a light off
                LATACLR = 1 << 3; 
            }
            */
            break;
        }
        
        case PROCESS_STATE_MESSAGES:
        {
            //Wait for something to appear on the receive queue.
            receiveFromProcessQ();            
            
            //Check that the message is for setting tokens
            if(processData.new_message[1] == TYPEC_TOKEN_NUMBER){
                processTokenMsg();
            }
            
            else if(processData.new_message[1] == TYPEC_MAP_DATA){
                processMapDataMsg();
            }
            
            else if(processData.new_message[1] == TYPEC_CLEAR_MAP){
                processData.new_object_location = 0;
            }
            
            //This function is from the Pi, asking the coordinator to 
            //Send the Pi the same message back, to update the map list
            else if(processData.new_message[1] == TYPEC_UPDATE_MAP){
                processUpdateMap();
            }
         
            break;
        }

        /* The default state should never be executed. */
        default:
        {
            /* TODO: Handle error in application's state machine. */
            dbgOutputVal(PROCESS_ENTERED_DEFAULT);
            stopAll();
            break;
        }
    }
}

/*******************************************************************************
 End of File
 */
