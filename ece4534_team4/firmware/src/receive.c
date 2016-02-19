/*******************************************************************************
  MPLAB Harmony Application Source File
  
  Company:
    Microchip Technology Inc.
  
  File Name:
    receive.c
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

#include "receive.h"
#include "proj_definitions.h"
//#include "receivePublic.h"

RECEIVE_DATA receiveData;
M_BUFFER messageBuffer;

void clearBuffer(){
    
    //Do anything you need to do to clear the message buffer in here
    messageBuffer.nextByteAt = 0;
    
}

void receiveSendToMsgQ() {
    // Convert sensor data to message format character array
    char data[MSG_LENGTH];
    data[0] = MSG_START;                         // Start byte
    data[1] = TYPE_COORDINATOR_MOTOR_CONTROL;    // Type byte
    data[2] = 0x67;  // Count byte
    // Dummy values
    data[3] = 0x20;
    data[4] = 0x40;
    data[5] = 0x60;
    data[6] = receiveData.sendCount << 1;
    data[7] = receiveData.sendCount >> 3;
    data[8] = receiveData.sendCount << 2;
    data[9] = MSG_STOP;             // Stop byte
                
    putDataOnMotorQ(data);
}


void receiveSendValFromISR(char* data){
    
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    if (xQueueSendFromISR( receiveData.xReceiveIntQ,
                            (void*) data,
                            &xHigherPriorityTaskWoken)
                                != pdPASS)//errQUEUE_FULL)
    {
        stopAll(); //failed to send to queue
    }

}

// Report number of good and bad messages to send queue to be transmitted to Pi via UART
void reportMsgData() {
    receiveData.sendCount++;
    
        // Convert message receive data to message format character array
        char data[MSG_LENGTH];
        data[0] = MSG_START;                  // Start byte
        data[1] = TYPE_MESSAGE_RECEIVE_DATA;  // Type byte
        data[2] = receiveData.sendCount;      // Count byte
        // Dummy values (data bytes x6)
        data[3] = 0x0;
        data[4] = 0x0;
        data[5] = ((receiveData.badMsg & 0xFF00) >> 8);
        data[6] = receiveData.badMsg & 0xFF; 
        data[7] = ((receiveData.goodMsg & 0xFF00) >> 8); 
        data[8] = (receiveData.goodMsg & 0xFF);
        data[9] = MSG_STOP;             // Stop byte
        
    putMsgOnSendQueue(data);  // Transfer message to Send task queue
    
    receiveData.badMsg = 0;
    receiveData.goodMsg = 0;
}

void RECEIVE_Initialize ( void )
{
    receiveData.state = RECEIVE_STATE_INIT;
    receiveData.goodMsg = 0;
    receiveData.longMsg = 0;
    receiveData.shortMsg = 0;
    receiveData.badMsg = 0;
    
    //Create a queue capable of holding 2500 characters (bytes))
    receiveData.xReceiveIntQ = xQueueCreate(2500, sizeof( char ) ); 
    if( receiveData.xReceiveIntQ == 0 ) {
        dbgOutputVal(RECEIVE_QUEUE_FAIL);
        stopAll(); //ERROR
    }
    
    messageBuffer.nextByteAt = 0;
    messageBuffer.start = MSG_START;
    messageBuffer.stop = MSG_STOP;
    receiveData.sendCount = 0x0;
    
    //Initialize any types you want to use right here? 
    
    //Create a timer
    receiveData.xReceiveTimer = xTimerCreate(  
                     "ReceiveTimer", //Just a text name
                     ( RECEIVE_TIMER_RATE / portTICK_PERIOD_MS ), //period in ms
                     pdTRUE, //auto-reload when expires
                     (void *) 27, //a unique id
                     receiveTimerCallback ); //pointer to callback function
    
    //Start the timer
    if( receiveData.xReceiveTimer == NULL ) {
        dbgOutputVal(RECEIVE_TIMERINIT_FAIL);
        stopAll();
    }
    else if( xTimerStart( receiveData.xReceiveTimer, 0 ) != pdPASS ) {
        dbgOutputVal(RECEIVE_TIMERINIT_FAIL);
        stopAll();
    }

}

void RECEIVE_Tasks ( void )
{
    
    //Function to receive data from the queue
    //Blocks until it receives a byte
    char qData;
    if (xQueueReceive(receiveData.xReceiveIntQ, &qData, portMAX_DELAY))
    {
        dbgOutputVal(qData);   

        if(qData == messageBuffer.start){
       
            LATASET = 1 << 3;
            clearBuffer();
            messageBuffer.buffer[0] = qData; // ~ 
            messageBuffer.nextByteAt = 1;         
        }
        else{

            if(messageBuffer.nextByteAt != sizeof(messageBuffer.buffer)-1){
                messageBuffer.buffer[messageBuffer.nextByteAt] = qData;
                messageBuffer.nextByteAt += 1; 
            }       
            else{

                //If this is true, we have a completed message!
                //Do something with it.
                if(qData == messageBuffer.stop){                

                    //Turn a light on
                    LATACLR = 1 << 3;       

                    receiveData.goodMsg++;
                    
                    //Clear the message
                    clearBuffer();               
                }
                else{
                    receiveData.badMsg++;
                    
                    //Bad message, clear it.
                    clearBuffer();
                }
            }
        } //end if qData receive
        
    }

    
    
    
    switch ( receiveData.state )
    {
        case RECEIVE_STATE_INIT:
        {
            break;
        }
        /* The default state should never be executed. */
        default:
        {
            dbgOutputVal(RECEIVE_ENTERED_DEFAULT);
            break;
        }
    }
}
 

/*******************************************************************************
 End of File
 */