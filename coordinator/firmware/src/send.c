/*******************************************************************************
  MPLAB Harmony Application Source File
  
  Company:
    Microchip Technology Inc.
  
  File Name:
    send.c

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


#include "send.h"
#include "sender.h"

SEND_DATA sendData;

// Called from TX interrupt. Read from transmit message queue and 
// output byte by byte to the transmit buffer
void receiveDataFromISR() {
    BaseType_t xTaskWokenByReceive = pdFALSE;
    char readChar;

    // Temporarily disable the transmit interrupt
    PLIB_INT_SourceDisable(INT_ID_0, INT_SOURCE_USART_1_TRANSMIT);
        
    // While the queue has available messages, read bytes and transmit
    while (uxQueueMessagesWaitingFromISR(sendData.transmitQ_CD) != 0){
        while (xQueueReceiveFromISR(sendData.transmitQ_CD, (void *) &readChar, &xTaskWokenByReceive) ) 
        {
            // A character was received.  Transmit the character now.
            if (sendData.testCount % BREAK_MESSAGE_DIV != 2) // Error simulation constant (missing byte)
                PLIB_USART_TransmitterByteSend(USART_ID_1, readChar);
            
            // Duplicate data error simulation constant
            if (sendData.testCount % ADD_MESSAGE_DIV == 1)
                PLIB_USART_TransmitterByteSend(USART_ID_1, readChar);
            
            // If removing the character from the queue woke the task that was
            // posting onto the queue cTaskWokenByReceive will have been set to
            // pdTRUE.  No matter how many times this loop iterates only one
            // task will be woken.
            sendData.testCount++;
        }
    }
}
   

// FROM SEND TASK TO TRANSMIT QUEUE (ON WAY TO UART)
void sendDataToTransmitQ(char* message) {
    
    // Check for proper message start - minor preliminary error check
    if (*message == '~') {
        char* k = message;
        int iter = 0;
        
        // SEND TO UART QUEUE
        if (sendData.transmitQ_CD != 0) {

            //for (k = message; *k; ++k) {
            // Add message to queue, character by character
            for (iter = 0; iter < MSG_LENGTH; iter++) {

                if( xQueueSend( sendData.transmitQ_CD, (void*) k, portMAX_DELAY) != pdPASS )
                {
                    dbgOutputVal(SEND_SENDTOTRANSMITQ_FAIL);
                }

                ++k; // Iterate to next char in message

                // Enable transmitter interrupt to signal empty buffer
                PLIB_INT_SourceEnable(INT_ID_0, INT_SOURCE_USART_1_TRANSMIT); 
            }
        }
    }
}

// Check queue holding the data from other component tasks
void receiveFromSendQ()
{    
   char newData = 0;
    char read[MSG_LENGTH];

    // Read the top message of the queue
    if (xQueueReceive(sendData.sendQ_CD, &read, portMAX_DELAY))
    {
        //dbgOutputVal(SEND_RECEIVEFROMQ);
        newData = 1;
    } 

    // If message received, push to UART transmit queue
    // If statement protects against repeat data.
    if (newData && ((read[1] != sendData.prevType) || (read[2] != sendData.prevCount))) {
        sendDataToTransmitQ(read);
    }
    
    sendData.prevType = read[1];
    sendData.prevCount = read[2];
}

// Remove oldest data on full local data queue
// Returns 1 if successful, 0 otherwise
uint8_t removeSendQueueData() {
    char readdata[MSG_LENGTH];
 
    if (xQueueReceive(sendData.sendQ_CD, &readdata, portMAX_DELAY))
    {
        return 0x1;
    } 
    
    return 0x0;
}

// Place data passed from other tasks into send queue
void putMsgOnSendQueue(char* data) {
    
    sendData.enqueueCount++;
    
    // Error simulation code, skip send enqueue
    if (sendData.enqueueCount % MESSAGE_SKIP_DIV == 1)
        return;
    
    if (sendData.sendQ_CD != 0) {
        
        // Check for full queue. If no spaces available, call to remove oldest data.
        if (uxQueueSpacesAvailable( sendData.sendQ_CD ) == 0) {
            // If message is not removed from queue, return and signal error
            if (removeSendQueueData() == 0) {
                dbgOutputVal(SEND_FULLQUEUE);
                //stopAll();
                return;
            }
        }
        
        // Send to queue, with at least one vacancy guaranteed for this data
        if( xQueueSend( sendData.sendQ_CD, (void*) data, portMAX_DELAY) != pdPASS )
        {
            dbgOutputVal(MOTOR_SENDTOSENDQ_FAIL);
        }
    }
}


void SEND_Initialize ( void )
{
    sendData.state = SEND_STATE_INIT;
    sendData.sendCount = 0x55;
    sendData.testCount = 0;
    sendData.enqueueCount = 0;
        sendData.prevType = 'a';
    sendData.prevCount = 'a';
    
    //Create a queue capable of holding 1000 characters
    sendData.transmitQ_CD = xQueueCreate( 1000, sizeof(char) );
    if( sendData.transmitQ_CD == 0 ) {
        dbgOutputVal(SEND_QUEUE_FAIL);
        stopAll();
    }
    
    // Create a queue capable of holding 250 messages
    sendData.sendQ_CD = xQueueCreate(250, MSG_LENGTH+1);
    if (sendData.sendQ_CD == 0) {
        dbgOutputVal(SEND_QUEUE_FAIL);
        stopAll();
    }
    
    //Create a timer
    sendData.xTimer100ms = xTimerCreate(  
                     "SendTimer100ms", //Just a text name
                     ( SEND_TIMER_RATE / portTICK_PERIOD_MS ), //period is 100ms
                     pdTRUE, //auto-reload when expires
                     (void *) 23, //a unique id
                     vTimerCallback ); //pointer to callback function
    
    //Start the timer
    if( sendData.xTimer100ms == NULL ) {
        dbgOutputVal(SEND_TIMERINIT_FAIL);
        stopAll();
    }
    else if( xTimerStart( sendData.xTimer100ms, 0 ) != pdPASS ) {
        dbgOutputVal(SEND_TIMERINIT_FAIL);
        stopAll();
    }
}

// Finite state machine definition, runs forever
void SEND_Tasks ( void )
{
   while (1)
    {
        switch ( sendData.state )
        {
            case SEND_STATE_INIT:
            {
                sendData.state = SEND_STATE_LOOP;
                break;
            }
            
            // Continually check data receive queue for new motor/sensor output
            case SEND_STATE_LOOP:
            {
                receiveFromSendQ();
                break;
            }

            default: /* The default state should never be executed. */
            {
                dbgOutputVal(SEND_ENTERED_DEFAULT);
                sendData.state = SEND_STATE_LOOP;
                break;
            }

        }//end switch
    }//end while
}


/*******************************************************************************
 End of File
 */
