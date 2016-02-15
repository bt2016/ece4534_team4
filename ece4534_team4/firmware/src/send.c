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
char cnt = 0x0;

void sendWriteMessage() {
    // Use transmit interrupt for UART 
    // Same interrupt as receiver - account for both (clear both), handle the one that was raised
    
    //writeMsgChar(msg_type, msg_data1, msg_data2);
    //writeString("roo");
    //writeMsgShortInt(msg_type, msg_datashort);
}

void SEND_Initialize ( void )
{
    sendData.state = SEND_STATE_INIT;

    sendData.letterPosition = 0;
    
    //Create a queue capable of holding 25 unsigned long numbers
    sendData.xTimerIntQ = xQueueCreate( 25, sizeof( unsigned int ) ); 
    if( sendData.xTimerIntQ == 0 ) stopAll();
    
    //Create a timer
    sendData.xTimer100ms = xTimerCreate(  
                     "SendTimer100ms", //Just a text name
                     ( 100 / portTICK_PERIOD_MS ), //period is 100ms
                     pdTRUE, //auto-reload when expires
                     (void *) 23, //a unique id
                     vTimerCallback ); //pointer to callback function
    
    //Start the timer
    if( sendData.xTimer100ms == NULL ) stopAll();
    else
    {
         if( xTimerStart( sendData.xTimer100ms, 0 ) != pdPASS ) stopAll();
    }
   
}

void SEND_Tasks ( void )
{
   while (1)
    {
        //unsigned int qData;
       char qData[MSG_LENGTH];

        switch ( sendData.state )
        {
            case SEND_STATE_INIT:
            {
                writeString("START");
                sendData.state = SEND_STATE_RECEIVE;
                break;
            }
            
            case SEND_STATE_RECEIVE:
            {
                //checkSourceQ();
                
                //writeString("Asking Queue...");
                if (xQueueReceive(sendData.xTimerIntQ, &qData, portMAX_DELAY))
                {
                    //writeString("Received\n");
                    dbgOutputVal(cnt);
                } 
                
                sendData.state = SEND_STATE_TRANSMIT;
                break;
            }

            case SEND_STATE_TRANSMIT:
            {
                //writeString("Received: ");
               writeMsgStr(cnt, qData);
               sendData.state = SEND_STATE_RECEIVE;
               break;
            }

            default: /* The default state should never be executed. */
            {
                writeString("DEFAULT_ERROR");
                sendData.state = SEND_STATE_TRANSMIT;
                break;
            }

        }//end switch
    }//end while
}

MESSAGE relay = {'~', 'm', 0x0, "123456", '.'};

void sendDataToMsgQ(char* message) {
    
    cnt++;
    unsigned int test = 20;
    
    if (sendData.xTimerIntQ != 0) {
        //writeString("Requesting send...");
        if( xQueueSend( sendData.xTimerIntQ, (void*) message, portMAX_DELAY) != pdPASS )
        {
            stopAll(); //failed to send to queue
        }
        //writeString("Posted\n");
    }
}
 
void checkSourceQ()
{    
    char readdata[MSG_LENGTH];
    //writeString("Asking Queue...");
    if (xQueueReceive(sendData.xTimerIntQ, &readdata, portMAX_DELAY))
    {
        //writeString("Received\n");
    } 
    
    sendDataToMsgQ(readdata);
}

char* test = "HEYYOUZZAA";

void sendTimerValToMsgQ(unsigned int* sendms)
{    
    cnt++;
    
    if (sendData.xTimerIntQ != 0) {
        //writeString("SENT:");
        if( xQueueSend( sendData.xTimerIntQ, (void*) test, portMAX_DELAY) != pdPASS )
        {
            stopAll(); //failed to send to queue
        }
    }

}

void sendValFromISR(unsigned int* message)
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    if (xQueueSendFromISR( sendData.xTimerIntQ,
                            (void*) message,
                            &xHigherPriorityTaskWoken) != pdPASS)//errQUEUE_FULL)
    {
        stopAll(); //failed to send to queue
    }
}

/*******************************************************************************
 End of File
 */
