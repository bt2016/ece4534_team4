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


#include "motor.h"
#include "sender.h"
#include "proj_definitions.h"

MOTOR_DATA motorData;

void MOTOR_Initialize ( void )
{
    motorData.sendCount = 0;
    
    motorData.state = MOTOR_STATE_INIT;
    
    //Create a queue capable of holding 25 unsigned long numbers
    motorData.xMotorQ = xQueueCreate( 250, MSG_LENGTH+1 ); 
    if( motorData.xMotorQ == 0 ) stopAll();
    
    //Create a timer
    motorData.xTimer200ms = xTimerCreate(  
                     "MotorTimer200ms", //Just a text name
                     ( 140 / portTICK_PERIOD_MS ), //period is 200ms
                     pdTRUE, //auto-reload when expires
                     (void *) 25, //a unique id
                     motorTimerCallback ); //pointer to callback function
    
    //Start the timer
    if( motorData.xTimer200ms == NULL ) stopAll();
    else
    {
         if( xTimerStart( motorData.xTimer200ms, 0 ) != pdPASS ) stopAll();
    }
   
}

void MOTOR_Tasks ( void )
{
   while (1)
    {
        switch ( motorData.state )
        {
            case MOTOR_STATE_INIT:
            {
                motorReceiveFromMsgQ();
                break;
            }

            default: /* The default state should never be executed. */
            {
                motorData.state = MOTOR_STATE_INIT;
                break;
            }

        }//end switch
    }//end while
}

void motorSendToMsgQ() {

        LATACLR = 1 << 3;
        
        motorData.sendCount++;
                
        // Convert sensor data to message format character array
        char data[MSG_LENGTH];
        data[0] = MSG_START;            // Start byte
        data[1] = 'm';                  // Type byte
        data[2] = motorData.sendCount;  // Count byte
        // Dummy values
        data[3] = 0x20;
        data[4] = 0x40;
        data[5] = 0x60;
        data[6] = motorData.sendCount << 1;
        data[7] = motorData.sendCount >> 3;
        data[8] = motorData.sendCount << 2;
        data[9] = MSG_STOP;             // Stop byte
                
        putDataOnQueue(data);
}

void putDataOnMotorQ(char* data) {
    if (motorData.xMotorQ != 0) {
        
        if( xQueueSend( motorData.xMotorQ, (void*) data, portMAX_DELAY) != pdPASS )
        {
            //stopAll(); //failed to send to queue
        }
    }
}


void motorReceiveFromMsgQ() {
    char readdata[MSG_LENGTH];
    char newData = 0;
    
    while (uxQueueMessagesWaiting(motorData.xMotorQ) != 0){
    
        if (xQueueReceive(motorData.xMotorQ, &readdata, portMAX_DELAY))
        {
            newData = 1;
            if (readdata[0] == '~' && readdata[9] == '.')
                LATASET = 1 << 3;   
        } 
    }
}




/*
MESSAGE motorRelay = {'~', 'm', 0x0, "123456", '.'};

char* motorTest = "ZZDHJGKFpo";

void motorSendTimerValToMsgQ(unsigned int* sendms)
{    
    if (motorData.xMotorQ != 0) {
        if (uxQueueSpacesAvailable(motorData.xMotorQ) != 0) {
            if( xQueueSend( motorData.xMotorQ, (void*) motorTest, portMAX_DELAY) != pdPASS )
            {
                //stopAll(); //failed to send to queue
            }
        }
    }
}

void motorSendValFromISR(unsigned int* message)
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    if (xQueueSendFromISR( motorData.xMotorQ,
                            (void*) message,
                            &xHigherPriorityTaskWoken) != pdPASS)//errQUEUE_FULL)
    {
        stopAll(); //failed to send to queue
    }
}
 * */

/*******************************************************************************
 End of File
 */
