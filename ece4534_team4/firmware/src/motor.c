/*******************************************************************************
  MPLAB Harmony Application Source File
  
  Company:
    Microchip Technology Inc.
  
  File Name:
    motor.c

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
    
    //Create a queue capable of holding 250 system messages
    motorData.xMotorQ = xQueueCreate( 250, MSG_LENGTH+1 ); 
    if( motorData.xMotorQ == 0 ) {
        dbgOutputVal(MOTOR_QUEUE_FAIL);
        stopAll();
    }
    
    //Create a timer
    motorData.xMotorTimer = xTimerCreate(  
                     "MotorTimer", //Just a text name
                     ( MOTOR_TIMER_RATE / portTICK_PERIOD_MS ), //period in ms
                     pdTRUE, //auto-reload when expires
                     (void *) 25, //a unique id
                     motorTimerCallback ); //pointer to callback function
    
    //Start the timer
    if( motorData.xMotorTimer == NULL ) {
        dbgOutputVal(MOTOR_TIMERINIT_FAIL);
        stopAll();
    }
    else if( xTimerStart( motorData.xMotorTimer, 0 ) != pdPASS ) {
        dbgOutputVal(MOTOR_TIMERINIT_FAIL);
        stopAll();
    }
   
}

// Finite state machine, runs forever
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
                dbgOutputVal(MOTOR_ENTERED_DEFAULT);
                break;
            }

        }//end switch
    }//end while
}

// Pack and transfer message to data queue in Send task
void motorSendToMsgQ() {

        //LATACLR = 1 << 3; // Clear LED
        
        motorData.sendCount++;
        
        // Error simulation constant - skip count byte in messages
        if (MESSAGE_COUNT_SKIP_DIV > (rand() % 20))
            motorData.sendCount++;
                
        // Convert sensor data to message format character array
        char data[MSG_LENGTH];
        data[0] = MSG_START;            // Start byte
        data[1] = TYPE_LR_ENCODER;      // Type byte
        data[2] = motorData.sendCount;  // Count byte
        // Dummy values (data bytes x6)
        data[3] = 0x22;
        data[4] = 0x33;
        data[5] = 0x44;
        data[6] = 0x55; 
        data[7] = 0x66; 
        data[8] = 0x77;
        data[9] = MSG_STOP;             // Stop byte
                
        putMsgOnSendQueue(data);  // Transfer message to Send task queue
}

// Remove oldest data on full local motor queue
// Returns 1 if successful, 0 otherwise
uint8_t motorRemoveQueueData() {
    char readdata[MSG_LENGTH];

    if (xQueueReceive(motorData.xMotorQ, &readdata, portMAX_DELAY))
    {
        return 0x1;
    } 
    
    return 0x0;
}

// Check local motor queue for vacancies. If none, remove oldest data.
// After, add the parameter char* message to local motor queue. 
void putDataOnMotorQ(char* data) {
    if (motorData.xMotorQ != 0) {
        
        // Check for full queue. If no spaces available, call to remove oldest data.
        if (uxQueueSpacesAvailable( motorData.xMotorQ ) == 0) {
            // If message is not removed from queue, return and signal error
            if (motorRemoveQueueData() == 0) {
                dbgOutputVal(MOTOR_FULLQUEUE);
                stopAll();
                return;
            }
        }
        
        // Send to queue, with at least one vacancy guaranteed for this data
        if( xQueueSend( motorData.xMotorQ, (void*) data, portMAX_DELAY) != pdPASS )
        {
            dbgOutputVal(RECEIVE_SENDTOMOTORQ_FAIL);
        }
    }
}

// Read value from local message queue
void motorReceiveFromMsgQ() {
    char readdata[MSG_LENGTH];
    char newData = 0;
    
    // Read from messages until most recent data (FIFO queue)
    while (uxQueueMessagesWaiting(motorData.xMotorQ) != 0){
    
        if (xQueueReceive(motorData.xMotorQ, &readdata, portMAX_DELAY))
        {
            dbgOutputVal(MOTOR_RECEIVEFROMQ);
            newData = 1;
            
            // Check for correct message format
            //if (readdata[0] == '~' && readdata[9] == '.')
                //LATASET = 1 << 3;   // Light LED
        } 
    }
}

/*******************************************************************************
 End of File
 */
