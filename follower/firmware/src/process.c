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
#include "sender.h"
#include "proj_definitions.h"



PROCESS_DATA processData; 
LD_POS leadPos;         // Running sensor data on lead rover's position.


// Update the running data on the lead rover's position in terms of IR received signals
// and distance sensing results
void processUpdateLeadPos(char type, char d1, char d2, char d3, char d4, char d5, char d6) {

    // Update front distance (in cm)
    if (type == (char)TYPE_FR_DIST) {
        leadPos.frontDistance = d6;
    }
    // Update directional IR data (all four at once)
    else if (type == (char)TYPE_FR_IR) {
        leadPos.frontSignal = d3;
        leadPos.rearSignal = d4;
        leadPos.leftSignal = d5;
        leadPos.rightSignal = d6;
    }
}

// Check queue holding the data from other component tasks
void receiveFromProcessQ()
{    
    char newData = 0;
    char read[MSG_LENGTH];

    // Read the top message of the queue
    if (xQueueReceive(processData.processQ_FR, &read, portMAX_DELAY))
    {
        //dbgOutputVal(SEND_RECEIVEFROMQ);
        newData = 1;
    } 

    // If new data was received, update the process.c's understanding of the lead's current whereabouts
    if (newData) {
        processUpdateLeadPos(read[1], read[3], read[4], read[5], read[6], read[7], read[8]);
    }
    
    //Used in error checking
    //processData.prevType = read[1];
    //processData.prevCount = read[2];
}

unsigned int bestMotorSignal() {
    unsigned int front = 0;
    unsigned int rear = 0;
    unsigned int right = 0;
    unsigned int left = 0;
    
    unsigned int max = 0;
    
    if (leadPos.leftSignal > leadPos.rightSignal) max = 3;
    else if (leadPos.rightSignal > leadPos.rearSignal) max = 2;
    else if (leadPos.rearSignal > leadPos.frontSignal) max = 1;
    
    if (max == 3) {
        if (leadPos.leftSignal > leadPos.rearSignal) {
            if (leadPos.leftSignal > leadPos.frontSignal) {}
            else max = 0;
        }
        else if (leadPos.rearSignal > leadPos.frontSignal) max = 1;
        else max = 0;
    }
    else if (max == 2) {
        if (leadPos.rightSignal > leadPos.frontSignal) {}
        else max = 0;
    }
    
    return max;
}
        
adjustSig() {
    
    leadPos.frontSignal = 0;
    leadPos.rightSignal = 0;
    leadPos.leftSignal = 0;
    leadPos.rearSignal = 0;
    
    if (TEST_FRONT) leadPos.frontSignal = 128;
    if (TEST_RIGHT) leadPos.rightSignal = 128;
    if (TEST_LEFT) leadPos.leftSignal = 128;
    if (TEST_REAR) leadPos.rearSignal = 128;
}
        
// Determine motor instructions to relay to motor queue.
// Includes right/left motor direction and power (to be interpreted with PWM in motor.c)
void processSendToMotorQ() {
    
    // Default values
    char rightDir = MOTOR_FORWARD;
    char leftDir = MOTOR_FORWARD;
    unsigned int rightSpeed = 0;
    unsigned int leftSpeed = 0;
    
    if (HC_SIGNALS) adjustSig();
    
  
    // Conditionals to determine motor action based on current sensor data
    // No credible directional data: 
    if ((leadPos.frontSignal < 10 && leadPos.rearSignal < 10 && leadPos.rightSignal < 10 && leadPos.leftSignal < 10)
    || (leadPos.frontSignal == 129 && leadPos.rearSignal == 129 && leadPos.rightSignal == 129 && leadPos.leftSignal == 129)) {
        if (leadPos.frontDistance > 7 && leadPos.frontDistance < 40) {
            rightSpeed = (leadPos.frontDistance << 1) + 20;
            leftSpeed = (leadPos.frontDistance << 1) + 20;
        }
        else if (leadPos.frontDistance < 6) {
            rightSpeed = 30;
            leftSpeed = 30;
            rightDir = MOTOR_REVERSE;
            leftDir = MOTOR_REVERSE;
        }
        else {
            rightDir = MOTOR_STOP;
            leftDir = MOTOR_STOP;
        }
    }
    // Directional data received from two opposing sensors: Stop rover
    else if ((leadPos.frontSignal > 40 && leadPos.rearSignal > 40) || (leadPos.rightSignal > 40 && leadPos.leftSignal > 40)) {
        rightDir = MOTOR_STOP;
        leftDir = MOTOR_STOP;
    }
    else {
        
        // Determine the strongest signal source from directional receivers.
        // If equal, priority goes Front > Rear > Right > Left
        unsigned int max;
        max = bestMotorSignal();

        // Strong signal in front of rover:
        if (max == LEAD_FRONT) {
            // Lead rover is directly in front
            if (leadPos.frontDistance < 6) {
                rightDir = MOTOR_REVERSE;
                leftDir = MOTOR_REVERSE;
                rightSpeed = 30;
                leftSpeed = 30;
            }
            if (leadPos.frontDistance < 8) {
                rightDir = MOTOR_STOP;
                leftDir = MOTOR_STOP;
            }
            // Distance sensor picks up lead rover farther than minimum desired tracking distance
            // Adjust for offset if the left or right receivers are also picking up data
            else if (leadPos.frontDistance > 12) {
                rightSpeed = 30 + (leadPos.frontDistance) + ((leadPos.leftSignal >> 2) & 0x3F);
                leftSpeed = 30 + (leadPos.frontDistance) + ((leadPos.rightSignal >> 2) & 0x3F);
            }
            // Lead rover is within short distance but not dangerously close
            else {
                rightSpeed = 30 + ((leadPos.leftSignal >> 2) & 0x3F);
                leftSpeed = 30 + ((leadPos.rightSignal >> 2) & 0x3F);
            }
        }
        // Strong signal behind rover: Follower turns around, direction depending on side data
        else if (max == LEAD_REAR) {
            // Right side favored: Turn right by rotating right wheel backward, left forward
            if (leadPos.rightSignal >= leadPos.leftSignal) {
                rightDir = MOTOR_REVERSE;
                rightSpeed = 80;
                leftSpeed = 80;
            }
            // Strong signal left: Commit to left turn
            else {
                leftDir = MOTOR_REVERSE;
                rightSpeed = 80;
                leftSpeed = 80;
            }
        }
        // Stronger signal right of rover:
        else if (max == LEAD_RIGHT) {
            rightDir = MOTOR_REVERSE;
            rightSpeed = 50;
            leftSpeed = 50;
        }
        else if (max == LEAD_LEFT) {
            leftDir = MOTOR_REVERSE;
            rightSpeed = 50;
            leftSpeed = 50;
        }
        // Default state: If this is true, something went wrong
        else {
            rightDir = MOTOR_STOP;
            leftDir = MOTOR_STOP;
        }
    }
    
    // Establish system message holding key motor data
    char data[MSG_LENGTH];
    data[0] = MSG_START;                       // Start byte
    data[1] = TYPEC_MOTOR_CONTROL;             // Type byte
    data[2] = processData.motorSendCount;      // Count byte
    data[3] = (leadPos.frontDistance & 0xFF);  // Sensor value (redundant)
    data[4] = 0x40;                 // Dummy
    data[5] = rightDir;             // Right motor forward/reverse/stop
    data[6] = (rightSpeed & 0xFF);  // Right motor speed
    data[7] = leftDir;              // Left motor forward/reverse/stop
    data[8] = (leftSpeed & 0xFF);   // Left motor speed
    data[9] = MSG_STOP;             // Stop byte
                
    processData.motorSendCount++;   // Increment sequence byte
    
    // Send to UART at specified interval
    if ((SEND_MOTOR == 0x1) && (processData.motorSendCount % MOTOR_MOD == 0))
        putMsgOnSendQueue(data);
    
    putDataOnMotorQ(data); // Send via queue to motor.c to process for actuators
}

// Remove oldest data on full local process queue
// Returns 1 if successful, 0 otherwise
uint8_t removeProcessQueueData() {
    char readdata[MSG_LENGTH];

    if (xQueueReceive(processData.processQ_FR, &readdata, portMAX_DELAY))
    {
        return 0x1;
    } 
    
    return 0x0;
}

// Check local process queue for vacancies. If none, remove oldest data.
// After, add the parameter char* message to local process queue. 
void putDataOnProcessQ(char* data) {
    if (processData.processQ_FR != 0) {
        
        // Check for full queue. If no spaces available, call to remove oldest data.
        if (uxQueueSpacesAvailable( processData.processQ_FR ) == 0) {
            // If message is not removed from queue, return and signal error
            if (removeProcessQueueData() == 0) {
                dbgOutputVal(MOTOR_FULLQUEUE);
                stopAll();
                return;
            }
        }
        
        // Send to queue, with at least one vacancy guaranteed for this data
        if( xQueueSend( processData.processQ_FR, (void*) data, portMAX_DELAY) != pdPASS )
        {
            dbgOutputVal(RECEIVE_SENDTOMOTORQ_FAIL);
        }
    }
}


void PROCESS_Initialize ( void )
{
    /* Place the App state machine in its initial state. */
    processData.state = PROCESS_STATE_INIT;
    
    processData.motorSendCount = 0;
    processData.prevType = 'a';
    processData.prevCount = 0x80;
    
    // Initialize default lead rover values
    leadPos.frontDistance = 0;
    leadPos.frontSignal = 129;
    leadPos.rearSignal = 129;
    leadPos.rightSignal = 129;
    leadPos.leftSignal = 129;
    
    //Create a queue capable of holding 1000 characters (bytes))
    processData.processQ_FR = xQueueCreate(1000, MSG_LENGTH+1 ); 
    if( processData.processQ_FR == 0 ) {
        dbgOutputVal(PROCESS_QUEUE_FAIL);
        stopAll(); //ERROR
    }
    
    //Timer to send to internal task queue
    processData.process_Timer_FR = xTimerCreate(  
                     "ProcessTimer", //Just a text name
                     ( PROC_TIMER_RATE / portTICK_PERIOD_MS ), //period in ms
                     pdTRUE, //auto-reload when expires
                     (void *) 31, //a unique id
                     processTimerCallback ); //pointer to callback function
    
    //Start the timer
    if( processData.process_Timer_FR == NULL ) {
        dbgOutputVal(PROCESS_TIMERINIT_FAIL);
        stopAll();
    }
    else if( xTimerStart( processData.process_Timer_FR, 0 ) != pdPASS ) {
        dbgOutputVal(PROCESS_TIMERINIT_FAIL);
        stopAll();
    }
    
}

void PROCESS_Tasks ( void )
{
    /* Check the application's current state. */
    switch ( processData.state )
    {
        /* Application's initial state. */
        case PROCESS_STATE_INIT:
        {
            processData.state = PROCESS_STATE_PROCESS;

            break;
        }
        
        case PROCESS_STATE_PROCESS:
        {
            // Receive data from the process queue from local sensors and motor tasks
            receiveFromProcessQ();
            // Call functions for rover identification and movement decision algorithms
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
