/*******************************************************************************
  MPLAB Harmony Application Source File
  
  Company:
    Microchip Technology Inc.
  
  File Name:
    sensor.c

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

#include "sensor.h"
#include "sender.h"
#include "debug.h"

SENSOR_DATA sensorData;

// Return enumeration for the best signal from IR directional data
unsigned int bestSignal(unsigned int ft, unsigned int rr, unsigned int lt, unsigned int rt) {

    // NONE = 4;
    // LEFT = 3;
    // RIGHT = 2;
    // REAR = 1;
    // FRONT = 0;
    
    if (ft < 10 && rr < 10 && lt < 10 && rt < 10) return 4; // NONE
    
    unsigned int max = 0;
    unsigned int front = ft; 
    unsigned int rear = rr;  
    unsigned int left = lt;  
    unsigned int right = rt; 
    
    if (left > right) max = 3;
    else if (right > rear) max = 2;
    else if (rear > front) max = 1;
    
    if (max == 3) {
        if (left > rear) {
            if (left > front) {}
            else max = 0;
        }
        else if (rear > front) max = 1;
        else max = 0;
    }
    else if (max == 2) {
        if (right > front) {}
        else max = 0;
    }
    
    return max;
}

//runs once
void SENSOR_Initialize ( void )
{
    // Initialize digital input pins A2-A5 (RB2-5)
    ODCBCLR = (0x3D);
    TRISBSET = (0x3D);
    LATBCLR = (0x3D);
   
    // Initialize sequence bytes
    sensorData.sendCount = 0;

	//Initialize task variables
    sensorData.state = SENSOR_STATE_INIT;
	
	//Create a queue capable of holding 25 unsigned long numbers
    sensorData.sensorQ_FR = xQueueCreate( 500, MSG_LENGTH ); 
    if( sensorData.sensorQ_FR == 0 ) {
        dbgOutputVal(SENSOR_QUEUE_FAIL);
        stopAll();
    }
	
    //Create a timer with rollover rate 100ms
	sensorData.sensor_Dist_Timer_FR = xTimerCreate(  
				 "SensorTimer", //Just a text name
				 ( FR_DIST_TIMER_RATE / portTICK_PERIOD_MS ), //period is 100ms
				 pdTRUE, //auto-reload when expires
				 (void *) 14, //a unique id
				 sensorDistTimerCallback ); //pointer to callback function
   
    //Start the timer
    if( sensorData.sensor_Dist_Timer_FR == NULL ) {
        dbgOutputVal(SENSOR_TIMERINIT_FAIL);
        stopAll();
    }
    else if( xTimerStart( sensorData.sensor_Dist_Timer_FR, 0 ) != pdPASS ) {
        dbgOutputVal(SENSOR_TIMERINIT_FAIL);
        stopAll();
    }

    PLIB_ADC_MuxAInputScanEnable(ADC_ID_1);
    DRV_ADC_Open();
    DRV_ADC_Start();
}

void setSendIRData(char send) {
    sensorData.sendIRData = send;
}
// Finite State Machine, runs forever.
void SENSOR_Tasks ( void )
{
    char qData[MSG_LENGTH];
    //char data[MSG_LENGTH];
    
	switch ( sensorData.state )
    {
        case SENSOR_STATE_INIT:
        {
            sensorData.state = SENSOR_STATE_READ;
            break;
        }
		
		case SENSOR_STATE_READ:
		{
            // Receive from IR distance sensor queue
		    if (xQueueReceive(sensorData.sensorQ_FR, &qData, portMAX_DELAY))
			{
                if (CUT_IR_DIST == 0) { 
                    
                    // Front, Rear, Left, Right
                    char best = bestSignal(qData[4], qData[5], qData[7], qData[6]);
        
                    // Convert sensor data to message format character array
                    char data[MSG_LENGTH];
                    data[0] = MSG_START;
                    data[1] = TYPE_ADC;
                    data[2] = sensorData.sendCount;
                    data[3] = best;
                    data[4] = qData[4];  // Front
                    data[5] = qData[5];  // Rear
                    data[6] = qData[6];  // Right
                    data[7] = qData[7];  // Left
                    data[8] = qData[8];  // Dist
                    data[9] = MSG_STOP;
                    
                    //dbgOutputVal(qData);  // Used for debug
                    
                    //putDataOnProcessQ(data);  // TYPEA
                    putDataOnMotorProcessQ(data); // Transfer message to Send task queue
                    
                    // Send to UART at specified interval
                    if ((sensorData.sendIRData == 0x1) && (sensorData.sendCount % DIST_MOD == 0))
                       putMsgOnSendQueue(data);
                    
                    // Increment sequence byte for this type. 
                    sensorData.sendCount++;
                }

			}
			break;
		}
			
        /* The default state should never be executed. */
        default:
        {
            dbgOutputVal(SENSOR_ENTERED_DEFAULT);
            break;
        }
    }//end switch
}//end SENSOR_Tasks()

/*
 * // LEGACY CODE - USED FOR Tri-state 38kHz demodulated IR receivers
// Read direction IR receivers and give data to the Process thread
void readIRReceiverData(){
   
    if (CUT_IR_DIR == 0x1) return;
             
    unsigned int max = 128;
    unsigned int front = max;
    unsigned int rear = max;
    unsigned int left = max;
    unsigned int right = max;
    unsigned int i;
    
    // Check I/O pins <max> number times. Decrement value if high
    for (i = 0; i < max; i++) {
        if (PORTB & 0x10) // 8  
            right--;
        if (PORTB & 0x4) // 20
            front--;
        if (PORTB & 0x8) // 4
            rear--;
        if (PORTB & 0x20) // 10
            left--;
    }
     
    char best = bestSignal(front, rear, left, right);
    
    // Create IR direction message with response data for all four receivers
    char data[MSG_LENGTH];
    data[0] = MSG_START;            // Start byte
    data[1] = TYPE_FR_IR;           // Type byte
    data[2] = sensorData.IRSendCount;  // Count byte
    data[3] = best & 0xF;
    data[4] = 0x33;  // Dummy
    data[5] = front & 0xFF; // ^ 0xFF;
    data[6] = rear & 0xFF; //^ 0xFF;
    data[7] = left & 0xFF; //^ 0xFF;
    data[8] = right & 0xFF; //^ 0xFF;
    data[9] = MSG_STOP;             // Stop byte
          
    //TYPEA
    putDataOnMotorProcessQ(data);  // Transfer message to Send task queue
            
    sensorData.IRSendCount++;   // Increment sequence byte
}
*/

void sendValToSensorTask(unsigned int* message)
{
    if( xQueueSend( sensorData.sensorQ_FR,
                             (void*) message,
                             portMAX_DELAY) != pdPASS )
    {
        dbgOutputVal(SENSOR_SENDTOSENSORQ_FAIL);
        //stopAll(); //failed to send to queue
    }
}

// Remove oldest data on full local sensor queue
// Returns 1 if successful, 0 otherwise
uint8_t removeSensorQueueData() {
    
    unsigned int* qData;
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    
    if (xQueueReceiveFromISR(sensorData.sensorQ_FR, &qData, &xHigherPriorityTaskWoken)) {
        return 0x1;
    }
    
    return 0x0;
}


// Place data read from Sensor onto local queue
void sendValToSensorTaskFromISR(char* data)
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    
    // Check for full queue. If no spaces available, call to remove oldest data.
    if (xQueueIsQueueFullFromISR(sensorData.sensorQ_FR) == pdTRUE) {
        // If message is not removed from queue, return and signal error
        if (removeSensorQueueData() == 0) {
            dbgOutputVal(SENSOR_FULLQUEUE);
            stopAll();
            return;
        }
    }
    
    // Send to queue, with at least one vacancy guaranteed for this data
    if (xQueueSendFromISR( sensorData.sensorQ_FR,
                            (void*) data,
                            &xHigherPriorityTaskWoken) != pdPASS)//errQUEUE_FULL)
    {
        dbgOutputVal(SENSOR_SENDTOSENSORQ_FAIL);
        //stopAll(); //failed to send to queue
    }
}
 

/*******************************************************************************
 End of File
 */
