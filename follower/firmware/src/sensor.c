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

SENSOR_DATA sensorData;

//runs once
void SENSOR_Initialize ( void )
{
    sensorData.sendCount = 0;
    sensorData.senseCount = 0;
    sensorData.IRSendCount = 0;
    sensorData.distSendCount = 0;
    
	//Initialize task variables
    sensorData.state = SENSOR_STATE_INIT;
	
	//Create a queue capable of holding 25 unsigned long numbers
    sensorData.sensorQ_FR = xQueueCreate( 2500, sizeof( unsigned int ) ); 
    if( sensorData.sensorQ_FR == 0 ) {
        dbgOutputVal(SENSOR_QUEUE_FAIL);
        stopAll();
    }
	
	//Create a timer with rollover rate 100ms
	sensorData.sensor_IR_Timer_FR = xTimerCreate(  
				 "SensorTimer", //Just a text name
				 ( FR_IR_TIMER_RATE / portTICK_PERIOD_MS ), //period is 100ms
				 pdTRUE, //auto-reload when expires
				 (void *) 29, //a unique id
				 sensorIRTimerCallback ); //pointer to callback function
				 
    //Create a timer with rollover rate 100ms
	sensorData.sensor_Dist_Timer_FR = xTimerCreate(  
				 "SensorTimer", //Just a text name
				 ( FR_DIST_TIMER_RATE / portTICK_PERIOD_MS ), //period is 100ms
				 pdTRUE, //auto-reload when expires
				 (void *) 29, //a unique id
				 sensorDistTimerCallback ); //pointer to callback function
    
    //Start the timer
    if( sensorData.sensor_IR_Timer_FR == NULL ) {
        dbgOutputVal(SENSOR_TIMERINIT_FAIL);
        stopAll();
    }
    else if( xTimerStart( sensorData.sensor_IR_Timer_FR, 0 ) != pdPASS ) {
        dbgOutputVal(SENSOR_TIMERINIT_FAIL);
        stopAll();
    }
    
    //Start the timer
    if( sensorData.sensor_Dist_Timer_FR == NULL ) {
        dbgOutputVal(SENSOR_TIMERINIT_FAIL);
        stopAll();
    }
    else if( xTimerStart( sensorData.sensor_Dist_Timer_FR, 0 ) != pdPASS ) {
        dbgOutputVal(SENSOR_TIMERINIT_FAIL);
        stopAll();
    }

    //Initialize ADC A0 = Pic32 pin 25, RB0. Manual Sample Start and TAD based Conversion Start
    PLIB_PORTS_PinDirectionInputSet(PORTS_ID_0, PORT_CHANNEL_B, PORTS_BIT_POS_0);
    PLIB_ADC_SampleAutoStartDisable(ADC_ID_1);
    PLIB_ADC_Enable(ADC_ID_1);
					 
}

// Finite State Machine, runs forever.
void SENSOR_Tasks ( void )
{
    unsigned int qData;
    char data[MSG_LENGTH];
    
	switch ( sensorData.state )
    {
        case SENSOR_STATE_INIT:
        {
			sensorData.state = SENSOR_STATE_READ;
            break;
        }
		
		case SENSOR_STATE_READ:
		{
            // Receive from sensor queue
		    if (xQueueReceive(sensorData.sensorQ_FR, &qData, portMAX_DELAY))
			{
                if (!CUT_SENSOR) {
                    
                    // Convert sensor data to message format character array
                    //qData = 0x55565758;
                    char data[10];
                    data[0] = MSG_START;
                    data[1] = TYPE_FR_DIST;
                    data[2] = sensorData.sendCount;
                    data[3] = 0x20;
                    data[4] = 0x20;
                    data[5] = 0x20;
                    data[6] = 0x20;
                    data[7] = 0x20;
                    data[8] = (qData & 0xFF);
                    data[9] = MSG_STOP;

                    // Error simulation constant - skip count byte in messages
                    putDataOnProcessQ(data); // Transfer message to Send task queue

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

// Read direction IR receivers and give data to the Process thread
void readIRReceiverData(){
    // If disable bool set, do not give message to Process queue
    if (CUT_SENSOR) return;
                
    // Convert sensor data to message format character array
    char data[MSG_LENGTH];
    data[0] = MSG_START;            // Start byte
    data[1] = TYPE_FR_IR;           // Type byte
    data[2] = sensorData.IRSendCount;  // Count byte
    // Dummy values (data bytes x6)
    data[3] = 0x22;
    data[4] = 0x33;
    data[5] = 0x44;
    data[6] = 0x55; 
    data[7] = 0x66; 
    data[8] = 0x77;
    data[9] = MSG_STOP;             // Stop byte
          
    putDataOnProcessQ(data);  // Transfer message to Send task queue
            
    sensorData.IRSendCount++;
}


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
void sendValToSensorTaskFromISR(unsigned int* message)
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
                            (void*) message,
                            &xHigherPriorityTaskWoken) != pdPASS)//errQUEUE_FULL)
    {
        dbgOutputVal(SENSOR_SENDTOSENSORQ_FAIL);
        //stopAll(); //failed to send to queue
    }
}
 

/*******************************************************************************
 End of File
 */
