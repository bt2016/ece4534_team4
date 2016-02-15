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
	//Initialize task variables
    sensorData.state = SENSOR_STATE_INIT;
	
	//Create a queue capable of holding 25 unsigned long numbers
    sensorData.q_adc_interrupt = xQueueCreate( 2500, sizeof( unsigned int ) ); 
    if( sensorData.q_adc_interrupt == 0 ) stopAll();
	
	//Create a timer with rollover rate 100ms
	sensorData.t_adc_interrupt = xTimerCreate(  
				 "Timer100ms", //Just a text name
				 ( 100 / portTICK_PERIOD_MS ), //period is 100ms
				 pdTRUE, //auto-reload when expires
				 (void *) 23, //a unique id
				 sensorTimerCallback ); //pointer to callback function
				 
    //Start the timer
    if( sensorData.t_adc_interrupt == NULL ) stopAll();
    else
    {
         if( xTimerStart( sensorData.t_adc_interrupt, 0 ) != pdPASS ) stopAll();
    }

    //Initialize ADC A0 = Pic32 pin 25, RB0. Manual Sample Start and TAD based Conversion Start
    PLIB_PORTS_PinDirectionInputSet(PORTS_ID_0, PORT_CHANNEL_B, PORTS_BIT_POS_0);
    PLIB_ADC_SampleAutoStartDisable(ADC_ID_1);
    PLIB_ADC_Enable(ADC_ID_1);
					 
}

//runs forever
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
            //dbgOutputVal(59);
		    if (xQueueReceive(sensorData.q_adc_interrupt, &qData, portMAX_DELAY))
			{
                // Convert sensor data to message format character array
                //qData = 0x55565758;
                char data[10];
                data[0] = MSG_START;
                data[1] = 's';
                data[2] = 0x20;
                data[3] = 0x20;
                data[4] = 0x20;
                data[5] = 0x20;
                data[6] = 0x20;
                data[7] = 0x20;
                data[8] = (qData & 0xFF);
                data[9] = MSG_STOP;
                
                putSensorStrOnQueue(data);
			}
			break;
		}
			
        /* The default state should never be executed. */
        default:
        {
            break;
        }
    }//end switch
}//end SENSOR_Tasks()


void sendValToSensorTask(unsigned int* message)
{
    if( xQueueSend( sensorData.q_adc_interrupt,
                             (void*) message,
                             portMAX_DELAY) != pdPASS )
    {
        stopAll(); //failed to send to queue
    }
}

void sendValToSensorTaskFromISR(unsigned int* message)
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    if (xQueueSendFromISR( sensorData.q_adc_interrupt,
                            (void*) message,
                            &xHigherPriorityTaskWoken) != pdPASS)//errQUEUE_FULL)
    {
        stopAll(); //failed to send to queue
    }
}
 

/*******************************************************************************
 End of File
 */
