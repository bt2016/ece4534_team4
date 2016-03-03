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
    
	//Initialize task variables
    sensorData.state = SENSOR_STATE_INIT;
	
	//Create a queue capable of holding 25 unsigned long numbers
    sensorData.sensorQ_SA = xQueueCreate( 2500, sizeof( unsigned int ) ); 
    if( sensorData.sensorQ_SA == 0 ) {
        dbgOutputVal(SENSOR_QUEUE_FAIL);
        stopAll();
    }
	
	//Create a timer with rollover rate 100ms
	sensorData.sensorDistTimer_SA = xTimerCreate(  
				 "SensorTimer", //Just a text name
				 ( SA_DIST_TIMER_RATE / portTICK_PERIOD_MS ), //period is 100ms
				 pdTRUE, //auto-reload when expires
				 (void *) 29, //a unique id
				 sensorTimerCallback ); //pointer to callback function
	sensorData.servoMovementTimer_SA = xTimerCreate(  
				 "ServoMovementTimer", //Just a text name
				 ( 200 / portTICK_PERIOD_MS ), //period is 200ms
				 pdFALSE, //do not auto-reload when expires
				 (void *) 30, //a unique id
				 servoMovementTimerCallback ); //pointer to callback function
				 
    //Start the timer
    if( sensorData.sensorDistTimer_SA == NULL ) {
        dbgOutputVal(SENSOR_TIMERINIT_FAIL);
        stopAll();
    }
    else if( xTimerStart( sensorData.sensorDistTimer_SA, 0 ) != pdPASS ) {
        dbgOutputVal(SENSOR_TIMERINIT_FAIL);
        stopAll();
    }

    //Initialize ADC A0 = Pic32 pin 25, RB0. Manual Sample Start and TAD based Conversion Start
    PLIB_PORTS_PinDirectionInputSet(PORTS_ID_0, PORT_CHANNEL_B, PORTS_BIT_POS_0);
    PLIB_ADC_SampleAutoStartDisable(ADC_ID_1);
    PLIB_ADC_Enable(ADC_ID_1);
	
    //Start PWM timer
    DRV_TMR0_Stop();
    DRV_TMR0_Initialize();
    DRV_TMR0_Start();
    
    //Initialize PWM
    DRV_OC0_Disable();
    DRV_OC0_Initialize();
    DRV_OC0_Enable();
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
            //set the servo to zero
            setServoAngle(SERVOANGLE_MIN);
            
            //start the servoMovementTimer
            startServoMovementTimer();
            
            //change state
			sensorData.state = SENSOR_STATE_TAKEREADINGS;
            break;
        }
		
        case SENSOR_STATE_TAKEREADINGS:
        {
            Obstacle o;
            //block until you receive a value from the ADC
            if (xQueueReceive(sensorData.sensorQ_SA, &qData, portMAX_DELAY)){
                //sensorData.r[sensorData.servo_angle - SERVOANGLE_MIN] = qData; //add the value to the array
                o.r = qData;
                o.theta = sensorData.servo_angle - SERVOANGLE_MIN;
                putDataOnProcessQ(&o);
                
                //check to see if we are finished panning
                if (sensorData.servo_angle < SERVOANGLE_MAX){
                    incrementServo();
                    startServoMovementTimer();
                }
                else{
                    //set the servo to zero and change state
                    setServoAngle(SERVOANGLE_MIN);
                    //sensorData.state = SENSOR_STATE_FINDOBSTACLES;
                }
            }
            break;
        }
        
        case SENSOR_STATE_FINDOBSTACLES:
        {
            break;
        }
        
		case SENSOR_STATE_READ:
		{
            // Receive from sensor queue
		    if (xQueueReceive(sensorData.sensorQ_SA, &qData, portMAX_DELAY))
			{
                if (!CUT_SENSOR) {
                    
                    // Convert sensor data to message format character array
                    char data[10];
                    data[0] = MSG_START;
                    data[1] = TYPE_SENSOR_DIST;
                    data[2] = sensorData.sendCount;
                    data[3] = 0x20;
                    data[4] = 0x20;
                    data[5] = 0x20;
                    data[6] = 0x20;
                    data[7] = ((qData & 0xFF00) >> 8);
                    data[8] = (qData & 0xFF);
                    data[9] = MSG_STOP;

                    // Error simulation constant - skip count byte in messages
                    // Only report token found to send queue (UART) every 10 readings (MS#2 TESTING)
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


void sendValToSensorTask(unsigned int* message)
{
    if( xQueueSend( sensorData.sensorQ_SA,
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
    
    if (xQueueReceiveFromISR(sensorData.sensorQ_SA, &qData, &xHigherPriorityTaskWoken)) {
        return 0x1;
    }
    
    return 0x0;
}

// Place data read from Sensor onto local queue
void sendValToSensorTaskFromISR(unsigned int* message)
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    
    // Check for full queue. If no spaces available, call to remove oldest data.
    if (xQueueIsQueueFullFromISR(sensorData.sensorQ_SA) == pdTRUE) {
        // If message is not removed from queue, return and signal error
        if (removeSensorQueueData() == 0) {
            dbgOutputVal(SENSOR_FULLQUEUE);
            stopAll();
            return;
        }
    }
    
    // Send to queue, with at least one vacancy guaranteed for this data
    if (xQueueSendFromISR( sensorData.sensorQ_SA,
                            (void*) message,
                            &xHigherPriorityTaskWoken) != pdPASS)//errQUEUE_FULL)
    {
        dbgOutputVal(SENSOR_SENDTOSENSORQ_FAIL);
        //stopAll(); //failed to send to queue
    }
}

void setServoAngle(int angle){
    //convert passed-in angle to a pulse time
    //time [ms] = (0.0103*degrees) + 0.5412;
    sensorData.servo_angle = angle;
    double time_ms = (0.0103*(double)angle) + 0.5412;
    int time_value = (6249*time_ms)/20;
    
    //restart the PWM module
    DRV_TMR0_Stop();
    DRV_TMR0_Initialize();
    DRV_OC0_SetPulseWidth(time_value);
    DRV_TMR0_Start();
}

//moves the servo ahead by one degree
//if the servo is at its maximum position, it will move back to the minimum position
void incrementServo(){
    if (sensorData.servo_angle == SERVOANGLE_MAX){
        sensorData.servo_angle = SERVOANGLE_MIN;
        setServoAngle(SERVOANGLE_MIN);
    }
    else{
        sensorData.servo_angle++;
        setServoAngle(sensorData.servo_angle);
    }
}

void startServoMovementTimer(){
    if( xTimerStart( sensorData.servoMovementTimer_SA, 0 ) != pdPASS ) {
        dbgOutputVal(SENSOR_TIMERINIT_FAIL);
        stopAll();
    }
}

/*******************************************************************************
 End of File
 */
