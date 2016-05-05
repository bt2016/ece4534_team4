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
				 ( 100 / portTICK_PERIOD_MS ), //period is 300ms
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

    //Initialize ADC Channel Scan for A0-A3
    //Manual Sample Start and TAD based Conversion Start
    DRV_ADC_Stop();
    DRV_ADC_Initialize();
    DRV_ADC_Open();
    
    //Start PWM timer. All of the servos use this same timer
    DRV_TMR0_Stop();
    DRV_TMR0_Initialize();
    DRV_TMR0_Start();
    
    //Initialize PWM
    DRV_OC0_Disable();
    DRV_OC0_Initialize();
    DRV_OC0_Enable();
    DRV_OC1_Disable();
    DRV_OC1_Initialize();
    DRV_OC1_Enable();
    DRV_OC2_Disable();
    DRV_OC2_Initialize();
    DRV_OC2_Enable();
    DRV_OC3_Disable();
    DRV_OC3_Initialize();
    DRV_OC3_Enable();
}

// Finite State Machine, runs forever.
void SENSOR_Tasks ( void )
{
    unsigned int qData;    //container for data coming into the Sensor Queue
    char data[MSG_LENGTH];
    int i=0, j=0;          //local iterators
    
	switch ( sensorData.state )
    {
        case SENSOR_STATE_INIT:
        {
            #ifdef SENSOR_DEBUG_ISOLATESENSOR
                setServoAngle(SENSOR_DEBUG_ISOLATESENSOR + SERVOANGLE_MIN);
            #else
                setServoAngle(SERVOANGLE_MIN); //set the servo to zero degrees
            #endif

            startServoMovementTimer(); //start the servoMovementTimer
			sensorData.state = SENSOR_STATE_TAKEREADINGS; //change state
            break;
        }
		
        case SENSOR_STATE_TAKEREADINGS:
        {
            //block until you receive a value from the ADC
            if (xQueueReceive(sensorData.sensorQ_SA, &qData, portMAX_DELAY))
                sensorData.ra[sensorData.servo_angle - SERVOANGLE_MIN] = qData;
            if (xQueueReceive(sensorData.sensorQ_SA, &qData, portMAX_DELAY))
                sensorData.rb[sensorData.servo_angle - SERVOANGLE_MIN] = qData;
            if (xQueueReceive(sensorData.sensorQ_SA, &qData, portMAX_DELAY))
                sensorData.rc[sensorData.servo_angle - SERVOANGLE_MIN] = qData;
            if (xQueueReceive(sensorData.sensorQ_SA, &qData, portMAX_DELAY))
                sensorData.rd[sensorData.servo_angle - SERVOANGLE_MIN] = qData;
            if (xQueueReceive(sensorData.sensorQ_SA, &qData, portMAX_DELAY))
                sensorData.ralr[sensorData.servo_angle - SERVOANGLE_MIN] = qData;
            if (xQueueReceive(sensorData.sensorQ_SA, &qData, portMAX_DELAY))
                sensorData.rblr[sensorData.servo_angle - SERVOANGLE_MIN] = qData;
            if (xQueueReceive(sensorData.sensorQ_SA, &qData, portMAX_DELAY))
                sensorData.rclr[sensorData.servo_angle - SERVOANGLE_MIN] = qData;
            if (xQueueReceive(sensorData.sensorQ_SA, &qData, portMAX_DELAY))
                sensorData.rdlr[sensorData.servo_angle - SERVOANGLE_MIN] = qData;

            //create and send objects with the values received above to Process Task
            #ifdef SENSOR_DEBUG_ISOLATESENSOR
                Obstacle o;
//                o.start_theta = 1;
//                o.end_theta = 1;
//                o.length_of_arc = 1;
                o.midpoint_x = 1;
                o.midpoint_y = 1;
//                o.slope = 1;
                o.midpoint_theta = sensorData.servo_angle - SERVOANGLE_MIN;

                o.type = OBSTACLE_TYPE_SERVOA;
                o.midpoint_r = sensorData.ra[sensorData.servo_angle - SERVOANGLE_MIN];
                putDataOnProcessQ(&o);
                o.type = OBSTACLE_TYPE_SERVOB;
                o.midpoint_r = sensorData.rb[sensorData.servo_angle - SERVOANGLE_MIN];
                putDataOnProcessQ(&o);
                o.type = OBSTACLE_TYPE_SERVOC;
                o.midpoint_r = sensorData.rc[sensorData.servo_angle - SERVOANGLE_MIN];
                putDataOnProcessQ(&o);
                o.type = OBSTACLE_TYPE_SERVOD;
                o.midpoint_r = sensorData.rd[sensorData.servo_angle - SERVOANGLE_MIN];
                putDataOnProcessQ(&o);
                
                o.type = OBSTACLE_TYPE_SERVOA_LR;
                o.midpoint_r = sensorData.ralr[sensorData.servo_angle - SERVOANGLE_MIN];
                putDataOnProcessQ(&o);
                o.type = OBSTACLE_TYPE_SERVOB_LR;
                o.midpoint_r = sensorData.rblr[sensorData.servo_angle - SERVOANGLE_MIN];
                putDataOnProcessQ(&o);
                o.type = OBSTACLE_TYPE_SERVOC_LR;
                o.midpoint_r = sensorData.rclr[sensorData.servo_angle - SERVOANGLE_MIN];
                putDataOnProcessQ(&o);
                o.type = OBSTACLE_TYPE_SERVOD_LR;
                o.midpoint_r = sensorData.rdlr[sensorData.servo_angle - SERVOANGLE_MIN];
                putDataOnProcessQ(&o);
                
                setServoAngle(SENSOR_DEBUG_ISOLATESENSOR + SERVOANGLE_MIN);
                startServoMovementTimer();
                break;
            #endif

            
            //check to see if we are finished panning
            if (sensorData.servo_angle < SERVOANGLE_MAX){                    
                incrementServo();
                startServoMovementTimer();
            }
            else{
                setServoAngle(SERVOANGLE_MIN); //set the servo to zero degrees
                sensorData.state = SENSOR_STATE_FINDOBSTACLES; //change state
            }
            break;
        }//end case SENSOR_STATE_TAKEREADINGS
            
        case SENSOR_STATE_FINDOBSTACLES:
        {
            Obstacle o;
            
            //correct for skewed sensor mounts in the LR (top-level) array
            for (i=0; i<(SERVO_DEGREES-7); i++){
                sensorData.ralr[SERVO_DEGREES-i] = sensorData.ralr[SERVO_DEGREES-i-7];
                sensorData.rdlr[i] = sensorData.rdlr[i+2];
                //sensorData.rdlr[SERVO_DEGREES-i] = sensorData.rdlr[SERVO_DEGREES-i-1];
            }
            
            //correct sensor A's and B's distance reading
            for (i=0; i<SERVO_DEGREES; i++){
                //sensorData.ra[i] = sensorData.ra[i]-5;
                //sensorData.rb[i] = sensorData.rb[i]+2;
                #ifdef SENSOR_DEBUG_DISABLE_SERVOA
                    sensorData.ra[i] = 0;
                    sensorData.ralr[i] = 0;
                #endif
                #ifdef SENSOR_DEBUG_DISABLE_SERVOB
                    sensorData.rb[i] = 0;
                    sensorData.rblr[i] = 0;
                #endif
                #ifdef SENSOR_DEBUG_DISABLE_SERVOC
                    sensorData.rc[i] = 0;
                    sensorData.rclr[i] = 0;
                #endif
                #ifdef SENSOR_DEBUG_DISABLE_SERVOD
                    sensorData.rd[i] = 0;
                    sensorData.rdlr[i] = 0;
                #endif
            }
            
            //if we are in SENSOR_DEBUG_SINGLEMAP mode, then also send the full array
            //from the sensor that was specified in the #define
            #ifdef SENSOR_DEBUG_SINGLEMAP
                int* SINGLEMAP_array;
                o.type = OBSTACLE_TYPE_MAP;
                //o.start_theta = 0;
                //o.end_theta = 0;
                //o.length_of_arc = 0;
                o.midpoint_x = 0;
                o.midpoint_y = 0;
                //o.slope = 0;
                
                if (SENSOR_DEBUG_SINGLEMAP == OBSTACLE_TYPE_SERVOA)      SINGLEMAP_array = sensorData.ra;
                else if (SENSOR_DEBUG_SINGLEMAP == OBSTACLE_TYPE_SERVOB) SINGLEMAP_array = sensorData.rb;
                else if (SENSOR_DEBUG_SINGLEMAP == OBSTACLE_TYPE_SERVOC) SINGLEMAP_array = sensorData.rc;
                else if (SENSOR_DEBUG_SINGLEMAP == OBSTACLE_TYPE_SERVOD) SINGLEMAP_array = sensorData.rd;
                else if (SENSOR_DEBUG_SINGLEMAP == OBSTACLE_TYPE_SERVOA_LR) SINGLEMAP_array = sensorData.ralr;
                else if (SENSOR_DEBUG_SINGLEMAP == OBSTACLE_TYPE_SERVOB_LR) SINGLEMAP_array = sensorData.rblr;
                else if (SENSOR_DEBUG_SINGLEMAP == OBSTACLE_TYPE_SERVOC_LR) SINGLEMAP_array = sensorData.rclr;
                else if (SENSOR_DEBUG_SINGLEMAP == OBSTACLE_TYPE_SERVOD_LR) SINGLEMAP_array = sensorData.rdlr;
                    
                findObstacles(SINGLEMAP_array, OBSTACLE_TYPE_SERVOA);
                for (i=0; i<SERVO_DEGREES; i++){
                    o.midpoint_theta = i;
                    o.midpoint_r = SINGLEMAP_array[i];
                    putDataOnProcessQ(&o);
                }
            #else
                //find and send obstacles from all raw data arrays ra, rb, rc, and rd
                findObstacles(sensorData.ra, OBSTACLE_TYPE_SERVOA);
                findObstacles(sensorData.rb, OBSTACLE_TYPE_SERVOB);
                findObstacles(sensorData.rc, OBSTACLE_TYPE_SERVOC);
                findObstacles(sensorData.rd, OBSTACLE_TYPE_SERVOD);
                findObstacles(sensorData.ralr, OBSTACLE_TYPE_SERVOA_LR);
                findObstacles(sensorData.rblr, OBSTACLE_TYPE_SERVOB_LR);
                findObstacles(sensorData.rclr, OBSTACLE_TYPE_SERVOC_LR);
                findObstacles(sensorData.rdlr, OBSTACLE_TYPE_SERVOD_LR);
                
            #endif
            
            //tell the Process task that we are finished sending data
            o.type = OBSTACLE_TYPE_END;
            putDataOnProcessQ(&o);
            
            sensorData.state = SENSOR_STATE_INIT;
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


//parses through an array from the sensor and finds obstacles
//immediately forwards these obstacles to the Process Task
int findObstacles(int* r, int sender){
    int length = 0;
    int startofline = 0;
    int endofline = 0;
    int middleofline = 0;
    int i=0;
    int runningsum = 0;

    while (i<SERVO_DEGREES-1){
        //if consecutive points exist, measure how long the line is
        if (abs(r[i]-r[i+1]) < LINE_MINDELTA_CM){
            startofline = i;
            middleofline = i;
            endofline = i+1;
            while (abs(r[middleofline]-r[endofline]) < LINE_MINDELTA_CM){
                middleofline++;
                endofline++;
                if (endofline == SERVO_DEGREES) break;
            }
            length = middleofline - startofline;
            
            //found a line of adequate length!
            if (length > LINE_MINLENGTH_CM){
                Obstacle o;
                o.type = sender;
                o.midpoint_theta = (startofline + middleofline)/2;
                o.midpoint_r = (r[startofline] + r[middleofline])/2;
                
                //the distance is the weighted average of all the points in the line
                //for (i=startofline; i<middleofline; i++) runningsum += r[i];
                //o.midpoint_r = runningsum/length;
                    

                //account for each sensor panning through a different quadrant
                if ((sender == OBSTACLE_TYPE_SERVOA) || (sender == OBSTACLE_TYPE_SERVOA_LR)){
                    o.midpoint_x = (int)(((double)o.midpoint_r*mycos(o.midpoint_theta))+0.5);
                    o.midpoint_y = (int)(((double)o.midpoint_r*mysin(o.midpoint_theta))+0.5);
                }
                else if ((sender == OBSTACLE_TYPE_SERVOB) || (sender == OBSTACLE_TYPE_SERVOB_LR)){
                    o.midpoint_theta += 90;
                    o.midpoint_x = (int)(((double)o.midpoint_r*mycos(o.midpoint_theta))+0.5);
                    o.midpoint_y = (int)(((double)o.midpoint_r*mysin(o.midpoint_theta))+0.5);
                    o.midpoint_x += 90;
                    o.midpoint_y += 0;
                }
                else if ((sender == OBSTACLE_TYPE_SERVOC) || (sender == OBSTACLE_TYPE_SERVOC_LR)){
                    o.midpoint_theta += 180;
                    o.midpoint_x = (int)(((double)o.midpoint_r*mycos(o.midpoint_theta))+0.5);
                    o.midpoint_y = (int)(((double)o.midpoint_r*mysin(o.midpoint_theta))+0.5);
                    o.midpoint_x += 90;
                    o.midpoint_y += 90;
                }
                else if ((sender == OBSTACLE_TYPE_SERVOD) || (sender == OBSTACLE_TYPE_SERVOD_LR)){
                    o.midpoint_theta += 270;
                    o.midpoint_x = (int)(((double)o.midpoint_r*mycos(o.midpoint_theta))+0.5);
                    o.midpoint_y = (int)(((double)o.midpoint_r*mysin(o.midpoint_theta))+0.5);
                    o.midpoint_x += 0;
                    o.midpoint_y += 90;
                }
                
                //send to Process Task unless it is a trivial line where r=0 constantly
                //if (o.midpoint_r != 0) putDataOnProcessQ(&o);
                if (o.midpoint_r > SERVO_MINRANGE_CM) putDataOnProcessQ(&o);
                i=endofline;
            }

            //found a line but it wasn't long enough. skip to the end of the line
            else i = endofline;
        }
        //else just ignore that point and move on
        else i++;
    }
}//end findObstacles()

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

//moves all servos to the specified angle, given in servo PWM ticks
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
    DRV_OC1_SetPulseWidth(time_value);
    DRV_OC2_SetPulseWidth(time_value);
    DRV_OC3_SetPulseWidth(time_value);
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

void startAdcReadingTimer(){
    if( xTimerStart( sensorData.takeAdcReadingTimer_SA, 0 ) != pdPASS ) {
        dbgOutputVal(SENSOR_TIMERINIT_FAIL);
        stopAll();
    }
}

void stopAdcReadingTimer(){
    if( xTimerStop( sensorData.takeAdcReadingTimer_SA, 0 ) != pdPASS ) {
        dbgOutputVal(SENSOR_TIMERINIT_FAIL);
        stopAll();
    }
}

int abs(int input){
    if (input<0) return (input*(-1));
    else return input;
}

/*******************************************************************************
 End of File
 */
