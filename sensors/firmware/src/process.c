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

PROCESS_DATA processData;

// Pack and transfer message to data queue in Send task
void processSensorData(char* sensorData) {
    
    // PROCESSING ALGORITHMS HERE
    
}

void sendClearMessage(){
    //Convert sensor data to message format character array
    char data[4];
    data[0] = MSG_START;
    data[1] = TYPE_SENSOR_CLEARALL;
    data[2] = processData.clearCount;
    data[3] = MSG_STOP;
    putMsgOnSendQueue(data);  // Transfer message to Send task queue]
    processData.clearCount++;
}

void sendDisplayMessage(){
    //Convert sensor data to message format character array
    char data[4];
    data[0] = MSG_START;
    data[1] = TYPE_SENSOR_DISPLAYSINGLEMAP;
    data[2] = processData.displayCount;
    data[3] = MSG_STOP;
    putMsgOnSendQueue(data);  // Transfer message to Send task queue
    processData.displayCount++;
}

void sendDisplayFieldMessage(){
    //Convert sensor data to message format character array
    char data[4];
    data[0] = MSG_START;
    data[1] = TYPE_SENSOR_DISPLAYFIELD;
    data[2] = processData.displayFieldCount;
    data[3] = MSG_STOP;
    putMsgOnSendQueue(data);  // Transfer message to Send task queue
    processData.displayFieldCount++;
}

void sendEchoMessage(Obstacle o){
    char data[7];
    data[0] = MSG_START;             //Start byte
    data[1] = TYPE_SENSOR_ECHO;      //Type byte
    data[2] = processData.echoCount; //Count byte
    data[3] = o.type;                //data1;
    data[4] = o.midpoint_r;          //data1;
    data[5] = o.midpoint_theta;      //data2;
    data[6] = MSG_STOP;              //Stop byte
    putMsgOnSendQueue(data);
    processData.echoCount++;
}

// Convert sensor data to message format character array
void sendProcessedData(Obstacle o) {
    //char data[MSG_LENGTH];
    char data[8];
    data[0] = MSG_START;                    // Start byte
    //data[1] = TYPE_SENSORARRAY_PROCESSED; // Type byte
    data[1] = 'a'; //TYPE_BROOKE_APPENDPOLAR;
    data[2] = processData.appendCount;      // Count byte
    data[3] = o.midpoint_r;                 //data1;
    data[4] = o.midpoint_theta;             //data2;
    data[5] = o.midpoint_x;                 //data3;
    data[6] = o.midpoint_y;                 //data4;
    data[7] = MSG_STOP;                     // Stop byte
    putMsgOnSendQueue(data);  // Transfer message to Send task queue
    processData.appendCount++;
}

void sendMapData(Obstacle o){
    char data[8];
    data[0] = MSG_START;                    // Start byte
    data[1] = TYPE_SENSOR_APPENDMAP;
    data[2] = processData.mapCount;      // Count byte
    data[3] = o.midpoint_r;                 //data1;
    data[4] = o.midpoint_theta;             //data2;
    data[5] = o.midpoint_x;                 //data3;
    data[6] = o.midpoint_y;                 //data4;
    data[7] = MSG_STOP;                     // Stop byte
    
    if (o.midpoint_r == 46)
        data[3] = 47;
    if (o.midpoint_theta == 46)
        data[4] = 47;
    if (o.midpoint_x == 46)
        data[5] = 47;
    if (o.midpoint_y == 46)
        data[6] = 47;
    
    putMsgOnSendQueue(data);  // Transfer message to Send task queue
    processData.mapCount++;
}

void sendLinesData(Obstacle o){
    char data[8];
    data[0] = MSG_START;                    // Start byte
    data[1] = TYPE_SENSOR_APPENDLINES;
    data[2] = processData.linesCount;      // Count byte
    data[3] = o.midpoint_r;                 //data1;
    data[4] = o.midpoint_theta;             //data2;
    data[5] = o.midpoint_x;                 //data3;
    data[6] = o.midpoint_y;                 //data4;
    data[7] = MSG_STOP;                     // Stop byte
    
    //correct for collisions with MSG_STOP
    if (o.midpoint_r == 46)
        data[3] = 47;
    if (o.midpoint_theta == 46)
        data[4] = 47;
    if (o.midpoint_x == 46)
        data[5] = 47;
    if (o.midpoint_y == 46)
        data[6] = 47;
    
    putMsgOnSendQueue(data);  // Transfer message to Send task queue
    processData.linesCount++;
}
void sendTargetsData(Obstacle o){
    char data[10];
    data[0] = MSG_START;                    // Start byte
    data[1] = TYPE_SENSOR_APPENDTARGETS;
    data[2] = processData.targetsCount;      // Count byte
    data[3] = o.midpoint_r;                 //data1;
    data[4] = o.midpoint_theta;             //data2;
    data[5] = o.midpoint_x;                 //data3;
    data[6] = o.midpoint_y;                 //data4;
    data[7] = o.standard_deviation;         //data5;
    data[8] = o.max_deviation;              //data6;
    data[9] = MSG_STOP;                     // Stop byte
    
    //correct for collisions with MSG_STOP
    if (o.midpoint_r == 46)
        data[3] = 47;
    if (o.midpoint_theta == 46)
        data[4] = 47;
    if (o.midpoint_x == 46)
        data[5] = 47;
    if (o.midpoint_y == 46)
        data[6] = 47;
    
    putMsgOnSendQueue(data);  // Transfer message to Send task queue
    processData.targetsCount++;
}

void sendRequestForUpdateToProcessTask(){
    Obstacle o;
    o.midpoint_x = 0;
    o.midpoint_y = 0;
    o.midpoint_theta = 0;
    o.midpoint_r = 0;
    o.type = OBSTACLE_TYPE_UPDATEREQUESTED;
    putDataOnProcessQ(&o);    
}

// Remove oldest data on full local process queue
// Returns 1 if successful, 0 otherwise
uint8_t removeProcessQueueData() {
    char readdata[MSG_LENGTH];

    if (xQueueReceive(processData.processQ_SA, &readdata, portMAX_DELAY))
    {
        return 0x1;
    } 
    
    return 0x0;
}

// Check local process queue for vacancies. If none, remove oldest data.
// After, add the parameter char* message to local process queue. 
void putDataOnProcessQ(char* data) {
    if (processData.processQ_SA != 0) {
        
        // Check for full queue. If no spaces available, call to remove oldest data.
        if (uxQueueSpacesAvailable( processData.processQ_SA ) == 0) {
            // If message is not removed from queue, return and signal error
            if (removeProcessQueueData() == 0) {
                dbgOutputVal(SENSOR_FULLQUEUE);
                stopAll();
                return;
            }
        }
        
        // Send to queue, with at least one vacancy guaranteed for this data
        if( xQueueSend( processData.processQ_SA, (void*) data, portMAX_DELAY) != pdPASS )
        {
            dbgOutputVal(SENSOR_SENDTOPROCESSQ_FAIL);
        }
    }
}

Obstacle getThreeCenterPoint(Obstacle a, Obstacle b, Obstacle c){
    Obstacle o;
    int x = a.midpoint_x + b.midpoint_x + c.midpoint_x;
    int y = a.midpoint_y + b.midpoint_y + c.midpoint_y;
    x = x/3;
    y = y/3;
    o.type = OBSTACLE_TYPE_PROCESSED;
    o.midpoint_r = 0;
    o.midpoint_theta = 0;
    o.midpoint_x = x;
    o.midpoint_y = y;
    return o;
}

Obstacle getTwoCenterPoint(Obstacle a, Obstacle b){
    Obstacle o;
    int x = a.midpoint_x + b.midpoint_x;
    int y = a.midpoint_y + b.midpoint_y;
    x = x/2;
    y = y/2;
    o.type = OBSTACLE_TYPE_PROCESSED;
    o.midpoint_r = 0;
    o.midpoint_theta = 0;
    o.midpoint_x = x;
    o.midpoint_y = y;
    return o;
}

int getDistanceSquared(Obstacle a, Obstacle b){
    int deltax = a.midpoint_x - b.midpoint_x;
    int deltay = a.midpoint_y - b.midpoint_y;
    return ( (deltax*deltax) + (deltay*deltay) );
}

/*
Obstacle getCenterPoint(Obstacle* inputArray, int length){
    Obstacle o;
    int i=0, x=0, y=0;
    for (i=0; i<length; i++){
        x += inputArray[i].midpoint_x;
        y += inputArray[i].midpoint_y;
    }
    o.type = OBSTACLE_TYPE_PROCESSED;
    o.start_theta = 0;
    o.end_theta = 0;
    o.length_of_arc = 0;
    o.slope = 0;
    o.midpoint_r = 0;
    o.midpoint_theta = 0;
    o.midpoint_x = (x/length);
    o.midpoint_y = (y/length);
    
    return o;
}
 * */

int processObstacleArray(Obstacle* inputArray, int inputArrayLength, Obstacle* outputArray, int start){
    Obstacle o;
    int outputArrayIndex = start;
    int i=0, j=0, k=0, l=0;
    int distance1, distance2, distance3;
    
    //for all combinations of 3 unique obstacles...
    for (i=0; i<inputArrayLength; i++){
        for (j=0; j<inputArrayLength; j++){
            for (k=0; k<inputArrayLength; k++){
                if ( (i!=j) && (i!=k) && (j!=k) ){
                    if (inputArray[i].type != OBSTACLE_TYPE_PROCESSED
                            && inputArray[j].type != OBSTACLE_TYPE_PROCESSED
                            && inputArray[k].type != OBSTACLE_TYPE_PROCESSED){
                        
                        o = getThreeCenterPoint(inputArray[i], inputArray[j], inputArray[k]);
                        distance1 = getDistanceSquared(o, inputArray[i]);
                        distance2 = getDistanceSquared(o, inputArray[j]);
                        distance3 = getDistanceSquared(o, inputArray[k]);
                                                
                        if (distance1 < TARGET_MIN_RADIUS_SQUARED
                                && distance2 < TARGET_MIN_RADIUS_SQUARED
                                && distance3 < TARGET_MIN_RADIUS_SQUARED){
                            //then o is an obstacle!
                            o.standard_deviation = (int)((double)((distance1+distance2+distance3)/3)+0.5);
                            o.max_deviation = maxDeviationThree(distance1, distance2, distance3);
                            inputArray[i].type = OBSTACLE_TYPE_PROCESSED;
                            inputArray[j].type = OBSTACLE_TYPE_PROCESSED;
                            inputArray[k].type = OBSTACLE_TYPE_PROCESSED;
                            
                            
                            //if there are more than 3 obstacles around this centerpoint, mark those as visited too so we don't get repeated targets
                            for (l=0; l<inputArrayLength; l++){
                                if ( (inputArray[l].type != OBSTACLE_TYPE_PROCESSED) && (getDistanceSquared(o,inputArray[l]) < TARGET_MIN_RADIUS_SQUARED) )
                                    inputArray[l].type = OBSTACLE_TYPE_PROCESSED;
                            }
                            
                            
                            //sendTargetsData(o);
                            outputArray[outputArrayIndex] = o;
                            outputArrayIndex++;
                        }
                    }
                }
            }
        }
    }
    
    //for all combinations of 2 unique obstacles...
    for (i=0; i<inputArrayLength; i++){
        for (j=0; j<inputArrayLength; j++){
            if (i != j){
                if (inputArray[i].type != OBSTACLE_TYPE_PROCESSED && inputArray[j].type != OBSTACLE_TYPE_PROCESSED){
                        o = getTwoCenterPoint(inputArray[i], inputArray[j]);
                        distance1 = getDistanceSquared(o, inputArray[i]);
                        distance2 = getDistanceSquared(o, inputArray[j]);
                    if (distance1 < TARGET_MIN_RADIUS_SQUARED
                            && distance2 < TARGET_MIN_RADIUS_SQUARED){
                        inputArray[i].type = OBSTACLE_TYPE_PROCESSED;
                        inputArray[j].type = OBSTACLE_TYPE_PROCESSED;
                        o.standard_deviation = (int)((double)((distance1+distance2)/2)+0.5);
                        o.max_deviation = distance1;
                        if (distance1 > distance2) o.max_deviation = distance1;
                        else if (distance2 > distance1) o.max_deviation = distance2;
                        
                        //sendTargetsData(o);
                        outputArray[outputArrayIndex] = o;
                        outputArrayIndex++;

                    }
                }
            }
        }
    }
    return outputArrayIndex;
}

int maxDeviationThree(int a, int b, int c){
    if ((a>b) && (a>c)) return a;
    if ((b>a) && (b>c)) return b;
    if ((c>a) && (c>b)) return c;
    return 0;
}
                                    
                                    
void PROCESS_Initialize ( void )
{
    /* Place the App state machine in its initial state. */
    processData.state = PROCESS_STATE_INIT;
    processData.clearCount = 0;
    processData.displayCount = 0;
    processData.displayFieldCount = 0;
    processData.displayAveragesCount = 0;
    processData.appendCount = 0;
    processData.echoCount = 0;
    processData.mapCount = 0;
    processData.linesCount = 0;
    processData.targetsCount = 0;
    
    processData.map_index = 0;
    processData.obstacles_from_sensors_index = 0;
    processData.processed_obstacles_index = 0;
    processData.lr_obstacles_from_sensors_index = 0;
    processData.lr_processed_obstacles_index = 0;
    //processData.averaged_obstacles_index = 0;
    
    processData.iterations = 0;
    
    //Create a queue capable of holding 1000 characters (bytes))
    //processData.processQ_SA = xQueueCreate(1000, MSG_LENGTH+1 ); 
    processData.processQ_SA = xQueueCreate(1000, sizeof(Obstacle) ); 
    if( processData.processQ_SA == 0 ) {
        dbgOutputVal(PROCESS_QUEUE_FAIL);
        stopAll(); //ERROR
    }
    
    //Timer to send to internal task queue
    processData.process_Timer_SA = xTimerCreate(  
                     "ProcessTimer", //Just a text name
                     ( SA_PROC_TIMER_RATE / portTICK_PERIOD_MS ), //5000ms period
                     pdTRUE, //auto-reload when expires
                     (void *) 31, //a unique id
                     processTimerCallback ); //pointer to callback function
    
    //Start the timer
    if( processData.process_Timer_SA == NULL ) {
        dbgOutputVal(PROCESS_TIMERINIT_FAIL);
        stopAll();
    }
    else if( xTimerStart( processData.process_Timer_SA, 0 ) != pdPASS ) {
        dbgOutputVal(PROCESS_TIMERINIT_FAIL);
        stopAll();
    }
    
}



void PROCESS_Tasks ( void )
{
    //local variables
    int i=0, j=0; //local iterator
    int std_temp; //temporary holding place for standard deviation calculations
    
    /* Check the application's current state. */
    switch ( processData.state )
    {
        Obstacle qData;
        
        /* Application's initial state. */
        case PROCESS_STATE_INIT:
        {
            //initialize variables
            processData.map_index = 0;
            processData.obstacles_from_sensors_index = 0;
            processData.processed_obstacles_index = 0;
            processData.lr_obstacles_from_sensors_index = 0;
            processData.lr_processed_obstacles_index = 0;
     
            //send a message to clear all of the text files
            sendClearMessage();
            processData.state = PROCESS_STATE_GATHERDATA;
            break;
        }
        
        case PROCESS_STATE_GATHERDATA:
        {            
            // Receive from process queue until we receive a message from coordinator asking for an update
		    if (xQueueReceive(processData.processQ_SA, &qData, portMAX_DELAY))
            {
                //if we are in DEBUG_ISOLATESENSOR mode, immediately forward to the PI
                #ifdef SENSOR_DEBUG_ISOLATESENSOR
                    if (qData.type == OBSTACLE_TYPE_SERVOA 
                            || qData.type == OBSTACLE_TYPE_SERVOB
                            || qData.type == OBSTACLE_TYPE_SERVOC
                            || qData.type == OBSTACLE_TYPE_SERVOD
                            || qData.type == OBSTACLE_TYPE_SERVOA_LR
                            || qData.type == OBSTACLE_TYPE_SERVOB_LR
                            || qData.type == OBSTACLE_TYPE_SERVOC_LR
                            || qData.type == OBSTACLE_TYPE_SERVOD_LR){
                        sendEchoMessage(qData);
                    }
                    break;
                #endif
                
                //else, decide where to put the data for later analysis
                if (qData.type == OBSTACLE_TYPE_SERVOA 
                        || qData.type == OBSTACLE_TYPE_SERVOB
                        || qData.type == OBSTACLE_TYPE_SERVOC
                        || qData.type == OBSTACLE_TYPE_SERVOD){
                    processData.obstacles_from_sensors[processData.obstacles_from_sensors_index] = qData;
                    processData.obstacles_from_sensors_index++;
                }
                else if (qData.type == OBSTACLE_TYPE_MAP){
                    processData.map[processData.map_index] = qData;
                    processData.map_index++;
                }
                else if (qData.type == OBSTACLE_TYPE_SERVOA_LR 
                        || qData.type == OBSTACLE_TYPE_SERVOB_LR
                        || qData.type == OBSTACLE_TYPE_SERVOC_LR
                        || qData.type == OBSTACLE_TYPE_SERVOD_LR){
                    processData.lr_obstacles_from_sensors[processData.lr_obstacles_from_sensors_index] = qData;
                    processData.lr_obstacles_from_sensors_index++;
                }
                else if (qData.type == OBSTACLE_TYPE_UPDATEREQUESTED){
                    processData.state = PROCESS_STATE_PROCESS;
                    break;
                }
                //else ignore the message
            }
			break;
        }//end case PROCESS_STATE_GATHERDATA
        
        case PROCESS_STATE_PROCESS:
        {
            #ifdef SENSOR_DEBUG_SINGLEMAP
                //send all obstacles found by sensors themselves
                for (i=0; i<processData.obstacles_from_sensors_index; i++){
                    sendLinesData(processData.obstacles_from_sensors[i]);
                    //sendEchoMessage(processData.obstacles_from_sensors[i]); //for debugging
                }
                //also send the map of the field
                for (i=0; i<processData.map_index; i++){
                    sendMapData(processData.map[i]);
                }
                sendDisplayMessage();
            #endif

            //if we are in SENSOR_DEBUG_NETWORK mode, then forward targets and lines to the pi, and display them
            #ifdef SENSOR_DEBUG_NETWORK
                //populate the processData.processed_obstacles array
                processData.processed_obstacles_index = processObstacleArray(processData.obstacles_from_sensors, processData.obstacles_from_sensors_index, processData.processed_obstacles, 0);
            
                for (i=0; i<processData.processed_obstacles_index; i++){
                    sendTargetsData(processData.processed_obstacles[i]);
                }

                for (i=0; i<processData.obstacles_from_sensors_index; i++){
                    sendLinesData(processData.obstacles_from_sensors[i]);
                }
                sendDisplayFieldMessage();
            #endif

            #ifdef SENSOR_DEBUG_NETWORK_LR
                processData.lr_processed_obstacles_index = processObstacleArray(processData.lr_obstacles_from_sensors, processData.lr_obstacles_from_sensors_index, processData.lr_processed_obstacles, 0);
                for (i=0; i<processData.lr_processed_obstacles_index; i++){
                    sendTargetsData(processData.lr_processed_obstacles[i]);
                }
                for (i=0; i<processData.lr_obstacles_from_sensors_index; i++){
                    sendLinesData(processData.lr_obstacles_from_sensors[i]);
                }
                sendDisplayFieldMessage();
            #endif  

            //if we are in SENSOR_DEBUG_NETWORKAVERAGES mode, then do statistics on the array then forward to the pi
            #ifdef SENSOR_DEBUG_NETWORKAVERAGES
                //process obstacles_from_sensors[] into averaged_obstacles[]
                processData.averaged_obstacles_index = processObstacleArray(processData.obstacles_from_sensors, processData.obstacles_from_sensors_index, processData.averaged_obstacles, processData.averaged_obstacles_index);

                if (processData.iterations < 2) processData.iterations++;
                else{
                    //process averaged_obstacles[] into processed_obstacles[]
                    for (i=0; i<processData.averaged_obstacles_index; i++) processData.averaged_obstacles[i].type = OBSTACLE_TYPE_SERVOA;
                    processData.processed_obstacles_index = processObstacleArray(processData.averaged_obstacles, processData.averaged_obstacles_index, processData.processed_obstacles, 0);

                    //send all intermediate targets and averaged targets to the pi
                    for (i=0; i<processData.processed_obstacles_index; i++) sendTargetsData(processData.processed_obstacles[i]);
                    for (i=0; i<processData.averaged_obstacles_index; i++)  sendLinesData(processData.averaged_obstacles[i]);

                    processData.iterations = 0;
                    processData.averaged_obstacles_index = 0;
                    sendDisplayFieldMessage();
                }
            #endif
                
            processData.state = PROCESS_STATE_INIT;
            break;
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
