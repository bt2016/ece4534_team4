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
#include "proj_definitions.h"

// *****************************************************************************
// *****************************************************************************
// Section: Global Data Definitions
// *****************************************************************************
// *****************************************************************************

// *****************************************************************************
/* Application Data

  Summary:
    Holds application data

  Description:
    This structure holds the application's data.

  Remarks:
    This structure should be initialized by the APP_Initialize function.
    
    Application strings and buffers are be defined outside this structure.
*/

PROCESS_DATA processData;







// 
// Remove oldest data on full local data queue
// Returns 1 if successful, 0 otherwise
uint8_t removeProcessQueueData() {
    char readdata[MSG_LENGTH];
 
    if (xQueueReceive(processData.processQ_CD, &readdata, portMAX_DELAY))
    {
        return 0x1;
    } 
    
    return 0x0;
}


// Check queue holding the data from other component tasks
// THIS IS BLOCKING
void receiveFromProcessQ()
{    
   char read[MSG_LENGTH]; 
   char newData = 0;

   // Read the top message of the queue
    if (xQueueReceive(processData.processQ_CD, &read, portMAX_DELAY))
    {
        newData = 1;
    } 
   
   
   char* k = read; 
   int i = 0;
   for(i=0; i<MSG_LENGTH; i++){
       
      processData.new_message[i] = *k;
      ++k;
   }
   

  
}



// Place data passed from other tasks into send queue
void putMsgOnProcessQueue(char* data) {
        
    if (processData.processQ_CD != 0) {
        
        // Check for full queue. If no spaces available, call to remove oldest data.
        if (uxQueueSpacesAvailable( processData.processQ_CD ) == 0) {
            // If message is not removed from queue, return and signal error
            if (removeProcessQueueData() == 0) {                
                stopAll();
                return;
            }
        }
        
        // Send to queue, with at least one vacancy guaranteed for this data
        if( xQueueSend( processData.processQ_CD, (void*) data, portMAX_DELAY) != pdPASS )
        {
            dbgOutputVal(SEND_QUEUE_FAIL);
        }
    }
}

//Call this whenever we get a message to set the tokens
void processTokenMsg(){
    char msgToSend[MSG_LENGTH];
    msgToSend[0] = MSG_START;
    msgToSend[MSG_LENGTH-1] = MSG_STOP;
    
    //Success! Set the number of tokens
    processData.number_of_tokens = processData.new_message[8];
    //Send ACK
    msgToSend[1] = TYPEC_ACK_TOKEN;
    putMsgOnSendQueue(msgToSend);

    processData.state = PROCESS_STATE_WAIT_FOR_CALIBRATION_START;

}


//Tell the pi to send us updated map data
void processUpdateMap(){
            
    
    //Tell the pi to send us updated map data
    processData.executeSensorTimer = TRUE;
                
    while(processData.new_message[1] != (char)ACK_FROM_SENSOR){
        receiveFromProcessQ();                    
    }

    processData.executeSensorTimer = FALSE;
    
    processData.number_of_sensor_pans = 2; //Reset this back to 2 incase it was heightened elsewhere

    
}

//Similar to Java's atan2 function. Note that y is passed in before x. 
//Returns a number from 0 to 360 in degrees. Returns -1 if x=0, y=0
int atan2_custom(double y, double x){
    
    double result;
    
    double YoverX; //Dont compute this yet
    
    if(x > 0){
        YoverX = y/x;
        result = atan(YoverX);
    }
    else if(x < 0){
        if(y >= 0){
            YoverX = y/x;
            result = atan(YoverX) + PI;
        }
        else if(y < 0){
            YoverX = y/x;
            result = atan(YoverX) - PI;
        }
    }
    else if(x == 0){
        if(y > 0){
            result = (PI / 2);
        }
        else if(y < 0){
            result = -(PI / 2);
        }
        else if(y == 0){
            return -1;
        }
    }
    
    if(result < 0){
        result = (result + (2*PI));
    }
        
    //Convert to degrees
    double val = 180 / PI;
    result = (result * val);
    
    return (int)result; 
}

void processMoveRoverTurn(int direction){
    
    
    
    if(processData.turn_rover_called == TRUE && processData.state != PROCESS_STATE_DIVERT){
        return; 
    }
    
    //TEST:
   // if(processData.rover_located == FALSE){
   //     return;       
   // }
    
    processData.turn_rover_called = TRUE;
    char msgToSend[MSG_LENGTH];
    msgToSend[0] = MSG_START;
    msgToSend[MSG_LENGTH-1] = MSG_STOP;
    msgToSend[1] = TYPEC_ROTATE;
    
        
    int degrees_to_rotate = 0;
    
    if(direction == LR_UP){
        processData.lr_last_direction = LR_UP; 
        
        degrees_to_rotate = 90 - processData.direction_degrees; 
               
    }
    else if(direction == LR_DOWN){
        processData.lr_last_direction = LR_DOWN; 
        degrees_to_rotate = 270 - processData.direction_degrees;
    }
    else if(direction == LR_LEFT){
        processData.lr_last_direction = LR_LEFT; 
        degrees_to_rotate = 180 - processData.direction_degrees;
    }
    else if(direction == LR_RIGHT){
        processData.lr_last_direction = LR_RIGHT; 
        degrees_to_rotate = 0 - processData.direction_degrees;
        
    }
    
    //Normally we won't need to worry about changing this, but keep track for diversion state
        //This breaks things for some reason??
    if(processData.state == PROCESS_STATE_DIVERT){
        processData.direction_degrees += degrees_to_rotate; 

        if(processData.direction_degrees < 0){
            processData.direction_degrees = 360 - abs(processData.direction_degrees);
        }
        if(processData.direction_degrees > 360){
            processData.direction_degrees = processData.direction_degrees - 360;
        }
     }
    
    
    
    if(degrees_to_rotate == 46){
        degrees_to_rotate = 47;
    }
    
    msgToSend[3] = 1;  //Rotate RIGHT
    if(degrees_to_rotate < 0){
        degrees_to_rotate = (-1 * degrees_to_rotate);       
    }
    else{
         msgToSend[3] = 0; //rotate LEFT
    }
            
    
    //Can't fit into the char sized buffer thingy 
    if(degrees_to_rotate < 220){
        msgToSend[4] = degrees_to_rotate;
        msgToSend[5] = 0;
    }
    else{
        msgToSend[4] = degrees_to_rotate - 200;
        msgToSend[5] = 200;
    }
    
    
    putMsgOnSendQueue(msgToSend);
    
}

void processFindRoverData(){
   
    int msg = 0;
    if(processData.rover_located == FALSE){
        
        
        int old_msg = 0;
        int rover_location_old_msg = processData.new_object_location; 
        for(old_msg=0; old_msg < processData.new_object_location; old_msg++){
            if(processData.object_locations[old_msg][5] == 'Y'){
                rover_location_old_msg = old_msg;
            }
        }
                
        //Needs to be revised to go over all data points as well as add in weights and last move amount
        for(msg=0; msg < processData.new_object_location_am; msg++){ 
             
            // if((processData.object_locations[msg][3] != processData.object_locations_am[msg][3]) || (processData.object_locations[msg][4] != processData.object_locations_am[msg][4])   ){
            if(processData.object_locations_am[msg][5] == 'Y'){ //Y
                
                    //Used to help in avoiding obstacles
                    processData.rover_located = TRUE;
                    
                    //This stops us from calling the turn function multiple times between readings
                    processData.turn_rover_called = FALSE;
                    
                    processData.rover_location_in_q = msg;
                                   
                    
                    processData.rover_x = processData.object_locations_am[msg][3];
                    processData.rover_y = processData.object_locations_am[msg][4];

                    processData.calc_rover_x = processData.object_locations_am[msg][3];
                    processData.calc_rover_y = processData.object_locations_am[msg][4];
                    
                    //Find the orientation
                    int old_x = processData.object_locations[msg][3]; //rover_location_old_msg
                    int old_y = processData.object_locations[msg][4]; //rover_location_old_msg

                    int new_x = processData.object_locations_am[msg][3]; 
                    int new_y = processData.object_locations_am[msg][4]; 

                   
                    //We need to calculate the rovers orientation below in degrees. 
                    //We can then use the degrees to determine direction (up, left, right, down)
                    //to be used in the rest of the program. 
                    double deltaX = new_x - old_x;
                    double deltaY = new_y - old_y; 
                                        
                    //If we get a -1 from this then direction is unknown
                    processData.direction_degrees = atan2_custom(deltaY, deltaX);
                    
                    int degrees = processData.direction_degrees;
                    
                    
                    //Rough degrees
                    if(degrees >= 45 && degrees <= 135){
                        processData.lr_last_direction = LR_UP;
                    }
                    else if(degrees >= 225 && degrees <= 315){
                        processData.lr_last_direction = LR_DOWN;
                    }
                    else if(degrees > 135 && degrees < 225){
                        processData.lr_last_direction = LR_LEFT;
                    }
                    else if((degrees < 45) || (degrees > 315)){
                        processData.lr_last_direction = LR_RIGHT;
                    }
                    else{
                        processData.lr_last_direction = LR_UNKNOWN;
                    }
                    
                    if(degrees == -1){
                        processData.lr_last_direction = LR_UNKNOWN;
                    }
                                     
                             

                }
                        
        }
        
           
    }
    
    
    //The rover was not found. Ask for another reading, and re-analyze the data.
    if(processData.rover_located == FALSE){
        
        processData.number_of_sensor_pans = 5; // Ask for more sensor pans 

        //grab new map data in AM (After move) queue
        processData.state = PROCESS_STATE_UPDATE_LOCATIONS;
        processData.next_state = PROCESS_STATE_LOCATE_ROVER_ANALYZE;
        processData.new_object_location_am = 0;
        processData.expecting_new_locations = TRUE; //We want locations to go to the new spot
        processUpdateMap();

    }
    
    
    //Protect against the "stuck against wall scenario
    if(processData.hit_origin == TRUE){
        if(processData.rover_x > 87){
            processMoveRoverTurn(LR_DOWN);
        }
        else if(processData.rover_x < 4){
            processMoveRoverTurn(LR_UP);
        }
        else if(processData.rover_y > 87){
            processMoveRoverTurn(LR_RIGHT);
        }
        else if(processData.rover_y < 4){
            processMoveRoverTurn(LR_LEFT);
        }
    }
    
}

int distanceToMove(int start, int end){
    return abs(start - end);
}

//Returns the max of two numbers 
int maximum(int a, int b){
    
    if(a > b){
        return a;
    }
    else{
        return b;
    }
}

//Returns distance between two points
double distanceBetweenPoints(int x1, int y1, int x2, int y2){
    double x_difference = x2 - x1; 
    double y_difference = y2 - y1; 
    
    double xsqrd = abs(x_difference * x_difference);
    double ysqrd = abs(y_difference * y_difference);
    double combined = xsqrd + ysqrd;
    
    processData.last_distance_calculated = sqrt(combined);
    double ans = processData.last_distance_calculated;
    return ans;
    
}

//Make sure the rover direction has been set before calling this function,
//And that the after move queues are populated with values. 
//This function will determine the safe moving distance before contact with an object
int distanceToObstacle(int distance_wanted){
    //Figure out the direction, go from there
    if(processData.lr_last_direction == LR_UP){
        
        //These two values will change
        int theoretical_x = processData.rover_x;
        int theoretical_y = processData.rover_y;        
        
        int move_increment = 0; //1st for loop iterator
        int max_move = distance_wanted; //return value
        
        //Go through the positions and figure out if there will be a collision 
        for(move_increment=0; move_increment < distance_wanted; move_increment++){
            
            //We are going up, so only y is changing 
            theoretical_y = processData.rover_y + move_increment;
            
            //Loop through obstacles and use cartesian distance formula to determine collisions
            int msg = 0;
            for(msg=0; msg < processData.new_object_location_am; msg++){ 

                int obs_x = processData.object_locations_am[msg][3];
                int obs_y = processData.object_locations_am[msg][4];
                
                //DEBUG
                processData.obs_x = obs_x;
                processData.obs_y = obs_y;
                
                double distance = distanceBetweenPoints(theoretical_x, theoretical_y, obs_x, obs_y);
                
                //Disregard the rover itself as an obstacle
                if((msg != processData.rover_location_in_q) && (distance > 2)){
                    
                    
                    //Make sure the obstacle is actually in front of us:
                    int obs_in_front = TRUE; 
                    
                    if(processData.lr_last_direction == LR_UP){
                        if(obs_y < processData.rover_y){
                            obs_in_front = FALSE;
                        }
                    }
                    else if(processData.lr_last_direction == LR_DOWN){
                        if(obs_y > processData.rover_y){
                            obs_in_front = FALSE;
                        }
                    }
                    else if(processData.lr_last_direction == LR_LEFT){
                        if(obs_x > processData.rover_x){
                            obs_in_front = FALSE;
                        }
                    }
                    else if(processData.lr_last_direction == LR_RIGHT){
                        if(obs_x < processData.rover_x){
                            obs_in_front = FALSE;
                        }
                    }
                    
                    //TODO: Add in the buffer zone here!!
                    if((distance < ROVER_WIDTH + BUFFER_ZONE) && (processData.need_divert == FALSE) && (obs_in_front == TRUE)){
                        //Obstacle ahead, divert after moving the max amount 
                        processData.need_divert = TRUE;                        
                        max_move = move_increment;
                        processData.max_move_amount_last_calculated = max_move;
                        return maximum(max_move, 1);
                                
                    }
                }
            }           
        }
        processData.max_move_amount_last_calculated = max_move;
        return maximum(max_move, 1);
        
    }
    else if(processData.lr_last_direction == LR_DOWN){
      
                
        //These two values will change
        int theoretical_x = processData.rover_x;
        int theoretical_y = processData.rover_y;        
        
        int move_increment = 0; //1st for loop iterator
        int max_move = distance_wanted; //return value
        
        //Go through the positions and figure out if there will be a collision 
        for(move_increment=0; move_increment < distance_wanted; move_increment++){
            
            //We are going up, so only y is changing 
            theoretical_y = processData.rover_y - move_increment;
            
            //Loop through obstacles and use cartesian distance formula to determine collisions
            int msg = 0;
            for(msg=0; msg < processData.new_object_location_am; msg++){ 

                int obs_x = processData.object_locations_am[msg][3];
                int obs_y = processData.object_locations_am[msg][4];
                
                //DEBUG
                processData.obs_x = obs_x;
                processData.obs_y = obs_y;
                
                double distance = distanceBetweenPoints(theoretical_x, theoretical_y, obs_x, obs_y);
                
                //Disregard the rover itself as an obstacle
                if((msg != processData.rover_location_in_q) && (distance > 2)){
                    //TODO: Add in the buffer zone here!!
                    if((distance < ROVER_WIDTH + BUFFER_ZONE) && (processData.need_divert == FALSE)){
                        //Obstacle ahead, divert after moving the max amount 
                        processData.need_divert = TRUE;                        
                        max_move = maximum((move_increment - 1), 0);
                        processData.max_move_amount_last_calculated = max_move;
                        return maximum(max_move - ERROR_AMT, 0);
                                
                    }
                }
            }           
        }
        processData.max_move_amount_last_calculated = max_move;
        return max_move;
        
        
        
        
    }
    else if(processData.lr_last_direction == LR_RIGHT){
       
                
        //These two values will change
        int theoretical_x = processData.rover_x;
        int theoretical_y = processData.rover_y;        
        
        int move_increment = 0; //1st for loop iterator
        int max_move = distance_wanted; //return value
        
        //Go through the positions and figure out if there will be a collision 
        for(move_increment=0; move_increment < distance_wanted; move_increment++){
            
            //We are going right, so only x is changing 
            theoretical_x = processData.rover_x + move_increment;
            
            //Loop through obstacles and use cartesian distance formula to determine collisions
            int msg = 0;
            for(msg=0; msg < processData.new_object_location_am; msg++){ 

                int obs_x = processData.object_locations_am[msg][3];
                int obs_y = processData.object_locations_am[msg][4];
                
                //DEBUG
                processData.obs_x = obs_x;
                processData.obs_y = obs_y;
                
                double distance = distanceBetweenPoints(theoretical_x, theoretical_y, obs_x, obs_y);
                
                //Disregard the rover itself as an obstacle
                if((msg != processData.rover_location_in_q) && (distance > 2)){
                    //TODO: Add in the buffer zone here!!
                    if((distance < ROVER_WIDTH + BUFFER_ZONE) && (processData.need_divert == FALSE)){
                        //Obstacle ahead, divert after moving the max amount 
                        processData.need_divert = TRUE;                        
                        max_move = maximum((move_increment - 1), 0);
                        processData.max_move_amount_last_calculated = max_move;
                        return maximum(max_move - ERROR_AMT, 0);
                                
                    }
                }
            }           
        }
        processData.max_move_amount_last_calculated = max_move;
        return max_move;
        
        
        
    }
    else if(processData.lr_last_direction == LR_LEFT){
      
                
        //These two values will change
        int theoretical_x = processData.rover_x;
        int theoretical_y = processData.rover_y;        
        
        int move_increment = 0; //1st for loop iterator
        int max_move = distance_wanted; //return value
        
        //Go through the positions and figure out if there will be a collision 
        for(move_increment=0; move_increment < distance_wanted; move_increment++){
            
            //We are going up, so only y is changing 
            theoretical_x = processData.rover_x - move_increment;
            
            //Loop through obstacles and use cartesian distance formula to determine collisions
            int msg = 0;
            for(msg=0; msg < processData.new_object_location_am; msg++){ 

                int obs_x = processData.object_locations_am[msg][3];
                int obs_y = processData.object_locations_am[msg][4];
                
                //DEBUG
                processData.obs_x = obs_x;
                processData.obs_y = obs_y;
                
                double distance = distanceBetweenPoints(theoretical_x, theoretical_y, obs_x, obs_y);
                
                //Disregard the rover itself as an obstacle
                if((msg != processData.rover_location_in_q) && (distance > 2)){
                    //TODO: Add in the buffer zone here!!
                    if((distance < ROVER_WIDTH + BUFFER_ZONE) && (processData.need_divert == FALSE)){
                        //Obstacle ahead, divert after moving the max amount 
                        processData.need_divert = TRUE;                        
                        max_move = maximum((move_increment - 1), 0);
                        processData.max_move_amount_last_calculated = max_move;
                        return maximum(max_move - ERROR_AMT, 0);                             
                    }
                }
            }           
        }
        processData.max_move_amount_last_calculated = max_move;
        return max_move;       
    }      
    else{
        return 1;
    }
    
}

/*TODO:
 * Figure out if we need to divert before we move by "amount" 
 * Move as far as we can before a divert, if its needed and raise a flag
 * 
 */ 

void DIVERT_MoveRoverForward(int amt){
    char msgToSend[MSG_LENGTH];
    msgToSend[0] = MSG_START;
    msgToSend[MSG_LENGTH-1] = MSG_STOP;
    msgToSend[1] = TYPEC_MOVE_FORWARD;
    
    //Figure out the move amount we can move before hitting something       
    if(processData.lr_last_direction == LR_UP){
        processData.rover_y += amt;
        if(processData.rover_y > 90){
            processData.rover_y = 90;
        }
    }
    else if(processData.lr_last_direction == LR_DOWN){
        processData.rover_y += -amt;
        if(processData.rover_y < 1){
            processData.rover_y = 1;
        }
    }
    else if(processData.lr_last_direction == LR_RIGHT){
        processData.rover_x += amt;
        if(processData.rover_x > 90){
            processData.rover_x = 90;
        }
    }
    else if(processData.lr_last_direction == LR_LEFT){
        processData.rover_x += -amt;
        if(processData.rover_x < 1){
            processData.rover_x = 1;
        }
        
    }    
   
    msgToSend[3] = amt; 
    processData.last_move_amount = amt;
    putMsgOnSendQueue(msgToSend);
}

void processMoveRoverForward(int amount){
    char msgToSend[MSG_LENGTH];
    msgToSend[0] = MSG_START;
    msgToSend[MSG_LENGTH-1] = MSG_STOP;
    msgToSend[1] = TYPEC_MOVE_FORWARD;
    
    //Only allow the rover to move forward by this much
    if(amount > MAX_MOVE){
        amount = MAX_MOVE;
    }
    
    //Figure out the move amount we can move before hitting something
    if(processData.need_divert == FALSE){
        
        if(processData.hit_origin == TRUE){
            amount = distanceToObstacle(amount);
            if(amount == 0){
                amount = 1; // we have to move.
            }
        }  
        
        if(processData.lr_last_direction == LR_UP){
            processData.calc_rover_y += amount;
        }
        else if(processData.lr_last_direction == LR_DOWN){
            processData.calc_rover_y += -amount;
            if(processData.calc_rover_y < 1){
                processData.calc_rover_y = 1;
            }
        }
        else if(processData.lr_last_direction == LR_RIGHT){
            processData.calc_rover_x += amount;
        }
        else if(processData.lr_last_direction == LR_LEFT){
            processData.calc_rover_x += -amount;
            if(processData.calc_rover_x < 1){
                processData.calc_rover_x = 1;
            }
        }    

        //Later on, constrain this to max amount
        msgToSend[3] = amount; 
        processData.last_move_amount = amount;
        putMsgOnSendQueue(msgToSend);
    }
    
}



//This function should be called AFTER a move is issued. It will take the data in the 
//After move map buffer and transfer it to the before move buffer. It will then free up the
//After move buffer
void processTransferMapDataBuffers(){
    int msg = 0;
    
    for(msg=0; msg < TRACKED_OBJECT_AMOUNT; msg++){ 
        strncpy(processData.object_locations[msg], processData.object_locations_am[msg], MSG_LENGTH);            
    }
    processData.new_object_location = processData.new_object_location_am; 
    //processData.new_object_location_am = 0; 
}

//Call this if you want to adjust map data
void processMapDataMsg(){
    
    processData.sensor_ack_number = processData.new_message[6]; 
    
    char msgToSend[MSG_LENGTH];
    msgToSend[0] = MSG_START;
    msgToSend[MSG_LENGTH-1] = MSG_STOP;
    msgToSend[1] = TYPEC_ACK_SENSOR_DATA;
    msgToSend[6] = processData.sensor_ack_number; 
    
    if(processData.expecting_new_locations == FALSE){
        if(processData.new_object_location <= TRACKED_OBJECT_AMOUNT-1){        
            strncpy(processData.object_locations[processData.sensor_ack_number], processData.new_message, MSG_LENGTH);
            processData.new_object_location = processData.sensor_ack_number+1;           
        }
        else{
            processData.new_object_location = 0;
            strncpy(processData.object_locations[processData.sensor_ack_number], processData.new_message, MSG_LENGTH);
            processData.new_object_location = processData.sensor_ack_number + 1;        
        }   
    }
    else{
        if(processData.new_object_location_am <= TRACKED_OBJECT_AMOUNT-1){        
            strncpy(processData.object_locations_am[processData.sensor_ack_number], processData.new_message, MSG_LENGTH);
            processData.new_object_location_am = processData.sensor_ack_number + 1;
        }
        else{
            processData.new_object_location_am = 0;
            strncpy(processData.object_locations_am[processData.sensor_ack_number], processData.new_message, MSG_LENGTH);
            processData.new_object_location_am = processData.sensor_ack_number + 1;        
        }
    }    
    
    putMsgOnSendQueue(msgToSend);
    
}

void processSendTokenFound(){
    
    
    
    if(processData.executeTimer == TRUE){
        
        
        //Send a message to the sensor telling it to pan and store values
        char msgToSend[MSG_LENGTH];
        msgToSend[0] = MSG_START;
        msgToSend[MSG_LENGTH-1] = MSG_STOP;

        //Tell the Pi we want the token number
        msgToSend[1] = TYPEC_LR_SENSOR_TO_FR; //type
        msgToSend[8] = processData.number_of_tokens;
        
        putMsgOnSendQueue(msgToSend);
                
    }
    
}

void processSendSensorRequest(){
    
    if(processData.executeSensorTimer == TRUE){
        
        //Send a message to the sensor telling it to pan and store values
        char msgToSend[MSG_LENGTH];
        msgToSend[0] = MSG_START;
        msgToSend[MSG_LENGTH-1] = MSG_STOP;

        
        msgToSend[1] = TYPE_SENSOR_MULTIPLEREQUESTED; //type
        msgToSend[3] = processData.number_of_sensor_pans; //D1, the number of pans 
        msgToSend[4] = processData.sensorStore; //D2, the "store" bit

        putMsgOnSendQueue(msgToSend);
                
        
        
    }
    
}


/*******************************************************************************
  Function:
    void PROCESS_Initialize ( void )

  Remarks:
    See prototype in process.h.
 */

void PROCESS_Initialize ( void )
{
    /* Place the App state machine in its initial state. */
    processData.state = PROCESS_STATE_INIT;
    processData.next_state = PROCESS_STATE_MESSAGES;
    
    processData.turn_rover_called = FALSE;
    
    processData.refresh_rate = 0;
    processData.need_divert = FALSE;    
    
    processData.calc_rover_x = 0;
    processData.calc_rover_y = 0;
    
    processData.number_of_sensor_pans = 2;
    
    processData.number_of_tokens = -1;
    processData.new_object_location = 0;
    processData.new_object_location_am = 0;
    
    processData.lr_last_direction = LR_UNKNOWN;
    processData.hit_origin = FALSE; 
    processData.rover_located = FALSE;
    processData.expecting_new_locations = FALSE;
    
    processData.finished = FALSE;
    
    processData.executeTimer = FALSE;
    processData.executeSensorTimer = FALSE;
    
    processData.sensorStore = 0; //this is a char, set to '1' to store data
    
    //CHANGE AS NEEDED
    processData.min_x = 8 + ROVER_WIDTH;
    processData.min_y = 8 + ROVER_WIDTH;
            
    
    processData.max_x = 82 - ROVER_WIDTH;
    processData.max_y = 82 - ROVER_WIDTH;     
    
    processData.next_move_amt = 30; //Set to 6 for initially finding rover
    
    processData.last_move_amount = 1;
    
    processData.sensor_ack_number = 0;
    
 
    /* TODO: Initialize your application's state machine and other
     * parameters.
     */
    
    //Create a Q capable of holding 250 messages
    processData.processQ_CD = xQueueCreate(250, MSG_LENGTH+1);
    if (processData.processQ_CD == 0) {
        stopAll();
    }
     
    
    
}


/******************************************************************************
  Function:
    void PROCESS_Tasks ( void )

  Remarks:
    See prototype in process.h.
 */
// Create a queue capable of holding 250 messages

    
void PROCESS_Tasks ( void )
{
    
    /* Check the application's current state. */
    switch ( processData.state )
    {
        /* Application's initial state. */
        case PROCESS_STATE_INIT:
        {
            
            processData.state = PROCESS_STATE_SET_TOKENS;
            break;
        }
        /* Request the number of tokens after connecting to RPI*/
        case PROCESS_STATE_SET_TOKENS:
        {
            char msgToSend[MSG_LENGTH];
            msgToSend[0] = MSG_START;
            msgToSend[MSG_LENGTH-1] = MSG_STOP;
            
            //Tell the Pi we want the token number
            msgToSend[1] = TYPEC_TOKEN_NUMBER;
            putMsgOnSendQueue(msgToSend);            
            
            //Wait for something to appear on the receive queue.
            receiveFromProcessQ();            
            
            //Check that the message is for setting tokens
            if(processData.new_message[1] == TYPEC_TOKEN_NUMBER){
                processTokenMsg();
            }
            
            //If it isn't, request from the Pi that we want token numbers
            else{
                msgToSend[1] = TYPEC_TOKEN_NUMBER;
                putMsgOnSendQueue(msgToSend); 
            }
          
            break;
        }
        
        case PROCESS_STATE_WAIT_FOR_CALIBRATION_START:
        {
            //Wait for the "GO" message to appear on receive queue
            receiveFromProcessQ();
            if(processData.new_message[1] == (char)TYPEC_CONTINUE){
            
                
                processData.number_of_sensor_pans = 3;
                processData.sensorStore = '1';
                processData.executeSensorTimer = TRUE;
                   
                
                while(processData.new_message[1] != (char)ACK_FROM_SENSOR){
                    receiveFromProcessQ();                    
                }
                
                processData.executeSensorTimer = FALSE;
                processData.sensorStore = 0; 
                
                
                //Off load the data and ACK it so we can move on. 
                 while(processData.new_message[1] != (char)TYPEC_END_MAP_DATA){
                    receiveFromProcessQ();
                    
                    char msgToSend[MSG_LENGTH];
                    msgToSend[0] = MSG_START;
                    msgToSend[MSG_LENGTH-1] = MSG_STOP;
                    msgToSend[6] = processData.new_message[6];

                    //Tell the Pi we want the token number
                    msgToSend[1] = TYPEC_ACK_SENSOR_DATA;
                    putMsgOnSendQueue(msgToSend); 

                }
                
                processData.number_of_sensor_pans = 2; 
                
                processData.state = PROCESS_STATE_SEND_FOLLOWER_GO;
            }
            else if(processData.new_message[1] == (char)TYPEC_SKIP){ 
                processData.state = PROCESS_STATE_SEND_FOLLOWER_GO;                
            }
            
            break;
        }
        
        case PROCESS_STATE_SEND_FOLLOWER_GO:
        {
            //Wait for the "GO" message to appear on receive queue
            receiveFromProcessQ();
            if(processData.new_message[1] == (char)TYPEC_CONTINUE){
            
                //Send a message to the sensor telling it to pan and store values
                char msgToSend[MSG_LENGTH];
                msgToSend[0] = MSG_START;
                msgToSend[MSG_LENGTH-1] = MSG_STOP;

                
                msgToSend[1] = TYPEC_GO; //type
                putMsgOnSendQueue(msgToSend);       
                
                processData.state = PROCESS_STATE_LOCATE_ROVER_MOVE; //PROCESS_STATE_MESSAGES;
            }
            else if(processData.new_message[1] == (char)TYPEC_SKIP){                
                processData.state = PROCESS_STATE_MESSAGES;                
            }
            
            break;
            
            
        }
        
        
        
        case PROCESS_STATE_MESSAGES:
        {
            //Wait for something to appear on the receive queue.
            receiveFromProcessQ();            
            
            //Check that the message is for setting tokens
            if(processData.new_message[1] == (char)TYPEC_TOKEN_NUMBER){
                processTokenMsg();
            }
            
            else if(processData.new_message[1] == (char)TYPEC_MAP_DATA){
               // processMapDataMsg();
            }
            
            else if(processData.new_message[1] == (char)TYPEC_TOKEN_FOUND){
                processData.number_of_tokens += -1;
                if(processData.number_of_tokens == 0){
                    processData.finished = TRUE; //Stop searching
                }
            }
            
            else if(processData.new_message[1] == (char)TYPEC_LR_MOVE_COMPLETE){
                //Check if we have finished searching the map
                /*
                if(processData.finished == FALSE){
                    //beginning
                    if(processData.rover_located == FALSE){
                        processData.state = PROCESS_STATE_LOCATE_ROVER_INIT;
                    }

                    //rest of moves
                    else{
                        if(processData.hit_origin == FALSE){
                            //Do nothing, this is for debugging
                        }
                        else{
                            processData.state = PROCESS_STATE_MOVE;
                        }                    
                    }                
                }
                 */
            }
            
            else if(processData.new_message[1] == (char)TYPEC_CLEAR_MAP){
               // processData.new_object_location = 0;
            }
            
            //This function is from the Pi, asking the coordinator to 
            //Send the Pi the same message back, to update the map list ON THE PI
            //The Pi will then update the map list on the coordinator
            else if(processData.new_message[1] == (char)TYPEC_UPDATE_MAP){
                //Just ACK messages 
                
                char msgToSend[MSG_LENGTH];
                msgToSend[0] = MSG_START;
                msgToSend[MSG_LENGTH-1] = MSG_STOP;
                msgToSend[1] = TYPEC_ACK_SENSOR_DATA;
                msgToSend[6] = processData.new_message[6]; 
                putMsgOnSendQueue(msgToSend);
                
            }
            
            //Transition into a command state
            else if(processData.new_message[1] == (char)TYPEC_MAKE_MOVE){
                
                //Check if we have finished searching the map
                if(processData.finished == FALSE){
                    //beginning
                    if(processData.rover_located == FALSE){
                        processData.state = PROCESS_STATE_LOCATE_ROVER_INIT;
                    }

                    //rest of moves
                    else{
                        if(processData.hit_origin == FALSE){
                            processData.state = PROCESS_STATE_MOVE_ORIGIN;
                        }
                        else{
                            processData.state = PROCESS_STATE_MOVE;
                        }                    
                    }                
                }
            }
           
         
            break;
        }
        
        case PROCESS_STATE_UPDATE_LOCATIONS:
        {
            //Turn a light on
            //LATASET = 1 << 3;
            
            //Wait for something to appear on the receive queue.
            receiveFromProcessQ();
            
            //Check that the message is for end of map data
            if(processData.new_message[1] == (char)TYPEC_END_MAP_DATA){
                processData.state = processData.next_state;
                
                char msgToSend[MSG_LENGTH];
                msgToSend[0] = MSG_START;
                msgToSend[MSG_LENGTH-1] = MSG_STOP;

                //Tell the Pi we want the token number
                msgToSend[1] = TYPEC_ACK_SENSOR_DATA;
                putMsgOnSendQueue(msgToSend); 
            }
            
            else if(processData.new_message[1] == (char)TYPEC_MAP_DATA){
                processMapDataMsg();
            }
            
            //Force move - not advised :< 
            else if(processData.new_message[1] == (char)TYPEC_MAKE_MOVE){
                processData.state = processData.next_state;
                //LATACLR = 1 << 3;
                //DO nothing, we need these queues to be updated perfectly
            }
            else if(processData.new_message[1] == (char)TYPEC_TOKEN_FOUND){
                processData.number_of_tokens += -1;
                if(processData.number_of_tokens == 0){
                    processData.finished = TRUE; //Stop searching
                }
            }
            break;
        }
        
        
        case PROCESS_STATE_LOCATE_ROVER_INIT:
        {
            //grab new map data in queue
            processData.state = PROCESS_STATE_UPDATE_LOCATIONS;
            processData.next_state = PROCESS_STATE_LOCATE_ROVER_MOVE;
            processData.new_object_location = 0;
            processData.expecting_new_locations = FALSE; //We want locations to go to the regular spot
            processUpdateMap();

             break;
        }
        
        case PROCESS_STATE_LOCATE_ROVER_MOVE:
        {
            //Tell the rover to move forward x amount, should be 6 initially 
            if(processData.rover_located == TRUE){
                processMoveRoverTurn(processData.lr_last_direction); //constantly call to make sure we actually facing proper direction
            }            
            processMoveRoverForward(processData.next_move_amt);
            //Wait for the rover to finish moving
            
            processData.rover_located = FALSE; //We no longer know where the rover is  
            LATASET = 1 << 3;
            
            
            while(processData.new_message[1] != (char)TYPEC_LR_MOVE_COMPLETE){
                receiveFromProcessQ(); //This ejects the "finish moving message from the Q. TODO change this later

                //Add token logic 
                if(processData.new_message[1] == (char)TYPEC_LR_SENSOR_TO_FR){
                
                    processData.executeTimer = TRUE;
                    
                    processSendTokenFound();
                    
                }
                else if(processData.new_message[1] == (char)TYPE_FR_ACK){
                    processData.executeTimer = FALSE; 
                }
                
                else if(processData.new_message[1] == (char)TYPE_FR_FOUND_TOKEN){
                    
                    processData.number_of_tokens = processData.new_message[8];
                    
                    if(processData.number_of_tokens <= 0){
                        processData.finished = TRUE; //Stop searching
                    }
                    
                    //Send an ACK to the FR that we received this message
                    //Send a message to the sensor telling it to pan and store values
                    char msgToSend[MSG_LENGTH];
                    msgToSend[0] = MSG_START;
                    msgToSend[MSG_LENGTH-1] = MSG_STOP;
                    msgToSend[1] = TYPEC_TOKEN_FOUND_ACK; //type
                    msgToSend[8] = processData.number_of_tokens;
                    putMsgOnSendQueue(msgToSend);       
                    
                }
                
            }
            
            
            LATACLR = 1 << 3;
            
             //grab new map data in AM (After move) queue
             processData.state = PROCESS_STATE_UPDATE_LOCATIONS;
             processData.next_state = PROCESS_STATE_LOCATE_ROVER_ANALYZE;
             processData.new_object_location_am = 0;
             processData.expecting_new_locations = TRUE; //We want locations to go to the new spot
             processUpdateMap();
             break;
        }
        
        case PROCESS_STATE_LOCATE_ROVER_ANALYZE:
        {            
            //Figure out what changed and the rovers direction
            processFindRoverData();
            
            
            //Transition to new state
            if(processData.hit_origin == TRUE){
                
                if(processData.finished == FALSE){
                    
                    if(processData.need_divert == FALSE){
                        processData.state = PROCESS_STATE_MOVE; //MESSAGES 
                    }
                    else{
                        processData.state = PROCESS_STATE_DIVERT;
                    }
                }
                else{
                    processData.state = PROCESS_STATE_MESSAGES;
                }
            }
            
            else{
                processData.state = PROCESS_STATE_MOVE_ORIGIN; 
            }
                                   
            break;  
        }
        
        //We are just starting out and want to get the rover to (0,0)
        case PROCESS_STATE_MOVE_ORIGIN:
        {   
            if(processData.rover_x > (processData.min_x + TOLERANCE)){ //The 3 is for tolerance 
                if(processData.lr_last_direction != LR_LEFT){
                    processMoveRoverTurn(LR_LEFT);
                } 
                
                int move_amt = distanceToMove(processData.rover_x, processData.min_x);

                //processMoveRoverForward(move_amt); //Keep going until we hit the left wall 
                
                //Transfer the newest map data into the old buffer since its now old data (after move)
                processTransferMapDataBuffers();
                processData.next_move_amt = move_amt; 
                processData.state = PROCESS_STATE_LOCATE_ROVER_MOVE;
                
            }           
            else{
                processData.hit_origin = TRUE;
                
                //Do this to set the rover in the correct direction to be taken over by the move commands
                processMoveRoverTurn(LR_UP);
                processData.lr_last_direction = LR_UP;
                processData.state = PROCESS_STATE_MESSAGES;
            }
            
            
            
            
            //TODO: ADD in the logic that checks for the refresh rate here
            //isDivertNeeded();
            
            
                      
       
            break;  
        }
        
        //We have already reached (1,1) and are searching the map for tokens
        case PROCESS_STATE_MOVE:
        {
            //Decide if we need to switch directions (obstacle or need to turn)
                       
            int decrement = ROVER_WIDTH + 11; //change as needed
            
            //Handle the "corners"
            // COMMENTED OUT : (processData.rover_y >= (processData.max_y - TOLERANCE)) &&
            if((processData.rover_x >= (processData.max_x - TOLERANCE)) &&  processData.lr_last_direction == LR_RIGHT){
                processMoveRoverTurn(LR_DOWN); //down
                processData.max_x += -decrement;  
            }
            //COMMENTED OUT : (processData.rover_x >= (processData.max_x - TOLERANCE)) &&
            else if((processData.rover_y <= (processData.min_y + TOLERANCE)) && processData.lr_last_direction == LR_DOWN){
                processMoveRoverTurn(LR_LEFT); //left
                processData.min_y += decrement;
            }
            //COMMENTED OUT: (processData.rover_x <= (processData.min_x + TOLERANCE)) &&
            else if((processData.rover_y >= (processData.max_y - TOLERANCE)) && processData.lr_last_direction == LR_UP){
                processMoveRoverTurn(LR_RIGHT); //right
                processData.max_y += -decrement;
            }
            //COMMENTED OUT : && (processData.rover_y <= (processData.min_y - TOLERANCE))
            else if((processData.rover_x <= (processData.min_x + TOLERANCE)) && processData.lr_last_direction == LR_LEFT){
                processMoveRoverTurn(LR_UP); //up
                processData.min_x += decrement;
            }
            
            //Incase we get stuck, turn the rover...
            if(processData.last_move_amount == 0){
                
                if(processData.lr_last_direction == LR_RIGHT){
                processMoveRoverTurn(LR_DOWN); //down 
                }
                else if(processData.lr_last_direction == LR_DOWN){
                    processMoveRoverTurn(LR_LEFT); //left
                }
                else if(processData.lr_last_direction == LR_UP){
                    processMoveRoverTurn(LR_RIGHT); //right
                }
                else if(processData.lr_last_direction == LR_LEFT){
                    processMoveRoverTurn(LR_UP); //up
                }                
                else if(processData.lr_last_direction == LR_UNKNOWN){ //TODO change me, this needs to be fixed
                    processData.lr_last_direction = LR_LEFT;
                    
                }
                    /*
                    //We need to relocate the rover and its direction
                    processData.state = PROCESS_STATE_LOCATE_ROVER_INIT;
                    processData.rover_located = FALSE;
                    processData.next_move_amt = 1;
                    
                    if(processData.rover_x > 85){
                         processMoveRoverTurn(LR_DOWN);
                    }
                    else if(processData.rover_x < 5){
                         processMoveRoverTurn(LR_UP);
                    }
                    else if(processData.rover_y > 85){
                         processMoveRoverTurn(LR_RIGHT);
                    }
                    else if(processData.rover_y < 5){
                         processMoveRoverTurn(LR_LEFT);
                    }
                }
                */
            }
            
            
            //Check to see if we've searched the entire map
            if((processData.rover_x >= 41) && (processData.rover_y >= 41) && (processData.rover_x <= 48) && (processData.rover_y <= 48)){
                processData.finished = TRUE; //We searched the whole map and are done moving 
            }
            
            
            //Maybe add in extra checks to verify direction is correct? TODO
            
            //Figure out the move amount
            if(processData.lr_last_direction == LR_UP){
                processData.next_move_amt = distanceToMove(processData.rover_y, processData.max_y); 
            }
            else if(processData.lr_last_direction == LR_DOWN){
                processData.next_move_amt = distanceToMove(processData.rover_y, processData.min_y); 
            }
            else if(processData.lr_last_direction == LR_RIGHT){
                 processData.next_move_amt = distanceToMove(processData.rover_x, processData.max_x);
            }
            else if(processData.lr_last_direction == LR_LEFT){
                processData.next_move_amt = distanceToMove(processData.rover_x, processData.min_x); 
            }
            
            //Let the other state handle moving as it will also update the known map
            //Locate the rover again. It will make it move forwards by move amt
            //processData.state = PROCESS_STATE_LOCATE_ROVER_INIT;   
            //CHANGED 4/18
            processData.state = PROCESS_STATE_LOCATE_ROVER_MOVE; 
            
            //Move our newest data into the "old" data queue, since the rover is about to move
            processTransferMapDataBuffers(); 
            
            
                   
            
            
            break;  
             
        }
        
        //Handle a diversion 
        case PROCESS_STATE_DIVERT:
        {
            processData.need_divert = FALSE;
            int broke = FALSE; //Turns true if we need to break off mid diversion to continue on path
            
            if((processData.lr_last_direction == LR_UP) && (broke == FALSE)){ //we want to go up but know there is an obstacle there
                
                //Divert around obstacle 
                //Decide which way to go around it
                
                int moves = abs(processData.calc_rover_x - processData.obs_x) + BUFFER_ZONE + ERROR_AMT;
                int next_dir;
                //See if we've covered that ground already
                if((processData.calc_rover_x - moves) <= processData.min_x){
                    processMoveRoverTurn(LR_RIGHT);
                    next_dir = LR_LEFT;
                }
                else if((processData.calc_rover_x + moves) >= processData.max_x){
                    processMoveRoverTurn(LR_LEFT); 
                    next_dir = LR_RIGHT;
                }
                else{ //if it doesn't matter, choose the best path.
                    if(processData.calc_rover_x < processData.obs_x){
                        processMoveRoverTurn(LR_LEFT);
                        next_dir = LR_RIGHT;
                    }
                    else{
                        processMoveRoverTurn(LR_RIGHT);
                        next_dir = LR_LEFT;

                    }
                }   
                
               DIVERT_MoveRoverForward(moves+1);
                

                int other_moves = abs(processData.obs_y - processData.calc_rover_y) + OBSTACLE_DIAMETER;
                processMoveRoverTurn(LR_UP);
                
                int increment = 0;                
                for(increment=0; increment < other_moves; increment++){
                    DIVERT_MoveRoverForward(1);
                    //Determine if we should instead make the next turn
                    if(processData.calc_rover_y >= processData.max_y){
                        processMoveRoverTurn(LR_RIGHT); //right
                        processData.max_y += -ROVER_WIDTH - 1;
                        broke = TRUE;
                        break;
                    }
                    
                }                
                if(broke == FALSE){
                    processMoveRoverTurn(next_dir); 

                    
                    DIVERT_MoveRoverForward(moves+1);
                    
                    //Return rover to last direction to continue on
                    processMoveRoverTurn(LR_UP);
                }
            }            
            else if((processData.lr_last_direction == LR_DOWN) && (broke == FALSE)){
                //Divert around obstacle 
                
                int moves = abs(processData.obs_x - processData.calc_rover_x) + BUFFER_ZONE + ERROR_AMT;
                int next_dir;
                //See if we've covered that ground already
                if((processData.calc_rover_x - moves) <= processData.min_x){
                    processMoveRoverTurn(LR_RIGHT);
                    next_dir = LR_LEFT;
                }
                else if((processData.calc_rover_x + moves) >= processData.max_x){
                    processMoveRoverTurn(LR_LEFT); 
                    next_dir = LR_RIGHT;
                }
                else{
                    if(processData.calc_rover_x < processData.obs_x){
                        processMoveRoverTurn(LR_LEFT);
                        next_dir = LR_RIGHT;
                    }
                    else{
                        processMoveRoverTurn(LR_RIGHT);
                        next_dir = LR_LEFT;
                    }              
                }
                               
                
                
                DIVERT_MoveRoverForward(moves+1);
                
                
                int other_moves = abs(processData.calc_rover_y - processData.obs_y) + OBSTACLE_DIAMETER;
                
                processMoveRoverTurn(LR_DOWN);
                
                int increment = 0;
                for(increment=0; increment < other_moves; increment++){
                    DIVERT_MoveRoverForward(1);
                    //Determine if we should instead make the next turn
                    if((processData.calc_rover_y <= processData.min_y)){
                        processMoveRoverTurn(LR_LEFT);
                        processData.min_y += ROVER_WIDTH - 1;
                        broke = TRUE;
                        break;
                    }
                }                   
                if(broke == FALSE){
                    processMoveRoverTurn(next_dir);                 

                    
                    DIVERT_MoveRoverForward(moves+1);
                            

                    processMoveRoverTurn(LR_DOWN);
                }
               
            }
            else if((processData.lr_last_direction == LR_RIGHT) && (broke == FALSE)){
                //Divert around obstacle 
                //Decide which way to go around it
                
                int moves = abs(processData.obs_y - processData.calc_rover_y) + BUFFER_ZONE + ERROR_AMT;
                int next_dir;
                //See if we've covered that ground already
                if((processData.calc_rover_y - moves) <= processData.min_y){
                    processMoveRoverTurn(LR_UP);
                    next_dir = LR_DOWN;
                }
                else if((processData.calc_rover_y + moves) >= processData.max_y){
                    processMoveRoverTurn(LR_DOWN); 
                    next_dir = LR_UP;
                }
                else{
                    if(processData.calc_rover_y > processData.obs_y){
                        processMoveRoverTurn(LR_UP);
                        next_dir = LR_DOWN;
                    }
                    else{
                        processMoveRoverTurn(LR_DOWN);
                        next_dir = LR_UP;
                    }              
                }
                                
                
                DIVERT_MoveRoverForward(moves+1);
                
                
                int other_moves = abs(processData.obs_x - processData.calc_rover_x) + OBSTACLE_DIAMETER;
                processMoveRoverTurn(LR_RIGHT);
                               
                int increment = 0;
                for(increment=0; increment < other_moves; increment++){
                    DIVERT_MoveRoverForward(1);
                    if((processData.calc_rover_x >= processData.max_x)){
                        processMoveRoverTurn(LR_DOWN);
                        processData.max_x += -ROVER_WIDTH - 1;
                        broke = TRUE;
                        break;
                    }
                }                 
                      
                if(broke == FALSE){
                    processMoveRoverTurn(next_dir); 

                    
                    DIVERT_MoveRoverForward(moves+1);
                   

                    processMoveRoverTurn(LR_RIGHT);   
                }
                
            }
            else if((processData.lr_last_direction == LR_LEFT) && (broke == FALSE)){
                //Divert around obstacle 
                
                int moves = abs(processData.obs_y - processData.calc_rover_y) + BUFFER_ZONE + ERROR_AMT;
                int next_dir; 
                //See if we've covered that ground already
                if((processData.calc_rover_y - moves) <= processData.min_y){
                    processMoveRoverTurn(LR_UP);
                    next_dir = LR_DOWN;
                }
                else if((processData.calc_rover_y + moves) >= processData.max_y){
                    processMoveRoverTurn(LR_DOWN); 
                    next_dir = LR_UP;
                }
                else{
                    if(processData.calc_rover_y > processData.obs_y){
                        processMoveRoverTurn(LR_UP);
                        next_dir = LR_DOWN;
                    }
                    else{
                        processMoveRoverTurn(LR_DOWN);
                        next_dir = LR_UP;
                    }    
                }
                
                
                DIVERT_MoveRoverForward(moves+1);
                
                
                int other_moves = abs(processData.calc_rover_x - processData.obs_x) + OBSTACLE_DIAMETER;
                processMoveRoverTurn(LR_LEFT);
                
                int increment = 0;
                for(increment=0; increment < other_moves; increment++){
                    DIVERT_MoveRoverForward(1);
                    if((processData.calc_rover_x <= processData.min_x)){
                        processMoveRoverTurn(LR_UP);
                        processData.min_x += ROVER_WIDTH - 1;
                        broke = TRUE;
                        break;
                    }
                }                   
                
                if(broke == FALSE){                    
                    processMoveRoverTurn(next_dir); 

                    
                    DIVERT_MoveRoverForward(moves+1);
                                   

                    processMoveRoverTurn(LR_LEFT); 
                }           
            }
            
           
            processData.state = PROCESS_STATE_MOVE;
            break;
        }
        

        /* The default state should never be executed. */
        default:
        {
            /* TODO: Handle error in application's state machine. */
            dbgOutputVal(PROCESS_ENTERED_DEFAULT);
            stopAll();
            break;
        }
    }
}

/*******************************************************************************
 End of File
 */
