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
#include "proj_definitions.h"
#include "peripheral/oc/plib_oc.h"
#include "peripheral/int/plib_int.h"

void putDataOnMotorProcessQ(char* data);
void processMotorInstruction(void);
void setMotors(int right, int left);
void setRightDirection(int enable);
void setLeftDirection(int enable);
void pid(int left, int right);
void pidReset(void);
void receiveFromMotorProcessQ(void);
void ackToken(char seq, char time1, char time2);
void stopMotors(void);
void processMotorControls(void);
void setMotorVars(void);
void searchOperation(void);
void followOperation(void);
void turnOperation(void);
void motorStop(void);

// ActuatorTimerCallback --> readEncoders > (checkTicks) > PID > processMotorInstruction > SetMotors > Set PWM Values
// MotortimerCallback --> Process Motor Controls > Receive from Q > Normal/Lock/Search Ops > SetMotorVars 

MOTOR_DATA motorData;
LEAD_POS leadPos;         // Running sensor data on lead rover's position.

/////////////////////////////// TEST FUNCTIONS /////////////////////////////////////////////
// Generic function to send data to send queue, message format via UART to Pi hub
void sendToSendQ(char type, char count, char d1, char d2, char d3, char d4, char d5, char d6) {
    char data[MSG_LENGTH];
    data[0] = MSG_START;          // Start byte
    data[1] = type;               // Type byte
    data[2] = count;              // Count byte
    data[3] = d1;                 // Sensor value (redundant)
    data[4] = d2;                 // Dummy
    data[5] = d3;                 // Right motor forward/reverse/stop
    data[6] = d4;                 // Right motor speed
    data[7] = d5;                 // Left motor forward/reverse/stop
    data[8] = d6;                 // Left motor speed
    data[9] = MSG_STOP;           // Stop byte

    putMsgOnSendQueue(data);
}

// DEBUG code - hardcode IR receiver read values
adjustSignals() {
    
    leadPos.frontSignal = 0;
    leadPos.rightSignal = 0;
    leadPos.leftSignal = 0;
    leadPos.rearSignal = 0;
    
    if (TEST_FRONT) {
        leadPos.frontSignal = 255;
        leadPos.bestSignal = LEAD_FRONT;
    }
    if (TEST_REAR) {
        leadPos.rearSignal = 255;
        leadPos.bestSignal = LEAD_REAR;
    }
    if (TEST_RIGHT) {
        leadPos.rightSignal = 255;
        leadPos.bestSignal = LEAD_RIGHT;
    }
    if (TEST_LEFT) {
        leadPos.leftSignal = 255;
        leadPos.bestSignal = LEAD_LEFT;
    }
}

// DEBUG code - Simulate token found message received
void testTokenFound() {
    if (SEND_FAKE_TOKEN)
        ackToken(0x44, 0x53, 0x56);
}

// DEBUG code - send current rover status (key control variables))
void sendStatus() {
    if (MODE_STATUS_UPDATE)
        sendToSendQ(TYPE_FR_STATUS, 0x50, motorData.rovState, 0x0, motorData.motorEnable, motorData.firstFollow, motorData.roundedTurn, motorData.stopOnToken);
}
// Toggle sharp turns and rounded turns (affected by IR side data or not)
void toggleTurn() {
    motorData.roundedTurn = motorData.roundedTurn ^ 1;
    sendStatus();
}
// Toggle stop on token or continuous mode (defaut)
void toggleStopOnToken() {
    motorData.stopOnToken = motorData.stopOnToken ^ 1;
    sendStatus();
}

// DEBUG code - Set data types to be sent via WiFly to Pi for visual error checking
void setOutputs(char IR, char TD, char ENC, char PID, char turn, char token) {
    motorData.roundedTurn = turn;
    motorData.stopOnToken = token;
    motorData.sendIRData = IR;
    motorData.sendTargetData = TD;
    motorData.sendEncoder = ENC;
    motorData.sendPID = PID;
    sendStatus();
}
/////////////////////////////// TEST FUNCTIONS /////////////////////////////////////////////




////////////////////////////// LOCATION TRACKING ///////////////////////////////////////////
// Set distance from target location (Lead rover or Token) 
void setTargetDistance() {
    if (motorData.rovState == ACQUIRE_TOKEN_LOC) {
        // Distance to location: Distance reported * 73.5 + 7 cm for rest of follower + 6 cm for extra half-length
        if (!motorData.tokenQueue) {
            motorData.ticksToToken = (leadPos.frontDist[0] * 73) + (leadPos.frontDist[0] >> 1) + 750;
        }
        leadPos.prevDir = PREV_NEITHER;
        motorData.rovState = TOKEN_LOCK;  // LOCKED
    }
    else if (motorData.rovState == ACQUIRE_LOCATION) {
        // For location lock and not token lock, make addition only +515
        motorData.timeSinceVanish = 200;
        motorData.ticksToTarget = (leadPos.frontDist[3] * 73) + (leadPos.frontDist[3] >> 1) + 515 + motorData.curEncoder + motorData.prevEncoder;
        leadPos.prevDir = PREV_NEITHER;
        motorData.rovState = POINT_LOCK;  // Locked unless lead appears on distance sensors again
    }
}

// Received token signal from Lead - Acknowledge, and set distance if rover can be seen
void ackToken(char sequence, char timeH, char timeL) {
    
    // If currently tracking token - Resent acknowledge signal
    // TODO? - Create queue of tokens in the case that they are close together
    if (motorData.rovState == TOKEN_LOCK || motorData.rovState == ACQUIRE_TOKEN_LOC) {
        sendToSendQ(TYPE_FR_ACK, motorData.prevTokenSequence, 0x0, 0x0, 0x0, 0x0, motorData.prevTokenHigh, motorData.prevTokenLow); //Acknowledge receive
    }
    // Currently locked onto rover position pre-turn - Use time since turn to estimate distance
    // after turn needed to move to pick up token
    else if (motorData.rovState == POINT_LOCK) {
        motorData.tokenQueue = 1;
        motorData.ticksToTokenLock = motorData.timeSinceVanish;
        
        motorData.prevTokenHigh = timeH;  // Time values - Not currently used
        motorData.prevTokenLow = timeL;
        motorData.prevTokenSequence = sequence;
        
        //motorData.rovState = ACQUIRE_TOKEN_LOC;
        sendToSendQ(TYPE_FR_ACK, sequence, 0x0, 0x0, 0x0, leadPos.frontDist[0] & 0xff, timeH, timeL); //Acknowledge receive
        
    }
    // Else normal operation - Token is where lead rover currently is, lock onto that position
    else {
        motorData.ticksToTokenLock = 0;
        
        motorData.prevTokenHigh = timeH;
        motorData.prevTokenLow = timeL;
        motorData.prevTokenSequence = sequence;
        
        motorData.rovState = ACQUIRE_TOKEN_LOC;
        sendToSendQ(TYPE_FR_ACK, sequence, 0x0, 0x0, 0x0, leadPos.frontDist[0] & 0xff, timeH, timeL); //Acknowledge receive
        
        if (leadPos.frontDist[0] < 40 && leadPos.frontDist[1] < 40) {
            setTargetDistance();
        }
    }
}

// Read motor encoder countdown value to determine next rover state
void checkTicks() {
    
    // TOKEN POSITION ACQUIRED
    // Send confirmation signal to Coordinator, search for lead rover, begin flashing LED
    if (motorData.rovState == TOKEN_LOCK && motorData.ticksToToken <= 0) {
       
        sendToSendQ(TYPE_FR_FOUNDTOKEN, motorData.tokenFindCount, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0);
        motorData.tokenFindCount++;
                
        motorData.ticksToToken = 0;
        motorData.tokenQueue = 0;
        motorData.ticksToTokenLock = 0;
        
        motorData.tokenCountdown = 20;
        
        // Halt motors and light LED if at location of token
        if (motorData.stopOnToken == 0x1) {
            motorStop();
            motorData.keep_blinking = 1;
        }
        
        if (leadPos.frontDist[0] < 40)
            motorData.rovState = FOLLOW;
        else
            motorData.rovState = SEARCH;
    }
    // PREVIOUS ROVER POSITION ACQUIRED
    // Begin searching for rover's new post-turn position
    else if (motorData.rovState == POINT_LOCK && motorData.ticksToTarget <= 0) {
        motorData.ticksToTarget = 0;
        
        if (motorData.tokenQueue) {
            motorData.ticksToToken = motorData.ticksToTokenLock;
            motorData.rovState = ACQUIRE_TOKEN_LOC;
        }
        
        else if (leadPos.frontDist[0] < 40)
            motorData.rovState = FOLLOW;
        else
            motorData.rovState = SEARCH;
    }
}

////////////////////////////// LOCATION TRACKING ///////////////////////////////////////////



///////////////////////////// PROCESSING ////////////////////////////////////////////

// Update the running data on the lead rover's position in terms of IR received signals
// and distance sensing results
void motorUpdateLeadPos(char type, char d1, char d2, char d3, char d4, char d5, char d6) {

    int i;
    // Update front distance (in cm)
    if (type == (char)TYPE_FR_DIST) {
        for (i = DIST_HISTORY - 2; i >= 0; i--) {
            leadPos.frontDist[i+1] = leadPos.frontDist[i];
        }
        leadPos.frontDist[0] = d6;
    }
    // ONLY ONE USED IN CURRENT BUILD
    else if (type == (char)TYPE_ADC) {
        for (i = DIST_HISTORY - 2; i >= 0; i--) {
            leadPos.frontDist[i+1] = leadPos.frontDist[i];
        }
        leadPos.frontDist[0] = d6;

        leadPos.prevFrontSignal = leadPos.frontSignal;
        leadPos.bestSignal = d1;
        leadPos.frontSignal = d2;
        leadPos.rearSignal = d3;
        leadPos.leftSignal = d5;
        leadPos.rightSignal = d4;
        
        // Determine previous relative side position of rover, if any. Analog values can fluctuate,
        // so minimum difference is mandatory. 
        if (leadPos.leftSignal > (leadPos.rightSignal + IROFFSET)) {
            leadPos.prevDir = PREV_LEFT;
        }
        else if ((leadPos.leftSignal + IROFFSET) < leadPos.rightSignal) {
            leadPos.prevDir = PREV_RIGHT;
        }
        
        motorData.IRsendCount++;
        // Send to UART on specified interval (Milestone #3)
        if ((SEND_IR == 0x1) && (motorData.IRsendCount % IR_MOD == 0))
            sendToSendQ(TYPE_HISTORY, leadPos.frontDist[0], leadPos.frontDist[1], leadPos.frontDist[2], leadPos.frontDist[3], leadPos.frontDist[4], leadPos.frontDist[5], leadPos.frontDist[6]);
    }
    // Update directional IR data (all four at once)
    else if (type == (char)TYPE_FR_IR) {
        leadPos.bestSignal = d1;
        leadPos.frontSignal = d3;
        leadPos.rearSignal = d4;
        leadPos.leftSignal = d5;
        leadPos.rightSignal = d6;
        
        if (leadPos.leftSignal > (leadPos.rightSignal + IROFFSET)) {
            leadPos.prevDir = PREV_LEFT;
        }
        else if ((leadPos.leftSignal + IROFFSET) < leadPos.rightSignal) {
            leadPos.prevDir = PREV_RIGHT;
        }
        
        motorData.IRsendCount++;
        // Send to UART on specified interval (Milestone #3)
        if ((SEND_IR == 0x1) && (motorData.IRsendCount % IR_MOD == 0))
            sendToSendQ(type, 0x44, d1, d2, d3, d4, d5, d6);
    }
}

// Operation conducted when the lead rover is not in view of distance sensors.
// Rely on IR receiver data to determine rotation. 
void noTargetOperation() {
    
    //53 * 2^6 -> Value of encoder ticks expected per 100ms
    int maxPID = MAXPID;
    int maxNEG = MAXNEG;
    
    // Conditionals to determine motor action based on current sensor data
    // No credible directional data: 
    if ((leadPos.frontSignal < IRMIN && leadPos.rearSignal < IRMIN && leadPos.rightSignal < IRMIN && leadPos.leftSignal < IRMIN)
    || (leadPos.frontSignal == IRMAX && leadPos.rearSignal == IRMAX && leadPos.rightSignal == IRMAX && leadPos.leftSignal == IRMAX)) {
        //SPIN - Not full speed
            motorData.pidRightSpd = maxPID - 500;
            motorData.pidLeftSpd = maxPID - 500;
                
            // Determine rotation direction based on left/right receiver data.
            // If neither is current majority, use previous stronger signal of the two.
            if (leadPos.leftSignal > (leadPos.rightSignal + IROFFSET)) {
                motorData.pidLeftSpd *= -1;
            } else if (leadPos.rightSignal > (leadPos.leftSignal + IROFFSET)) {
                motorData.pidRightSpd *= -1;
            } else {
                if (leadPos.prevDir == PREV_LEFT) {
                    motorData.pidLeftSpd *= -1;
                } else {
                    motorData.pidRightSpd *= -1;
                }
            }
    }
    else {
        // Strong signal in front of rover:
        if (leadPos.bestSignal == LEAD_FRONT) {
            motorData.pidRightSpd = maxPID >> 1;
            motorData.pidLeftSpd = maxPID >> 1;
                
            if (leadPos.leftSignal > (leadPos.rightSignal + IROFFSET)) {
                motorData.pidLeftSpd *= -1;
            } else if (leadPos.rightSignal > (leadPos.leftSignal + IROFFSET)) {
                motorData.pidRightSpd *= -1;
            } else {
                if (leadPos.prevDir == PREV_LEFT) {
                    motorData.pidLeftSpd *= -1;
                } else {
                    motorData.pidRightSpd *= -1;
                }
            }
        }
        // Strong signal behind rover: Follower turns around, direction depending on side data
        else if (leadPos.bestSignal == LEAD_REAR) {
            motorData.pidRightSpd = maxPID;
            motorData.pidLeftSpd = maxPID;
                
            if (leadPos.leftSignal > (leadPos.rightSignal + IROFFSET)) {
               motorData.pidLeftSpd *= -1;
            } else if (leadPos.rightSignal > (leadPos.leftSignal + IROFFSET)) {
                motorData.pidRightSpd *= -1;
            } else {
                if (leadPos.prevDir == PREV_LEFT) {
                    motorData.pidLeftSpd *= -1;
                } else {
                    motorData.pidRightSpd *= -1;
                }
            }
        }
        // Stronger signal right of rover:
        else if (leadPos.bestSignal == LEAD_RIGHT) {
            motorData.pidRightSpd = maxNEG;
            motorData.pidLeftSpd = maxPID;
        }
        // Stronger signal left of rover
        else if (leadPos.bestSignal == LEAD_LEFT) {
            motorData.pidRightSpd = maxPID;
            motorData.pidLeftSpd = maxNEG;
        }
        // Default state: If this is true, something went wrong
        else {
            motorData.pidRightSpd = maxNEG;
            motorData.pidLeftSpd = maxPID;
        }
    }
    
    // If the last IR distance data reported valid, slow more to keep on target for the second reading
    if (leadPos.frontDist[0] > 20 && leadPos.frontDist[0] < 40) {
        motorData.pidRightSpd >>= 1;
        motorData.pidLeftSpd >>= 1;
        
        
    }
}

// Operation when locked onto target position (Previous lead position or Token location)
void lockOperation() {
    
    //53 * 2^6
    int maxPID = MAXPID;
    int maxNEG = MAXNEG;
    
    // If not locked onto token and lead appears in front, switch to follow mode
    if (motorData.rovState != TOKEN_LOCK && leadPos.frontDist[0] < 40 && leadPos.frontDist[1] < 40) {
        motorData.rovState = FOLLOW;
        followOperation();
        return;
    }
   
    // If first follow (initial bootup) -> Slower rotations, greater allowance for angular adjustment
    // because we aren't 'Following' just yet
    if (motorData.firstFollow) {
        if (leadPos.bestSignal == LEAD_LEFT) {
            motorData.pidLeftSpd = -2000;
            motorData.pidRightSpd = 2000;
        }
        else if (leadPos.bestSignal == LEAD_RIGHT) {
            motorData.pidLeftSpd = 2000;
            motorData.pidRightSpd = -2000;
        }
    }
    // Always reverse if something's directly in front
    else if (leadPos.frontDist[0] < TARG_DIST) {
        motorData.pidRightSpd = maxNEG;
        motorData.pidLeftSpd = maxNEG;
    }
    // Always stop if something's in the target range in front
    else if (leadPos.frontDist[0] == TARG_DIST || leadPos.frontDist[0] == TARG_DIST_SEC) {
        motorData.pidRightSpd = 0;
        motorData.pidLeftSpd = 0;
    }
    // Else, full speed ahead.
    else {
        motorData.pidRightSpd = maxPID;
        motorData.pidLeftSpd = maxPID;
    }
    
    // If rounded turns enabled, decrease the motor whose side favors the lead's orientation
    // This allows for gradual turning without changing states on the fly
    if (motorData.rovState == POINT_LOCK && motorData.roundedTurn != 0) {
        if (leadPos.rightSignal > leadPos.leftSignal) {
            motorData.pidRightSpd >>= 1;
        }
        if (leadPos.leftSignal > leadPos.rightSignal) {
            motorData.pidLeftSpd >>= 1;
        }
    }
    
    setMotorVars();
    
}

// Follow operation -> Lead rover position currently known, no token lock
void followOperation() {
     //53 * 2^6
    int maxPID = MAXPID;
    int maxNEG = MAXNEG;
    
    // Debug code
    if (HC_SIGNALS) adjustSignals();
    
    // Front signal too low for the object in front to be the lead - Rotate (change mode) until we find the real target
    if (leadPos.frontSignal < 10 && (leadPos.rearSignal > 10 || leadPos.rightSignal > 200 || leadPos.leftSignal > 200)) {
        motorData.rovState = SEARCH;
        turnOperation();
        return;
    }

    // If in normal operation
    if (MODE_LAST_LOC == 0x1 && !motorData.firstFollow) {
        // Last two readings have lost the lead rover, after the previous two had it locked.
        // We've lost his position and want to set the last known location as our destination
        if (leadPos.frontDist[0] == 40 && leadPos.frontDist[1] == 40 && leadPos.frontDist[2] < 40 && leadPos.frontDist[3] < 40) {
            motorData.rovState = ACQUIRE_LOCATION;

            setTargetDistance();
            
            lockOperation();
            return;
        }
        // If we have one signal that seems to have lost the lead, slow down to read again and make sure 
        // it wasn't an erratic reading.
        if (leadPos.frontDist[0] == 40 && leadPos.frontDist[1] < 40 && leadPos.frontDist[2] < 40) {
            motorData.pidRightSpd >>= 1;
            motorData.pidLeftSpd >>= 1;
            
            setMotorVars();
            
            return;
        }
    }
    
    // Always reverse if there's something right in front of the sensor
    if (leadPos.frontDist[0] < TARG_DIST) {
        motorData.pidRightSpd = maxNEG;
        motorData.pidLeftSpd = maxNEG;
        
        // Test mode - Try to reverse and also adjust direction if there's a side offset
        if (MODE_NUANCE == 0x1) {
            if (leadPos.bestSignal == LEAD_RIGHT) {
                motorData.pidLeftSpd += 1000;
            }
            if (leadPos.bestSignal == LEAD_LEFT) {
                motorData.pidRightSpd += 1000;
            }
        }
    }
    // Always stop if something's in the target zone
    else if (leadPos.frontDist[0] == TARG_DIST || leadPos.frontDist[0] == TARG_DIST_SEC) {
        motorData.pidRightSpd = 0;
        motorData.pidLeftSpd = 0;
        
        if (MODE_NUANCE == 0x1) {
            if (leadPos.bestSignal == LEAD_RIGHT) {
                motorData.pidRightSpd = -800;
                motorData.pidLeftSpd = 800;
            }
            if (leadPos.bestSignal == LEAD_LEFT) {
                motorData.pidRightSpd = 800;
                motorData.pidLeftSpd = -800;
            }
        }
    }
    // Else, full speed ahead
    else if (leadPos.frontDist[0] < 40) {
        motorData.pidRightSpd = maxPID;
        motorData.pidLeftSpd = maxPID;
        
        // IF AT STARTUP, Continue adjusting rotation until the front signal is the strongest
        // The key at first is to line up accurately
        if (motorData.firstFollow) {
            if (leadPos.bestSignal == LEAD_LEFT) {
                motorData.pidLeftSpd = -2000;
                motorData.pidRightSpd = 2000;
            }
            else if (leadPos.bestSignal == LEAD_RIGHT) {
                motorData.pidLeftSpd = 2000;
                motorData.pidRightSpd = -2000;
            }
            else if (leadPos.bestSignal == LEAD_FRONT) {
                if (leadPos.bestSignal == LEAD_LEFT) {
                    motorData.pidLeftSpd -= 2000;
                }
                else if (leadPos.bestSignal == LEAD_RIGHT) {
                    motorData.pidRightSpd -= 2000;
                }
            }
        }
        else if (MODE_NUANCE == 0x1) {
            if (leadPos.bestSignal == LEAD_RIGHT) {
                motorData.pidRightSpd -= 1000;
            }
            if (leadPos.bestSignal == LEAD_LEFT) {
                motorData.pidLeftSpd -= 1000;
            }
        }
    }
    //Else, not locked onto front target - Pursue based on directional data
    else {
        motorData.rovState = SEARCH;
        
        noTargetOperation();
    }
    
    setMotorVars();
    
}

// Search state -> Determine if we need to get out of the state, else move to noTargetOperation())
void turnOperation() {
    
    //53 * 2^6
    int maxPID = MAXPID;
    int maxNEG = MAXNEG;
    
    if (HC_SIGNALS) adjustSignals();
    
    // If last two signals were positive, and the most recent says the lead is close
    if (leadPos.frontDist[0] < 22 && leadPos.frontDist[1] < 40) {
        
        // If the IR receivers corroborate the distance sensor's beliefs, switch to follow
        if (!(leadPos.rearSignal > 10) && !(leadPos.frontSignal < 10 && leadPos.rightSignal > 10) && !(leadPos.frontSignal < 10 && leadPos.leftSignal > 10)) {
            motorData.rovState = FOLLOW;
            followOperation();
            return;
        }
    }
    // If first follow, trust the first read that states accuracy. If the IR receivers say otherwise,
    // other code will adjust turns for now. 
    else if (leadPos.frontDist[0] < 40 && motorData.firstFollow) {
            motorData.rovState = FOLLOW;
            followOperation();
            return;
    }
    // If the last three readings were positive for the lead rover and the IR receivers aren't going crazy,
    // Trust that the lead is actually there and follow
    else if (leadPos.frontDist[0] < 40 && leadPos.frontDist[1] < 40 && leadPos.frontDist[2] < 40) {
        if (!(leadPos.rearSignal > 10) && !(leadPos.frontSignal < 10 && leadPos.rightSignal > 10) && !(leadPos.frontSignal < 10 && leadPos.leftSignal > 10)) {
            motorData.rovState = FOLLOW;
            followOperation();
            return;
        }
    }
    
    // Else, commence rotation instructions
    noTargetOperation();
    
    setMotorVars();
}

// TOKEN SEARCH OPERATION - Lead position not known, but token signal received
void searchOperation() {
    
   //53 * 2^6
    int maxPID = MAXPID;
    int maxNEG = MAXNEG;
    
    if (HC_SIGNALS) adjustSignals();
    
    // Follow similar guidelines for the regular search operation, but set a target rather 
    // than the lead itself (still using its current position, though)
    if (leadPos.frontDist[0] < 22 && leadPos.frontDist[1] < 40) {
        
        if (!(leadPos.rearSignal > 10) && !(leadPos.frontSignal < 10 && leadPos.rightSignal > 10) && !(leadPos.frontSignal < 10 && leadPos.leftSignal > 10)) {
            setTargetDistance();
            lockOperation();
            return;
        }
    }
    else if (leadPos.frontDist[0] < 40 && leadPos.frontDist[1] < 40 && motorData.firstFollow) {
        motorData.rovState = FOLLOW;
        followOperation();
        return;
    }
    else if (leadPos.frontDist[0] < 40 && leadPos.frontDist[1] < 40 && leadPos.frontDist[2] < 40) {
        if (!(leadPos.rearSignal > 10) && !(leadPos.frontSignal < 10 && leadPos.rightSignal > 10) && !(leadPos.frontSignal < 10 && leadPos.leftSignal > 10)) {
            motorData.rovState = FOLLOW;
            followOperation();
            return;
        }
    }
    
    // else, proceed with turning operation
    noTargetOperation();
    
    setMotorVars();
}
    
   
// Set motor speed and direction control variables
void setMotorVars() {
    
    // Default values - Aim forward, but don't move
    char rightDir = MOTOR_FORWARD;
    char leftDir = MOTOR_FORWARD;
    int rightSpeed = 0;
    int leftSpeed = 0;

    //53 * 2^6
    int maxPID = MAXPID;
    int maxNEG = MAXNEG;
    
    // PID variables
    int error;
    int deriv;
    int out = 0;
    
    // Cap the ideal right/left speeds for the motors
    if (motorData.pidRightSpd >= maxPID) motorData.pidRightSpd = maxPID;
    else if (motorData.pidRightSpd <= maxNEG) motorData.pidRightSpd = maxNEG;
    if (motorData.pidLeftSpd >= maxPID) motorData.pidLeftSpd = maxPID;
    else if (motorData.pidLeftSpd <= maxNEG) motorData.pidLeftSpd = maxNEG;
    
    // Convert the pid values into encoder ticks desired per reading (100ms)
    rightSpeed = motorData.pidRightSpd >> 6;
    leftSpeed = motorData.pidLeftSpd >> 6;
    
    // PID values are signed integers, so set direction accordingly
    // And then adjust the encoder tick values to be positive at all times
    if (motorData.pidRightSpd < 0) {
        rightDir = MOTOR_REVERSE;
        rightSpeed *= -1;
    }
    else if (motorData.pidRightSpd == 0) {
        rightDir = MOTOR_STOP;
    }
    else rightDir = MOTOR_FORWARD;
    
    if (motorData.pidLeftSpd < 0) {
        leftDir = MOTOR_REVERSE;
        leftSpeed *= -1;
    }
    else if (motorData.pidLeftSpd == 0) {
        leftDir = MOTOR_STOP;
    }
    else leftDir = MOTOR_FORWARD;
    
    // If first follow and stopped, confirm that first follow has successfully concluded
    if (motorData.firstFollow) {
        if (motorData.pidLeftSpd == 0 && motorData.pidRightSpd == 0) {
            motorData.firstFollow = 0;
            sendStatus();
        }
    }
    
    // # Encoder ticks desired per 100ms read
    motorData.rightIdeal = rightSpeed;
    motorData.leftIdeal = leftSpeed;
    
    // Direction set
    motorData.rightDir = rightDir;
    motorData.leftDir = leftDir;
    
    // Maintain previous pid value
    motorData.pidPrevRight = motorData.pidRightSpd;
    motorData.pidPrevLeft = motorData.pidLeftSpd;
    
    // If variable active, send PID calculated values for front distance measurement
    if (SEND_DISTPID == 0x1 && (motorData.motorSendCount % DISTPID_MOD == 0)) {
        sendToSendQ(TYPE_DISTPID, 0x90, error & 0xFF, out & 0xFF, leftSpeed & 0xFF, rightSpeed & 0xFF,
                leadPos.pidIntegral & 0xff, deriv & 0xFF);
    }
            
    motorData.motorSendCount++;   // Increment sequence byte
    
    // Send to UART at specified interval
    if ((SEND_MOTOR == 0x1) && (motorData.motorSendCount % MOTOR_MOD == 0))
        sendToSendQ(TYPEC_MOTOR_CONTROL, motorData.motorSendCount, (leadPos.frontDist[0] & 0xFF), 
                0x40, rightDir, (rightSpeed & 0xFF), leftDir, (leftSpeed & 0xFF));    
}

// Read current values in the process queue
// Call function based on current state
void processMotorControls() {
    receiveFromMotorProcessQ();
    
    if (motorData.rovState == TOKEN_LOCK || motorData.rovState == POINT_LOCK) {
        lockOperation();
    } else if (motorData.rovState == ACQUIRE_LOCATION || motorData.rovState == ACQUIRE_TOKEN_LOC) {
        searchOperation();
    } else if (motorData.rovState == FOLLOW) {
        followOperation();
    } else {
        turnOperation();
    }
}

// Read the values of the hardware timers tied to the encoder values, act on them, and reset the timers
void motorReadEncoders() {

    motorData.timeSinceVanish += 100; // 100ms per read, track if the lead rover is out of range
    
    // Maintain previous encoder value for posterity
    motorData.prevEncoder = motorData.curEncoder;
    
    // Set encoder values from current timer reads (# ticks since last read)
    int rightEnc = DRV_TMR3_CounterValueGet();
    int leftEnc = DRV_TMR2_CounterValueGet();

    // Estimate distance traveled by taking the average of the two encoder values
    motorData.curEncoder = (rightEnc + leftEnc) >> 1;
    
    motorData.sendCount++;
 
    if (motorData.sendCount % TARGETDATA_MOD == 0 && (motorData.sendTargetData == 0x1)) {
        sendToSendQ(TYPE_FR_TARGETDATA, motorData.sendCount, 0x33, motorData.rovState, ((motorData.ticksToToken >> 8) & 0xFF), 
                motorData.ticksToToken & 0xFF, ((motorData.ticksToTarget >> 8) & 0xFF), motorData.ticksToTarget & 0xFF);
    }
    
    // TEST
    //if (motorData.rovState == POINT_LOCK || motorData.rovState == ACQUIRE_LOCATION) {
    if (motorData.sendCount % ENCODER_MOD == 0 && (motorData.sendEncoder == 0x1)) {
        sendToSendQ(TYPE_FR_ENCODER, motorData.sendCount, motorData.leftIdeal, motorData.rightIdeal, ((leftEnc >> 8) & 0xFF), 
                leftEnc & 0xFF, ((rightEnc >> 8) & 0xFF), rightEnc & 0xFF);
    }
    
    // If going in opposite directions, this is a rotation and no distance has been moved
    if (motorData.rightDir != motorData.leftDir) {
        motorData.curEncoder = 0;
    }
    // If moving forward, convert ticks to negative distance because we're counting down to zero
    else if (motorData.rightDir != MOTOR_REVERSE) {
        motorData.curEncoder *= -1;
    }
        
    // If locked onto a token, increment/decrement the # ticks expected left to pick up the token
    if (motorData.rovState == TOKEN_LOCK) {
        
        motorData.ticksToToken += motorData.curEncoder;
        
        checkTicks();
    }
    // If locked onto previous lead position, do the same but for the turn location
    else if (motorData.rovState == POINT_LOCK) {
                
        motorData.ticksToTarget += motorData.curEncoder;
        
        checkTicks();
    }
    
    // Clear timers for clean reads
    DRV_TMR2_CounterClear();
    DRV_TMR3_CounterClear();
    
    // Call pid function to act on the currently read values and ideal numbers processed
    pid(leftEnc, rightEnc);
}

// This is the function that actually sets the direction and speed of the motors for the current cycle.
// It is the closest abstraction to the physical components in terms of the processing functions.
void processMotorInstruction() {

    // If locked onto token or following lead rover, the motors should always be in the same direction.
    if (motorData.rovState == TOKEN_LOCK || motorData.rovState == POINT_LOCK || motorData.rovState == FOLLOW) {
        motorData.rightDir = motorData.leftDir;
    }
    else if (motorData.leftDir == MOTOR_REVERSE) motorData.rightDir = MOTOR_FORWARD;
    else if (motorData.leftDir == MOTOR_FORWARD) motorData.rightDir = MOTOR_REVERSE;

    // Set motor directions on the appropriate pins using the current member variables
    switch (motorData.rightDir) {
        case MOTOR_FORWARD: 
            setRightDirection(SET_HIGH);
            break;
        case MOTOR_REVERSE: 
            setRightDirection(SET_LOW);
            break;
        default: 
            setRightDirection(SET_LOW);
            //rSpeed = 0x0;
            break;
    }
    switch (motorData.leftDir) {
        case MOTOR_FORWARD: 
            setLeftDirection(SET_HIGH);
            break;
        case MOTOR_REVERSE: 
            setLeftDirection(SET_LOW);
            break;
        default: 
            setLeftDirection(SET_LOW);
            //lSpeed = 0x0;
            break;
    }
        
    // If right/left are in the MOTOR_STOP state, force their speed values to be zero
    int r, l;
    if (motorData.rightDir == (char)MOTOR_STOP) r = 0; 
    else r = motorData.rightSpd;

    if (motorData.leftDir == (char)MOTOR_STOP) l = 0; 
    else l = motorData.leftSpd;
       
    // Set the motor speed values for both sides
    if (motorData.motorEnable == 0x1) 
        setMotors(r, l);
}

// MOTOR ENCODER PID CALCULATION
void pid(int l_measured, int r_measured) {
    int l_error;
    int l_deriv;
    int l_out = 0;
    int l_SP = motorData.leftIdeal;  // set point for left motor
    int r_error;
    int r_deriv;
    int r_out = 0;
    int r_SP = motorData.rightIdeal;  // Set point for right motor
    int r_int_off;
    int l_int_off;
    
    int KP = 35;  // Coefficient for proportion (primary)
    int KI = 4;   // Coefficient for integral
    int KD = 12;  // Coefficient for derivative
    
    
    // If totally stopped, reset previous values, integral value, and ideal values to 
    // avoid accumulation of error over time
    if (motorData.leftDir == MOTOR_STOP) {
        motorData.pidLeftSpd = 0;
        motorData.leftSpd = 0;
        motorData.lpid_PrevError = 0;
        motorData.leftIdeal = 0;
        motorData.lpid_Integral = 0;
    }
    // If previously stopped, set the current speed to be the processed ideal
    // for the initial burst, to speed up the process
    else if (motorData.pidPrevLeft == 0) {
        motorData.leftSpd = motorData.pidLeftSpd >> 6;
    }
    else {
        // Core PID value
        l_error = l_SP - l_measured;
        
        if (l_error < 0) l_int_off = -1;
        else l_int_off = 1;
        
        motorData.lpid_Integral = motorData.lpid_Integral + ((l_error*FR_ENCODER_TIMER_RATE) >> 10) + l_int_off;
        l_deriv = (l_error - motorData.lpid_PrevError);   // / FR_ENCODER_TIMER_RATE;
        l_out = ((KP * l_error) + (KI * motorData.lpid_Integral) + (KD * l_deriv));// >> 5;
        motorData.lpid_PrevError = l_error;

        // Set a maximum allotted change per read to avoid overshooting
        if (l_out > MAX_CHANGE) l_out = MAX_CHANGE;
        else if (l_out < -MAX_CHANGE) l_out = -MAX_CHANGE;

        // Update motor speed value, with a maximum cap
        motorData.leftSpd = motorData.leftSpd + l_out;
        if (motorData.leftSpd > MAX_MOTOR) motorData.leftSpd = MAX_MOTOR;
    }
    
    // Right motor follows an equivalent process as the left
    if (motorData.rightDir == MOTOR_STOP) {
        motorData.pidRightSpd = 0;
        motorData.rightSpd = 0;
        motorData.rpid_PrevError = 0;
        motorData.rightIdeal = 0;
        motorData.rpid_Integral = 0;
    }
    else if (motorData.pidPrevRight == 0) {
       motorData.rightSpd = motorData.pidRightSpd >> 6;
    }
    else {
        r_error = r_SP - r_measured;
        
        if (r_error < 0) r_int_off = -1;
        else r_int_off = 1;
        
        motorData.rpid_Integral = motorData.rpid_Integral + ((r_error*FR_ENCODER_TIMER_RATE) >> 10) + r_int_off;
        r_deriv = (r_error - motorData.rpid_PrevError);   // / FR_ENCODER_TIMER_RATE;
        r_out = ((KP * r_error) + (KI * motorData.rpid_Integral) + (KD * r_deriv));// >> 5;
        motorData.rpid_PrevError = r_error;

        if (r_out > MAX_CHANGE) r_out = MAX_CHANGE;
        else if (r_out < -MAX_CHANGE) r_out = -MAX_CHANGE;

        motorData.rightSpd = motorData.rightSpd + r_out;
        if (motorData.rightSpd > MAX_MOTOR) motorData.rightSpd = MAX_MOTOR;
    }
    
    // Used for messaging (to keep within 1 byte each)
    int lo = l_out/2;
    int ro = r_out/2;
    
    // If variable is active, send PID values to WiFly
    if (motorData.sendPID && (motorData.sendCount % PID_DIV == 0)) 
        sendToSendQ(TYPE_PID, 0x22, 0x33, r_error, motorData.rpid_Integral & 0xff, r_deriv & 0xff, lo & 0xFF, ro & 0xFF );
    
    // Set motor controls if the disable signal is inactive
    if (!CUT_MOTOR)
        processMotorInstruction();
}

// Reset PID values (i.e., on restart)
void pidReset() {
    motorData.lpid_Integral = 0;
    motorData.rpid_Integral = 0;
    motorData.rpid_PrevError = 0;
    motorData.lpid_PrevError = 0;
}
///////////////////////////// PROCESSING ////////////////////////////////////////////



////////////////////////////////// QUEUE FUNCTIONS ////////////////////////////////////

// Remove oldest data on full local process queue
// Returns 1 if successful, 0 otherwise
uint8_t removeMotorProcessQueueData() {
    char readdata[MSG_LENGTH];

    if (xQueueReceive(motorData.processQ_FR, &readdata, portMAX_DELAY))
    {
        return 0x1;
    } 
    
    return 0x0;
}

// Check local process queue for vacancies. If none, remove oldest data.
// After, add the parameter char* message to local process queue. 
void putDataOnMotorProcessQ(char* data) {
    if (motorData.processQ_FR != 0) {
        
        // Check for full queue. If no spaces available, call to remove oldest data.
        if (uxQueueSpacesAvailable( motorData.processQ_FR ) == 0) {
            // If message is not removed from queue, return and signal error
            if (removeMotorProcessQueueData() == 0) {
                dbgOutputVal(MOTOR_FULLQUEUE);
                stopAll();
                return;
            }
        }
        
        // Send to queue, with at least one vacancy guaranteed for this data
        if( xQueueSend( motorData.processQ_FR, (void*) data, portMAX_DELAY) != pdPASS )
        {
            dbgOutputVal(RECEIVE_SENDTOMOTORQ_FAIL);
        }
    }
}

// Check queue holding the data from other component tasks
void receiveFromMotorProcessQ()
{    
    char newData = 0;
    char newDir = 0;
    char newDist = 0;
    char tokenFound = 0;
    char newIR = 0;
    char read[MSG_LENGTH];
    char dirData[MSG_LENGTH];
    char distData[MSG_LENGTH];
    char tokenData[MSG_LENGTH];
    char IRData[MSG_LENGTH];
    int i;

    // Read the top message of the queue
    // Continue until all current messages are read.
    // Retain the most recent message of each specific type discovered (IR data, token receive, etc)
    while (uxQueueMessagesWaiting(motorData.processQ_FR) != 0){
        if (xQueueReceive(motorData.processQ_FR, &read, portMAX_DELAY))
        {
            if (read[1] == (char)TYPE_FR_IR) {
                //stopAll();
                for(i = 0; i<10; i++) 
                    dirData[i] = read[i];
                newDir = 1;
            }   
            else if (read[1] == (char)TYPE_FR_DIST) {
                
                for(i = 0; i<10; i++) 
                    distData[i] = read[i];
                newDist = 1;
            }
            else if (read[1] == (char)TYPEC_LR_SENSOR_TO_FR) {
                for(i = 0; i<10; i++) 
                    tokenData[i] = read[i];
                tokenFound = 1;                
            }
            else if (read[1] == (char)TYPE_ADC) {
                for(i = 0; i<10; i++) 
                    IRData[i] = read[i];
                newIR = 1;                
            }
            
            newData = 1;
        } 
    }

    // If new data was received, update the process.c's understanding of the lead's current whereabouts
    if (newDir) {
        motorUpdateLeadPos(dirData[1], dirData[3], dirData[4], dirData[5], dirData[6], dirData[7], dirData[8]);
    }
    if (newDist) {
        motorUpdateLeadPos(distData[1], distData[3], distData[4], distData[5], distData[6], distData[7], distData[8]);
    }
    if (tokenFound) {
        ackToken(tokenData[2], tokenData[7], tokenData[8]);
    }
    if (newIR) {
        motorUpdateLeadPos(IRData[1], IRData[3], IRData[4], IRData[5], IRData[6], IRData[7], IRData[8]);
    }
    
    //Used in error checking
    motorData.prevType = read[1];
    motorData.prevCount = read[2];
}
////////////////////////////////// QUEUE FUNCTIONS ////////////////////////////////////





///////////////////// MOTOR CONTROLS ///////////////////////////////

// Set the PWM values for the motor speeds.
void setPWMLeft(uint16_t time_val) {
   PLIB_OC_PulseWidth16BitSet(OC_ID_1, time_val);
}
void setPWMRight(uint16_t time_val) {
   PLIB_OC_PulseWidth16BitSet(OC_ID_2, time_val);
}

// Set motors by stopping timers, resetting PWM values, and restarting timers
void setMotors(int right, int left) {
        DRV_TMR0_Stop();
        DRV_TMR0_Initialize();
        setPWMLeft(left);
        DRV_TMR0_Start();

        DRV_TMR1_Stop();
        DRV_TMR1_Initialize();
        setPWMRight(right);
        DRV_TMR1_Start();
}

// Full stop on both motors.
void stopMotors() {
    motorData.rightDir = MOTOR_STOP;
    motorData.leftDir = MOTOR_STOP;
    setMotors(0,0);
}

// Set direction of left tread
void setLeftDirection(int enable) {
    LATFCLR = (1 << 3);
    LATFSET = (enable << 3);
    
    // Enable = 1 --> FORWARD
    // Enable = 0 --> REVERSE
}
// Set direction of right read
void setRightDirection(int enable) {
    LATGCLR = (1 << 1);
    LATGSET = (enable << 1);
}
// Set control for left/right motor independence
void setJP1(int enable) {
    LATCCLR = (1 << 2);
    LATCSET = (enable << 2);
}
void setJP2(int enable) {
    LATCCLR = (1 << 3);
    LATCSET = (enable << 3);
}

// Initialize PWM values and all four timers (two for PWM, two tied to encoders)
void pwm_Init() {
    DRV_TMR0_Stop();
    DRV_TMR0_Initialize();
    DRV_TMR0_Start();
    
    DRV_TMR1_Stop();
    DRV_TMR1_Initialize();
    DRV_TMR1_Start();
    
    DRV_OC0_Disable();
    DRV_OC0_Initialize();
    DRV_OC0_Enable();
    
    DRV_OC1_Disable();
    DRV_OC1_Initialize();
    DRV_OC1_Enable();
    
    // Left encoder
    DRV_TMR2_Stop();
    DRV_TMR2_Initialize();
    DRV_TMR2_Start();
    
    // Right encoder
    DRV_TMR3_Stop();
    DRV_TMR3_Initialize();
    DRV_TMR3_Start();
}
///////////////////// MOTOR CONTROLS ///////////////////////////////



///////////////////////////// CORE ROUTINE ////////////////////////////////////
void MOTOR_Initialize ( void )
{
    motorData.IRsendCount = 0;
    motorData.sendCount = 0;
    motorData.motorSendCount = 0;
    
    motorData.state = MOTOR_STATE_INIT;
    motorData.rovState = SEARCH;
    
    motorData.motorEnable = 0x1;
    motorData.prevCount = 0x0;
    motorData.prevType = 0x0;
    
    // PID INITIALIZATIONS
    motorData.rpid_Integral = 0;
    motorData.rpid_PrevError = 0;
    motorData.lpid_Integral = 0;
    motorData.lpid_PrevError = 0;
    
    motorData.leftIdeal = 0;
    motorData.rightIdeal = 0;
    motorData.rightSpd = 0;
    motorData.leftSpd = 0;
    
    // Zero entire queue of lead rover distance readings
    int i;
    for (i = 0; i < DIST_HISTORY; i++) {
        leadPos.frontDist[i] = 0;
    }
    leadPos.pidIntegral = 0;
    leadPos.pidPrevError = 0;
    
    motorData.pidRightSpd = 0;
    motorData.pidLeftSpd = 0;
    
    motorData.ticksToToken = 0;
    motorData.ticksToTokenLock = 0;
    motorData.ticksToTarget = 0;
    
    motorData.curEncoder = 0;
    motorData.prevEncoder = 0;
    
    motorData.prevTokenHigh = 0x0;
    motorData.prevTokenLow = 0x0;
    motorData.prevTokenSequence = 0x0;
    
    motorData.tokenCountdown = 0;
    motorData.keep_blinking = 0;
    
    leadPos.prevFrontSignal = 0;
    motorData.tokenFindCount = 0;
    
    motorData.firstFollow = 1;
    
    motorData.roundedTurn = MODE_ROUNDED;
    motorData.stopOnToken = STOP_ON_TOKEN;
    
    motorData.timeSinceVanish = 0;
    motorData.tokenQueue = 0;
    //motorData.timeToToken = 0;
    
    // Initialize data send enablers
    motorData.sendIRData = SEND_IRDATA;
    motorData.sendTargetData = SEND_TARGETDATA;
    motorData.sendPID = SEND_PID;
    motorData.sendEncoder = SEND_ENCODERS;

    //Create a queue capable of holding 1000 characters (bytes))
    motorData.processQ_FR = xQueueCreate(1000, MSG_LENGTH+1 ); 
    if( motorData.processQ_FR == 0 ) {
        dbgOutputVal(MOTOR_QUEUE_FAIL);
        stopAll(); //ERROR
    }
   
    //Create a timer
    motorData.MotorTimer_FR = xTimerCreate(  
                     "MotorTimer", //Just a text name
                     ( FR_MOTOR_TIMER_RATE / portTICK_PERIOD_MS ), //period in ms
                     pdTRUE, //auto-reload when expires
                     (void *) 25, //a unique id
                     motorTimerCallback ); //pointer to callback function
    
        //Create a timer
    motorData.actuatorTimer_FR = xTimerCreate(  
                     "ActuatorTimer", //Just a text name
                     ( FR_ENCODER_TIMER_RATE / portTICK_PERIOD_MS ), //period in ms
                     pdTRUE, //auto-reload when expires
                     (void *) 47, //a unique id
                     actuatorTimerCallback ); //pointer to callback function
 
    
    //Start the timer
    if( motorData.MotorTimer_FR == NULL ) {
        dbgOutputVal(MOTOR_TIMERINIT_FAIL);
        stopAll();
    }
    else if( xTimerStart( motorData.MotorTimer_FR, 0 ) != pdPASS ) {
        dbgOutputVal(MOTOR_TIMERINIT_FAIL);
        stopAll();
    }
    
        //Start the timer
    if( motorData.actuatorTimer_FR == NULL ) {
        dbgOutputVal(MOTOR_TIMERINIT_FAIL);
        stopAll();
    }
    else if( xTimerStart( motorData.actuatorTimer_FR, 0 ) != pdPASS ) {
        dbgOutputVal(MOTOR_TIMERINIT_FAIL);
        stopAll();
    }
   
        
    // Initialize appropriate pins for input/output for motor control
    
    // RF3 - Direction 1 control (0 - Reverse, 1 - Forward)
    // RC14 = Direction 1 control
    ODCFCLR = 0x8;
    TRISFCLR = 0x8;
    
    // RG1 - Direction 2 control (0 - Reverse, 1 - Forward)
    ODCGCLR = 0x2;
    TRISGCLR = 0x2;
    
    PLIB_PORTS_PinDirectionOutputSet (PORTS_ID_0, PORT_CHANNEL_C, PORTS_BIT_POS_14);
    PLIB_PORTS_PinClear(PORTS_ID_0, PORT_CHANNEL_C, PORTS_BIT_POS_14);
    
    // Used in manual motor enabling
    ODCACLR = 0x4;
    TRISACLR = 0x4;
    
    // Pin for LED control
    ODCGCLR = (1 << 7);
    TRISGCLR = (1 << 7);
    
    pwm_Init();    // Initalize PWM
    stopMotors();  // Begin with motors disabled
    
    motorData.ledState = 0;
    motorData.ledRolls = 0;
    
}

// Finite state machine, runs forever
void MOTOR_Tasks ( void )
{
    switch ( motorData.state )
    {
        case MOTOR_STATE_INIT:
        {
            // Independent direction control enable
            setJP1(SET_HIGH);
            setLeftDirection(SET_HIGH);
            setRightDirection(SET_HIGH);

            motorStop();
            
            motorData.state = MOTOR_LOOP;
            break;
        }
            
        case MOTOR_LOOP:
        {
            // Uh.
            break;
        }

        default: /* The default state should never be executed. */
        {
            motorData.state = MOTOR_LOOP;
            dbgOutputVal(MOTOR_ENTERED_DEFAULT);
            break;
        }

    }//end switch
}
///////////////////////////// CORE ROUTINE ////////////////////////////////////


// Stop rover and don't allow further movement until a start command is given
void motorStop() {
    stopMotors();
    motorData.motorEnable = 0x0;
    sendStatus();
}

// Start/restart rover. Zero key values, begin in search mode, activate First Follow
void motorStart() {
    motorData.tokenQueue = 0;
    motorData.rovState = SEARCH;
    motorData.firstFollow = 1;
    motorData.keep_blinking = 0;
    motorData.tokenCountdown = 0;
    motorData.motorEnable = 0x1;
    pidReset();
    sendStatus();
}

// Determine activity on the LED attached to the rover.
void setLED() {
    
    motorData.ledRolls++; // Tracking for blink enabled functions
    
    // If found a token and blink variable is not zero, blink on every rollover
    if (motorData.tokenCountdown != 0) {
        motorData.ledState = motorData.ledState ^ 1;
        
        if (motorData.keep_blinking != 1) motorData.tokenCountdown--;
    }
    else {
        switch(motorData.rovState) {
            case SEARCH: // Search - LED off
                motorData.ledState = 0;
                break;
            case ACQUIRE_LOCATION:
            case ACQUIRE_TOKEN_LOC: // Acquiring token/rover location - LED blinks quickly
                if (motorData.ledRolls > 3) {
                    motorData.ledRolls = 0;
                    motorData.ledState = motorData.ledState ^ 1;
                }
                break;
            case FOLLOW: // Following lead rover - LED blinks slowly
                if (motorData.ledRolls > 10) {
                    motorData.ledRolls = 0;
                    motorData.ledState = motorData.ledState ^ 1;
                }
                break;
            case TOKEN_LOCK:
            case POINT_LOCK:  // Locked onto position - LED always high
                motorData.ledState = 1;
                break;
        }    
    }
    
    // Set LED pin
    LATGCLR = (1 << 7);
    LATGSET = (motorData.ledState << 7);
}


/*******************************************************************************
 End of File
 */
