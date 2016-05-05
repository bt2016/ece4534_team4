/*******************************************************************************
  MPLAB Harmony Application Header File

  Company:
    Microchip Technology Inc.

  File Name:
    motor.h

  Summary:
    This header file provides prototypes and definitions for the application.

  Description:
    This header file provides function prototypes and data type definitions for
    the application.  Some of these are required by the system (such as the
    "APP_Initialize" and "APP_Tasks" prototypes) and some of them are only used
    internally by the application (such as the "APP_STATES" definition).  Both
    are defined here for convenience.
*******************************************************************************/

//DOM-IGNORE-BEGIN
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
//DOM-IGNORE-END

#ifndef _MOTOR_H
#define _MOTOR_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdlib.h>
#include "system_config.h"
#include "system_definitions.h"

#include "queue.h"        //FreeRTOS file
#include "timers.h"       //FreeRTOS file
#include "debug.h"        //Created by me
#include "timerCallback.h"//Created by me
#include "proj_definitions.h"

// DOM-IGNORE-BEGIN
#ifdef __cplusplus  // Provide C++ Compatibility
extern "C" {
#endif
// DOM-IGNORE-END 

#define MAXPID 3392
#define MAXNEG -3392
    
#define PREV_NEITHER 0x0
#define PREV_RIGHT 0x1
#define PREV_LEFT 0x2
    
typedef enum {
    SEARCH = 0,
    FOLLOW,
    ACQUIRE_LOCATION,
    ACQUIRE_TOKEN_LOC,
    TOKEN_LOCK,
    POINT_LOCK,
    DISABLE
} ROV_STATE;

typedef enum
{
	MOTOR_STATE_INIT=0,
    MOTOR_LOOP
} MOTOR_STATES;

typedef struct
{
  int pidIntegral;
  int pidPrevError;
  
  uint8_t frontDistL[DIST_HISTORY];
  uint8_t frontDistR[DIST_HISTORY];
  
  int leftLostPos;
  int rightLostPos;
  
  int prevDir;
  int prevFrontSignal;

  unsigned int leftSignal;
  unsigned int rightSignal;
  unsigned int frontSignal;
  unsigned int rearSignal;
  unsigned int bestSignal;
    
} LEAD_POS;


typedef struct
{
    char tokenMode;
    char foundTokenNoAck;
    unsigned short int tokenFindSendCount;
    
    unsigned short int motorSendCount;
    unsigned short int IRsendCount;
    
    /* The application's current state */
    MOTOR_STATES state;
    unsigned short int sendCount;
    char motorEnable;
    
    int leftSpd;
    int rightSpd;
    int leftIdeal;
    int rightIdeal;
    char leftDir;
    char rightDir;

    /* TODO: Define any additional data used by the application. */
    QueueHandle_t MotorQ_FR;
    QueueHandle_t actuatorQ_FR;
    TimerHandle_t MotorTimer_FR;
    TimerHandle_t actuatorTimer_FR;
    
    QueueHandle_t leftEncoderQ;
    QueueHandle_t rightEncoderQ;
    
    //TYPEA
    QueueHandle_t processQ_FR;
    
    char prevType;
    char prevCount;
    int keep_blinking;
    
    int rpid_Integral;
    int rpid_PrevError;
    
    int lpid_Integral;
    int lpid_PrevError;
    
    int lpid_KP;
    int lpid_KI;
    int lpid_KD;
    
    int rpid_KP;
    int rpid_KI;
    int rpid_KD;
    
    int pidRightSpd;
    int pidLeftSpd;
    int pidPrevRight;
    int pidPrevLeft;
    
    int ticksToToken;
    int ticksToTokenLock;
    char tokenQueue;
    int ticksToTarget;
    ROV_STATE rovState;
    
    int curEncoder;
    int prevEncoder;
    
    
    char prevTokenSequence;
    char prevTokenHigh;
    char prevTokenLow;
    int tokenCountdown;

    int ledState;
    int ledRolls;
    
    uint8_t tokenFindCount;
    uint8_t firstFollow;
    
    char roundedTurn;
    char stopOnToken;
    
    int timeSinceVanish;
    int timeToToken;
    
    char sendIRData;
    char sendTargetData;
    char sendPID;
    char sendEncoder;
   
} MOTOR_DATA;

void MOTOR_Initialize ( void );
void MOTOR_Tasks( void );


#endif /* _MOTOR_H */

//DOM-IGNORE-BEGIN
#ifdef __cplusplus
}
#endif
//DOM-IGNORE-END

/*******************************************************************************
 End of File
 */

