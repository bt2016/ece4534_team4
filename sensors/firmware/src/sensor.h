/*******************************************************************************
  MPLAB Harmony Application Header File

  Company:
    Microchip Technology Inc.

  File Name:
    sensor.h

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

#ifndef _SENSOR_H
#define _SENSOR_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdlib.h>
#include "system_config.h"
#include "system_definitions.h"

#include "queue.h"        //FreeRTOS file
#include "timers.h"       //FreeRTOS file
#include "sensor_public.h"//Created by me
#include "debug.h"        //Created by me
#include "timerCallback.h"//Created by me

// DOM-IGNORE-BEGIN
#ifdef __cplusplus  // Provide C++ Compatibility
extern "C" {
#endif
// DOM-IGNORE-END 

//Defines for moving servo
#define SERVOANGLE_MIN 80
#define SERVOANGLE_MAX 170
#define SERVO_DEGREES 90
    
//Defines for interpreting sensor data
#define SERVO_MAXRANGE_CM 100
#define LINE_MINDELTA_CM  20
#define LINE_MINLENGTH_CM 4
    
//Defines for debugging
//#define SENSOR_DEBUG_ISOLATESENSOR
#define SENSOR_DEBUG_FULLMAP

typedef enum
{
	SENSOR_STATE_INIT=0,
    SENSOR_STATE_TAKEREADINGS=1,
    SENSOR_STATE_FINDOBSTACLES=2,
	SENSOR_STATE_READ=3,
} SENSOR_STATES;

typedef struct
{
    SENSOR_STATES state;
	QueueHandle_t sensorQ_SA;
	TimerHandle_t sensorDistTimer_SA;
    TimerHandle_t servoMovementTimer_SA;
    TimerHandle_t takeAdcReadingTimer_SA;
    unsigned int senseCount;
    unsigned short int sendCount;
    int sendToSensorQ_Err;
    
    int r[90]; //stores polar coordinate points where the index is the angle, and the value is the radius
    int servo_angle;

} SENSOR_DATA;

void SENSOR_Initialize ( void );
void SENSOR_Tasks( void );

//helper functions
void sendValToSensorTask(unsigned int* message);
uint8_t removeSensorQueueData();
void sendValToSensorTaskFromISR(unsigned int* message);
void setServoAngle(int angle);
void incrementServo();
void startServoMovementTimer();

#endif /* _SENSOR_H */

//DOM-IGNORE-BEGIN
#ifdef __cplusplus
}
#endif
//DOM-IGNORE-END

/*******************************************************************************
 End of File
 */

