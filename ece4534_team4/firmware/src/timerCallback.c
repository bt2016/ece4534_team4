
/* ************************************************************************** */
/** 
  @Project
     ECE 4534 Embedded Design
     Team 4 - 11:00 TR

  @File Name
    filename.c

  @Summary
    Brief description of the file.

  @Description
    Describe the purpose of this file.
 */

#include "timerCallback.h"
#include "proj_definitions.h"

unsigned int sendms = 0;
unsigned int sensorTick = 0;
unsigned int motorTick = 0;

// Called from Send Timer rollover
void vTimerCallback(TimerHandle_t pxTimer) {
    sendms += 1; 
}

// Called on Sensor Timer rollover
void sensorTimerCallback(TimerHandle_t sTimer){
    sensorTick++;
    
    // Error simulation constant - straddle message rate
    if (sensorTick % MESSAGE_RATE_DIV == 0)
        PLIB_ADC_SamplingStart(0);  // Sample from sensor
}

// Called on Motor Timer rollover
void motorTimerCallback(TimerHandle_t mTimer) {
    motorTick++;
    
    // Error simulation constant - straddle message rate
    if (motorTick % MESSAGE_RATE_DIV == 0) 
        motorSendToMsgQ(); // Send data to Send task
}

// Report message received stats on rollover
void distributeCallback(TimerHandle_t dTimer) {
    if (!CUT_RECEIVE_DATA)
        reportMsgDataToSendQ();
}

// Called on Receiver Timer rollover
void receiveTimerCallback(TimerHandle_t rTimer) {
    receiveSendToMotorQ(); // Send dummy data to motor
}


/* *****************************************************************************
 End of File
 */
