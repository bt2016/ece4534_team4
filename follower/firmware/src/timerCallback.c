
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

unsigned int sensorIRTick = 0;
unsigned int sensorDistTick = 0;
unsigned int motorTick = 0;

// Called on Sensor Timer rollover
void sensorIRTimerCallback(TimerHandle_t sTimer){
    sensorIRTick++;
    
    // Error simulation constant - straddle message rate
    if (sensorIRTick % MESSAGE_RATE_DIV == 0)
        readIRReceiverData();  // Read from IR receivers
}

void sensorDistTimerCallback(TimerHandle_t dTimer) {
    sensorDistTick++;
    
    // Error simulation constant - straddle message rate
    if (sensorDistTick % MESSAGE_RATE_DIV == 0)
        PLIB_ADC_SamplingStart(0);  // Sample from sensor
}

// Called on Motor Timer rollover
void motorTimerCallback(TimerHandle_t mTimer) {
    motorTick++;
    
    // Error simulation constant - straddle message rate
    if (motorTick % MESSAGE_RATE_DIV == 0) 
        motorSendToProcessQ(); // Send data to Send task
}

void processTimerCallback(TimerHandle_t pTimer) {
    processSendToMotorQ();
}

// Report message received stats on rollover
void receiveTimerCallback(TimerHandle_t rTimer) {
    if (!CUT_RECEIVE_DATA)
        reportMsgDataToSendQ();
}

void actuatorTimerCallback(TimerHandle_t aTimer) {
    sendMotorControls();
}



/* *****************************************************************************
 End of File
 */
