
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
#include "process_public.h"
#include "send_public.h"
#include "proj_definitions.h"

unsigned int sendms = 0;
unsigned int IRSensorTick = 0;
unsigned int distSensorTick = 0;
unsigned int motorTick = 0;
unsigned int processTick = 0;

// Called from Send Timer rollover
void vTimerCallback(TimerHandle_t pxTimer) {
    //sendms += 1; 
}

void sendTimerCallback(TimerHandle_t sTimer){
    sendStoredMessageToTransmitQ();
}

// Called on Sensor Timer rollover
void sensorTimerCallback(TimerHandle_t sTimer){
    distSensorTick++;
    
    
    // Error simulation constant - straddle message rate
    if (distSensorTick % MESSAGE_RATE_DIV == 0){

        //PLIB_ADC_SamplingStart(0);  // Sample from sensor
    }
}

void servoMovementTimerCallback(TimerHandle_t sTimer){
    DRV_ADC_Start(); //Sample from sensor
}

// Called on Sensor Timer rollover
void IRArrayTimerCallback(TimerHandle_t sTimer){
    IRSensorTick++;
    
    // Error simulation constant - straddle message rate
    if (IRSensorTick % MESSAGE_RATE_DIV == 0)
        PLIB_ADC_SamplingStart(1);  // Sample from sensor
}


// Report message received stats on rollover
void receiveTimerCallback(TimerHandle_t rTimer) {
    if (!CUT_RECEIVE_DATA)
        reportMsgDataToSendQ();
}

void processTimerCallback(TimerHandle_t aTimer) {
    processTick++;
    if (processTick % MESSAGE_RATE_DIV == 0){}
        //sendProcessedData();
    //sendRequestForUpdateToProcessTask();
    
}


/* *****************************************************************************
 End of File
 */
