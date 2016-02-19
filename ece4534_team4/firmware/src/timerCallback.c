
/* ************************************************************************** */
/** Descriptive File Name

  @Company
    Company Name

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
unsigned int receiveTick = 0;

// Called from App Timer rollover
void vTimerCallback(TimerHandle_t pxTimer) {
    sendms += 1; //Timer is called every 100ms
    /*
    if (sendms % 600 == 0) {
        writeString("TIME");
    }
     */
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

// Called on Receiver Timer rollover
void receiveTimerCallback(TimerHandle_t rTimer) {
    receiveTick++;
    
    if (receiveTick % 8 == 0) 
        reportMsgData();
        
    receiveSendToMsgQ(); // Send dummy data to motor
}


/* *****************************************************************************
 End of File
 */
