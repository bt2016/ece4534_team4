
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
unsigned int motorTick = 0;

// Called from Send Timer rollover
void vTimerCallback(TimerHandle_t pxTimer) {
    sendms += 1; 
}

// Distribute messages to internal task queues
void receiveMotorCallback(TimerHandle_t dTimer) {
    motorTick++;
    
    // Error simulation constant - straddle message rate
    if (motorTick % MESSAGE_RATE_DIV == 0)
        sendLRMotorInstruction();
}

// Report message receive statistics to send queue
void receiveTimerCallback(TimerHandle_t rTimer) {
     if (!CUT_RECEIVE_DATA)
        reportMsgDataToSendQ();
}


/* *****************************************************************************
 End of File
 */
