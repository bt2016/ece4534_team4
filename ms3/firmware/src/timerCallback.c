
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

unsigned int sendms = 0;
unsigned int sensorTick = 0;
unsigned int motorTick = 0;

void pwmTimerCallback(TimerHandle_t pxTimer) {
    sendms++;
    
    if (sendms & 0x1) {
        LATESET = 0x10;
    }
    else {
        LATECLR = 0x10;
    }
}

/* *****************************************************************************
 End of File
 */
