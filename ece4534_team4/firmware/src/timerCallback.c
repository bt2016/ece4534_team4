
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

unsigned int sendms = 0;

void vTimerCallback(TimerHandle_t pxTimer) {
    sendms += 1; //Timer is called every 100ms
    if (sendms % 600 == 0) {
        writeString("TIME");
    }
}

void sensorTimerCallback(TimerHandle_t sTimer){
    PLIB_ADC_SamplingStart(0);
}

void motorTimerCallback(TimerHandle_t mTimer) {
    motorSendToMsgQ();
}

void receiveTimerCallback(TimerHandle_t rTimer) {
    receiveSendToMsgQ();
}


/* *****************************************************************************
 End of File
 */
