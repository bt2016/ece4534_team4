
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
    //sendTimerValToMsgQ(sendms);
    checkSourceQ();
    //sendWriteMessage();
}

void motorTimerCallback(TimerHandle_t mTimer) {
    motorSendTimerValToMsgQ(sendms);
}


/* *****************************************************************************
 End of File
 */
