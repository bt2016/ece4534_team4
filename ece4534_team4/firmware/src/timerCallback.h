/* ************************************************************************** */
/** 
  @Project
     ECE 4534 Embedded Design
     Team 4 - 11:00 TR

  @File Name
    filename.h

  @Summary
    Brief description of the file.

  @Description
    Describe the purpose of this file.
 */
/* ************************************************************************** */

#ifndef TIMER_CALLBACK_H    /* Guard against multiple inclusion */
#define TIMER_CALLBACK_H

#include <FreeRTOS.h>
#include <timers.h>

/* Provide C++ Compatibility */
#ifdef __cplusplus
extern "C" {
#endif

void vTimerCallback(TimerHandle_t pxTimer);
void motorTimerCallback(TimerHandle_t mTimer);
void sendToMotorQ();
void motorSendToMsgQ();
void motorReceiveFromMsgQ();
void sensorTimerCallback(TimerHandle_t sTimer);
void receiveTimerCallback(TimerHandle_t rTimer);
void distributeCallback(TimerHandle_t dTimer);

/* Provide C++ Compatibility */
#ifdef __cplusplus
}
#endif

#endif /* TIMER_CALLBACK_H */

/* *****************************************************************************
 End of File
 */
