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

void pwmTimerCallback(TimerHandle_t pxTimer);

/* Provide C++ Compatibility */
#ifdef __cplusplus
}
#endif

#endif /* TIMER_CALLBACK_H */

/* *****************************************************************************
 End of File
 */
