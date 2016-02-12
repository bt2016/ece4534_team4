/* ************************************************************************** */
/** Descriptive File Name
  @File Name
    app1_public.h

  @Description
    This file contains public routines for interacting with the app's queue
 */
/* ************************************************************************** */

#ifndef APP1_PUBLIC_H    /* Guard against multiple inclusion */
#define APP1_PUBLIC_H

/* Provide C++ Compatibility */
#ifdef __cplusplus
extern "C" {
#endif
   
void app1SendTimerValToMsgQ(unsigned int* millisecondsElapsed);
void app1SendValFromISR(unsigned int* message);

/* Provide C++ Compatibility */
#ifdef __cplusplus
}
#endif

#endif //APP1_PUBLIC_H

/* *****************************************************************************
 End of File
 */
