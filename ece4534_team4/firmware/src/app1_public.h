/* ************************************************************************** */
/** 
  @Project
     ECE 4534 Embedded Design
     Team 4 - 11:00 TR
 
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
    
void sendTimerValToMsgQ(unsigned int* sendms);
void sendValFromISR(unsigned int* message);

/* Provide C++ Compatibility */
#ifdef __cplusplus
}
#endif

#endif //APP1_PUBLIC_H

/* *****************************************************************************
 End of File
 */
