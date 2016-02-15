/* ************************************************************************** */
/** Descriptive File Name
  @File Name
    sensor_public.h

  @Description
    This file contains public routines for interacting with the app's queue
 */
/* ************************************************************************** */

#ifndef SENSOR_PUBLIC_H    /* Guard against multiple inclusion */
#define SENSOR_PUBLIC_H

/* Provide C++ Compatibility */
#ifdef __cplusplus
extern "C" {
#endif
    
void sendValToSensorTask(unsigned int* message);
void sendValToSensorTaskFromISR(unsigned int* message);

/* Provide C++ Compatibility */
#ifdef __cplusplus
}
#endif

#endif //SENSOR_PUBLIC_H

/* *****************************************************************************
 End of File
 */
