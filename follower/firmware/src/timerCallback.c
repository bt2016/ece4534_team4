
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

unsigned int sensorIRTick = 0;
unsigned int sensorDistTick = 0;
unsigned int motorTick = 0;
unsigned int ackTick = 0;


// Read analog inputs - IR distance sensor and IR receivers
// Start auto sample to scan and trigger INTERRUPT
void sensorDistTimerCallback(TimerHandle_t dTimer) {
    sensorDistTick++;
    
    // Error simulation constant - straddle message rate
    if (sensorDistTick % MESSAGE_RATE_DIV == 0) {
        PLIB_ADC_SampleAutoStartEnable(0);
    }
}

// Called on Motor Timer rollover
// Activate motor control read & Motor variable calculation
void motorTimerCallback(TimerHandle_t mTimer) {
    ackTick++;
    
    processMotorControls();
    
    if (ackTick % 2) {
        sendTokenFoundMsg();
    }
}

// Report message received stats on rollover (debug)
void receiveTimerCallback(TimerHandle_t rTimer) {
    if (!CUT_RECEIVE_DATA)
        reportMsgDataToSendQ();
}

// Activate encoder reading
void actuatorTimerCallback(TimerHandle_t aTimer) {
    motorTick++;
    
    motorReadEncoders(); // Send data to Send task
    setLED();
}

/* *****************************************************************************
 End of File
 */
