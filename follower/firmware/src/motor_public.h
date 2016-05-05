/* ************************************************************************** */
/** Descriptive File Name

  @Company
    Company Name

  @File Name
    filename.h

  @Summary
    Brief description of the file.

  @Description
    Describe the purpose of this file.
 */
/* ************************************************************************** */

#ifndef _MOTOR_PUBLIC_H    /* Guard against multiple inclusion */
#define _MOTOR_PUBLIC_H


/* Provide C++ Compatibility */
#ifdef __cplusplus
extern "C" {
#endif

// Interface functions
//void putDataOnMotorQ(char* data);
void putDataOnMotorProcessQ(char* data);
void motorSendToMsgQ();
void motorStart();
void motorStop();
void setLED();
void toggleTurn();
void toggleStopOnToken();
void setOutputs(char IR, char TD, char ENC, char PID, char turn, char token);
                                
void testTokenFound();

    /* Provide C++ Compatibility */
#ifdef __cplusplus
}
#endif

#endif /* _EXAMPLE_FILE_NAME_H */

/* *****************************************************************************
 End of File
 */
