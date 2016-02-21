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

#ifndef _SENDER_H    /* Guard against multiple inclusion */
#define _SENDER_H


/* ************************************************************************** */
/* ************************************************************************** */
/* Section: Included Files                                                    */
/* ************************************************************************** */
/* ************************************************************************** */

/* This section lists the other files that are included in this file.
 */

/* TODO:  Include other files here if needed. */


/* Provide C++ Compatibility */
#ifdef __cplusplus
extern "C" {
#endif
    
    #include "send.h"
    #include "proj_definitions.h"

    void writeString(char* sendval);
    void writeMsgStr(char type, char count, char* sendval);
    void writeMsgChar(char type, char count, char dataa, char datab, char datac, char datad, char datae, char dataf);
    void writeMsgShortInt(char type, char count, unsigned int data);
    void writeMESSAGE(MESSAGE message);

    /* Provide C++ Compatibility */
#ifdef __cplusplus
}
#endif

#endif /* _SENDER_H */

/* *****************************************************************************
 End of File
 */
