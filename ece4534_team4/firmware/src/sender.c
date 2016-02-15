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
/* ************************************************************************** */

/* ************************************************************************** */
/* ************************************************************************** */
/* Section: Included Files                                                    */
/* ************************************************************************** */
/* ************************************************************************** */

/* This section lists the other files that are included in this file.
 */

#include "sender.h"

/* ************************************************************************** */
/* ************************************************************************** */
/* Section: File Scope or Global Data                                         */
/* ************************************************************************** */
/* ************************************************************************** */

/*  A brief description of a section can be given directly below the section
    banner.
 */

/* ************************************************************************** */
/** Descriptive Data Item Name

  @Summary
    Brief one-line summary of the data item.
    
  @Description
    Full description, explaining the purpose and usage of data item.
    <p>
    Additional description in consecutive paragraphs separated by HTML 
    paragraph breaks, as necessary.
    <p>
    Type "JavaDoc" in the "How Do I?" IDE toolbar for more information on tags.
    
  @Remarks
    Any additional remarks
 */
int global_data;


/* ************************************************************************** */
/* ************************************************************************** */
// Section: Local Functions                                                   */
/* ************************************************************************** */
/* ************************************************************************** */

/*  A brief description of a section can be given directly below the section
    banner.
 */

/* ************************************************************************** */

/** 
  @Function
    int ExampleLocalFunctionName ( int param1, int param2 ) 

  @Summary
    Brief one-line description of the function.

  @Description
    Full description, explaining the purpose and usage of the function.
    <p>
    Additional description in consecutive paragraphs separated by HTML 
    paragraph breaks, as necessary.
    <p>
    Type "JavaDoc" in the "How Do I?" IDE toolbar for more information on tags.

  @Precondition
    List and describe any required preconditions. If there are no preconditions,
    enter "None."

  @Parameters
    @param param1 Describe the first parameter to the function.
    
    @param param2 Describe the second parameter to the function.

  @Returns
    List (if feasible) and describe the return values of the function.
    <ul>
      <li>1   Indicates an error occurred
      <li>0   Indicates an error did not occur
    </ul>

  @Remarks
    Describe any special behavior not described above.
    <p>
    Any additional remarks.

  @Example
    @code
    if(ExampleFunctionName(1, 2) == 0)
    {
        return 3;
    }
 */
void writeString(char* sendvalue) {
    char *k;
    for (k = sendvalue; *k; ++k) {
        DRV_USART0_WriteByte(*k);
    }
}

void writeMsgStr(char count, char* sendvalue) {
    char *k;
    
    DRV_USART0_WriteByte(MSG_START);
    DRV_USART0_WriteByte(count);
    
    for (k = sendvalue; *k; ++k) {
        DRV_USART0_WriteByte(*k);
    }
    
    DRV_USART0_WriteByte(MSG_STOP);
}

void writeMsgChar(char type, char count, char dataa, char datab) {
    DRV_USART0_WriteByte(MSG_START);
    DRV_USART0_WriteByte(type);
    DRV_USART0_WriteByte(count);
    DRV_USART0_WriteByte(dataa);
    DRV_USART0_WriteByte(datab);
    DRV_USART0_WriteByte(MSG_STOP);
}

void writeMsgShortInt(char type, char count, unsigned short int data) {
    DRV_USART0_WriteByte(MSG_START);
    DRV_USART0_WriteByte(type);
    DRV_USART0_WriteByte(count);
    DRV_USART0_WriteByte( data >> 8);
    DRV_USART0_WriteByte( data );
    DRV_USART0_WriteByte(MSG_STOP);
}

void writeMESSAGE(MESSAGE message) {
    DRV_USART0_WriteByte(message.start);
    DRV_USART0_WriteByte(message.type);
    DRV_USART0_WriteByte(message.count);
    writeString(message.data);
    DRV_USART0_WriteByte(message.stop);
}



/* *****************************************************************************
 End of File
 */
