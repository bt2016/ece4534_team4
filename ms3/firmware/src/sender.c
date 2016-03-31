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

int global_data;


/* ************************************************************************** */
/* ************************************************************************** */
// Section: Local Functions                                                   */
/* ************************************************************************** */
/* ************************************************************************** */

void writeString(char* sendvalue) {
    char *k;
    for (k = sendvalue; *k; ++k) {
        DRV_USART0_WriteByte(*k);
    }
}

void writeMsgStr(char type, char count, char* sendvalue) {
    char *k;
    
    DRV_USART0_WriteByte(MSG_START);
    DRV_USART0_WriteByte(type);
    DRV_USART0_WriteByte(count);
    
    for (k = sendvalue; *k; ++k) {
        DRV_USART0_WriteByte(*k);
    }
    
    DRV_USART0_WriteByte(MSG_STOP);
}

void writeMsgChar(char type, char count, char dataa, char datab, char datac, char datad, char datae, char dataf) {
    DRV_USART0_WriteByte(MSG_START);
    DRV_USART0_WriteByte(type);
    DRV_USART0_WriteByte(count);
    DRV_USART0_WriteByte(dataa);
    DRV_USART0_WriteByte(datab);
    DRV_USART0_WriteByte(datac);
    DRV_USART0_WriteByte(datad);
    DRV_USART0_WriteByte(datae);
    DRV_USART0_WriteByte(dataf);
    DRV_USART0_WriteByte(MSG_STOP);
}

void writeMsgShortInt(char type, char count, unsigned int data) {
    DRV_USART0_WriteByte(MSG_START);
    DRV_USART0_WriteByte(type);
    DRV_USART0_WriteByte(count);
    DRV_USART0_WriteByte( 0x20);
    DRV_USART0_WriteByte( 0x20);
    DRV_USART0_WriteByte( data >> 24);
    DRV_USART0_WriteByte( data >> 16);
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
