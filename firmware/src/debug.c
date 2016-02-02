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
#include "debug.h"

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
static int ExampleLocalFunction(int param1, int param2) {
    return 0;
}

/*
 * This function sends the 8 bits of unsigned char outVal to 8 I/O lines
 * on the pic. Be sure to check that the harmony configuration has these pins
 * as outputs, latched low.
     * Pin 30 -> Bit position 7. Pic32 pin 5.  Signal E7 
     * Pin 31 -> Bit position 6. Pic32 pin 4.  Signal E6
     * Pin 32 -> Bit position 5. Pic32 pin 3.  Signal E5
     * Pin 33 -> Bit position 4. Pic32 pin 100.Signal E4
     * Pin 34 -> Bit position 3. Pic32 pin 99. Signal E3
     * Pin 35 -> Bit position 2. Pic32 pin 98. Signal E2
     * Pin 36 -> Bit position 1. Pic32 pin 94. Signal E1
     * Pin 37 -> Bit position 0. Pic32 pin 93. Signal E0     
*/
int dbgOutputVal(unsigned char outVal){
    //NOTE: Check out SYS_PORTS 
    LATECLR = 0xff;
    LATESET = outVal;
}

//Stops everything else from running and blinks the LED
// in FreeRTOS look up vTaskAllSuspend or something like that
void stopAll(){
    while(1){
        LATASET = 1 << 3;
        int i = 0;
        for(i=0; i<10000000; i++);
        LATACLR = 1 << 3;
        for(i=0; i<10000000; i++);
    }
}


/* *****************************************************************************
 End of File
 */
