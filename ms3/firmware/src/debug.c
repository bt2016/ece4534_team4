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

#include "debug.h"

int global_data;

/*
 * This function sends the 8 bits of unsigned char outVal to 8 I/O lines
 * on the pic. Be sure to check that the harmony configuration has these pins
 * as outputs, latched low.
     * Pin 28 -> Bit position 9. Pic32 pin 01. Signal G15
     * Pin 29 -> Bit position 8. Pic32 pin 11. Signal G7
     * Pin 30 -> Bit position 7. Pic32 pin 05. Signal E7 
     * Pin 31 -> Bit position 6. Pic32 pin 04. Signal E6
     * Pin 32 -> Bit position 5. Pic32 pin 03. Signal E5
     * Pin 33 -> Bit position 4. Pic32 pin 100.Signal E4
     * Pin 34 -> Bit position 3. Pic32 pin 99. Signal E3
     * Pin 35 -> Bit position 2. Pic32 pin 98. Signal E2
     * Pin 36 -> Bit position 1. Pic32 pin 94. Signal E1
     * Pin 37 -> Bit position 0. Pic32 pin 93. Signal E0     
*/
int dbgOutputVal(unsigned int outVal){
    LATECLR = 0xff;
    LATGCLR = 0xff;
    LATGCLR = (1<<15);
    LATESET = outVal;
    if (outVal & 0x0100) LATGSET = (1<<7);
    if (outVal & 0x0200) LATGSET = (1<<15);
}

//Stops everything else from running and blinks the LED
void stopAll()
{
    vTaskSuspendAll();
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
