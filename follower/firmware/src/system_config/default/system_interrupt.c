/*******************************************************************************
 System Interrupts File

  File Name:
    system_int.c

  Summary:
    Raw ISR definitions.

  Description:
    This file contains a definitions of the raw ISRs required to support the
    interrupt sub-system.

  Summary:
    This file contains source code for the interrupt vector functions in the
    system.

  Description:
    This file contains source code for the interrupt vector functions in the
    system.  It implements the system and part specific vector "stub" functions
    from which the individual "Tasks" functions are called for any modules
    executing interrupt-driven in the MPLAB Harmony system.

  Remarks:
    This file requires access to the systemObjects global data structure that
    contains the object handles to all MPLAB Harmony module objects executing
    interrupt-driven in the system.  These handles are passed into the individual
    module "Tasks" functions to identify the instance of the module to maintain.
 *******************************************************************************/

// DOM-IGNORE-BEGIN
/*******************************************************************************
Copyright (c) 2011-2014 released Microchip Technology Inc.  All rights reserved.

Microchip licenses to you the right to use, modify, copy and distribute
Software only when embedded on a Microchip microcontroller or digital signal
controller that is integrated into your product or third party product
(pursuant to the sublicense terms in the accompanying license agreement).

You should refer to the license agreement accompanying this Software for
additional information regarding your rights and obligations.

SOFTWARE AND DOCUMENTATION ARE PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
EITHER EXPRESS OR IMPLIED, INCLUDING WITHOUT LIMITATION, ANY WARRANTY OF
MERCHANTABILITY, TITLE, NON-INFRINGEMENT AND FITNESS FOR A PARTICULAR PURPOSE.
IN NO EVENT SHALL MICROCHIP OR ITS LICENSORS BE LIABLE OR OBLIGATED UNDER
CONTRACT, NEGLIGENCE, STRICT LIABILITY, CONTRIBUTION, BREACH OF WARRANTY, OR
OTHER LEGAL EQUITABLE THEORY ANY DIRECT OR INDIRECT DAMAGES OR EXPENSES
INCLUDING BUT NOT LIMITED TO ANY INCIDENTAL, SPECIAL, INDIRECT, PUNITIVE OR
CONSEQUENTIAL DAMAGES, LOST PROFITS OR LOST DATA, COST OF PROCUREMENT OF
SUBSTITUTE GOODS, TECHNOLOGY, SERVICES, OR ANY CLAIMS BY THIRD PARTIES
(INCLUDING BUT NOT LIMITED TO ANY DEFENSE THEREOF), OR OTHER SIMILAR COSTS.
 *******************************************************************************/
// DOM-IGNORE-END


// *****************************************************************************
// *****************************************************************************
// Section: Included Files
// *****************************************************************************
// *****************************************************************************

#include <xc.h>
#include <sys/attribs.h>
#include "receive.h"
#include "send.h"
#include "motor.h"
#include "sensor.h"
#include "system_definitions.h"

// *****************************************************************************
// *****************************************************************************
// Section: System Interrupt Vector Functions
// *****************************************************************************
// *****************************************************************************




void IntHandlerDrvUsartInstance0(void)
{
    /* TODO: Add code to process interrupt here */
    //if (!DRV_USART0_ReceiverBufferIsEmpty()){
                //stopAll();
        char incomingByte = DRV_USART0_ReadByte();
        receiveSendValFromISR(&incomingByte);      
   // }
 
    // If transmitter not enabled, interrupt never occurs here (messages still go through over wifly)
    // Transmitter enable -> Empty Flag up      
    if (PLIB_USART_TransmitterIsEmpty(USART_ID_1)) {
        receiveDataFromISR();
    }
    
    PLIB_INT_SourceFlagClear(INT_ID_0, INT_SOURCE_USART_1_RECEIVE);  
    PLIB_INT_SourceFlagClear(INT_ID_0, INT_SOURCE_USART_1_TRANSMIT);
    PLIB_INT_SourceFlagClear(INT_ID_0, INT_SOURCE_USART_1_ERROR);
}
 
void IntHandlerDrvAdc(void)
{
    DRV_ADC_Stop();
    //TODO: make sure this corresponds to the Harmony config!!!
    int numberSamplesPerInterrupt = 3;
        
    //clear the interrupt flag
    PLIB_INT_SourceFlagClear(INT_ID_0, INT_SOURCE_ADC_1);
    //get the ADC value
    unsigned int potValue = 0;
    unsigned int potValue1 = 0;
    unsigned int potValue2 = 0;
    unsigned int potValue3 = 0;
    unsigned int potValue4 = 0;
    int i = 0;
    
        for(i=0;i<5;i+=5)
    {
        potValue4 += DRV_ADC_SamplesRead(i);
        potValue += DRV_ADC_SamplesRead(i+1);
        potValue1 += DRV_ADC_SamplesRead(i+2);
        potValue3 += DRV_ADC_SamplesRead(i+3);
        potValue2 += DRV_ADC_SamplesRead(i+4);
    }
    /*
    for(i=0; i<numberSamplesPerInterrupt; i += 3) {
        //potValue += PLIB_ADC_ResultGetByIndex(ADC_ID_1, i);
        potValue += DRV_ADC_SamplesRead(i);
        potValue1 += DRV_ADC_SamplesRead(i+1);
        potValue2 += DRV_ADC_SamplesRead(i+2);
    }
     * */
    //potValue = potValue/numberSamplesPerInterrupt; //the output is a 16-bit int
    
    unsigned int distance;
    
    //convert ADC steps to distance in cm
    // distance = (unsigned int) (63.404-((double)potValue*0.058)); //cm
    // distance = (unsigned int) (24.952-((double)potValue*0.0227)); //in
    
    //distance = (unsigned int) potValue;
    //distance = distance & 0xFF;
    
    // DIST 
    if (potValue < 89)
        distance = 40;
    else
        distance = (unsigned int) (3530.0 / ((double)potValue));
    
    if (potValue2 < 833) potValue2 = 0;
    else potValue2 = potValue2 - 832;

    char data[MSG_LENGTH];
    data[0] = MSG_START;          // Start byte
    data[1] = TYPE_ADC;               // Type byte
    data[2] = 0x20;              // Count byte
    data[3] = 0x10;                // Sensor value (redundant)
    data[4] = (potValue1 & 0x03FC) >> 2;  //FRONT
    data[5] = (potValue2 & 0x03FC) >> 2;  //REAR
    data[6] = (potValue3 & 0x03FC) >> 2;  //RIGHT
    data[7] = (potValue4 & 0x03FC) >> 2;  //LEFT
    data[8] = distance & 0xFF;                 // Left motor speed
    data[9] = MSG_STOP;           // Stop byte
    
    if (SEND_ADC == 0x1)
       putMsgOnSendQueue(data);
 
    //send the value to the queue
    sendValToSensorTaskFromISR(data);
    
    //PLIB_ADC_SampleAutoStartEnable(ADC_ID_1);
    
}

 
/*******************************************************************************
 End of File
*/

