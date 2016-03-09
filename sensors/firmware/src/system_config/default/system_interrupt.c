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
#include "process.h"
#include "send.h"
#include "irarray.h"
#include "sensor.h"
#include "receive.h"
#include "system_definitions.h"

// *****************************************************************************
// *****************************************************************************
// Section: System Interrupt Vector Functions
// *****************************************************************************
// *****************************************************************************

void IntHandlerDrvUsartInstance0(void)
{
    /* TODO: Add code to process interrupt here */
    if (!DRV_USART0_ReceiverBufferIsEmpty()){
        char incomingByte = DRV_USART0_ReadByte();
        
        receiveSendValFromISR(&incomingByte);
        
        PLIB_INT_SourceFlagClear(INT_ID_0, INT_SOURCE_USART_1_RECEIVE);        
    }
 
    // If transmitter not enabled, interrupt never occurs here (messages still go through over wifly)
    // Transmitter enable -> Empty Flag up      
    if (PLIB_USART_TransmitterIsEmpty(USART_ID_1)) {
        receiveDataFromISR();
    }
    
    PLIB_INT_SourceFlagClear(INT_ID_0, INT_SOURCE_USART_1_TRANSMIT);
    PLIB_INT_SourceFlagClear(INT_ID_0, INT_SOURCE_USART_1_ERROR);
}
 
void IntHandlerDrvAdc(void)
{
    //TODO: make sure this corresponds to the Harmony config!!!
    int numberSamplesPerInterrupt = 10;
        
    //clear the interrupt flag
    PLIB_INT_SourceFlagClear(INT_ID_0, INT_SOURCE_ADC_1);
    
    //get the ADC value
    unsigned int runningSum = 0;
    int i = 0;
    for(i=0; i<numberSamplesPerInterrupt; i++)
        runningSum += PLIB_ADC_ResultGetByIndex(ADC_ID_1, i);
    double averageADCVal = (double)runningSum/(double)numberSamplesPerInterrupt; //the output is a 16-bit int
    
    //convert ADC steps to distance in cm
    averageADCVal = (averageADCVal/(double)1024)*5; //convert ADC val to voltage
    unsigned int distance = ((16.211*averageADCVal*averageADCVal*averageADCVal*averageADCVal) - (127.77*averageADCVal*averageADCVal*averageADCVal) + (371.33*averageADCVal*averageADCVal) - (494.66*averageADCVal) + 297.73);
    
    //send the value to the queue
    sendValToSensorTaskFromISR(&distance);
}
 
 
/*
     if (PLIB_USART_TransmitterIsEmpty(USART_ID_1)) {
        //stopAll(); //Yes
        receiveDataFromMsgQ();
        //stopAll(); //No
        PLIB_INT_SourceDisable(INT_ID_0, INT_SOURCE_USART_1_TRANSMIT);   
    }
*/
 
 
  
/*******************************************************************************
 End of File
*/

