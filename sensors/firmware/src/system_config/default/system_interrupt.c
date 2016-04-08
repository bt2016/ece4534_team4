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
    //clear the interrupt flag
    PLIB_INT_SourceFlagClear(INT_ID_0, INT_SOURCE_ADC_1);
    
    unsigned int ADCVal; //raw output from the adc (0-1023)
    double ADCVoltage;   //calculated voltage on the adc pin
    double distance;     //converting voltage to distance based on datasheet specs
    unsigned int retval; //rounded value to put o the queue
    int i; //iterator
    
    for (i=0; i<8; i++){
        ADCVal = PLIB_ADC_ResultGetByIndex(ADC_ID_1, i);
        ADCVoltage = (ADCVal/(double)1024)*3.3;
        distance = ((16.211*ADCVoltage*ADCVoltage*ADCVoltage*ADCVoltage) - (127.77*ADCVoltage*ADCVoltage*ADCVoltage) + (371.33*ADCVoltage*ADCVoltage) - (494.66*ADCVoltage) + 297.73);
        if (distance > SERVO_MAXRANGE_CM) distance = 0;
        retval = (unsigned int)(distance+0.5);
        sendValToSensorTaskFromISR(&retval);
    }
    
        
    /*
    //get the ADC value   
    unsigned int ADCVal = PLIB_ADC_ResultGetByIndex(ADC_ID_1, 0);
    
    //convert ADC steps to distance in cm
    double ADCVoltage = (ADCVal/(double)1024)*3.3; //convert ADC val to voltage
    double distance = ((16.211*ADCVoltage*ADCVoltage*ADCVoltage*ADCVoltage) - (127.77*ADCVoltage*ADCVoltage*ADCVoltage) + (371.33*ADCVoltage*ADCVoltage) - (494.66*ADCVoltage) + 297.73);
    
    //define maximum sensor distance
    if (distance >90) distance = 0;
    
    //ensure accurate rounding when casting to unsigned int
    unsigned int retval = (unsigned int)(distance+0.5);
    
    //send the value to the queue
    sendValToSensorTaskFromISR(&retval);
     * */
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

