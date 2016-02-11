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
#include "app.h"
#include "system_definitions.h"

// *****************************************************************************
// *****************************************************************************
// Section: Global variables
// *****************************************************************************
// *****************************************************************************

RX_DATA rxData;


// *****************************************************************************
// *****************************************************************************
// Section: System Interrupt Vector Functions
// *****************************************************************************
// *****************************************************************************

void IntHandlerDrvUsartInstance0(void)
{

    /* Check the application's current state. */
    switch ( rxData.state )
    {
        /* Application's initial state. */
        case RX_STATE_INIT:                        // application state after system and application are initialized
        {
            rxData.state = RX_STATE_START;           // change state to receive after initializing application
            rxData.start = '~';
            rxData.types[0] = 'r'; //you can add more types in if you'd like
            rxData.stop = '.'; //this is our stop value
            
            rxData.buffer[0] = '\0';
            rxData.buffer[1] = '\0';
            rxData.buffer[2] = '\0';
            rxData.buffer[3] = '\0';
            rxData.buffer[4] = '\0';
            
 
            //Dont want to break here.
        }
        
                
        case RX_STATE_START:  // USART receive start state
        {          
            char new = DRV_USART0_ReadByte();
            if(new == rxData.start){
                rxData.buffer[0] = new;
                rxData.last_rx_byte = new;
                rxData.state = RX_STATE_TYPE;
            }
            /*
            else if(rxData.buffer[0] == rxData.start){
                rxData.state = RX_STATE_TYPE;
            }      
            */
            break;
        }
        
        case RX_STATE_TYPE:  // USART receive type state
        {          
            char new = DRV_USART0_ReadByte();
            
            rxData.last_rx_byte = new;

            
            if(new == rxData.start){         //BAD, we need to restart       
                rxData.state = RX_STATE_START;
            }
            else if(new == rxData.stop){  //BAD, we need to restart
                rxData.state = RX_STATE_START;
                rxData.buffer[0] = '\0';
            }  
            else{
                rxData.buffer[1] = new; //this is the type of message byte
                rxData.state = RX_STATE_D1;
            }
                     
            break;

        }
        
        case RX_STATE_D1:  // USART receive type state
        {          
            char new = DRV_USART0_ReadByte();
            
            rxData.last_rx_byte = new;

            
            if(new == rxData.start){         //BAD, we need to go back      
                rxData.state = RX_STATE_TYPE;
                rxData.buffer[1] = '\0'; //clear type byte 
            }
            else if(new == rxData.stop){  //BAD, we need to restart
                rxData.state = RX_STATE_START; //go back to start
                
                //clear the buffer
                rxData.buffer[0] = '\0';
                rxData.buffer[1] = '\0';
            }  
            else{
                rxData.buffer[2] = new;
                rxData.state = RX_STATE_D2;
            }
                     
            break;
        }
        
        case RX_STATE_D2:  // USART receive type state
        {          
            char new = DRV_USART0_ReadByte();
            
            rxData.last_rx_byte = new;

            
            if(new == rxData.start){         //BAD, we need to go back      
                rxData.state = RX_STATE_TYPE;
                rxData.buffer[1] = '\0'; //clear type byte 
                rxData.buffer[2] = '\0'; //clear last data byte
            }
            else if(new == rxData.stop){  //BAD, we need to restart
                rxData.state = RX_STATE_START; //go back to start
                
                //clear the buffer
                rxData.buffer[0] = '\0';
                rxData.buffer[1] = '\0';
                rxData.buffer[2] = '\0';
            }  
            else{
                rxData.buffer[3] = new;
                rxData.state = RX_STATE_INTERPRET;
            }
                     
            break;
        }
        
        
        case RX_STATE_INTERPRET:  // USART receive type state
        {          
            char new = DRV_USART0_ReadByte();
            
            rxData.last_rx_byte = new;

            
            if(new == rxData.stop){  //YAY! 
                rxData.state = RX_STATE_START;
                
                
                //DO WHAT EVER PROCESSING YOU NEED TO DO HERE:
                if(rxData.buffer[2] == 'o'){
                   LATASET = 1 << 3;
                } 
                else if(rxData.buffer[2] == 'f'){
                   LATACLR = 1 << 3;    
                }
                
                //Clear the buffer
                rxData.buffer[0] = '\0'; //start
                rxData.buffer[1] = '\0'; //type
                rxData.buffer[2] = '\0'; //d1
                rxData.buffer[3] = '\0'; //d2
            }
            else{ //If it isn't the stop byte, we don't care. Erase everything and restart
                if(new == rxData.start){
                    rxData.state = RX_STATE_TYPE;
                    rxData.buffer[1] = '\0';
                    rxData.buffer[2] = '\0';
                    rxData.buffer[3] = '\0';
                }
                else{
                    rxData.state = RX_STATE_START;
                    rxData.buffer[0] = '\0';
                    rxData.buffer[1] = '\0';
                    rxData.buffer[2] = '\0';
                    rxData.buffer[3] = '\0';
                }
                
                
                
            }
                     
            break;
        }
        
        
    } // end of switch case statement

    /* Clear pending interrupt */
    PLIB_INT_SourceFlagClear(INT_ID_0, INT_SOURCE_USART_1_TRANSMIT);
    PLIB_INT_SourceFlagClear(INT_ID_0, INT_SOURCE_USART_1_RECEIVE);
    PLIB_INT_SourceFlagClear(INT_ID_0, INT_SOURCE_USART_1_ERROR);
    
    
            
}
 
 
 

 
 

 

 
 
  
/*******************************************************************************
 End of File
*/

