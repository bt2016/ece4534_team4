/*******************************************************************************
  MPLAB Harmony Application Source File
  
  Company:
    Microchip Technology Inc.
  
  File Name:
    pwm.c

  Summary:
    This file contains the source code for the MPLAB Harmony application.

  Description:
    This file contains the source code for the MPLAB Harmony application.  It 
    implements the logic of the application's state machine and it may call 
    API routines of other MPLAB Harmony modules in the system, such as drivers,
    system services, and middleware.  However, it does not call any of the
    system interfaces (such as the "Initialize" and "Tasks" functions) of any of
    the modules in the system or make any assumptions about when those functions
    are called.  That is the responsibility of the configuration-specific system
    files.
 *******************************************************************************/

// DOM-IGNORE-BEGIN
/*******************************************************************************
Copyright (c) 2013-2014 released Microchip Technology Inc.  All rights reserved.

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

#include "pwm.h"

// *****************************************************************************
// *****************************************************************************
// Section: Global Data Definitions
// *****************************************************************************
// *****************************************************************************

// *****************************************************************************
/* Application Data

  Summary:
    Holds application data

  Description:
    This structure holds the application's data.

  Remarks:
    This structure should be initialized by the APP_Initialize function.
    
    Application strings and buffers are be defined outside this structure.
*/

PWM_DATA pwmData;

// *****************************************************************************
// *****************************************************************************
// Section: Application Callback Functions
// *****************************************************************************
// *****************************************************************************

/* TODO:  Add any necessary callback funtions.
*/

// *****************************************************************************
// *****************************************************************************
// Section: Application Local Functions
// *****************************************************************************
// *****************************************************************************

/* TODO:  Add any necessary local functions.
*/

// *****************************************************************************
// *****************************************************************************
// Section: Application Initialization and State Machine Functions
// *****************************************************************************
// *****************************************************************************

/*******************************************************************************
  Function:
    void PWM_Initialize ( void )

  Remarks:
    See prototype in pwm.h.
 */
unsigned int hi;
unsigned int yo;
unsigned int a,b,c,d,e,f,g;

void pwm_Init() {
    DRV_TMR0_Stop();
    DRV_TMR0_Initialize();
    DRV_TMR0_Start();

    DRV_OC0_Disable();
    DRV_OC0_Initialize();
    DRV_OC0_Enable();
    
        DRV_OC1_Disable();
    DRV_OC1_Initialize();
    DRV_OC1_Enable();
    
        DRV_OC2_Disable();
    DRV_OC2_Initialize();
    DRV_OC2_Enable();
    
        DRV_OC3_Disable();
    DRV_OC3_Initialize();
    DRV_OC3_Enable();
}

void OC0_SetPulseWidth(uint16_t time_val) {
   PLIB_OC_PulseWidth16BitSet(OC_ID_1, time_val);
}
void OC1_SetPulseWidth(uint16_t time_val) {
   PLIB_OC_PulseWidth16BitSet(OC_ID_2, time_val);
}
void OC2_SetPulseWidth(uint16_t time_val) {
   PLIB_OC_PulseWidth16BitSet(OC_ID_3, time_val);
}
void OC3_SetPulseWidth(uint16_t time_val) {
   PLIB_OC_PulseWidth16BitSet(OC_ID_4, time_val);
}



//void 
void PWM_Initialize ( void )
{
    /* Place the App state machine in its initial state. */
    pwmData.state = PWM_STATE_INIT;
    
    /*
    // RF1 - Direction 1 control (0 - Forward, 1 - Reverse)
    ODCFCLR = 0x2;
    TRISFCLR = 0x2;
     * */
    
    // RF3 - Direction 1 control (0 - Reverse, 1 - Forward)
    ODCFCLR = 0x8;
    TRISFCLR = 0x8;
    
    // RG1 - Direction 2 control (0 - Reverse, 1 - Forward)
    ODCGCLR = 0x2;
    TRISGCLR = 0x2;
    
    // CHIPKIT 22, 23 = PIC 7 (RC2), 8(RC3)
    // RC3 = JP2, RC2 = JP1
    // RC2 = 0 --> Independent speeds
    // RC3 = 1 --> Independent direction control
    //ODCCCLR = 0xC;
    //TRISCCLR = 0xC;
    
    //ODCCCLR = (1 << 14);
    //TRISCCLR = (1 << 14);
    
    ODCCCLR = 0xffff;
    TRISCCLR = 0xFFFF;
    
    pwm_Init();
}

void setDirection1(int enable) {
    //LATCSET = (enable << 14);
    LATFSET = (enable << 3);
}
void setDirection2(int enable) {
    LATGSET = (enable << 1);
}
void setJP1(int enable) {
    LATCSET = (enable << 2);
}
void setJP2(int enable) {
    LATCSET = (enable << 3);
}

/******************************************************************************
  Function:
    void PWM_Tasks ( void )

  Remarks:
    See prototype in pwm.h.
 */

void PWM_Tasks ( void )
{
    /* Check the application's current state. */
    switch ( pwmData.state )
    {
        /* Application's initial state. */
        case PWM_STATE_INIT:
        {
            /*
            uint16_t time_val2 = 600;
            uint16_t time_val = 3000;
            
            DRV_TMR0_Stop();
            DRV_TMR0_Initialize();
            OC0_SetPulseWidth(time_val);
            DRV_TMR0_Start();
            
            DRV_TMR1_Stop();
            DRV_TMR1_Initialize();
            OC1_SetPulseWidth(time_val2);
            DRV_TMR1_Start();
            
            // Independent direction control enable
            setJP1(SET_HIGH);
            setJP2(SET_HIGH);
            setDirection1(SET_HIGH);
            setDirection2(SET_LOW);
             
             */
            pwmData.state = PWM_STATE_STAY;
            
        }

        case PWM_STATE_STAY:
        {}
        /* TODO: implement your application state machine.*/

        /* The default state should never be executed. */
        default:
        {
            /* TODO: Handle error in application's state machine. */
            break;
        }
    }
}
 

/*******************************************************************************
 End of File
 */
