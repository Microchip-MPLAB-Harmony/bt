/*******************************************************************************
  MPLAB Harmony Application Source File

  Company:
    Microchip Technology Inc.

  File Name:
    app.c

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

// *****************************************************************************
// *****************************************************************************
// Section: Included Files
// *****************************************************************************
// *****************************************************************************

#include <string.h>
#include "app.h"        // also beings in app_tone_lookup_table.h
#include "ble/non_gui/ble.h"
extern BLE_DATA bleData;

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

APP_DATA appData;

// *****************************************************************************
// *****************************************************************************
// Section: Application Callback Functions
// *****************************************************************************
// *****************************************************************************

/// *****************************************************************************
 /*
  Function:
        static void App_TimerCallback
        (
            uintptr_t context
        )

  Summary:
    Implements the handler for timer callback function.

  Description:
    Called every 1 ms by timer services.  It then decrements WM8904Delay if
    non-zero.

  Parameters:
    context      - Contains index into driver's instance object array

  Remarks:
    None
*/
static void App_TimerCallback( uintptr_t context)
{
    if (appData.buttonDelay)
    {      
        appData.buttonDelay--;
    }
    
    if (appData.waitingToConnect)
    {
        appData.waitingToConnectTimer++;
        if ((appData.waitingToConnectTimer & 0xFF) == 0)    // about every 1/4 second
        {
            LED1_Toggle();
        }
    }
}

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
    void APP_Initialize ( void )

  Remarks:
    See prototype in app.h.
 */

void APP_Initialize ( void )
{
    /* TODO: Initialize your application's state machine and other
     * parameters.
     */   
    printf("\n----APP_Initialize----\r\n");
	LED1_On();       // initially on, until connected
    appData.state = APP_STATE_INIT;   
 
    appData.buttonState = BUTTON_STATE_IDLE;   
    appData.buttonDelay = 0;        // used for button debounce
    appData.buttonMode = false;
    appData.queryDelay = 0;         // used for status queries

    appData.waitingToConnect = false;
    appData.waitingToConnectTimer = 0;

    bleInitialize(true);
}

/******************************************************************************
  Function:
    void APP_Tasks ( void )

  Remarks:
    See prototype in app.h.
 */
DRV_HANDLE tmrHandle;

void APP_Tasks ( void )
{    
    bleTasks(); 
    
    /* Check the application's current state. */
    switch ( appData.state )
    {       
        /* Application's initial state. */
        case APP_STATE_INIT:
        {
            /* Open the timer Driver */
            tmrHandle = SYS_TIME_CallbackRegisterMS(App_TimerCallback, 
                    (uintptr_t)0, 1/*ms*/, SYS_TIME_PERIODIC);

            if ( SYS_TIME_HANDLE_INVALID != tmrHandle )
            {
               appData.state = APP_STATE_WAIT_INIT;
            }            
        }
        break;
        
        case APP_STATE_WAIT_INIT:
        {
            if (DRV_BT_STATUS_READY == DRV_BT_GetPowerStatus())
            {           
                appData.state=APP_STATE_IDLE;
            }
        }
        break;
        
        // Initialized 
        case APP_STATE_IDLE:
        {
            APP_Button_Tasks();
            
            if (appData.queryDelay == 0)
            {
                DRV_BT_BLE_QueryStatus(bleData.bt.handle);
                appData.queryDelay = QUERY_DELAY;
            }
		}
		break;
         
        default:
        {
            /* TODO: Handle error in application's state machine. */
        }
        break;             
    }
}

#define BUTTON_DEBOUNCE 50
#define LONG_BUTTON_PRESS 1000

void APP_Button_Tasks()
{
    char* test_string;
        
   //BUTTON PROCESSING
    /* Check the buttons' current state. */      
    switch ( appData.buttonState )
    {
        case BUTTON_STATE_IDLE:
        {
            if ( (appData.buttonDelay==0)&&
                 (SWITCH_Get()==SWITCH_STATE_PRESSED))                
            {
                appData.buttonDelay=BUTTON_DEBOUNCE;       
                appData.buttonState=BUTTON_STATE_PRESSED;               
            }
        }
        break;
        
        case BUTTON_STATE_PRESSED:
        { 
            if (appData.buttonDelay>0)
            {
                break;      // still debouncing
            }
            
            if(SWITCH_Get()==SWITCH_STATE_PRESSED) 
            {                
                appData.buttonDelay=LONG_BUTTON_PRESS;          // 1 sec is long press
                appData.buttonState=BUTTON_STATE_BUTTON0_PRESSED;                  
            }
        }
        break;
          
        case BUTTON_STATE_BUTTON0_PRESSED:
        {
            if ((appData.buttonDelay>0)&&
                (SWITCH_Get()!=SWITCH_STATE_PRESSED))     // SW01 pressed and released < 1 sec
            {
                if (appData.buttonMode)
                {
	                // future feature    
                }
                else
                {
	                test_string = "Button pressed";
	                DRV_BT_SendDataOverBLE(bleData.bt.handle, (uint8_t*)test_string, strlen(test_string));
                }
                appData.buttonDelay=BUTTON_DEBOUNCE;                
                appData.buttonState=BUTTON_STATE_IDLE;              
            }
            else if ((appData.buttonDelay==0)&&
                     (SWITCH_Get()==SWITCH_STATE_PRESSED))  // SW0 still pressed after 1 sec
            {
                // long press reserved for future feature
                //appData.buttonMode = !appData.buttonMode;             
                
                appData.buttonState=BUTTON_STATE_WAIT_FOR_RELEASE;                
            }                          
        } 
        break;

        case BUTTON_STATE_WAIT_FOR_RELEASE:
        {
            if (SWITCH_Get()!=SWITCH_STATE_PRESSED)
            {
                appData.buttonDelay=BUTTON_DEBOUNCE;
                appData.buttonState=BUTTON_STATE_IDLE;
            }
        }

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
