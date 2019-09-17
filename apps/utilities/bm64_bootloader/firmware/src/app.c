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

#include "app.h"

/* NOTE:  UART/USART references in the source code are generic, e.g. UARTn_Read
 * These are resolved into actual calls in user.h using #defines, e.g.
 * #define UARTn_Read          UART1_Read 
 */
#include "user.h"

// *****************************************************************************
// *****************************************************************************
// Section: Global Data Definitions
// *****************************************************************************
// *****************************************************************************

uint8_t CACHE_ALIGN cdcReadBuffer[APP_READ_BUFFER_SIZE];
uint8_t CACHE_ALIGN cdcWriteBuffer[APP_READ_BUFFER_SIZE];

uint8_t CACHE_ALIGN uartReceivedData;

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
}

void APP_WriteCallback(uintptr_t context)
{
	appData.isUARTWriteComplete = true;
}

void APP_ReadCallback(uintptr_t context)
{
	appData.isUARTReadComplete = true;
}
/*******************************************************
 * USB CDC Device Events - Application Event Handler
 *******************************************************/

USB_DEVICE_CDC_EVENT_RESPONSE APP_USBDeviceCDCEventHandler(USB_DEVICE_CDC_INDEX index,
    USB_DEVICE_CDC_EVENT event, void *pData, uintptr_t userData)
{
    APP_DATA * appDataObject;

    USB_CDC_CONTROL_LINE_STATE * controlLineStateData;
    appDataObject = (APP_DATA *)userData;

    switch(event)
    {
        case USB_DEVICE_CDC_EVENT_GET_LINE_CODING:

            /* This means the host wants to know the current line
             * coding. This is a control transfer request. Use the
             * USB_DEVICE_ControlSend() function to send the data to
             * host.  */

            USB_DEVICE_ControlSend(appDataObject->usbDevHandle,
                    &appDataObject->getLineCodingData, sizeof(USB_CDC_LINE_CODING));
            break;

        case USB_DEVICE_CDC_EVENT_SET_LINE_CODING:

            /* This means the host wants to set the line coding.
             * This is a control transfer request. Use the
             * USB_DEVICE_ControlReceive() function to receive the
             * data from the host */
            appData.isSetLineCodingCommandInProgress = true;
            appData.isBaudrateDataReceived = false;
            USB_DEVICE_ControlReceive(appDataObject->usbDevHandle,
                    &appDataObject->setLineCodingData, sizeof(USB_CDC_LINE_CODING));
            break;

        case USB_DEVICE_CDC_EVENT_SET_CONTROL_LINE_STATE:

            /* This means the host is setting the control line state.
             * Read the control line state. We will accept this request
             * for now. */

            controlLineStateData = (USB_CDC_CONTROL_LINE_STATE *)pData;
            appDataObject->controlLineStateData.dtr = controlLineStateData->dtr;
            appDataObject->controlLineStateData.carrier =
                    controlLineStateData->carrier;

            USB_DEVICE_ControlStatus(appDataObject->usbDevHandle, USB_DEVICE_CONTROL_STATUS_OK);
            break;

        case USB_DEVICE_CDC_EVENT_SEND_BREAK:

            /* This means that the host is requesting that a break of the
             * specified duration be sent. Read the break duration */

            appDataObject->breakData = ((USB_DEVICE_CDC_EVENT_DATA_SEND_BREAK *)pData)->breakDuration;
            
            /* Complete the control transfer by sending a ZLP  */
            USB_DEVICE_ControlStatus(appDataObject->usbDevHandle, USB_DEVICE_CONTROL_STATUS_OK);         
            break;

        case USB_DEVICE_CDC_EVENT_READ_COMPLETE:

            /* This means that the host has sent some data*/
            appDataObject->isCDCReadComplete = true;
            appDataObject->readLength =
                       ((USB_DEVICE_CDC_EVENT_DATA_READ_COMPLETE*)pData)->length;				
            break;

        case USB_DEVICE_CDC_EVENT_CONTROL_TRANSFER_DATA_RECEIVED:

            /* The data stage of the last control transfer is
             * complete. */
            if (appData.isSetLineCodingCommandInProgress == true)
            {
               /* We have received set line coding command from the Host.
                * DRV_UART_BaudSet() function is not interrupt safe and it
                * should not be called here. It is called in APP_Tasks()
                * function. The ACK for Status stage of the control transfer is
                * send in the APP_Tasks() function.  */
                appData.isSetLineCodingCommandInProgress = false;
                appData.isBaudrateDataReceived = true;
            }
            else
            {
				/* ACK the Status stage of the Control transfer */
                USB_DEVICE_ControlStatus(appDataObject->usbDevHandle, USB_DEVICE_CONTROL_STATUS_OK);
            }
            break;

        case USB_DEVICE_CDC_EVENT_CONTROL_TRANSFER_DATA_SENT:

            /* This means the GET LINE CODING function data is valid. We don't
             * do much with this data in this demo. */
            break;
        case USB_DEVICE_CDC_EVENT_CONTROL_TRANSFER_ABORTED:
            /* The control transfer has been aborted */
            if (appData.isSetLineCodingCommandInProgress == true)
            {
                appData.isSetLineCodingCommandInProgress = false;
                appData.isBaudrateDataReceived = false;
            }
            break;
        case USB_DEVICE_CDC_EVENT_WRITE_COMPLETE:

            /* This means that the data write got completed. We can schedule
             * the next read. */

            appDataObject->isCDCWriteComplete = true;
            break;

        default:
            break;
    }

    return USB_DEVICE_CDC_EVENT_RESPONSE_NONE;
}

/*********************************************
 * Application USB Device Layer Event Handler
 *********************************************/

void APP_USBDeviceEventHandler(USB_DEVICE_EVENT event, void * eventData, uintptr_t context)
{
    USB_DEVICE_EVENT_DATA_CONFIGURED *configuredEventData;

    switch(event)
    {
        case USB_DEVICE_EVENT_RESET:
            /* USB device is reset or device is deconfigured.
             * This means that USB device layer is about to deininitialize
             * all function drivers. Update LEDs to indicate
             * reset/deconfigured state. */
			 
            /* Update LED to show reset state */
            LED1_Off();

            appData.deviceIsConfigured = false;
            break;

        case USB_DEVICE_EVENT_CONFIGURED:

            /* Check the configuration. We only support configuration 1 */
            configuredEventData = (USB_DEVICE_EVENT_DATA_CONFIGURED*)eventData;
            
            if ( configuredEventData->configurationValue == 1)
            {
                /* The device is in configured state. Update LED indication */
                LED1_On();
                
                /* Register the CDC Device application event handler here.
                 * Note how the appData object pointer is passed as the
                 * user data */

                USB_DEVICE_CDC_EventHandlerSet(USB_DEVICE_CDC_INDEX_0,
                        APP_USBDeviceCDCEventHandler, (uintptr_t)&appData);

                /* mark that set configuration is complete */
                appData.deviceIsConfigured = true;

            }       
            break;

        case USB_DEVICE_EVENT_POWER_DETECTED:

            /* VBUS was detected. We can attach the device */
            USB_DEVICE_Attach(appData.usbDevHandle);           
            break;

        case USB_DEVICE_EVENT_POWER_REMOVED:

            /* VBUS is not available any more. Detach the device. */
            USB_DEVICE_Detach(appData.usbDevHandle);
            
            LED1_Off();            
            break;

        case USB_DEVICE_EVENT_SUSPENDED:

            /* Switch LED to show suspended state */
            LED1_Off();           
            break;

        case USB_DEVICE_EVENT_RESUMED:
        case USB_DEVICE_EVENT_ERROR:
        default:           
            break;
    }
}

// *****************************************************************************
// *****************************************************************************
// Section: Application Local Functions
// *****************************************************************************
// *****************************************************************************


/*****************************************************
 * This function is called in every step of the
 * application state machine.
 *****************************************************/

bool APP_StateReset(void)
{
    /* This function returns true if the device
     * was reset  */

    bool retVal;

    if(appData.deviceIsConfigured == false)
    {
        appData.state = APP_STATE_WAIT_FOR_CONFIGURATION;
        appData.readTransferHandle = USB_DEVICE_CDC_TRANSFER_HANDLE_INVALID;
        appData.writeTransferHandle = USB_DEVICE_CDC_TRANSFER_HANDLE_INVALID;
        appData.isCDCReadComplete = true;
        appData.isCDCWriteComplete = true;
        retVal = true;

        appData.isSetLineCodingCommandInProgress = false;
        appData.isBaudrateDataReceived = false;
    }
    else
    {
        retVal = false;
    }

    return(retVal);
}

/***************************************************************************
 * This function Handles the Set Line coding command from Host.
 ***************************************************************************/
void _APP_SetLineCodingHandler(void)
{
    UART_SERIAL_SETUP UartSetup;

    UartSetup.baudRate = appData.setLineCodingData.dwDTERate;
    UartSetup.parity = appData.setLineCodingData.bParityType;
    UartSetup.dataWidth = appData.setLineCodingData.bDataBits;
    UartSetup.stopBits = appData.setLineCodingData.bCharFormat;

    if (true == UARTn_SerialSetup(&UartSetup, CHIP_FREQ_CPU_MAX))
    {
        /* Baudrate is changed successfully. Update Baudrate info in the
         * Get line coding structure. */
        appData.getLineCodingData.dwDTERate = appData.setLineCodingData.dwDTERate;

        /* Acknowledge the Status stage of the Control transfer */
        USB_DEVICE_ControlStatus(appData.usbDevHandle, USB_DEVICE_CONTROL_STATUS_OK);
    }
    else
    {
        /* Baudrate was not set. There are two ways that an unsupported
         * baud rate could be handled.  The first is just to ignore the
         * request and ACK the control transfer.  That is what is currently
         * implemented below. */
         USB_DEVICE_ControlStatus(appData.usbDevHandle, USB_DEVICE_CONTROL_STATUS_OK);

        /* The second possible method is to stall the STATUS stage of the
         * request. STALLing the STATUS stage will cause an exception to be
         * thrown in the requesting application. Some programs, like
         * HyperTerminal, handle the exception properly and give a pop-up
         * box indicating that the request settings are not valid.  Any
         * application that does not handle the exception correctly will
         * likely crash when this request fails.  For the sake of example
         * the code required to STALL the status stage of the request is
         * provided below.  It has been left out so that this demo does not
         * cause applications without the required exception handling to
         * crash.*/
         //USB_DEVICE_ControlStatus(appData.usbDevHandle, USB_DEVICE_CONTROL_STATUS_ERROR);
    } 
}

// *****************************************************************************
// *****************************************************************************
// Section: Application Initialization and State Machine Functions
// *****************************************************************************
// *****************************************************************************

/*******************************************************************************
  Function:
    void APP_Initialize(void)

  Remarks:
    See prototype in app.h.
 */

void APP_Initialize(void)
{
    /* Place the App state machine in its initial state. */
    appData.state = APP_STATE_INIT;
    
    /* Device Layer Handle  */
    appData.usbDevHandle = USB_DEVICE_HANDLE_INVALID ;

    /* Device configured status */
    appData.deviceIsConfigured = false;

    /* Initial get line coding state */
    appData.getLineCodingData.dwDTERate = 9600;
    appData.getLineCodingData.bDataBits = 8;
    appData.getLineCodingData.bParityType = 0;
    appData.getLineCodingData.bCharFormat = 0;

    /* Read Transfer Handle */
    appData.readTransferHandle = USB_DEVICE_CDC_TRANSFER_HANDLE_INVALID;

    /* Write Transfer Handle */
    appData.writeTransferHandle = USB_DEVICE_CDC_TRANSFER_HANDLE_INVALID;

    /* Initialize the read complete flag */
    appData.isCDCReadComplete = true;

    /*Initialize the write complete flag*/
    appData.isCDCWriteComplete = true;
    
    /*Initialize the buffer pointers */
    appData.uartReceivedData = &uartReceivedData;   

    /* Set up the read buffer */
    appData.cdcReadBuffer = &cdcReadBuffer[0];

    /* Set up the read buffer */
    appData.cdcWriteBuffer = &cdcWriteBuffer[0];
    
    UARTn_WriteCallbackRegister(APP_WriteCallback, 0);
    UARTn_ReadCallbackRegister(APP_ReadCallback, 0);
    
	appData.isUARTWriteComplete = false;
    appData.isUARTReadComplete = false;
    
    appData.buttonState = BUTTON_STATE_IDLE;   
    appData.buttonDelay = 0;        // used for button debounce

    BM64_MFB_Clear();
    
    STBYRST_Set(); 

    LED1_Off();       
}

/******************************************************************************
  Function:
    void APP_Tasks(void)

  Remarks:
    See prototype in app.h.
 */
DRV_HANDLE tmrHandle;

void APP_Tasks(void)
{
    APP_Button_Tasks();

    if ((appData.deviceIsConfigured) && (appData.isBaudrateDataReceived))
    {
		 appData.isBaudrateDataReceived = false;
        _APP_SetLineCodingHandler();
    }

    /* Update the application state machine based
     * on the current state */
    
    switch(appData.state)
    {
        case APP_STATE_INIT:        
            /* Open the timer Driver */
            tmrHandle = SYS_TIME_CallbackRegisterMS(App_TimerCallback, 
                    (uintptr_t)0, 1/*ms*/, SYS_TIME_PERIODIC);

            if ( SYS_TIME_HANDLE_INVALID != tmrHandle )
            {                
               appData.state = APP_STATE_OPENUSB;
            }  
            break;
            
        case APP_STATE_OPENUSB:

            /* Open the device layer */
            appData.usbDevHandle = USB_DEVICE_Open( USB_DEVICE_INDEX_0, DRV_IO_INTENT_READWRITE );

            if(appData.usbDevHandle != USB_DEVICE_HANDLE_INVALID)
            {
                /* Register a callback with device layer to get event notification (for end point 0) */
                USB_DEVICE_EventHandlerSet(appData.usbDevHandle, APP_USBDeviceEventHandler, 0);

                /* Application waits for device configuration. */
                appData.state = APP_STATE_WAIT_FOR_CONFIGURATION;
            }
            else
            {
                /* The Device Layer is not ready to be opened. We should try
                 * again later. */
            }
            break;

        case APP_STATE_WAIT_FOR_CONFIGURATION:

            /* Check if the device is configured */
            if(appData.deviceIsConfigured)
            {
                /* Schedule the first read from CDC function driver */

                appData.state = APP_STATE_CHECK_CDC_READ;
                appData.isCDCReadComplete = false;
				appData.isUARTReadComplete = false;

                UARTn_Read(appData.uartReceivedData, 1);
                USB_DEVICE_CDC_Read (USB_DEVICE_CDC_INDEX_0, &appData.readTransferHandle,
                        appData.cdcReadBuffer, APP_READ_BUFFER_SIZE);
            }
            break;

        case APP_STATE_CHECK_CDC_READ:

            if(APP_StateReset())
            {
                break;
            }

            /* If CDC read is complete, send the received data to the UART. */
            if(appData.isCDCReadComplete == true)
            {               
                if(true == UARTn_Write(&appData.cdcReadBuffer[0], appData.readLength))                   
				{
					appData.isCDCReadComplete = false;

					appData.isUARTWriteComplete = false;
					
					//LED1_Toggle();   // testing only
					
                    /* This means we have sent all the data. We schedule the next
                     * CDC Read. */
                    USB_DEVICE_CDC_Read (USB_DEVICE_CDC_INDEX_0, &appData.readTransferHandle,
                        appData.cdcReadBuffer, APP_READ_BUFFER_SIZE);

                    appData.state = APP_STATE_CHECK_UART_RECEIVE;

				}
            }
            else
            {
                /* We did not get any data from CDC. Check if any data was
                 * received from the UART. */
                appData.state = APP_STATE_CHECK_UART_RECEIVE;
            }
            break;

        case APP_STATE_CHECK_UART_RECEIVE:

            if(APP_StateReset())
            {
                break;
            }

            /* Check if a character was received on the UART */
            if(appData.isUARTReadComplete == true)
            {
                //LED1_Toggle();     // testing only
                USB_DEVICE_CDC_Write(USB_DEVICE_CDC_INDEX_0, &appData.writeTransferHandle,
                        appData.uartReceivedData, 1,
                        USB_DEVICE_CDC_TRANSFER_FLAGS_DATA_COMPLETE);

				appData.isUARTReadComplete = false;
                
                UARTn_Read(appData.uartReceivedData, 1);
            }
			else
			{
				appData.state = APP_STATE_CHECK_CDC_READ;
			}
            break;

        case APP_STATE_ERROR:
        default:
            break;
    }
}

#define BUTTON_DEBOUNCE 50
#define LONG_BUTTON_PRESS 1000

void APP_Button_Tasks()
{      
   //BUTTON PROCESSING
    /* Check the buttons' current state. */         
    switch ( appData.buttonState )
    {
        case BUTTON_STATE_IDLE:
        {
            if ( (appData.buttonDelay==0)&&
                 (SWITCH1_Get()==SWITCH1_STATE_PRESSED))                
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
            
            if(SWITCH1_Get()==SWITCH1_STATE_PRESSED) 
            {                
                appData.buttonState=BUTTON_STATE_BUTTON0_PRESSED;                  
            }
        }
        break;
          
        case BUTTON_STATE_BUTTON0_PRESSED:
        {
            if (SWITCH1_Get()!=SWITCH1_STATE_PRESSED)     // SW0 pressed and released < 1 sec
            {

                appData.buttonDelay=200;                
                STBYRST_Clear();     // normally high
                appData.buttonState=BUTTON_STATE_WAIT_RESET_LOW;              
            }
        } 
        break;
        
        case BUTTON_STATE_WAIT_RESET_LOW:
        {
            if (appData.buttonDelay>0)
            {
                break;
            }            
            appData.buttonDelay=50;                
            STBYRST_Set();
            appData.buttonState=BUTTON_STATE_WAIT_RESET_HIGH;              
        } 
        break; 
        
        case BUTTON_STATE_WAIT_RESET_HIGH:
        {
            if (appData.buttonDelay>0)
            {
                break;
            }            
            appData.buttonDelay=200;                
            BM64_MFB_Set();     // normally low           
            appData.buttonState=BUTTON_STATE_WAIT_MFB_HIGH;              
        } 
        break;

        case BUTTON_STATE_WAIT_MFB_HIGH:
        {
            if (appData.buttonDelay>0)
            {
                break;
            }            
            appData.buttonDelay=20;                
            BM64_MFB_Clear();            
            appData.buttonState=BUTTON_STATE_IDLE;              
        } 
        break;         

        /* The default state should never be executed. */
        default:
        {
            /* TODO: Handle error in application's state machine. */
            break;
        }
    }
}

