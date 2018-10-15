/*******************************************************************************
  BM64 Bluetooth Static Driver implementation

  Company:
    Microchip Technology Inc.

  File Name:
    drv_bm64.c

  Summary:
   BM64 Bluetooth Static Driver main source file

  Description:
    This file is the implementation of the external (public) API of the static 
    implementation of the BM64 driver (plus some local functions).

    The BM64 is a Bluetooth 4.2 Stereo Module that supports classic A2DP, AVRCP,
    HFP, HSP, and SPP protocols as well as BLE (Bluetooth Low Energy).
    
    The BM64 streams I2S audio at up to 24-bit, 96 kHz.  It uses a UART to 
    receive commands from the host microcontroller (PIC32) and and send events
    back.

*******************************************************************************/
// DOM-IGNORE-BEGIN
/*******************************************************************************
Copyright (c) 2017 released Microchip Technology Inc.  All rights reserved.

Microchip licenses to you the right to use, modify, copy and distribute
Software only when embedded on a Microchip microcontroller or digital signal
controller that is integrated into your product or third party product
(pursuant to the sublicense terms in the accompanying license agreement).

You should refer to the license agreement accompanying this Software for
additional information regarding your rights and obligations.

SOFTWARE AND DOCUMENTATION ARE PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
EITHER EXPRESS  OR  IMPLIED,  INCLUDING  WITHOUT  LIMITATION,  ANY  WARRANTY  OF
MERCHANTABILITY, TITLE, NON-INFRINGEMENT AND FITNESS FOR A  PARTICULAR  PURPOSE.
IN NO EVENT SHALL MICROCHIP OR  ITS  LICENSORS  BE  LIABLE  OR  OBLIGATED  UNDER
CONTRACT, NEGLIGENCE, STRICT LIABILITY, CONTRIBUTION,  BREACH  OF  WARRANTY,  OR
OTHER LEGAL  EQUITABLE  THEORY  ANY  DIRECT  OR  INDIRECT  DAMAGES  OR  EXPENSES
INCLUDING BUT NOT LIMITED TO ANY  INCIDENTAL,  SPECIAL,  INDIRECT,  PUNITIVE  OR
CONSEQUENTIAL DAMAGES, LOST  PROFITS  OR  LOST  DATA,  COST  OF  PROCUREMENT  OF
SUBSTITUTE  GOODS,  TECHNOLOGY,  SERVICES,  OR  ANY  CLAIMS  BY  THIRD   PARTIES
(INCLUDING BUT NOT LIMITED TO ANY DEFENSE  THEREOF),  OR  OTHER  SIMILAR  COSTS.
*******************************************************************************/
// DOM-IGNORE-END

/*************************************************************
 * Include files.
 ************************************************************/
#include <xc.h>
#include "driver/driver_common.h"
#include "configuration.h"
#include "driver/bm64/drv_bm64.h"
#include "driver/bm64/drv_bm64_local.h"
#include "driver/bm64/drv_bm64_command_send.h"
#include "driver/bm64/drv_bm64_command_decode.h"
#include "driver/bm64/drv_bm64_ble.h"
#include "driver/bm64/drv_bm64_uart.h"
#include "driver/bm64/drv_bm64_gpio.h"
#include "system/time/sys_time.h"

#define BM64_BLE_JUMPER_ENABLED 0

#define SYS_DEBUG(x,y)      // not implemented

// all #defines, enums and non-static functions and variables prefixed by
// DRV_BM64 to avoid name conflicts

DRV_HANDLE DRV_BM64_tmrHandle;

static uint16_t _timer1ms;

static void _initStatus( void );
static void _initSPP( void );
static void _BM64ControlTasks( void );
static uint8_t _localBDAddr[6];
static uint8_t _masterBDAddr[6];
static uint8_t _linkbackBDAddr[6];
static char _localBDName[DRV_BM64_MAXBDNAMESIZE+1]; // +1 for trailing null char

#define DRV_BM64_SPP_RxFifoSize 200                       //receive buffer max. size
uint8_t DRV_BM64_SPP_RxFifo[DRV_BM64_SPP_RxFifoSize];          //receive buffer
uint32_t DRV_BM64_SPP_RxFifoHead, DRV_BM64_SPP_RxFifoTail, DRV_BM64_SPP_RxCounter;

static void _switchSampleFrequency(DRV_BM64_SAMPLE_FREQUENCY sampleFreq);
static void _clientCallBack(DRV_BM64_EVENT event, uint32_t param);

static uint8_t _volumeFormatTo4bits(uint8_t volume);
static uint8_t _volumeFormatTo7bits(uint8_t volume);
static uint8_t _volumeFormatFrom7bits(uint8_t vol_7bits);
static void _volumeUp(DRV_BM64_VOLUME_MODE mode);
static void _volumeDown(DRV_BM64_VOLUME_MODE mode);
static void _set4bitVol(uint8_t vol, DRV_BM64_VOLUME_MODE mode);
static void _set7bitVol(uint8_t vol, DRV_BM64_VOLUME_MODE mode);
static void _setCodecVolCurrentMode( void );
static uint8_t _get7bitsVolCurrentMode( void );

//*****************************************************************************
/* Driver Hardware instance objects.

  Summary:
    Defines the hardware instances objects for the BM64 Bluetooth module

  Description:
    This data type defines the hardware instance objects that are available for
    BM64 Bluetooth module, so as to capture the hardware state of the instance.
 */

//DRV_BM64_OBJ gDrvBm64Obj[DRV_BM64_INSTANCES_NUMBER];
DRV_BM64_OBJ    gDrvBm64Obj;


// *****************************************************************************
/* Driver Client instance objects.

  Summary:
    Defines the client instances objects

  Description:
    This data type defines the client instance objects that are available for
    BM64 Bluetooth module, so as to capture the client state of the instance.
    It uses the configuration of maximum number of clients which can get
    registered per hardware instance.
 */
DRV_BM64_CLIENT_OBJ gDrvBm64ClientObj[DRV_BM64_CLIENTS_NUMBER];


// *****************************************************************************
/* Driver common data object

  Summary:
    Defines the common data object

  Description:
    This object maintains data that is required by all BM64 Bluetooth
   driver instances

  Remarks:
    None
 */

DRV_BM64_COMMON_DATA_OBJ gDrvBm64CommonDataObj;

// *****************************************************************************
// *****************************************************************************
// Section: BM64 Bluetooth Driver System Routine Implementations
// *****************************************************************************
// *****************************************************************************

// *****************************************************************************
/* Function DRV_BM64_Initialize:

        void DRV_BM64_Initialize( void );

  Summary:
    Initializes hardware and data for the instance of the BM64 Bluetooth module

  Description:
    This routine initializes the BM64 driver instance for the specified driver
    index, making it ready for clients to open and use it. The initialization
    data is specified by the init parameter. The initialization may fail if the
    number of driver objects allocated are insufficient or if the specified
    driver instance is already initialized.

  Precondition:
    None.

  Parameters:
    None. 

  Returns:
    None.

  Remarks:
    This routine must be called before any other BM64 driver routine is called.
    This routine should only be called once during system initialization.
    This routine will never block for hardware access.

 */

void DRV_BM64_Initialize()
{
    /* Populate the driver object with the required data */
    gDrvBm64Obj.inUse                           = true;
    gDrvBm64Obj.status                          = SYS_STATUS_UNINITIALIZED;
    gDrvBm64Obj.numClients                      = 0;
    gDrvBm64Obj.isInInterruptContext            = false;

    gDrvBm64Obj.linkIndex = 0; 
    gDrvBm64Obj.request = 0;
    gDrvBm64Obj.linkStatus = DRV_BM64_NO_LINK_STATUS;
    gDrvBm64Obj.linkedMode = 0;
    gDrvBm64Obj.pairedRecordNumber = 0;
    gDrvBm64Obj.nSPKLinkedCounter = 0;
    gDrvBm64Obj.nSPKLinkingBackCounter = 0;    
    DRV_BM64_UART_Initialize(); 
        
    DRV_BM64_TaskReq(DRV_BM64_REQ_SYSTEM_ON);
    gDrvBm64Obj.state = DRV_BM64_STATE_INITIALIZE_START;
    
    _initStatus();
    DRV_BM64_CommandDecodeInit();
    DRV_BM64_CommandSendInit();
    
    gDrvBm64Obj.volume.a2dpVol = 20;
    gDrvBm64Obj.volume.hfpVol = 20;
    gDrvBm64Obj.volume.lineInVol = 20;
    gDrvBm64Obj.volume.currentVolMode = DRV_BM64_VOLUME_A2DP;
    
    gDrvBm64Obj.port1.port = 0;
    gDrvBm64Obj.port3.port = 0;
    
    gDrvBm64Obj.nextCommandReq.value = 0; 
    gDrvBm64Obj.nextMMIActionReq.value = 0;
    gDrvBm64Obj.nextMMIActionReq2.value = 0;   
    gDrvBm64Obj.playingStatus = DRV_BM64_PLAYING_STOPPED;

    _initSPP();

    /* Create the hardware instance mutex. */
     if(OSAL_MUTEX_Create(&(gDrvBm64Obj.mutexDriverInstance)) != OSAL_RESULT_TRUE)
     {
        return;
     }

    /* Check if the global mutexes have been created. If not
       then create these. */
     if(!gDrvBm64CommonDataObj.membersAreInitialized)
     {
         /* This means that mutexes where not created. Create them. */
         if((OSAL_MUTEX_Create(&(gDrvBm64CommonDataObj.mutexClientObjects)) != OSAL_RESULT_TRUE))
         {
            return;
         }
         /* Set this flag so that global mutexes get allocated only once */
         gDrvBm64CommonDataObj.membersAreInitialized = true;
     }

    gDrvBm64Obj.status = SYS_STATUS_READY;      // okay to open driver 
} /* DRV_BM64_Initialize */

// *****************************************************************************
/* Function DRV_BM64_Tasks:
 
        void  DRV_BM64_Tasks( void );

  Summary:
    Maintains the driver's control and data interface state machine.

  Description:
    This routine is used to maintain the driver's internal control and data
    interface state machine and implement its control and data interface
    implementations.  
 
    This function should be called from the SYS_Tasks() function.
  
  Precondition:
    None.

  Parameters:
    None. 

  Returns:
    None.

  Remarks:
    This routine is not normally called directly by an application.  Instead it
    is called by the system's Tasks routine (SYS_Tasks).
*/

void DRV_BM64_Tasks( void )
{
    if((false == gDrvBm64Obj.inUse))
    {
        /* This instance of the driver is not initialized. Don't do anything */
        return;
    }

    _BM64ControlTasks();    
}

// *****************************************************************************
/* Function DRV_BM64_Status:
 
        SYS_STATUS DRV_BM64_Status( void );

  Summary:
    Gets the current status of the BM64 Bluetooth driver module.

  Description:
    This routine provides the current status of the BM64 Bluetooth driver module,
    passed back as type SYS_STATUS.
 
  Precondition:
    None.

  Parameters:
    None. 

  Returns:
    Driver status, encoded as type SYS_STATUS enum:

    SYS_STATUS_DEINITIALIZED  - Indicates that the driver has been
                                deinitialized
    SYS_STATUS_READY          - Indicates that any previous module operation
                                for the specified module has completed
    SYS_STATUS_BUSY           - Indicates that a previous module operation for
                                the specified module has not yet completed
    SYS_STATUS_ERROR          - Indicates that the specified module is in an
                                error state

  Remarks:
    A driver can opened only when its status is SYS_STATUS_READY.
*/

SYS_STATUS DRV_BM64_Status( void )
{
    /* Return the status of the driver object */
    return gDrvBm64Obj.status;
} /* DRV_BM64_Status */


// *****************************************************************************
/* Function DRV_BM64_GetPowerStatus:

        DRV_BM64_DRVR_STATUS DRV_BM64_GetPowerStatus( void );

  Summary:
    Gets the current status of the BM64 Bluetooth driver module (BM64-specific).

  Description:
    This routine provides the current status (power on/off/ready) of the BM64
    Bluetooth driver module passed back as type DRV_BM64_DRVR_STATUS enum.

  Precondition:
    DRV_BM64_Initialize must have been called to initialize the driver instance.

  Parameters:
    None. 

  Returns:
    Driver status, encoded as type DRV_BM64_DRVR_STATUS enum.

  Remarks:
    None.
*/

DRV_BM64_DRVR_STATUS DRV_BM64_GetPowerStatus( void )
{
    if(gDrvBm64Obj.state == DRV_BM64_STATE_POWER_OFF)
        return DRV_BM64_STATUS_OFF;
    else if(gDrvBm64Obj.state >= DRV_BM64_STATE_POWER_ON && gDrvBm64Obj.state < DRV_BM64_STATE_BT_RUNNING)
        return DRV_BM64_STATUS_ON;
    else if(gDrvBm64Obj.state == DRV_BM64_STATE_BT_RUNNING)
        return DRV_BM64_STATUS_READY;
    else
        return DRV_BM64_STATUS_NONE;
    }

// *****************************************************************************
/* Function DRV_BM64_TaskReq:

        void DRV_BM64_TaskReq(DRV_BM64_REQUEST request);

  Summary:
    Make a power on/power off task request.

  Description:
    Make a power on/power off task request using the DRV_BM64_REQUEST enum.
  
  Precondition:
    DRV_BM64_Initialize must have been called to initialize the driver instance.

  Parameters:
    request        - power on/off request of type DRV_BM64_REQUEST

  Returns:
    None.

  Remarks:
    None.
*/

void DRV_BM64_TaskReq(DRV_BM64_REQUEST request)
{
    gDrvBm64Obj.request = request;
}

// *****************************************************************************
// *****************************************************************************
// Section: BM64 Bluetooth Driver Client Routines
// *****************************************************************************
// *****************************************************************************
// *****************************************************************************
/* Function DRV_BM64_Open: 

        DRV_HANDLE DRV_BM64_Open(const DRV_IO_INTENT ioIntent,
                          const DRV_BM64_PROTOCOL protocol);

  Summary:
    Opens the specified BM64 driver instance and returns a handle to it

  Description:
    This routine opens the specified BM64 Bluetooth driver instance and provides
    a handle that must be provided to all other client-level operations to
    identify the caller and the instance of the driver. The ioIntent
    parameter defines how the client interacts with this driver instance.

    Only DRV_IO_INTENT_READ is a valid ioIntent option as the BM64 Bluetooth
    driver audio stream is read-only.

    Specifying a DRV_IO_INTENT_EXCLUSIVE will cause the driver to provide
    exclusive access to this client. The driver cannot be opened by any
    other client.

  Precondition:
    DRV_BM64_Initialize must have been called to initialize the driver instance.

  Parameters:
    ioIntent        - valid handle to an opened BM64 device driver unique to client
    protocol        - specifies which protocol(s) the client intends to use
                      with this driver.  One of the various DRV_BM64_PROTOCOL 
                      enum values, including DRV_BM64_PROTOCOL_ALL.

  Returns:
    valid handle to an opened BM64 device driver unique to client

  Remarks:
    The handle returned is valid until the DRV_BM64_Close routine is called.
    This routine will never block waiting for hardware.  If the requested intent
    flags are not supported, the routine will return DRV_HANDLE_INVALID.  This
    function is thread safe in a RTOS application. It should not be called in an
    ISR.
 
    Currently only one client is allowed at a time.
*/

DRV_HANDLE DRV_BM64_Open
(
    const DRV_IO_INTENT ioIntent,
    const DRV_BM64_PROTOCOL protocol
)
{
    DRV_BM64_CLIENT_OBJ *hClient;
    uint32_t iClient;

    if (gDrvBm64Obj.status == SYS_STATUS_BUSY)
    {
        return DRV_HANDLE_INVALID;
    }
    
    if (gDrvBm64Obj.status != SYS_STATUS_READY)
    {
        /* The BM64  module should be ready */
        SYS_DEBUG(0, "Was the driver initialized? \r\n");
        return DRV_HANDLE_INVALID;
    }
    
    if ((gDrvBm64Obj.numClients > 0) && (true == gDrvBm64Obj.isExclusive))
    {
        /* Driver already opened in exclusive mode. Cannot open a new client. */
        SYS_DEBUG(0, "Cannot open a new client in exclusive mode \r\n");
        return DRV_HANDLE_INVALID;
    }

    if ((gDrvBm64Obj.numClients > 0) &&
        (DRV_IO_INTENT_EXCLUSIVE == (ioIntent & DRV_IO_INTENT_EXCLUSIVE)))
    {
        /*  A client Instance of driver is open.
            Cannot open the new client in exclusive mode */
            SYS_DEBUG(0, "Cannot open a new client in exclusive mode \r\n");
            return DRV_HANDLE_INVALID;
    }

    iClient = 0;
    hClient = (DRV_BM64_CLIENT_OBJ *)&gDrvBm64ClientObj[iClient];

    /* Grab client object mutex here */
    if(OSAL_MUTEX_Lock(&(gDrvBm64CommonDataObj.mutexClientObjects), OSAL_WAIT_FOREVER) == OSAL_RESULT_TRUE)
    {
        /* Setup client operations */
        /* Find available slot in array of client objects */
        for (; iClient < DRV_BM64_CLIENTS_NUMBER; iClient++)
        {
            if (false == hClient->inUse)
            {
                /* Remember which BM64 driver instance owns me */
                hClient->inUse  = true;
                hClient->pEventCallBack = NULL;
                hClient->protocol = protocol;
                gDrvBm64Obj.numClients++; 
                
                /* We have found a client object
                 * Release the mutex and return with
                 * the driver handle */
                /* An operation mode is needed */
                if((OSAL_MUTEX_Unlock(&(gDrvBm64CommonDataObj.mutexClientObjects))) != OSAL_RESULT_TRUE)
                {
                    SYS_DEBUG(0, "Unable to unlock open routine mutex \r\n");
                    return DRV_HANDLE_INVALID;
                }
                /* Return the client object */
                return (DRV_HANDLE) hClient;
            }
            hClient++;
        }
        /* Could not find a client object. Release the mutex and
         * return with an invalid handle. */
        if((OSAL_MUTEX_Unlock(&(gDrvBm64CommonDataObj.mutexClientObjects))) != OSAL_RESULT_TRUE)
        {
            SYS_DEBUG(0, "Unable to unlock open routine mutex \r\n");
        }
    }
    return DRV_HANDLE_INVALID;
}

// *****************************************************************************
/* Function DRV_BM64_Close: 

        void DRV_BM64_Close(DRV_HANDLE handle);

  Summary:
    Closes an opened-instance of the BM64 Bluetooth driver

  Description:
    This routine closes an opened-instance of the BM64 driver, invalidating the
    handle. Any buffers in the driver queue that were submitted by this client
    will be removed.  After calling this routine, the handle passed in "handle"
    must not be used with any of the remaining driver routines.  A new handle must
    be obtained by calling DRV_BM64_Open before the caller may use the driver
    again

  Precondition:
    DRV_BM64_Open must have been called to obtain a valid opened device handle.

  Parameters:
    handle          - valid handle to an opened BM64 device driver unique to client

  Returns:
    None.

  Remarks:
    Usually there is no need for the driver client to verify that the Close
    operation has completed.  The driver will abort any ongoing operations
    when this routine is called.
*/
void DRV_BM64_Close( const DRV_HANDLE handle)
{
    DRV_BM64_CLIENT_OBJ *clientObj;

    if(handle == DRV_HANDLE_INVALID || (DRV_HANDLE)NULL == handle)
    {
        SYS_DEBUG(0, "Invalid Driver Handle \r\n");
        return;
    }

    clientObj = (DRV_BM64_CLIENT_OBJ *) handle;
    if (false == clientObj->inUse)
    {
        SYS_DEBUG(0, "Invalid Driver Handle \r\n");
        return;
    }

    /* De-allocate the object */
    clientObj->inUse = false;
    /* Reduce the number of clients */
    gDrvBm64Obj.numClients--;
} /* DRV_BM64_Close */

// *****************************************************************************
/* Function DRV_BM64_BufferEventHandlerSet:

       void DRV_BM64_BufferEventHandlerSet(DRV_HANDLE handle,
            const DRV_BM64_BUFFER_EVENT_HANDLER eventHandler,
            const uintptr_t contextHandle);

  Summary:
    This function allows a client to identify a event handling function
    for the driver to call back.

  Description:
    This function allows a client to identify a command event handling function
    for the driver to call back when the last submitted command have finished.

    When a client calls DRV_BM64_BufferAddRead function, it is provided with
    a handle identifying the buffer that was added to the driver's buffer queue.
    The driver will pass this handle back to the client by calling "eventHandler"
    function when the buffer transfer has completed.
 
    The context parameter contains a handle to the client context,
    provided at the time the event handling function is registered using the
    DRV_BM64_BufferEventHandlerSet function.  This context handle value is
    passed back to the client as the "context" parameter.  It can be any value
    necessary to identify the client context or instance (such as a pointer to
    the client's data) instance of the client.

    The event handler should be set before the client performs any "BM64 Bluetooth
    Specific Client Routines" operations that could generate events.
    The event handler once set, persists until the client closes the driver or
    sets another event handler (which could be a "NULL" pointer to indicate no callback).
 
  Precondition:
    DRV_BM64_Open must have been called to obtain a valid opened device handle.

  Parameters:
    handle          - valid handle to an opened BM64 device driver unique to client
    eventHandler    - pointer to a function to be called back (prototype defined
                      by DRV_BM64_BUFFER_EVENT_HANDLER)
    contextHandle   - handle to the client context

  Returns:
    None.

  Remarks:
    If the client does not want to be notified when the command has completed, 
    it does not need to register a callback.
*/


// *****************************************************************************
/*
Function:
	void DRV_BM6_BufferAddRead(const DRV_HANDLE handle,
		DRV_BM6_BUFFER_HANDLE *bufferHandle, void *buffer, size_t size)

  Summary:
    Schedule a non-blocking driver write operation.

  Description:
    This function schedules a non-blocking read operation. The function returns
    with a valid buffer handle in the bufferHandle argument if the read request
    was scheduled successfully. The function adds the request to the hardware
    instance receive queue and returns immediately. While the request is in the
    queue, the application buffer is owned by the driver and should not be
    modified.  The function returns DRV_BM64_BUFFER_HANDLE_INVALID
    - if a buffer could not be allocated to the request
    - if the input buffer pointer is NULL
    - if the buffer size is 0.
    - if the queue is full or the queue depth is insufficient
    If the requesting client registered an event callback with the driver,
    the driver will issue a DRV_BM64_BUFFER_EVENT_COMPLETE event if the buffer
    was processed successfully of DRV_BM64_BUFFER_EVENT_ERROR event if the
    buffer was not processed successfully.

  Precondition:
    DRV_BM64_Open must have been called to obtain a valid opened device handle.

  Parameters:
    handle       - valid handle to an opened BM64 device driver unique to client
    bufferHandle - pointer to an argument that contains the return buffer handle
    buffer       - pointer to buffer that will contain the received data
    size         - buffer size in bytes.

  Returns:
    The bufferHandle parameter will contain the return buffer handle. This will
    be DRV_BM64_BUFFER_HANDLE_INVALID if the function was not successful.

   Remarks:
    This function is thread safe in a RTOS application. It can be called from
    within the BM64 Driver Buffer Event Handler that is registered by this
    client. It should not be called in the event handler associated with another
    BM64 driver instance. It should not otherwise be called directly in an ISR.

*/


// *****************************************************************************
/* Function DRV_BM64_EventHandlerSet:

        void DRV_BM64_EventHandlerSet(DRV_HANDLE handle,
            const DRV_BM64_EVENT_HANDLER eventHandler,
            const uintptr_t contextHandle);

  Summary:
    This function allows a client to identify a event handling function
    for the driver to call back.

  Description:
    This function allows a client to identify a command event handling function
    for the driver to call back when an event has been received from the BM64.
 
   The context parameter contains a handle to the client context,
    provided at the time the event handling function is registered using the
    DRV_BM64_BufferEventHandlerSet function.  This context handle value is
    passed back to the client as the "context" parameter.  It can be any value
    necessary to identify the client context or instance (such as a pointer to
    the client's data) instance of the client.* 

    The event handler should be set before the client performs any "BM64 Bluetooth
    Specific Client Routines" operations that could generate events.
    The event handler once set, persists until the client closes the driver or
    sets another event handler (which could be a "NULL" pointer to indicate no callback).
 
  Precondition:
    DRV_BM64_Open must have been called to obtain a valid opened device handle.

  Parameters:
    handle          - valid handle to an opened BM64 device driver unique to client
    eventHandler    - pointer to a function to be called back (prototype defined
                      by DRV_BM64_EVENT_HANDLER)
    contextHandle   - handle to the client context

  Returns:
    None.

  Remarks:
    If the client does not want to be notified when an event has occurred, it
    does not need to register a callback.
*/

void DRV_BM64_EventHandlerSet
(
	DRV_HANDLE handle,
	const DRV_BM64_EVENT_HANDLER eventHandler,   // TEMP!!
	const uintptr_t contextHandle
)
{
    DRV_BM64_CLIENT_OBJ *clientObj;

    if((DRV_HANDLE_INVALID == handle) || (0 == handle))
    {
        /* This means the handle is invalid */
        SYS_DEBUG(0, "Handle is invalid \r\n");
        return;
    }

    clientObj = (DRV_BM64_CLIENT_OBJ *) handle;
    /* Assing the event handler and the context */
    if(false == clientObj->inUse)
    {
        SYS_DEBUG(0, "Invalid driver handle \r\n");
        return;
    }

    clientObj->pEventCallBack = eventHandler;
    clientObj->hClientArg = contextHandle;
}

// *****************************************************************************
/* Function DRV_BM64_volumeUp:

        void DRV_BM64_volumeUp(const DRV_HANDLE handle);

  Summary:
    Turn the volume up on the host device.

  Description:
    Turn the volume up on the host device by one increment (about 3% of
    full-scale).
 
  Precondition:
    DRV_BM64_Open must have been called to obtain a valid opened device handle.

  Parameters:
    handle      - valid handle to an opened BM64 device driver unique to client

  Returns:
    None.

  Remarks:
    None.
*/

void DRV_BM64_volumeUp(const DRV_HANDLE handle)
{
    switch(gDrvBm64Obj.volume.currentVolMode)
    {
        case DRV_BM64_VOLUME_A2DP:
            if (DRV_BM64_eCSBStatus.nspk_link != DRV_BM64_NSPK_NO_LINK && gDrvBm64Obj.state >= DRV_BM64_STATE_POWER_ON ) //NSPK mode
            {
                if (DRV_BM64_IsAllowedToSendCommand()) {
                    DRV_BM64_MMI_ActionCommand(DRV_BM64_MMI_INC_SPK_GAIN, gDrvBm64Obj.linkIndex);
                }
                else {
                    gDrvBm64Obj.nextMMIActionReq.IncSpkGainReq = 1;
                }
            }
            else //single speaker mode or NSPK connecting
            {
                _volumeUp(DRV_BM64_VOLUME_A2DP);
                if(gDrvBm64Obj.state >= DRV_BM64_STATE_POWER_ON)
                {
                    if (DRV_BM64_IsAllowedToSendCommand()) {
                        DRV_BM64_updateA2DPGain(_volumeFormatTo7bits(gDrvBm64Obj.volume.a2dpVol));
                    }
                    else {
                        gDrvBm64Obj.nextCommandReq.updateA2DPGainReq = 1;
                    }
                }
            }
            break;
        case DRV_BM64_VOLUME_HFP:
            _volumeUp(DRV_BM64_VOLUME_HFP);
            if(gDrvBm64Obj.state >= DRV_BM64_STATE_POWER_ON)
            {
                if (DRV_BM64_IsAllowedToSendCommand()) {
                    DRV_BM64_updateHFPGain(_volumeFormatTo4bits(gDrvBm64Obj.volume.hfpVol));
                }
                else {
                    gDrvBm64Obj.nextCommandReq.updateHFPGainReq = 1;
                }
            }
            break;
        case DRV_BM64_VOLUME_LINEIN:
            _volumeUp(DRV_BM64_VOLUME_LINEIN);
            if(gDrvBm64Obj.state >= DRV_BM64_STATE_POWER_ON)
            {
                if (DRV_BM64_IsAllowedToSendCommand()) {
                    DRV_BM64_updateLineInGain(_volumeFormatTo4bits(gDrvBm64Obj.volume.lineInVol));
                }
                else {
                    gDrvBm64Obj.nextCommandReq.updateLineInGainReq = 1;
                }
            }
            break;
    }
    _setCodecVolCurrentMode();      // call back to client to set codec volume
}

// *****************************************************************************
/* Function DRV_BM64_volumeDown:

        void DRV_BM64_volumeDown(const DRV_HANDLE handle);

  Summary:
    Turn the volume down on the host device.

  Description:
    Turn the volume down on the host device by one increment (about 3% of
    full-scale).
 
  Precondition:
    DRV_BM64_Open must have been called to obtain a valid opened device handle.

  Parameters:
    handle      - valid handle to an opened BM64 device driver unique to client

  Returns:
    None.

  Remarks:
    None.
*/

void DRV_BM64_volumeDown(const DRV_HANDLE handle)
{
    switch(gDrvBm64Obj.volume.currentVolMode)
    {
        case DRV_BM64_VOLUME_A2DP:
            if (DRV_BM64_eCSBStatus.nspk_link != DRV_BM64_NSPK_NO_LINK && gDrvBm64Obj.state >= DRV_BM64_STATE_POWER_ON ) //NSPK mode
            {
                if (DRV_BM64_IsAllowedToSendCommand()) {
                    DRV_BM64_MMI_ActionCommand(DRV_BM64_MMI_DEC_SPK_GAIN, gDrvBm64Obj.linkIndex);
                }
                else {
                    gDrvBm64Obj.nextMMIActionReq.DecSpkGainReq  = 1;
                }
            }
            else //single speaker mode or NSPK connecting
            {
				_volumeDown(DRV_BM64_VOLUME_A2DP);
                if(gDrvBm64Obj.state >= DRV_BM64_STATE_POWER_ON)
                {
                    if (DRV_BM64_IsAllowedToSendCommand()) {
                        DRV_BM64_updateA2DPGain(_volumeFormatTo7bits(gDrvBm64Obj.volume.a2dpVol));
                    }
                    else {
                        gDrvBm64Obj.nextCommandReq.updateA2DPGainReq = 1;
                    }
                }
            }
            break;
        case DRV_BM64_VOLUME_HFP:
            _volumeDown(DRV_BM64_VOLUME_HFP);
            if(gDrvBm64Obj.state >= DRV_BM64_STATE_POWER_ON)
            {
                if (DRV_BM64_IsAllowedToSendCommand()) {
                    DRV_BM64_updateHFPGain(_volumeFormatTo4bits(gDrvBm64Obj.volume.hfpVol));
                }
                else {
                    gDrvBm64Obj.nextCommandReq.updateHFPGainReq = 1;
                }
            }
            break;
        case DRV_BM64_VOLUME_LINEIN:
            _volumeDown(DRV_BM64_VOLUME_LINEIN);
            if(gDrvBm64Obj.state >= DRV_BM64_STATE_POWER_ON)
            {
                if (DRV_BM64_IsAllowedToSendCommand()) {
                    DRV_BM64_updateLineInGain(_volumeFormatTo4bits(gDrvBm64Obj.volume.lineInVol));
                }
                else {
                    gDrvBm64Obj.nextCommandReq.updateLineInGainReq = 1;
                }
            }
            break;
    }
    _setCodecVolCurrentMode();      // call back to client to set codec volume
}

// *****************************************************************************
/* Function DRV_BM64_VolumeGet:

        uint8_t DRV_BM64_VolumeGet(const DRV_HANDLE handle);

  Summary:
    Return current volume level.

  Description:
    Returns volume for current mode (A2DP, HFP etc.) in percent (0-100).
 
  Precondition:
    DRV_BM64_Open must have been called to obtain a valid opened device handle.

  Parameters:
    handle      - valid handle to an opened BM64 device driver unique to client

  Returns:
    volume level in percent, 0-100

  Remarks:
    None.
*/

uint8_t DRV_BM64_volumeGet(DRV_HANDLE handle)
{
    uint8_t volume = _get7bitsVolCurrentMode();
    volume = (100*volume) / 127;        // convert to percent
    return volume;
}

// *****************************************************************************
/* Function DRV_BM64_VolumeSet:

        void DRV_BM64_VolumeSet(const DRV_HANDLE handle, uint8_t volume);

  Summary:
    Set current volume.

  Description:
    Set volume for current mode (A2DP, HFP etc.) in percent (0-100).
 
  Precondition:
    DRV_BM64_Open must have been called to obtain a valid opened device handle.

  Parameters:
    handle      - valid handle to an opened BM64 device driver unique to client
    volume      - volume level in percent, 0-100

  Returns:
    None.

  Remarks:
    None.
*/
void DRV_BM64_volumeSet(DRV_HANDLE handle, uint8_t volume)
{
    uint8_t *p = NULL;
    uint16_t volume16bits;
    
    volume16bits = (127*volume)/100;        // convert from percent to 0-127    
    
    switch(gDrvBm64Obj.volume.currentVolMode)
    {
        case DRV_BM64_VOLUME_A2DP:
            DRV_BM64_updateA2DPGain(volume16bits);
            p = &gDrvBm64Obj.volume.a2dpVol;
            break;
        case DRV_BM64_VOLUME_HFP:
            DRV_BM64_updateHFPGain(volume16bits);
            p = &gDrvBm64Obj.volume.hfpVol;
            break;
        case DRV_BM64_VOLUME_LINEIN:
            DRV_BM64_updateLineInGain(volume16bits);
            p = &gDrvBm64Obj.volume.lineInVol;
            break;
    }
    if (p != NULL)
    {
    	*p = _volumeFormatFrom7bits(volume16bits);
	}
}

// *****************************************************************************
/* Function DRV_BM64_SamplingRateGet:

        uint32_t DRV_BM64_SamplingRateGet(DRV_HANDLE handle);

  Summary:
    Return the current sampling rate.

  Description:
    Return the current sampling rate as a 32-bit integer.
 
  Precondition:
    DRV_BM64_Open must have been called to obtain a valid opened device handle.

  Parameters:
    handle      - valid handle to an opened BM64 device driver unique to client

  Returns:
    None.

  Remarks:
    xxx
*/

uint32_t DRV_BM64_SamplingRateGet(DRV_HANDLE handle)
{
    /* Return the sampling rate */
    return gDrvBm64Obj.samplingRate;
}

// *****************************************************************************
/* Function DRV_BM64_SamplingRateSet:

        void DRV_BM64_SamplingRateSet(DRV_HANDLE handle, uint32_t samplingRate);

  Summary:
    Set the current sampling rate.

  Description:
    Set the current sampling rate (passed as a 32-bit integer).
 
  Precondition:
    DRV_BM64_Open must have been called to obtain a valid opened device handle.

  Parameters:
    handle      - valid handle to an opened BM64 device driver unique to client
    samplingRate - sampling rate in Hz (8000, 16000, 44100 or 48000)

  Returns:
    None.

  Remarks:
    xxx
*/


// *****************************************************************************
/* Function DRV_BM64_EnterBTPairingMode:

        void DRV_BM64_EnterBTPairingMode(const DRV_HANDLE handle);

  Summary:
    Enter Bluetooth pairing mode.

  Description:
    Starting the pairing process, making this BM64 available for pairing with a
    Bluetooth host.
 
  Precondition:
    DRV_BM64_Open must have been called to obtain a valid opened device handle.

  Parameters:
    handle      - valid handle to an opened BM64 device driver unique to client

  Returns:
    None.

  Remarks:
    None.
*/

void DRV_BM64_EnterBTPairingMode(const DRV_HANDLE handle)
{
    if(gDrvBm64Obj.state == DRV_BM64_STATE_BT_RUNNING)
    {
        if (DRV_BM64_IsAllowedToSendCommand()) {
            DRV_BM64_MMI_ActionCommand(DRV_BM64_MMI_ANY_MODE_ENTERING_PAIRING, gDrvBm64Obj.linkIndex);
            DRV_BM64_LinkbackStatus = DRV_BM64_PAIRING_START;
        }
        else {
            gDrvBm64Obj.nextMMIActionReq.PairReq = 1;
        }
    }
}

// *****************************************************************************
/* Function DRV_BM64_ForgetAllLinks:

        void DRV_BM64_ForgetAllLinks(const DRV_HANDLE handle);

  Summary:
    Forget all links.

  Description:
    Forget (erase) all links and pairings stored in EEPROM.
 
  Precondition:
    DRV_BM64_Open must have been called to obtain a valid opened device handle.

  Parameters:
    handle      - valid handle to an opened BM64 device driver unique to client

  Returns:
    None.

  Remarks:
    After this is called, one must call DRV_BM64_EnterBTPairingMode to establish
    a connection to a Bluetooth host again. 
*/

void DRV_BM64_ForgetAllLinks(const DRV_HANDLE handle)       // was DRV_BM64_ResetEEPROMtoDefault
{
    if (DRV_BM64_IsAllowedToSendCommand()) {
        DRV_BM64_MMI_ActionCommand ( DRV_BM64_MMI_RESET_EEPROM_SETTING, gDrvBm64Obj.linkIndex);
    }
    else {
        gDrvBm64Obj.nextMMIActionReq.ResetEEPROMReq = 1;
    }
}

// *****************************************************************************
/* Function DRV_BM64_LinkLastDevice:

        void DRV_BM64_LinkLastDevice(const DRV_HANDLE handle);

  Summary:
    Link last device.

  Description:
    Link (connect) to last device that was previously linked.
 
  Precondition:
    DRV_BM64_Open must have been called to obtain a valid opened device handle.

  Parameters:
    handle      - valid handle to an opened BM64 device driver unique to client

  Returns:
    None.

  Remarks:
    None.
*/

void DRV_BM64_LinkLastDevice(const DRV_HANDLE handle)       // new
{
    if (DRV_BM64_IsAllowedToSendCommand()) {
        DRV_BM64_MMI_ActionCommand ( DRV_BM64_MMI_LINK_BACK_DEVICE, gDrvBm64Obj.linkIndex);
    }
    else {
        gDrvBm64Obj.nextMMIActionReq.LinkLastDeviceReq = 1;
    }
}

// *****************************************************************************
/* Function DRV_BM64_DisconnectAllLinks:

        void DRV_BM64_DisconnectAllLinks(const DRV_HANDLE handle);

  Summary:
    Disconnect all links.

  Description:
    Disconnect all current links to a Bluetooth host.
 
  Precondition:
    DRV_BM64_Open must have been called to obtain a valid opened device handle.

  Parameters:
    handle      - valid handle to an opened BM64 device driver unique to client

  Returns:
    None.

  Remarks:
    Does not unpair the device, just disconnects.  Use DRV_BM64_LinkLastDevice
    to reconnect.  Use DRV_BM64_ForgetAllLinks to forget all pairings.
*/

void DRV_BM64_DisconnectAllLinks(const DRV_HANDLE handle)       // new
{
    if (DRV_BM64_IsAllowedToSendCommand()) {
        DRV_BM64_MMI_ActionCommand ( DRV_BM64_MMI_DISCONNECT_ALL_LINK, gDrvBm64Obj.linkIndex);
    }
    else {
        gDrvBm64Obj.nextMMIActionReq.DisconnectAllLinksReq = 1;
    }
}

// *****************************************************************************
/* Function DRV_BM64_GetLinkStatus:

        DRV_BM64_LINKSTATUS DRV_BM64_GetLinkStatus(const DRV_HANDLE handle);

  Summary:
    Return link status.

  Description:
    Returns a 8-bit value containing current link status as bit flags for
        SCO (bit 0), ACL, HFP, A2DP, AVRCP, SPP, IAP, MAP (bit 7)
 
  Precondition:
    DRV_BM64_Open must have been called to obtain a valid opened device handle.

  Parameters:
    handle      - valid handle to an opened BM64 device driver unique to client

  Returns:
    8-bit value defined by DRV_BM64_LINKSTATUS enum.

  Remarks:
    None.
*/

DRV_BM64_LINKSTATUS DRV_BM64_GetLinkStatus(const DRV_HANDLE handle)
{
    return gDrvBm64Obj.linkStatus;
}


// *****************************************************************************
/* Function DRV_BM64_Play:

        DRV_BM64_Play(const DRV_HANDLE handle);

  Summary:
    Toggle play/pause mode.

  Description:
    Send an AVRCP command to the host device to initiate or resume playback.
 
  Precondition:
    DRV_BM64_Open must have been called to obtain a valid opened device handle.

  Parameters:
    handle      - valid handle to an opened BM64 device driver unique to client

  Returns:
    None.

  Remarks:
    None.
*/

/*-----------------------------------------------------------------------------*/
void DRV_BM64_Play(const DRV_HANDLE handle)
{
    if (DRV_BM64_IsAllowedToSendCommand())
    {
        DRV_BM64_MusicControlCommand(0x05);
    }
    else
    {
        gDrvBm64Obj.nextCommandReq.musicCtrlReq_05 = 1;
    }
}

// *****************************************************************************
/* Function DRV_BM64_Pause:

        void DRV_BM64_Pause(const DRV_HANDLE handle);

  Summary:
    Pause playback.

  Description:
    Send an AVRCP command to the host device to pause.
 
  Precondition:
    DRV_BM64_Open must have been called to obtain a valid opened device handle.

  Parameters:
    handle      - valid handle to an opened BM64 device driver unique to client

  Returns:
    None.

  Remarks:
    None.
*/

void DRV_BM64_Pause(const DRV_HANDLE handle)
{
    if (DRV_BM64_IsAllowedToSendCommand())
    {
        DRV_BM64_MusicControlCommand(0x06);
    }
    else
    {
        gDrvBm64Obj.nextCommandReq.musicCtrlReq_06 = 1;
    }
}

// *****************************************************************************
/* Function DRV_BM64_Stop:

        void DRV_BM64_Stop(const DRV_HANDLE handle);

  Summary:
    Stop playback.

  Description:
    Send an AVRCP command to the host device to stop playback.
 
  Precondition:
    DRV_BM64_Open must have been called to obtain a valid opened device handle.

  Parameters:
    handle      - valid handle to an opened BM64 device driver unique to client

  Returns:
    None.

  Remarks:
    None.
*/

void DRV_BM64_Stop(const DRV_HANDLE handle)
{
    if (DRV_BM64_IsAllowedToSendCommand())
    {
        DRV_BM64_MusicControlCommand(0x08);
    }
    else
    {
        gDrvBm64Obj.nextCommandReq.musicCtrlReq_08 = 1;
    }
}

// *****************************************************************************
/* Function DRV_BM64_PlayPause:

        void DRV_BM64_PlayPause(const DRV_HANDLE handle);

  Summary:
    Toggle play/pause mode.

  Description:
    Send an AVRCP command to the host device to toggle the play/pause mode.
 
  Precondition:
    DRV_BM64_Open must have been called to obtain a valid opened device handle.

  Parameters:
    handle      - valid handle to an opened BM64 device driver unique to client

  Returns:
    None.

  Remarks:
    None.
*/

void DRV_BM64_PlayPause(const DRV_HANDLE handle)
{
    if(DRV_BM64_eCSBStatus.nspk_link == DRV_BM64_NSPK_SLAVE_LINK_TO_MASTER || DRV_BM64_eCSBStatus.nspk_link == DRV_BM64_BROADCAST_SLAVE)    //slave
    {
        if (DRV_BM64_IsAllowedToSendCommand())
            DRV_BM64_MMI_ActionCommand ( 0x32, gDrvBm64Obj.linkIndex);
        else
            gDrvBm64Obj.nextMMIActionReq2.PlayPauseReq = 1;
    }
    else
    {
        if (DRV_BM64_IsAllowedToSendCommand())
        {
            DRV_BM64_MusicControlCommand(0x07);
        }
        else
        {
            gDrvBm64Obj.nextCommandReq.musicCtrlReq_07 = 1;
        }
    }
}

// *****************************************************************************
/* Function DRV_BM64_PlayPreviousSong:

        void DRV_BM64_PlayPreviousSong(const DRV_HANDLE handle);

  Summary:
    Play the previous song.

  Description:
    Send an AVRCP command to the host device to play the previous song in a 
    playlist.
 
  Precondition:
    DRV_BM64_Open must have been called to obtain a valid opened device handle.

  Parameters:
    handle      - valid handle to an opened BM64 device driver unique to client

  Returns:
    None.

  Remarks:
    None.
*/

void DRV_BM64_PlayPreviousSong(const DRV_HANDLE handle)
{
    if (DRV_BM64_IsAllowedToSendCommand()) {
        DRV_BM64_MMI_ActionCommand ( DRV_BM64_MMI_PREVIOUS_SONG, gDrvBm64Obj.linkIndex);
    }
    else{
        gDrvBm64Obj.nextMMIActionReq.PreviousSongReq = 1;
    }
}

// *****************************************************************************
/* Function DRV_BM64_PlayNextSong:

        void DRV_BM64_PlayNextSong(const DRV_HANDLE handle);

  Summary:
    Play the next song.

  Description:
    Send an AVRCP command to the host device to play the next song in a 
    playlist.
 
  Precondition:
    DRV_BM64_Open must have been called to obtain a valid opened device handle.

  Parameters:
    handle      - valid handle to an opened BM64 device driver unique to client

  Returns:
    None.

  Remarks:
    None.
*/

/*-----------------------------------------------------------------------------*/
void DRV_BM64_PlayNextSong(const DRV_HANDLE handle)
{
    if (DRV_BM64_IsAllowedToSendCommand()) {
        DRV_BM64_MMI_ActionCommand ( DRV_BM64_MMI_NEXT_SONG, gDrvBm64Obj.linkIndex);
    }
    else {
        gDrvBm64Obj.nextMMIActionReq.NextSongReq = 1;
    }
}

// *****************************************************************************
/* Function DRV_BM64_Rewind:

        void DRV_BM64_Rewind(const DRV_HANDLE handle);

  Summary:
    Rewind the media.

  Description:
    Send an AVRCP command to the host device to rewind the media.
 
  Precondition:
    DRV_BM64_Open must have been called to obtain a valid opened device handle.

  Parameters:
    handle      - valid handle to an opened BM64 device driver unique to client

  Returns:
    None.

  Remarks:
    None.
*/

void DRV_BM64_Rewind(const DRV_HANDLE handle)
{
    if (DRV_BM64_IsAllowedToSendCommand())
    {
        DRV_BM64_MusicControlCommand(0x03);
    }
    else
    {
        gDrvBm64Obj.nextCommandReq.musicCtrlReq_03 = 1;
    }
}

// *****************************************************************************
/* Function DRV_BM64_FastForward:

        void DRV_BM64_FastForward(const DRV_HANDLE handle);

  Summary:
    Fast forward the media.

  Description:
    Send an AVRCP command to the host device to Fast forward the media.
 
  Precondition:
    DRV_BM64_Open must have been called to obtain a valid opened device handle.

  Parameters:
    handle      - valid handle to an opened BM64 device driver unique to client

  Returns:
    None.

  Remarks:
    None.
*/

void DRV_BM64_FastForward(const DRV_HANDLE handle)
{
    if (DRV_BM64_IsAllowedToSendCommand())
    {
        DRV_BM64_MusicControlCommand(0x01);
    }
    else
    {
        gDrvBm64Obj.nextCommandReq.musicCtrlReq_01 = 1;
    }
}

// *****************************************************************************
/* Function DRV_BM64_CancelForwardOrRewind:

        void DRV_BM64_CancelForwardOrRewind(const DRV_HANDLE handle);

  Summary:
    Cancel previous fast forward or rewind request.

  Description:
    Send an AVRCP command to the host device to cancel a previous fast forward
    or rewind request.
 
  Precondition:
    DRV_BM64_Open must have been called to obtain a valid opened device handle.

  Parameters:
    handle      - valid handle to an opened BM64 device driver unique to client

  Returns:
    None.

  Remarks:
    None.
*/

void DRV_BM64_CancelForwardOrRewind(const DRV_HANDLE handle)
{
    if (DRV_BM64_IsAllowedToSendCommand())
    {
        DRV_BM64_MusicControlCommand(0x00);
    }
    else
    {
        gDrvBm64Obj.nextCommandReq.musicCtrlReq_00 = 1;
    }
}

// *****************************************************************************
/* Function DRV_BM64_GetPlayingStatus:

        void DRV_BM64_GetPlayingStatus(const DRV_HANDLE handle);

  Summary:
    Return the current playing status of the device.

  Description:
    Return the current AVRCP playing status of the device, e.g. stopped,
    playing, paused, fast forward or rewind, encoded as as the enum 
    DRV_BM64_PLAYINGSTATUS.
 
  Precondition:
    DRV_BM64_Open must have been called to obtain a valid opened device handle.

  Parameters:
    handle      - valid handle to an opened BM64 device driver unique to client

  Returns:
    None.

  Remarks:
    None.
 */

DRV_BM64_PLAYINGSTATUS DRV_BM64_GetPlayingStatus(const DRV_HANDLE handle)
{
    return gDrvBm64Obj.playingStatus;
}

// *****************************************************************************
/* Function DRV_BM64_GetBDAddress:

        void DRV_BM64_GetBDAddress(const DRV_HANDLE handle, char* buffer);

  Summary:
    Return the Bluetooth address.

  Description:
    Return the Bluetooth address of the device as an ASCII string.
 
  Precondition:
    DRV_BM64_Open must have been called to obtain a valid opened device handle.

  Parameters:
    handle      - valid handle to an opened BM64 device driver unique to client
    buffer      - pointer to a char buffer at least 18 bytes long

  Returns:
    None.

  Remarks:
    Buffer must be at least 18 bytes in length (6 octets separated by ?:?, e.g.
    able to hold "12:34:56:78:90:12\0").
*/

void DRV_BM64_GetBDAddress(const DRV_HANDLE handle, char* buffer)
{
    buffer[0] = ((_localBDAddr[5]>>4)&0xF)+'0';
    if (buffer[0] > '9') buffer[0]+=7;
    buffer[1] = (_localBDAddr[5]&0xF)+'0';
    if (buffer[1] > '9') buffer[1]+=7;
    buffer[2] = ':';
    
    buffer[3] = ((_localBDAddr[4]>>4)&0xF)+'0';
    if (buffer[3] > '9') buffer[3]+=7;
    buffer[4] = (_localBDAddr[4]&0xF)+'0';
    if (buffer[4] > '9') buffer[4]+=7;
    buffer[5] = ':'; 
    
    buffer[6] = ((_localBDAddr[3]>>4)&0xF)+'0';
    if (buffer[6] > '9') buffer[6]+=7;
    buffer[7] = (_localBDAddr[3]&0xF)+'0';
    if (buffer[7] > '9') buffer[7]+=7;
    buffer[8] = ':';
    
    buffer[9] = ((_localBDAddr[2]>>4)&0xF)+'0';
    if (buffer[9] > '9') buffer[9]+=7;
    buffer[10] = (_localBDAddr[2]&0xF)+'0';
    if (buffer[10] > '9') buffer[10]+=7;
    buffer[11] = ':';

    buffer[12] = ((_localBDAddr[1]>>4)&0xF)+'0';
    if (buffer[12] > '9') buffer[12]+=7;
    buffer[13] = (_localBDAddr[1]&0xF)+'0';
    if (buffer[13] > '9') buffer[13]+=7;
    buffer[14] = ':';
    
    buffer[15] = ((_localBDAddr[0]>>4)&0xF)+'0';
    if (buffer[15] > '9') buffer[15]+=7;
    buffer[16] = (_localBDAddr[0]&0xF)+'0';
    if (buffer[16] > '9') buffer[16]+=7;
    buffer[17] = '\0';
}

// *****************************************************************************
/* Function DRV_BM64_GetBDName:

        void DRV_BM64_GetBDName(const DRV_HANDLE handle, char* buffer, 
                const uint8_t buflen);

  Summary:
    Return Bluetooth device name.

  Description:
    Return the Bluetooth device name as an ASCII string.
 
  Precondition:
    DRV_BM64_Open must have been called to obtain a valid opened device handle.

  Parameters:
    handle      - valid handle to an opened BM64 device driver unique to client
    buffer      - pointer to a char buffer at least buflen bytes long
    buflen      - length of buffer (including terminating \0 byte)

  Returns:
    None.

  Remarks:
    If name is longer than buflen-1 bytes long, it will be truncated to fit
    inside the buffer.
 */


void DRV_BM64_GetBDName(const DRV_HANDLE handle, char* buffer, const uint8_t buflen)
{
    uint8_t i;
    char ch;
    for (i=0; i < (buflen-1); i++)
{
        ch = _localBDName[i];      
        if (ch == '\0')
{
            break;
        }
        buffer[i] = ch;
    }
    buffer[i] = '\0';     
}

// *****************************************************************************
/* Function DRV_BM64_SetBDName:

        void DRV_BM64_SetBTName(const DRV_HANDLE handle, const char* buffer);

  Summary:
    Set the Bluetooth device name.

  Description:
    Set a temporary Bluetooth device name from an ASCII string buffer.
 
  Precondition:
    DRV_BM64_Open must have been called to obtain a valid opened device handle.

  Parameters:
    handle      - valid handle to an opened BM64 device driver unique to client
    buffer      - pointer to a char buffer containing the new name

  Returns:
    None.

  Remarks:
    The name is set for this session only; if the BM64 is reset (e.g. power is
    lost) the name will revert to the Bluetooth name stored in EEPROM.
*/

void DRV_BM64_SetBDName(const DRV_HANDLE handle, const char* buffer)
{
    char ch;
    uint8_t i;
    for (i=0; i < DRV_BM64_MAXBDNAMESIZE; i++)
{
        ch = buffer[i];
        if (ch=='\0')
    {
            break;
        }
        _localBDName[i] = buffer[i];    // make local copy 
    }
    for (; i < DRV_BM64_MAXBDNAMESIZE+1; i++)
    {
        _localBDName[i] = '\0'; 
    }    
    DRV_BM64_ChangeDeviceNameCommand(_localBDName);   
}

// *****************************************************************************
/* Function DRV_BM64_ClearBLEData:

        void DRV_BM64_ClearBLEData( const DRV_HANDLE handle );

  Summary:
    Clear the BLE receive buffer.

  Description:
    Clears the buffer used when receiving characters via the 
    DRV_BM64_ReadByteFromBLE and DRV_BM64_ReadDataFromBLE calls.
 
  Precondition:
    DRV_BM64_Open must have been called to obtain a valid opened device handle.

  Parameters:
    handle      - valid handle to an opened BM64 device driver unique to client

  Returns:
    None.

  Remarks:
    None.
*/

void DRV_BM64_ClearBLEData( const DRV_HANDLE handle )
{
    DRV_BM64_SPPBuffClear();
}

// *****************************************************************************
/* Function DRV_BM64_ReadByteFromBLE:

        bool DRV_BM64_ReadByteFromBLE(const DRV_HANDLE handle, uint8_t* byte);

  Summary:
    Read a byte over BLE.

  Description:
    Read one byte over BLE using the BM64's "Transparent Service" feature.
 
  Precondition:
    DRV_BM64_Open must have been called to obtain a valid opened device handle.

  Parameters:
    handle      - valid handle to an opened BM64 device driver unique to client
    byte        - pointer to a uint8_t to receive the data

  Returns:
    bool        - true if a byte was returned, false if receive buffer empty

  Remarks:
    None.
*/

bool DRV_BM64_ReadByteFromBLE(const DRV_HANDLE handle, uint8_t* byte)
{
    if(DRV_BM64_SPP_RxCounter)
    {
        *byte = DRV_BM64_SPP_RxFifo[DRV_BM64_SPP_RxFifoHead];
        DRV_BM64_SPP_RxCounter--;
        if(DRV_BM64_SPP_RxFifoHead < DRV_BM64_SPP_RxFifoSize-1)
            DRV_BM64_SPP_RxFifoHead++;
        else
            DRV_BM64_SPP_RxFifoHead = 0;
        return true;
    }
    else
        return false;
}

// *****************************************************************************
/* Function DRV_BM64_ReadDataFromBLE:

        bool DRV_BM64_ReadDataFromBLE(const DRV_HANDLE handle, uint8_t* bytes, 
                uint16_t size );

  Summary:
    Read data over BLE.

  Description:
    Read data over BLE using the BM64's "Transparent Service" feature.
 
  Precondition:
    DRV_BM64_Open must have been called to obtain a valid opened device handle.

  Parameters:
    handle      - valid handle to an opened BM64 device driver unique to client
    bytes       - pointer to a uint8_t buffer at least size bytes long
    size        - length of buffer (including

  Returns:
    bool        - true if data was returned, false if receive buffer empty

  Remarks:
    No more than size bytes will be returned, even if more are available.
*/

bool DRV_BM64_ReadDataFromBLE(const DRV_HANDLE handle, uint8_t* byte, uint16_t size )
{
    if(DRV_BM64_SPP_RxCounter)
    {
        while (DRV_BM64_SPP_RxCounter)
        {
            *byte++ = DRV_BM64_SPP_RxFifo[DRV_BM64_SPP_RxFifoHead];
            *byte= '\0';
            DRV_BM64_SPP_RxCounter--;
            if(DRV_BM64_SPP_RxFifoHead < DRV_BM64_SPP_RxFifoSize-1)
                DRV_BM64_SPP_RxFifoHead++;
            else
                DRV_BM64_SPP_RxFifoHead = 0;        
            if (--size==0)
            {
                break;      // reached end of buffer
            }
        }
        return true;
    }
    else
        return false;
}

// *****************************************************************************
/* Function DRV_BM64_SendByteOverBLE:

        void DRV_BM64_SendByteOverBLE(const DRV_HANDLE handle, uint8_t byte); 

  Summary:
    Send a byte over BLE.

  Description:
    Send one byte over BLE using the BM64's "Transparent Service" feature.
 
  Precondition:
    DRV_BM64_Open must have been called to obtain a valid opened device handle.

  Parameters:
    handle      - valid handle to an opened BM64 device driver unique to client
    byte        - uint8_t of data to be sent

  Returns:
    None.

  Remarks:
    None.
*/

void DRV_BM64_SendByteOverBLE(const DRV_HANDLE handle, uint8_t byte)
{
    DRV_BM64_SendSPPData(&byte, 1, gDrvBm64Obj.linkIndex);
}

// *****************************************************************************
/* Function DRV_BM64_SendDataOverBLE:

        void DRV_BM64_SendDataOverBLE(const DRV_HANDLE handle, uint8_t* bytes,
                uint16_t size);

  Summary:
    Send data over BLE.

  Description:
    Send data over BLE using the BM64's "Transparent Service" feature.
 
  Precondition:
    DRV_BM64_Open must have been called to obtain a valid opened device handle.

  Parameters:
    handle      - valid handle to an opened BM64 device driver unique to client
    bytes       - pointer to a uint8_t buffer at least size bytes long
    size        - length of buffer (including

  Returns:
    None.

  Remarks:
    None.
*/

void DRV_BM64_SendDataOverBLE(const DRV_HANDLE handle, uint8_t* addr, uint16_t size)
{
    DRV_BM64_SendSPPData(addr, size, gDrvBm64Obj.linkIndex);
}

// *****************************************************************************
// *****************************************************************************
// local routines
// *****************************************************************************
// *****************************************************************************

/*------------------------------------------------------------*/
static void initConnection(uint8_t index)
{
    DRV_BM64_AllConnections.allConnection[index].DeviceID_LinkIndex = 0;
    DRV_BM64_AllConnections.allConnection[index].LinkedProfileStatus = 0;
}

/*------------------------------------------------------------*/
static void initAllConnections(void)
{
    uint8_t i;
    DRV_BM64_AllConnections.activeIndex = 0;
    for(i=0; i<DRV_BM64_CONNECTION_MAX; i++)
        initConnection(i);
}

 /*------------------------------------------------------------*/
static uint8_t returnCurrentActiveConnectionIndex(void)
{
    return DRV_BM64_AllConnections.activeIndex;
}

 /*------------------------------------------------------------*/
static void setCurrentActiveConnectionIndex(uint8_t index)
{
    DRV_BM64_AllConnections.activeIndex = index;
}

 /*------------------------------------------------------------*/
static uint8_t findAvailableConnectionIndex(void)
{
    uint8_t i;
    for(i=0; i<DRV_BM64_CONNECTION_MAX; i++)
    {
        if(DRV_BM64_AllConnections.allConnection[i].LinkedProfileStatus == 0)
            return i;
    }
    return 0xff;              //-1: not available
}

/*------------------------------------------------------------*/
static uint8_t findExistConnectionByInfo(uint8_t deviceID_databaseIndex)
{
    uint8_t i;
    for(i=0; i<DRV_BM64_CONNECTION_MAX; i++)
    {
        if(DRV_BM64_AllConnections.allConnection[i].DeviceID_LinkIndex == deviceID_databaseIndex)
            return i;
    }
    return 0xff;            //-1: not found
}

 /*------------------------------------------------------------*/
static void updateWhenProfileConnected(uint8_t profile_status, uint8_t deviceID_databaseIndex)
{
    uint8_t index = findExistConnectionByInfo(deviceID_databaseIndex);
    if(index != 0xff)
    {
        DRV_BM64_AllConnections.allConnection[index].LinkedProfileStatus |= profile_status;
    }
    else
    {
        index = findAvailableConnectionIndex();
        if(index != 0xff)
        {
            //new added a connection
            DRV_BM64_AllConnections.allConnection[index].DeviceID_LinkIndex = deviceID_databaseIndex;     //save new info(device_id and linked database index)
            DRV_BM64_AllConnections.allConnection[index].LinkedProfileStatus = 0;
            DRV_BM64_AllConnections.allConnection[index].LinkedProfileStatus |= profile_status;           //save profile linked status
            setCurrentActiveConnectionIndex(index);
        }
        else
        {
            //ignore
        }
    }
}

/*------------------------------------------------------------*/
static void updateWhenProfileDisconnected(uint8_t profile_status, uint8_t deviceID_databaseIndex)
{
    uint8_t index = findExistConnectionByInfo(deviceID_databaseIndex);
    if(index != 0xff)
    {
        DRV_BM64_AllConnections.allConnection[index].LinkedProfileStatus &= (profile_status^0xff);
        if(DRV_BM64_AllConnections.allConnection[index].LinkedProfileStatus == 0)
        {
            //erase a connection
            DRV_BM64_AllConnections.allConnection[index].DeviceID_LinkIndex = 0;
            if(index == returnCurrentActiveConnectionIndex())
                setCurrentActiveConnectionIndex(0xff);          //set active index to NONE (-1)
        }
    }
    else
    {
        //ignore
    }
}

/*------------------------------------------------------------*/
/*
 * volume-related functions
 * internally, volume stored as 0-30 (approx. 3% per step)
*/
/*------------------------------------------------------------*/

static uint8_t _volumeFormatTo4bits(uint8_t volume)
{
    return volume>>1;
}

/*------------------------------------------------------------*/
static uint8_t _volumeFormatTo7bits(uint8_t volume)
{
    return volume*127/30;
}

/*------------------------------------------------------------*/
static uint8_t _volumeFormatFrom7bits(uint8_t vol_7bits)
{
    vol_7bits &= 0x7f;
    return (uint8_t)(vol_7bits*30/127);
}

/*------------------------------------------------------------*/
static void _volumeUp(DRV_BM64_VOLUME_MODE mode)
{
    uint8_t* p;
    switch(mode)
    {
        case DRV_BM64_VOLUME_A2DP:
            p = &gDrvBm64Obj.volume.a2dpVol;
            break;
        case DRV_BM64_VOLUME_HFP:
            p = &gDrvBm64Obj.volume.hfpVol;
            break;
        case DRV_BM64_VOLUME_LINEIN:
            p = &gDrvBm64Obj.volume.lineInVol;
            break;
    }
    if(*p < 30)
    {
        (*p)++;
    }
}

/*------------------------------------------------------------*/
static void _volumeDown(DRV_BM64_VOLUME_MODE mode)
{
    uint8_t* p;
    switch(mode)
    {
        case DRV_BM64_VOLUME_A2DP:
            p = &gDrvBm64Obj.volume.a2dpVol;
            break;
        case DRV_BM64_VOLUME_HFP:
            p = &gDrvBm64Obj.volume.hfpVol;
            break;
        case DRV_BM64_VOLUME_LINEIN:
            p = &gDrvBm64Obj.volume.lineInVol;
            break;
    }
    if(*p > 0)
    {
        (*p)--;
    }
}

/*------------------------------------------------------------*/
static void _set4bitVol(uint8_t vol, DRV_BM64_VOLUME_MODE mode)
{
    uint8_t* p;
    switch(mode)
    {
        case DRV_BM64_VOLUME_A2DP:
            p = &gDrvBm64Obj.volume.a2dpVol;
            break;
        case DRV_BM64_VOLUME_HFP:
            p = &gDrvBm64Obj.volume.hfpVol;
            break;
        case DRV_BM64_VOLUME_LINEIN:
            p = &gDrvBm64Obj.volume.lineInVol;
            break;
    }
    vol &= 0x0f;
    *p = vol<<1;
}

/*------------------------------------------------------------*/
static void _set7bitVol(uint8_t vol, DRV_BM64_VOLUME_MODE mode)
{  
    uint8_t* p;
    switch(mode)
    {
        case DRV_BM64_VOLUME_A2DP:
            p = &gDrvBm64Obj.volume.a2dpVol;
            break;
        case DRV_BM64_VOLUME_HFP:
            p = &gDrvBm64Obj.volume.hfpVol;
            break;
        case DRV_BM64_VOLUME_LINEIN:
            p = &gDrvBm64Obj.volume.lineInVol;
            break;
    }
    vol &= 0x7f;
    *p = vol*30/127;
}

/*------------------------------------------------------------*/
static uint8_t _get7bitsVolCurrentMode( void )
{
    uint8_t* p = NULL;
    switch(gDrvBm64Obj.volume.currentVolMode)
    {
        case DRV_BM64_VOLUME_A2DP:
            p = &gDrvBm64Obj.volume.a2dpVol;
            break;
        case DRV_BM64_VOLUME_HFP:
            p = &gDrvBm64Obj.volume.hfpVol;
            break;
        case DRV_BM64_VOLUME_LINEIN:
            p = &gDrvBm64Obj.volume.lineInVol;
            break;
    }
    if (p != NULL)
    {
    	return _volumeFormatTo7bits(*p);
	}
    else
    {
        return 0;
    }
}

/*------------------------------------------------------------*/
static void _setCodecVolCurrentMode( void )
    {
    uint16_t volume = (uint16_t)_get7bitsVolCurrentMode();
    volume = (100*volume)/127;      // convert to percent 0-100
    _clientCallBack(DRV_BM64_EVENT_VOLUME_CHANGED, (uint8_t)volume);
 }

/*------------------------------------------------------------*/
/*
 * _clientCallBack
*/
/*------------------------------------------------------------*/

static void _clientCallBack(DRV_BM64_EVENT event, uint32_t param)
{
    DRV_BM64_CLIENT_OBJ *hClient;
    uint32_t iClient;
    
    iClient = 0;
    hClient = (DRV_BM64_CLIENT_OBJ *)&gDrvBm64ClientObj[iClient];

    for (; iClient < DRV_BM64_CLIENTS_NUMBER; iClient++)
    {
        if (true == hClient->inUse)
        {
            if (NULL != hClient->pEventCallBack)
            {
                hClient->pEventCallBack(event, param, hClient->hClientArg);
            }
        }
        hClient++;
    }
}

/*------------------------------------------------------------*/
static void _initStatus( void )
{
    DRV_BM64_SystemStatus = DRV_BM64_SYSTEM_POWER_OFF;
    DRV_BM64_eCSBStatus.nspk_link = DRV_BM64_NSPK_NO_LINK;
    DRV_BM64_eCSBStatus.snpk_event = DRV_BM64_CSB_EVENT_STANDBY;
    DRV_BM64_eCSBStatus.nspk_status = DRV_BM64_CSB_STATUS_STANDBY;
    DRV_BM64_LinkbackStatus = DRV_BM64_LINKBACK_INIT;
    initAllConnections();
    DRV_BM64_CallStatus = DRV_BM64_CALL_IDLE;
    DRV_BM64_LineInStatus = LINE_IN_INACTIVE;
    DRV_BM64_A2DPStatus = DRV_BM64_A2DP_IDLE;
}

/*------------------------------------------------------------*/
/*
 * static void _nextCommandReqCheck(void)
 */
/*------------------------------------------------------------*/
static void _nextCommandReqCheck( void )
{
    if(gDrvBm64Obj.nextCommandReq.value)
    {
        if (gDrvBm64Obj.nextCommandReq.updateA2DPGainReq) {     //from event, button
            if (DRV_BM64_IsAllowedToSendCommand()) {
                gDrvBm64Obj.nextCommandReq.updateA2DPGainReq = 0;
                DRV_BM64_updateA2DPGain(_volumeFormatTo7bits(gDrvBm64Obj.volume.a2dpVol));
            }
        }
        if (gDrvBm64Obj.nextCommandReq.updateHFPGainReq) {      //from button
            if (DRV_BM64_IsAllowedToSendCommand()) {
                gDrvBm64Obj.nextCommandReq.updateHFPGainReq = 0;
                DRV_BM64_updateHFPGain(_volumeFormatTo4bits(gDrvBm64Obj.volume.hfpVol));
            }
        }  
        if (gDrvBm64Obj.nextCommandReq.updateLineInGainReq) {       //from button
            if (DRV_BM64_IsAllowedToSendCommand()) {
                gDrvBm64Obj.nextCommandReq.updateLineInGainReq = 0;
                DRV_BM64_updateLineInGain(_volumeFormatTo4bits(gDrvBm64Obj.volume.lineInVol));
            }
    	}
        if (gDrvBm64Obj.nextCommandReq.linkbackToDevAddr) {     //from event
            if (DRV_BM64_IsAllowedToSendCommand()) {
                gDrvBm64Obj.nextCommandReq.linkbackToDevAddr = 0;
                DRV_BM64_LinkBackToDeviceByBTAddress(_linkbackBDAddr);
            }
        }
        if (gDrvBm64Obj.nextCommandReq.musicCtrlReq_00) {       //from button
            if (DRV_BM64_IsAllowedToSendCommand()) {
                gDrvBm64Obj.nextCommandReq.musicCtrlReq_00 = 0;
                DRV_BM64_MusicControlCommand(0x00);
            }
        }
        if (gDrvBm64Obj.nextCommandReq.musicCtrlReq_01) {       //from button
            if (DRV_BM64_IsAllowedToSendCommand()) {
                gDrvBm64Obj.nextCommandReq.musicCtrlReq_01 = 0;
                DRV_BM64_MusicControlCommand(0x01);
            }
        }
        if (gDrvBm64Obj.nextCommandReq.musicCtrlReq_03) {       //from button
            if (DRV_BM64_IsAllowedToSendCommand()) {
                gDrvBm64Obj.nextCommandReq.musicCtrlReq_03 = 0;
                DRV_BM64_MusicControlCommand(0x03);
            }
        }
        if (gDrvBm64Obj.nextCommandReq.musicCtrlReq_05) {       //from button
            if (DRV_BM64_IsAllowedToSendCommand()) {
                gDrvBm64Obj.nextCommandReq.musicCtrlReq_05 = 0;
                DRV_BM64_MusicControlCommand(0x05);
            }
        }
        if (gDrvBm64Obj.nextCommandReq.musicCtrlReq_06) {       //from button
            if (DRV_BM64_IsAllowedToSendCommand()) {
                gDrvBm64Obj.nextCommandReq.musicCtrlReq_06 = 0;
                DRV_BM64_MusicControlCommand(0x06);
            }
        }
        if (gDrvBm64Obj.nextCommandReq.musicCtrlReq_07) {       //from button
            if (DRV_BM64_IsAllowedToSendCommand()) {
                gDrvBm64Obj.nextCommandReq.musicCtrlReq_07 = 0;
                DRV_BM64_MusicControlCommand(0x07);
            }
        }
        if (gDrvBm64Obj.nextCommandReq.musicCtrlReq_08) {       //from button
            if (DRV_BM64_IsAllowedToSendCommand()) {
                gDrvBm64Obj.nextCommandReq.musicCtrlReq_08 = 0;
                DRV_BM64_MusicControlCommand(0x08);
            }
        }
        if (gDrvBm64Obj.nextCommandReq.SPPLinkBackReq) {        //from event
            if (DRV_BM64_IsAllowedToSendCommand()) {
                gDrvBm64Obj.nextCommandReq.SPPLinkBackReq = 0;
                    DRV_BM64_ProfileLinkBack ( 0x02, gDrvBm64Obj.linkIndex);
                }
        }
        if (gDrvBm64Obj.nextCommandReq.DisconnectAllLinksReq) {  //from button
            if (DRV_BM64_IsAllowedToSendCommand()) {
                gDrvBm64Obj.nextCommandReq.DisconnectAllLinksReq = 0;
                DRV_BM64_DisconnectAllProfile();
            }
        }
        if (gDrvBm64Obj.nextCommandReq.LinkLastDeviceReq) {  //from button
            if (DRV_BM64_IsAllowedToSendCommand()) {
                gDrvBm64Obj.nextCommandReq.LinkLastDeviceReq = 0;
                DRV_BM64_LinkBackToLastDevice();
            }
        }                    
	}
    if(gDrvBm64Obj.nextMMIActionReq.value)
    {
        if (gDrvBm64Obj.nextMMIActionReq.BroadcastModeCommandReq) {     //from sequential command
            if (DRV_BM64_IsAllowedToSendCommand()) {
                gDrvBm64Obj.nextMMIActionReq.BroadcastModeCommandReq = 0;
                DRV_BM64_MMI_ActionCommand(DRV_BM64_MMI_NSPK_ADD_SPEAKER, gDrvBm64Obj.linkIndex);
            }
        }
        if (gDrvBm64Obj.nextMMIActionReq.SpeakerAddCommandReq) {    //from sequential command
            if (DRV_BM64_IsAllowedToSendCommand()) {
                gDrvBm64Obj.nextMMIActionReq.SpeakerAddCommandReq = 0;
                DRV_BM64_MMI_ActionCommand(DRV_BM64_MMI_NSPK_ADD_SPEAKER, gDrvBm64Obj.linkIndex);
            }
        }
        if (gDrvBm64Obj.nextMMIActionReq.PairReq) {     //from button, event
            if (DRV_BM64_IsAllowedToSendCommand()) {
                gDrvBm64Obj.nextMMIActionReq.PairReq = 0;
                DRV_BM64_MMI_ActionCommand(DRV_BM64_MMI_ANY_MODE_ENTERING_PAIRING, gDrvBm64Obj.linkIndex);
                DRV_BM64_LinkbackStatus = DRV_BM64_PAIRING_START;
            }
        }
         
        if (gDrvBm64Obj.nextMMIActionReq.ResetEEPROMReq) {  //from button
            if (DRV_BM64_IsAllowedToSendCommand()) {
                gDrvBm64Obj.nextMMIActionReq.ResetEEPROMReq = 0;
                DRV_BM64_MMI_ActionCommand(DRV_BM64_MMI_RESET_EEPROM_SETTING, gDrvBm64Obj.linkIndex);
            }
        }
        if (gDrvBm64Obj.nextMMIActionReq.DisconnectAllLinksReq) {  //from button
            if (DRV_BM64_IsAllowedToSendCommand()) {
                gDrvBm64Obj.nextMMIActionReq.DisconnectAllLinksReq = 0;
                DRV_BM64_MMI_ActionCommand(DRV_BM64_MMI_DISCONNECT_ALL_LINK, gDrvBm64Obj.linkIndex);
            }
        }
        if (gDrvBm64Obj.nextMMIActionReq.LinkLastDeviceReq) {  //from button
            if (DRV_BM64_IsAllowedToSendCommand()) {
                gDrvBm64Obj.nextMMIActionReq.LinkLastDeviceReq = 0;
                DRV_BM64_MMI_ActionCommand(DRV_BM64_MMI_LINK_BACK_DEVICE, gDrvBm64Obj.linkIndex);
            }
        }    
        if (gDrvBm64Obj.nextMMIActionReq.NextSongReq) {     //from button
            if (DRV_BM64_IsAllowedToSendCommand()) {
                gDrvBm64Obj.nextMMIActionReq.NextSongReq = 0;
                DRV_BM64_MMI_ActionCommand(DRV_BM64_MMI_NEXT_SONG, gDrvBm64Obj.linkIndex);
            }
        }
        if (gDrvBm64Obj.nextMMIActionReq.PreviousSongReq) {     //from button
            if (DRV_BM64_IsAllowedToSendCommand()) {
                gDrvBm64Obj.nextMMIActionReq.PreviousSongReq = 0;
                DRV_BM64_MMI_ActionCommand(DRV_BM64_MMI_PREVIOUS_SONG, gDrvBm64Obj.linkIndex);
            }
        }
        if (gDrvBm64Obj.nextMMIActionReq.IncSpkGainReq) {       //from button
            if (DRV_BM64_IsAllowedToSendCommand()) {
                gDrvBm64Obj.nextMMIActionReq.IncSpkGainReq = 0;
                DRV_BM64_MMI_ActionCommand(DRV_BM64_MMI_INC_SPK_GAIN, gDrvBm64Obj.linkIndex);
            }
        }
        if (gDrvBm64Obj.nextMMIActionReq.DecSpkGainReq ) {      //from button
            if (DRV_BM64_IsAllowedToSendCommand()) {
                gDrvBm64Obj.nextMMIActionReq.DecSpkGainReq  = 0;
                DRV_BM64_MMI_ActionCommand(DRV_BM64_MMI_DEC_SPK_GAIN, gDrvBm64Obj.linkIndex);
            }
        }
        if (gDrvBm64Obj.nextMMIActionReq.AcceptCallReq) {       //from button
            if (DRV_BM64_IsAllowedToSendCommand()) {
                gDrvBm64Obj.nextMMIActionReq.AcceptCallReq = 0;
                DRV_BM64_MMI_ActionCommand(DRV_BM64_MMI_ACCEPT_CALL, gDrvBm64Obj.linkIndex);
            }
        }
        if (gDrvBm64Obj.nextMMIActionReq.ForceEndCallReq) {     //from button
            if (DRV_BM64_IsAllowedToSendCommand()) {
                gDrvBm64Obj.nextMMIActionReq.ForceEndCallReq = 0;
                DRV_BM64_MMI_ActionCommand(DRV_BM64_MMI_FORCE_END_CALL, gDrvBm64Obj.linkIndex);
            }
        }
        if (gDrvBm64Obj.nextMMIActionReq.LastNumberRedialReq) {     //from button
            if (DRV_BM64_IsAllowedToSendCommand()) {
                gDrvBm64Obj.nextMMIActionReq.LastNumberRedialReq = 0;
                DRV_BM64_MMI_ActionCommand(DRV_BM64_MMI_LAST_NUMBER_REDIAL, gDrvBm64Obj.linkIndex);
            }
        }
        if (gDrvBm64Obj.nextMMIActionReq.RejectCallReq) {       //from button
            if (DRV_BM64_IsAllowedToSendCommand()) {
                gDrvBm64Obj.nextMMIActionReq.RejectCallReq = 0;
                DRV_BM64_MMI_ActionCommand(DRV_BM64_MMI_REJECT_CALL, gDrvBm64Obj.linkIndex);
            }
        }
    }
    if(gDrvBm64Obj.nextMMIActionReq2.value)
    {
        if (gDrvBm64Obj.nextMMIActionReq2.TransferCallReq) {        //from button
            if (DRV_BM64_IsAllowedToSendCommand()) {
                gDrvBm64Obj.nextMMIActionReq2.TransferCallReq = 0;
                DRV_BM64_MMI_ActionCommand(0x0E, gDrvBm64Obj.linkIndex);
            }
        }
        if (gDrvBm64Obj.nextMMIActionReq2.VoiceDialReq) {       //from button
            if (DRV_BM64_IsAllowedToSendCommand()) {
                gDrvBm64Obj.nextMMIActionReq2.VoiceDialReq = 0;
                DRV_BM64_MMI_ActionCommand(0x0A, gDrvBm64Obj.linkIndex);
            }
        }
        if (gDrvBm64Obj.nextMMIActionReq2.cancelVoiceDialReq ) {        //from button
            if (DRV_BM64_IsAllowedToSendCommand()) {
                gDrvBm64Obj.nextMMIActionReq2.cancelVoiceDialReq  = 0;
                DRV_BM64_MMI_ActionCommand(0x0B, gDrvBm64Obj.linkIndex);
            }
        }
        if (gDrvBm64Obj.nextMMIActionReq2.PlayPauseReq) {       //from button
            if (DRV_BM64_IsAllowedToSendCommand()) {
                gDrvBm64Obj.nextMMIActionReq2.PlayPauseReq = 0;
                DRV_BM64_MMI_ActionCommand(0x32, gDrvBm64Obj.linkIndex);
            }
        }
        if (gDrvBm64Obj.nextMMIActionReq2.CancelNSPKReq) {      //from button
            if (DRV_BM64_IsAllowedToSendCommand()) {
                gDrvBm64Obj.nextMMIActionReq2.CancelNSPKReq = 0;
                DRV_BM64_MMI_ActionCommand(0xE3, gDrvBm64Obj.linkIndex);
            }
        }
        if (gDrvBm64Obj.nextMMIActionReq2.TerminateNSPKReq) {       //from button
            if (DRV_BM64_IsAllowedToSendCommand()) {
                gDrvBm64Obj.nextMMIActionReq2.TerminateNSPKReq = 0;
                DRV_BM64_MMI_ActionCommand(0xE4, gDrvBm64Obj.linkIndex);
            }
        }
        if (gDrvBm64Obj.nextMMIActionReq2.TerminateCancelNSPKReq) {     //from button
            if (DRV_BM64_IsAllowedToSendCommand()) {
                gDrvBm64Obj.nextMMIActionReq2.TerminateCancelNSPKReq = 0;
                DRV_BM64_MMI_ActionCommand(0xE5, gDrvBm64Obj.linkIndex);
            }
        }
        if (gDrvBm64Obj.nextMMIActionReq2.switchNSPKChannel) {      //from button
            if (DRV_BM64_IsAllowedToSendCommand()) {
                gDrvBm64Obj.nextMMIActionReq2.switchNSPKChannel = 0;
                DRV_BM64_MMI_ActionCommand(0xEC, gDrvBm64Obj.linkIndex);
            }
        }
        if (gDrvBm64Obj.nextMMIActionReq2.enterNSPKModeReq) {       //...
            if (DRV_BM64_IsAllowedToSendCommand()) {
                gDrvBm64Obj.nextMMIActionReq2.enterNSPKModeReq = 0;
                DRV_BM64_MMI_ActionCommand(0xF4, gDrvBm64Obj.linkIndex);
            }
        }
        if (gDrvBm64Obj.nextMMIActionReq2.enterBroadcastModeReq) {      //...
            if (DRV_BM64_IsAllowedToSendCommand()) {
                gDrvBm64Obj.nextMMIActionReq2.enterBroadcastModeReq = 0;
                DRV_BM64_MMI_ActionCommand(0xF5, gDrvBm64Obj.linkIndex);
            }
        }
        if (gDrvBm64Obj.nextMMIActionReq2.MMI_F6_Req) {     //from event, button
            if (DRV_BM64_IsAllowedToSendCommand()) {
                gDrvBm64Obj.nextMMIActionReq2.MMI_F6_Req = 0;
                DRV_BM64_MMI_ActionCommand(0xF6, gDrvBm64Obj.linkIndex);
            }
        }
        if (gDrvBm64Obj.nextMMIActionReq2.MMI_F7_Req) {     //from button
            if (DRV_BM64_IsAllowedToSendCommand()) {
                gDrvBm64Obj.nextMMIActionReq2.MMI_F7_Req = 0;
                DRV_BM64_MMI_ActionCommand(0xF7, gDrvBm64Obj.linkIndex);
            }
        }
    }
    
    //unused ...
    /*if (TimeCommandReq.ReqDisableDiscoverableTimer == 0 && TimeCommandReq.ReqDisableDiscoverable) {
        if (DRV_BM64_IsAllowedToSendCommand()) {
            TimeCommandReq.ReqDisableDiscoverable = 0;
            DRV_BM64_SendDiscoverableCommand(0); //disable discoverable when time is out.
        }
    }*/
}
//================================================
// Init
//================================================

void DRV_BM64_Timer_1ms( uintptr_t context)
{ 
    //drive BM64 Module Initialization
    DRV_BM64_Timer1MS_event();

    //UART TX send to BM64
    DRV_BM64_CommandSend1MS_event();
}

//================================================
// Task
//================================================
static void _BM64ControlTasks(void)
{        
#if defined( ENABLE_SYS_LOG ) 
    static uint16_t laststate = 0xffff;
    if (gDrvBm64Obj.state != laststate)    
    {
        SYS_LOG2("DRV_BM64_Task: state=%d, request=%d",gDrvBm64Obj.state,gDrvBm64Obj.request);
        laststate = gDrvBm64Obj.state;
    }    
#endif
    
    DRV_BM64_UART_Tasks();       // take care of UART first
    
    switch (gDrvBm64Obj.state) {
        case DRV_BM64_STATE_INITIALIZE_START:
            /* Open the timer Driver */
            DRV_BM64_tmrHandle = SYS_TIME_CallbackRegisterMS(DRV_BM64_Timer_1ms, 
                    (uintptr_t)0, 1/*ms*/, SYS_TIME_PERIODIC);

            if (SYS_TIME_HANDLE_INVALID == DRV_BM64_tmrHandle)
            {
                while(1);
            }
            
            _timer1ms = 200;
            DRV_BM64_RESET_SetLow();                        // Reset is active low
            gDrvBm64Obj.state = DRV_BM64_STATE_INIT_MFB_HIGH;
            break;

        case DRV_BM64_STATE_INIT_MFB_HIGH:
            if (!_timer1ms) {
                _timer1ms = 20;
                DRV_BM64_MFB_SetHigh();                     // MFB is active high
                gDrvBm64Obj.state = DRV_BM64_STATE_INIT_RESET_HIGH;
            }
            break;

        case DRV_BM64_STATE_INIT_RESET_HIGH:
            if (!_timer1ms) {
                DRV_BM64_RESET_SetHigh();
                DRV_BM64_CommandDecodeInit();
                _timer1ms = 500; //wait 500ms
                gDrvBm64Obj.state = DRV_BM64_STATE_INIT_RESET_HIGH_WAIT;
            }
            break;

        case DRV_BM64_STATE_INIT_RESET_HIGH_WAIT:
            if (!_timer1ms) //check 500ms times up
            {
                gDrvBm64Obj.state = DRV_BM64_STATE_INIT_COMMAND_START;
            }
            break;
                                 
        case DRV_BM64_STATE_INIT_COMMAND_START:
            //if(DRV_BM64_SystemStatus == DRV_BM64_SYSTEM_POWER_OFF){
                if(DRV_BM64_IsAllowedToSendCommand()){
                    DRV_BM64_SetRXBufferSize();
                    gDrvBm64Obj.state = DRV_BM64_STATE_INIT_SETUP_GPIO;
                }
            //}
            break;        

        case DRV_BM64_STATE_INIT_SETUP_GPIO:
            if(DRV_BM64_IsAllowedToSendCommand()){
                DRV_BM64_SetupBTMGPIO();
                _timer1ms = 1000;      //set 1000ms time out
                gDrvBm64Obj.state = DRV_BM64_STATE_INIT_WAIT_GPIO_EVENT;
            }
            break;

        case DRV_BM64_STATE_INIT_WAIT_GPIO_EVENT:
            if(!_timer1ms)     //time out
            {
                gDrvBm64Obj.state = DRV_BM64_STATE_INIT_READ_DEVICE_ADDR;
                break;
            }
            break;

        case DRV_BM64_STATE_INIT_READ_DEVICE_ADDR:
            if(DRV_BM64_IsAllowedToSendCommand()){
                _timer1ms = 1000;      //set 1000ms time out
                DRV_BM64_ReadDeviceAddressCommand();
                gDrvBm64Obj.state = DRV_BM64_STATE_INIT_READ_DEVICE_ADDR_WAIT;
            }
            break;

        case DRV_BM64_STATE_INIT_READ_DEVICE_ADDR_WAIT:
            if(!_timer1ms)
            {
                _timer1ms = 1000;
                DRV_BM64_ReadDeviceAddressCommand();        //retry
            }
            break;

        case DRV_BM64_STATE_INIT_READ_DEVICE_NAME:
            if(DRV_BM64_IsAllowedToSendCommand()){
                _timer1ms = 1000;      //set 1000ms time out
                DRV_BM64_ReadDeviceNameCommand();
                gDrvBm64Obj.state = DRV_BM64_STATE_INIT_READ_DEVICE_NAME_WAIT;
            }
            break;

        case DRV_BM64_STATE_INIT_READ_DEVICE_NAME_WAIT:
            if(!_timer1ms)
            {
                _timer1ms = 1000;
                DRV_BM64_ReadDeviceNameCommand();        //retry
            }
            break;            

        case DRV_BM64_STATE_INIT_BLE_ADV_START:
            if(BM64_BLE_JUMPER_ENABLED)
            {
                DRV_BM64_BLE_advUpdateLocalUniqueID(_localBDAddr);
                DRV_BM64_BLE_UpdateAdvType(CONNECTABLE_UNDIRECT_ADV);
                DRV_BM64_BLE_advUpdateBTconnectable(BT_CONNECTABLE);     //enable BLE and it is connectable
                gDrvBm64Obj.state = DRV_BM64_STATE_POWER_OFF;
            }
            else
            {
                gDrvBm64Obj.state = DRV_BM64_STATE_POWER_OFF;       //don't enable BLE
            }
            break;

        case DRV_BM64_STATE_POWER_ON_START:
            if ( DRV_BM64_IsAllowedToSendCommand() )
            {
                DRV_BM64_MMI_ActionCommand(DRV_BM64_MMI_POWERON_BUTTON_PRESS, gDrvBm64Obj.linkIndex); //POWER ON button pressed command
                gDrvBm64Obj.state = DRV_BM64_STATE_POWER_ON_BUTTON_PRESSED;
                break;
            }
            if (gDrvBm64Obj.request == DRV_BM64_REQ_SYSTEM_OFF)
            {
                gDrvBm64Obj.state = DRV_BM64_STATE_POWER_OFF;
                gDrvBm64Obj.request = DRV_BM64_REQ_NONE; //clear request
            }
            break;

        case DRV_BM64_STATE_POWER_ON_BUTTON_PRESSED:
            if ( DRV_BM64_IsAllowedToSendCommand() )
            {
                DRV_BM64_MMI_ActionCommand(DRV_BM64_MMI_POWERON_BUTTON_RELEASE, gDrvBm64Obj.linkIndex); //POWER ON button released command
                //_timer1ms = 5;
                gDrvBm64Obj.state = DRV_BM64_STATE_POWER_ON;
                break;
            }
            break;

        case DRV_BM64_STATE_POWER_ON:
            // not getting DRV_BM64_SystemStatus events?
            if(1/*DRV_BM64_SystemStatus == DRV_BM64_SYSTEM_STANDBY*/)        //wait until status is standby
            {
                if(BM64_BLE_JUMPER_ENABLED)
                {
                    //BTAPP_timer1ms = 50;
                    gDrvBm64Obj.state = DRV_BM64_STATE_VOL_SYNC;
                }
                else
                {
                    DRV_BM64_BLE_advUpdateLocalUniqueID(_localBDAddr);
                    DRV_BM64_BLE_UpdateAdvType(CONNECTABLE_UNDIRECT_ADV);
                    DRV_BM64_BLE_advUpdateBTconnectable(BT_CONNECTABLE);
                    gDrvBm64Obj.state = DRV_BM64_STATE_BLE_ADV_WAIT;
                }
            }
            break;

        case DRV_BM64_STATE_BLE_ADV_WAIT:
            if(DRV_BM64_BLE_advUpdateIsEnd())
            {
                //BTAPP_timer1ms = 50;
                gDrvBm64Obj.state = DRV_BM64_STATE_VOL_SYNC;
            }
            break;

        case DRV_BM64_STATE_VOL_SYNC:
			gDrvBm64Obj.state = DRV_BM64_STATE_READ_PAIR_RECORD;
            break;

        case DRV_BM64_STATE_READ_PAIR_RECORD:
            if(DRV_BM64_IsAllowedToSendCommand()){
                DRV_BM64_GetPairRecordCommand();
                gDrvBm64Obj.state = DRV_BM64_STATE_READ_PAIR_RECORD_WAIT;
                _timer1ms = 1000; //set 1000 time out
            }
            break;

        case DRV_BM64_STATE_READ_PAIR_RECORD_WAIT:
            if (!_timer1ms) {              //time out
                gDrvBm64Obj.pairedRecordNumber = 0;
                DRV_BM64_GetPairRecordCommand();
                _timer1ms = 1000; //set time out
            }
            break;

        case DRV_BM64_STATE_READ_LINKED_MODE:
            if(DRV_BM64_IsAllowedToSendCommand()) {
                gDrvBm64Obj.nSPKLinkedCounter = 0;
                gDrvBm64Obj.nSPKLinkingBackCounter = 0;
                DRV_BM64_ReadBTMLinkModeCommand();
                gDrvBm64Obj.state = DRV_BM64_STATE_READ_LINKED_MODE_WAIT;
                _timer1ms = 1000;          //set 1 seconds for time out waiting
            }
            break;

        case DRV_BM64_STATE_READ_LINKED_MODE_WAIT:
            if(!_timer1ms)         //time out
            {
                if(gDrvBm64Obj.pairedRecordNumber != 0)
                {
                    DRV_BM64_LinkBackToLastDevice();
                    gDrvBm64Obj.state = DRV_BM64_STATE_LINKBACK_TO_LAST_DEVICE;
                }
                else
                {
                    gDrvBm64Obj.state = DRV_BM64_STATE_BT_RUNNING;
                }
            }
            break;

        case DRV_BM64_STATE_LINKBACK_START:
            if(DRV_BM64_IsAllowedToSendCommand())
            {
                switch(gDrvBm64Obj.linkedMode)
                {
                    case 0:     //single mode
                        if(gDrvBm64Obj.pairedRecordNumber != 0)
                        {
                            DRV_BM64_LinkBackToLastDevice();
                            gDrvBm64Obj.state = DRV_BM64_STATE_LINKBACK_TO_LAST_DEVICE;
                        }
                        else
                        {
                            gDrvBm64Obj.state = DRV_BM64_STATE_BT_RUNNING;
                        }
                        break;
                    case 1:     //multi-point mode
                        if(gDrvBm64Obj.pairedRecordNumber != 0)
                        {
                            DRV_BM64_LinkBackToLastDevice();//DRV_BM64_LinkBackMultipoint();        spec change 0603
                            gDrvBm64Obj.state = DRV_BM64_STATE_LINKBACK_TO_LAST_DEVICE;
                        }
                        else
                        {
                            gDrvBm64Obj.state = DRV_BM64_STATE_BT_RUNNING;
                        }
                        break;
                    case 2:     //NSPK master
                        DRV_BM64_MMI_ActionCommand ( 0xF4, gDrvBm64Obj.linkIndex);
                        gDrvBm64Obj.nextMMIActionReq.NSpeakerTriggerMasterReq = 1;
                        gDrvBm64Obj.state = DRV_BM64_STATE_POWERBACK_TO_NSPK_MODE;
                        break;
                    case 3:     //NSPK slave
                        DRV_BM64_MMI_ActionCommand ( 0xF4, gDrvBm64Obj.linkIndex);
                        gDrvBm64Obj.nextMMIActionReq.NSpeakerTriggerSlaveReq = 1;
                        gDrvBm64Obj.state = DRV_BM64_STATE_POWERBACK_TO_NSPK_MODE;
                        break;
                    case 4:     //Broadcast master
                        DRV_BM64_MMI_ActionCommand ( 0xF5, gDrvBm64Obj.linkIndex);
                        gDrvBm64Obj.nextMMIActionReq.BroadcastTriggerMasterReq = 1;
                        gDrvBm64Obj.state = DRV_BM64_STATE_POWERBACK_TO_NSPK_MODE;
                        break;
                    case 5:     //Broadcast slave
                        DRV_BM64_MMI_ActionCommand ( 0xF5, gDrvBm64Obj.linkIndex);
                        gDrvBm64Obj.nextMMIActionReq.BroadcastTriggerSlaveReq = 1;
                        gDrvBm64Obj.state = DRV_BM64_STATE_POWERBACK_TO_NSPK_MODE;
                        break;
                }
            }
            break;

        case DRV_BM64_STATE_LINKBACK_TO_LAST_DEVICE:
            /*if(DRV_BM64_GetAckStatusLinkBack() != ACK_STS_OK)
            {
                break;
            }*/
            gDrvBm64Obj.state = DRV_BM64_STATE_BT_RUNNING;
            break;

        case DRV_BM64_STATE_POWERBACK_TO_NSPK_MODE:
            if(!DRV_BM64_IsAllowedToSendCommand())
                break;
            if (gDrvBm64Obj.nextMMIActionReq.NSpeakerTriggerMasterReq) {
                gDrvBm64Obj.nextMMIActionReq.NSpeakerTriggerMasterReq = 0;
                DRV_BM64_MMI_ActionCommand(DRV_BM64_MMI_MASTERSPK_ENTER_CSB_PAGE, gDrvBm64Obj.linkIndex);
                _timer1ms = 63000;
                gDrvBm64Obj.state = DRV_BM64_STATE_POWERBACK_NSPK_MASTER_WAITING;
            } else if (gDrvBm64Obj.nextMMIActionReq.NSpeakerTriggerSlaveReq) {
                gDrvBm64Obj.nextMMIActionReq.NSpeakerTriggerSlaveReq = 0;
                DRV_BM64_MMI_ActionCommand(DRV_BM64_MMI_SLAVESPK_ENTER_CSB_PAGESCAN, gDrvBm64Obj.linkIndex);
                _timer1ms = 63000;
                gDrvBm64Obj.state = DRV_BM64_STATE_POWERBACK_NSPK_SLAVE_WAITING;
            } else if (gDrvBm64Obj.nextMMIActionReq.BroadcastTriggerMasterReq) {
                gDrvBm64Obj.nextMMIActionReq.BroadcastTriggerMasterReq = 0;
                DRV_BM64_MMI_ActionCommand(DRV_BM64_MMI_MASTERSPK_ENTER_CSB_PAGE, gDrvBm64Obj.linkIndex);
                _timer1ms = 63000;
                gDrvBm64Obj.state = DRV_BM64_STATE_POWERBACK_BROADCAST_MASTER_WAITING;
            } else if (gDrvBm64Obj.nextMMIActionReq.BroadcastTriggerSlaveReq) {
                gDrvBm64Obj.nextMMIActionReq.BroadcastTriggerSlaveReq = 0;
                DRV_BM64_MMI_ActionCommand(DRV_BM64_MMI_SLAVESPK_ENTER_CSB_PAGESCAN, gDrvBm64Obj.linkIndex);
                _timer1ms = 63000;
                gDrvBm64Obj.state = DRV_BM64_STATE_POWERBACK_BROADCAST_SLAVE_WAITING;
            }
            break;

        case DRV_BM64_STATE_POWERBACK_NSPK_MASTER_WAITING:
            if(!_timer1ms)
            {
                gDrvBm64Obj.state = DRV_BM64_STATE_BT_RUNNING;
                break;
            }
            if(gDrvBm64Obj.nSPKLinkedCounter > 0 && gDrvBm64Obj.nSPKLinkingBackCounter == gDrvBm64Obj.nSPKLinkedCounter)
            {
                if(DRV_BM64_IsAllowedToSendCommand())
                {
                    DRV_BM64_MMI_ActionCommand ( 0xE3, gDrvBm64Obj.linkIndex);
                    gDrvBm64Obj.state = DRV_BM64_STATE_BT_RUNNING;
                    _timer1ms = 0;         //clear time out timer
                    break;
                }
            }
            if(gDrvBm64Obj.request == DRV_BM64_REQ_SYSTEM_OFF)
                gDrvBm64Obj.state =  DRV_BM64_STATE_POWER_OFF_START;
            gDrvBm64Obj.request = DRV_BM64_REQ_NONE;
            break;

        case DRV_BM64_STATE_POWERBACK_NSPK_SLAVE_WAITING:
            if(!_timer1ms)
            {
                gDrvBm64Obj.state = DRV_BM64_STATE_BT_RUNNING;
                break;
            }
            if(gDrvBm64Obj.request == DRV_BM64_REQ_SYSTEM_OFF)
                gDrvBm64Obj.state =  DRV_BM64_STATE_POWER_OFF_START;
            gDrvBm64Obj.request = DRV_BM64_REQ_NONE;
            break;

        case DRV_BM64_STATE_POWERBACK_BROADCAST_MASTER_WAITING:
            if(!_timer1ms)
            {
                gDrvBm64Obj.state = DRV_BM64_STATE_BT_RUNNING;
                break;
            }
            if(gDrvBm64Obj.nSPKLinkingBackCounter == gDrvBm64Obj.nSPKLinkedCounter)
            {
                if(DRV_BM64_IsAllowedToSendCommand())
                {
                    DRV_BM64_MMI_ActionCommand ( 0xE3, gDrvBm64Obj.linkIndex);
                    gDrvBm64Obj.state = DRV_BM64_STATE_BT_RUNNING;
                    _timer1ms = 0;         //clear time out timer
                    break;
                }
            }
            if(gDrvBm64Obj.request == DRV_BM64_REQ_SYSTEM_OFF)
                gDrvBm64Obj.state =  DRV_BM64_STATE_POWER_OFF_START;
            gDrvBm64Obj.request = DRV_BM64_REQ_NONE;
            break;

        case DRV_BM64_STATE_POWERBACK_BROADCAST_SLAVE_WAITING:
            if(!_timer1ms)
            {
                gDrvBm64Obj.state = DRV_BM64_STATE_BT_RUNNING;
                break;
            }
            if(gDrvBm64Obj.request == DRV_BM64_REQ_SYSTEM_OFF)
                gDrvBm64Obj.state =  DRV_BM64_STATE_POWER_OFF_START;
            gDrvBm64Obj.request = DRV_BM64_REQ_NONE;
            break;

        case DRV_BM64_STATE_BT_RUNNING:
            if(gDrvBm64Obj.request == DRV_BM64_REQ_SYSTEM_OFF)
                gDrvBm64Obj.state =  DRV_BM64_STATE_POWER_OFF_START;
            gDrvBm64Obj.request = DRV_BM64_REQ_NONE;
            break;

        case DRV_BM64_STATE_POWER_OFF_WAIT_NSPK_EVENT:
            break;

        case DRV_BM64_STATE_POWER_OFF_START:
        case DRV_BM64_STATE_POWER_OFF_START_NSPK:
            //if (!_timer1ms) {
            if(DRV_BM64_IsAllowedToSendCommand()){
                //_timer1ms = 50;
                if(gDrvBm64Obj.state == DRV_BM64_STATE_POWER_OFF_START_NSPK)        //event comes from NSPK exist
                {
                    DRV_BM64_MMI_ActionCommand(DRV_BM64_MMI_POWEROFF_BUTTON_PRESS, gDrvBm64Obj.linkIndex);
                    gDrvBm64Obj.state = DRV_BM64_STATE_POWER_OFF_WAIT_NSPK;
                }
                else if(DRV_BM64_eCSBStatus.nspk_link != DRV_BM64_NSPK_NO_LINK)     //NSPK exist
                {
                    if(DRV_BM64_eCSBStatus.nspk_link == DRV_BM64_BROADCAST_SLAVE)
                    {
                        DRV_BM64_MMI_ActionCommand(DRV_BM64_MMI_POWEROFF_BUTTON_PRESS, gDrvBm64Obj.linkIndex);      //broadcast slave don't support POWER_OFF_ALL_SPK function
                        gDrvBm64Obj.state = DRV_BM64_STATE_POWER_OFF_WAIT;
                    }
                    else
                    {
                        DRV_BM64_MMI_ActionCommand(DRV_BM64_MMI_POWER_OFF_ALL_SPK, gDrvBm64Obj.linkIndex);
                        gDrvBm64Obj.state = DRV_BM64_STATE_POWER_OFF_WAIT_NSPK_EVENT;
                    }
                }
                else if(DRV_BM64_eCSBStatus.nspk_link == DRV_BM64_NSPK_NO_LINK && DRV_BM64_eCSBStatus.snpk_event == DRV_BM64_CSB_EVENT_CONNECTING)        //NSPK connecting
                {
                    DRV_BM64_MMI_ActionCommand(DRV_BM64_MMI_POWEROFF_BUTTON_PRESS, gDrvBm64Obj.linkIndex);
                    gDrvBm64Obj.state = DRV_BM64_STATE_POWER_OFF_WAIT;
                }
                else        //NSPK none or single speaker
                {
                    DRV_BM64_MMI_ActionCommand(DRV_BM64_MMI_POWEROFF_BUTTON_PRESS, gDrvBm64Obj.linkIndex);
                    gDrvBm64Obj.state = DRV_BM64_STATE_POWER_OFF_WAIT;
                }
            }
            break;

        case DRV_BM64_STATE_POWER_OFF_WAIT:
        case DRV_BM64_STATE_POWER_OFF_WAIT_NSPK:
            //if (!_timer1ms) //50ms waiting
            if(DRV_BM64_IsAllowedToSendCommand())
            {
                _timer1ms = 150;
                DRV_BM64_MMI_ActionCommand(DRV_BM64_MMI_POWEROFF_BUTTON_RELEASE, gDrvBm64Obj.linkIndex);
                if(gDrvBm64Obj.state == DRV_BM64_STATE_POWER_OFF_WAIT)
                    gDrvBm64Obj.state = DRV_BM64_STATE_POWER_OFF_WAIT2;
                else
                    gDrvBm64Obj.state = DRV_BM64_STATE_POWER_OFF_WAIT2_NSPK;
            }
            break;

        case DRV_BM64_STATE_POWER_OFF_WAIT2:
        case DRV_BM64_STATE_POWER_OFF_WAIT2_NSPK:
            if (!_timer1ms)
            {
                gDrvBm64Obj.state = DRV_BM64_STATE_POWER_OFF;              

                if(BM64_BLE_JUMPER_ENABLED)            //check jumper setting to determine if need to enable BLE when power off
                {
                    DRV_BM64_BLE_UpdateAdvType(CONNECTABLE_UNDIRECT_ADV);
                    DRV_BM64_BLE_advUpdateBTconnectable(BT_CONNECTABLE);
                }
            }
            break;

        case DRV_BM64_STATE_POWER_OFF:
            if (gDrvBm64Obj.request != DRV_BM64_REQ_NONE) {
                if (gDrvBm64Obj.request == DRV_BM64_REQ_SYSTEM_ON) {
                    gDrvBm64Obj.state = DRV_BM64_STATE_POWER_ON_START;
                    gDrvBm64Obj.request = DRV_BM64_REQ_NONE; //clear request
                }
            }
            break;

        default:
            break;

    }

    if (gDrvBm64Obj.state >= DRV_BM64_STATE_INIT_COMMAND_START)
    {
        DRV_BM64_CommandDecodeMain();
        DRV_BM64_CommandSendTask();
		DRV_BM64_BLE_advertiserUpdateTask();
        _nextCommandReqCheck();
}
}

//================================================
// BM64 Event Handler
//================================================
void DRV_BM64_EventHandler(uint8_t event, uint16_t para, uint8_t* para_full)
{
    uint8_t lowByte, highByte;
    
#if defined( ENABLE_SYS_LOG )    
    SYS_LOG2("DRV_BM64_EventHandler: event=%d, para=%d",event,para);
#endif     
    
    switch(event)
	{
        case DRV_BM64_DEC_EVENT_NSPK_STATUS:
            DRV_BM64_eCSBStatus.nspk_link = (uint8_t)(para>>8);
            DRV_BM64_eCSBStatus.snpk_event = (uint8_t)(para&0x00ff);
            if(DRV_BM64_eCSBStatus.nspk_link)     //NSPK exist
            {
                switch(DRV_BM64_eCSBStatus.nspk_link)                     {
                    case DRV_BM64_NSPK_MASTER_LINK_TO_SLAVE2:// = 1,
                    case DRV_BM64_NSPK_MASTER_LINK_TO_SLAVE3:// = 2,
                    case DRV_BM64_NSPK_MASTER_LINK_TO_BOTH:// = 3,
                        if (DRV_BM64_eCSBStatus.snpk_event == DRV_BM64_CSB_EVENT_CONNECTED) {
                            DRV_BM64_eCSBStatus.nspk_status = DRV_BM64_CSB_STATUS_CONNECTED_AS_NSPK_MASTER;

                            if (gDrvBm64Obj.state == DRV_BM64_STATE_POWERBACK_NSPK_MASTER_WAITING) {
                                gDrvBm64Obj.nSPKLinkingBackCounter = para_full[2];
                            }

                            _masterBDAddr[0] = _localBDAddr[0];
                            _masterBDAddr[1] = _localBDAddr[1];
                            _masterBDAddr[2] = _localBDAddr[2];
                            _masterBDAddr[3] = _localBDAddr[3];
                            _masterBDAddr[4] = _localBDAddr[4];
                            _masterBDAddr[5] = _localBDAddr[5];
                            DRV_BM64_BLE_advUpdateMasterUniqueID(_masterBDAddr);
                            DRV_BM64_BLE_advUpdateGroupStatus(DRV_BM64_BLE_GROUP);
                            DRV_BM64_BLE_advUpdateNumOfPlayer(para_full[2]);
                            DRV_BM64_BLE_advUpdateRoleInGroup(DRV_BM64_BLE_MASTER_ROLE);
                        }
                        else if(DRV_BM64_eCSBStatus.snpk_event == DRV_BM64_CSB_EVENT_CONTINUE_CONNECTING) {
                            DRV_BM64_eCSBStatus.nspk_status = DRV_BM64_CSB_STATUS_NSPK_MASTER_CONNECTING;
                        }

                        break;

                    case DRV_BM64_NSPK_SLAVE_LINK_TO_MASTER:// = 4,
                        if (DRV_BM64_eCSBStatus.snpk_event == DRV_BM64_CSB_EVENT_CONNECTED) {
                            DRV_BM64_eCSBStatus.nspk_status = DRV_BM64_CSB_STATUS_CONNECTED_AS_NSPK_SLAVE;

                            _masterBDAddr[0] = para_full[3];
                            _masterBDAddr[1] = para_full[4];
                            _masterBDAddr[2] = para_full[5];
                            _masterBDAddr[3] = para_full[6];
                            _masterBDAddr[4] = para_full[7];
                            _masterBDAddr[5] = para_full[8];
                            DRV_BM64_BLE_advUpdateMasterUniqueID(_masterBDAddr);
                            DRV_BM64_BLE_advUpdateGroupStatus(DRV_BM64_BLE_GROUP);
                            DRV_BM64_BLE_advUpdateNumOfPlayer(0);
                            DRV_BM64_BLE_advUpdateRoleInGroup(DRV_BM64_BLE_PLAYER_ROLE);
                        }

                        DRV_BM64_BLE_UpdateAdvType(SCANNABLE_UNDIRECT_ADV); //non-connectable
                        DRV_BM64_BLE_advUpdateBTconnectable(BT_NON_CONNECTABLE);

                        if (gDrvBm64Obj.state == DRV_BM64_STATE_POWERBACK_NSPK_SLAVE_WAITING) {
                            _timer1ms = 0; //clear time out timer
                            gDrvBm64Obj.state = DRV_BM64_STATE_BT_RUNNING;
                        }
                        break;

                    case DRV_BM64_BROADCAST_MASTER:// = 5,
                        if (DRV_BM64_eCSBStatus.snpk_event == DRV_BM64_CSB_EVENT_CONNECTED) {
                            DRV_BM64_eCSBStatus.nspk_status = DRV_BM64_CSB_STATUS_CONNECTED_AS_BROADCAST_MASTER;

                            if (gDrvBm64Obj.state == DRV_BM64_STATE_POWERBACK_BROADCAST_MASTER_WAITING) {
                                gDrvBm64Obj.nSPKLinkingBackCounter = para_full[2];
                            }
#if 0       //SPEC change 0603
                            BT_masterBDAddr[0] = BT_localBDAddr[0];
                            BT_masterBDAddr[1] = BT_localBDAddr[1];
                            BT_masterBDAddr[2] = BT_localBDAddr[2];
                            BT_masterBDAddr[3] = BT_localBDAddr[3];
                            BT_masterBDAddr[4] = BT_localBDAddr[4];
                            BT_masterBDAddr[5] = BT_localBDAddr[5];
                            BLE_advUpdateMasterUniqueID(BT_masterBDAddr);
                            BLE_advUpdateGroupStatus(GROUP);
                            BLE_advUpdateRoleInGroup(MASTER_ROLE);
                            BLE_advUpdateNumOfPlayer(para_full[2]);
#endif
                        }
                        else if(DRV_BM64_eCSBStatus.snpk_event == DRV_BM64_CSB_EVENT_CONTINUE_CONNECTING) {
                            DRV_BM64_eCSBStatus.nspk_status = DRV_BM64_CSB_STATUS_BROADCAST_MASTER_CONNECTING;
                        }
                        break;

                    case DRV_BM64_BROADCAST_SLAVE:// = 6
                        if (DRV_BM64_eCSBStatus.snpk_event == DRV_BM64_CSB_EVENT_CONNECTED) {
                            DRV_BM64_eCSBStatus.nspk_status = DRV_BM64_CSB_STATUS_CONNECTED_AS_BROADCAST_SLAVE;
#if 0       //SPEC change 0603
                            BT_masterBDAddr[0] = para_full[3];
                            BT_masterBDAddr[1] = para_full[4];
                            BT_masterBDAddr[2] = para_full[5];
                            BT_masterBDAddr[3] = para_full[6];
                            BT_masterBDAddr[4] = para_full[7];
                            BT_masterBDAddr[5] = para_full[8];
                            BLE_advUpdateMasterUniqueID(BT_masterBDAddr);
                            BLE_advUpdateGroupStatus(GROUP);
                            BLE_advUpdateRoleInGroup(PLAYER_ROLE);
                            BLE_advUpdateNumOfPlayer(0);
#endif
                        }

                        DRV_BM64_BLE_UpdateAdvType(SCANNABLE_UNDIRECT_ADV); //non-connectable
                        DRV_BM64_BLE_advUpdateBTconnectable(BT_NON_CONNECTABLE);

                        if (gDrvBm64Obj.state == DRV_BM64_STATE_POWERBACK_BROADCAST_SLAVE_WAITING) {
                            _timer1ms = 0; //clear time out timer
                            gDrvBm64Obj.state = DRV_BM64_STATE_BT_RUNNING;
                        }
                        break;
                    default:
                        break;
                }
            }
            else
            {
                if(DRV_BM64_eCSBStatus.snpk_event == DRV_BM64_CSB_EVENT_STANDBY){
                    DRV_BM64_eCSBStatus.nspk_status = DRV_BM64_CSB_STATUS_STANDBY;
                }
                else if(DRV_BM64_eCSBStatus.snpk_event == DRV_BM64_CSB_EVENT_CONNECTING){
                    DRV_BM64_eCSBStatus.nspk_status = DRV_BM64_CSB_STATUS_CONNECTING;
                }
            }
            break;

        case DRV_BM64_DEC_EVENT_NSPK_CHANNEL_SETTING:
            DRV_BM64_BLE_advUpdateOutputChannel((uint8_t)(para&0x00ff));
            break;

        case DRV_BM64_DEC_EVENT_LINE_IN_STATUS:
            DRV_BM64_LineInStatus = (uint8_t)(para&0x00ff);
            if(DRV_BM64_LineInStatus != LINE_IN_INACTIVE)
            {
                gDrvBm64Obj.volume.currentVolMode = DRV_BM64_VOLUME_LINEIN;
                _setCodecVolCurrentMode();
            }
            break;

        case DRV_BM64_DEC_EVENT_A2DP_STATUS:
            DRV_BM64_A2DPStatus = (uint8_t)(para&0x00ff);
            if(DRV_BM64_A2DPStatus == DRV_BM64_A2DP_ACTIVE)
            {
                gDrvBm64Obj.volume.currentVolMode = DRV_BM64_VOLUME_A2DP;
                _setCodecVolCurrentMode();
            }
            break;

        case DRV_BM64_DEC_EVENT_CALL_STATUS_CHANGED:
            DRV_BM64_CallStatus = (uint8_t)para;
            if(DRV_BM64_CallStatus != DRV_BM64_CALL_IDLE)
            {
                gDrvBm64Obj.volume.currentVolMode = DRV_BM64_VOLUME_HFP;
                _setCodecVolCurrentMode();
            }
            else
            {
                gDrvBm64Obj.volume.currentVolMode = DRV_BM64_VOLUME_A2DP;
                _setCodecVolCurrentMode();
            }
            break;

        case DRV_BM64_DEC_EVENT_CODEC_TYPE:
            {
                uint8_t mode = (uint8_t)(para&0x00ff);
                if ((mode == 4)||(mode == 6))
                {
                    _switchSampleFrequency((DRV_BM64_SAMPLE_FREQUENCY)(para>>8));
                }
            }
            break;            

        case DRV_BM64_DEC_EVENT_HFP_CONNECTED:
            gDrvBm64Obj.linkStatus |= DRV_BM64_HFP_LINK_STATUS;
            updateWhenProfileConnected(DRV_BM64_HFP_LINK_STATUS, (uint8_t)para);
            //if(IsConnectionsFull())
            //    DRV_BM64_SendDiscoverableCommand(0);
            DRV_BM64_LinkbackStatus = DRV_BM64_LINK_CONNECTED;
            DRV_BM64_SystemStatus = DRV_BM64_SYSTEM_CONNECTED;           
            break;
        case DRV_BM64_DEC_EVENT_HFP_DISCONNECTED:
            gDrvBm64Obj.linkStatus &= (DRV_BM64_HFP_LINK_STATUS^0xff);
            updateWhenProfileDisconnected(DRV_BM64_HFP_LINK_STATUS, (uint8_t)para);            
            break;
        case DRV_BM64_EVENT_SPP_CONNECTED:
            gDrvBm64Obj.linkStatus |= DRV_BM64_SPP_LINK_STATUS;//BT_SPP_Connected = true;
            updateWhenProfileConnected(DRV_BM64_SPP_LINK_STATUS, (uint8_t)para);
            //if(IsConnectionsFull())
            //    BT_SendDiscoverableCommand(0);
            break;
        case DRV_BM64_EVENT_IAP_CONNETED:
            gDrvBm64Obj.linkStatus |= DRV_BM64_IAP_LINK_STATUS;
            updateWhenProfileConnected(DRV_BM64_IAP_LINK_STATUS, (uint8_t)para);
            //if(IsConnectionsFull())
            //    BT_SendDiscoverableCommand(0);
            break;
        case DRV_BM64_EVENT_SPP_IAP_DISCONNECTED:
            gDrvBm64Obj.linkStatus &= (DRV_BM64_SPP_LINK_STATUS^0xff);//BT_SPP_Connected = false;
            gDrvBm64Obj.linkStatus &= (DRV_BM64_IAP_LINK_STATUS^0xff);
            updateWhenProfileDisconnected(DRV_BM64_SPP_LINK_STATUS, (uint8_t)para);
            updateWhenProfileDisconnected(DRV_BM64_IAP_LINK_STATUS, (uint8_t)para);
            break;
        case DRV_BM64_DEC_EVENT_A2DP_CONNECTED:
            gDrvBm64Obj.linkStatus |= DRV_BM64_A2DP_LINK_STATUS;
            if(!(gDrvBm64Obj.linkStatus & DRV_BM64_SPP_LINK_STATUS || gDrvBm64Obj.linkStatus & DRV_BM64_IAP_LINK_STATUS))
            {
                if (DRV_BM64_IsAllowedToSendCommand()) {
                    DRV_BM64_ProfileLinkBack ( 0x02, gDrvBm64Obj.linkIndex);
                }
                else {
                    gDrvBm64Obj.nextCommandReq.SPPLinkBackReq = 1;
                }
            }
            updateWhenProfileConnected(DRV_BM64_A2DP_LINK_STATUS, (uint8_t)para);
            //if(IsConnectionsFull())
            //    DRV_BM64_SendDiscoverableCommand(0);
            DRV_BM64_LinkbackStatus = DRV_BM64_LINK_CONNECTED;
            DRV_BM64_SystemStatus = DRV_BM64_SYSTEM_CONNECTED;           
            break;
        case DRV_BM64_DEC_EVENT_A2DP_DISCONNECTED:
            gDrvBm64Obj.linkStatus &= (DRV_BM64_A2DP_LINK_STATUS^0xff);
            updateWhenProfileDisconnected(DRV_BM64_A2DP_LINK_STATUS, (uint8_t)para);           
            break;
        case DRV_BM64_DEC_EVENT_AVRCP_CONNECTED:
            gDrvBm64Obj.linkStatus |= DRV_BM64_AVRCP_LINK_STATUS;
            updateWhenProfileConnected(DRV_BM64_AVRCP_LINK_STATUS, (uint8_t)para);
            //if(IsConnectionsFull())
            //    DRV_BM64_SendDiscoverableCommand(0);
            break;
        case DRV_BM64_DEC_EVENT_AVRCP_DISCONNECTED:
            gDrvBm64Obj.linkStatus &= (DRV_BM64_AVRCP_LINK_STATUS^0xff);
            updateWhenProfileDisconnected(DRV_BM64_AVRCP_LINK_STATUS, (uint8_t)para);
            break;
        case DRV_BM64_DEC_EVENT_ACL_CONNECTED:
            gDrvBm64Obj.linkStatus |= DRV_BM64_ACL_LINK_STATUS;
            updateWhenProfileConnected(DRV_BM64_ACL_LINK_STATUS, (uint8_t)para);
            //if(IsConnectionsFull())
            //    DRV_BM64_SendDiscoverableCommand(0);
            break;
        case DRV_BM64_DEC_EVENT_ACL_DISCONNECTED:
            gDrvBm64Obj.linkStatus &= (DRV_BM64_ACL_LINK_STATUS^0xff);
            updateWhenProfileDisconnected(DRV_BM64_ACL_LINK_STATUS, (uint8_t)para);
            break;
        case DRV_BM64_DEC_EVENT_SCO_CONNECTED:
            gDrvBm64Obj.linkStatus |= DRV_BM64_SCO_LINK_STATUS;
            updateWhenProfileConnected(DRV_BM64_SCO_LINK_STATUS, (uint8_t)para);
            //if(IsConnectionsFull())
            //    DRV_BM64_SendDiscoverableCommand(0);
            break;
        case DRV_BM64_DEC_EVENT_SCO_DISCONNECTED:
            gDrvBm64Obj.linkStatus &= (DRV_BM64_SCO_LINK_STATUS^0xff);
            updateWhenProfileDisconnected(DRV_BM64_SCO_LINK_STATUS, (uint8_t)para);
            break;
        case DRV_BM64_EVENT_MAP_CONNECTED:
            gDrvBm64Obj.linkStatus |= DRV_BM64_MAP_LINK_STATUS;
            updateWhenProfileConnected(DRV_BM64_MAP_LINK_STATUS, (uint8_t)para);
            //if(IsConnectionsFull())
            //    BT_SendDiscoverableCommand(0);
            break;
        case DRV_BM64_EVENT_MAP_DISCONNECTED:
            gDrvBm64Obj.linkStatus &= (DRV_BM64_MAP_LINK_STATUS^0xff);
            updateWhenProfileDisconnected(DRV_BM64_MAP_LINK_STATUS, (uint8_t)para);
            break;
        case DRV_BM64_DEC_EVENT_SYS_POWER_ON:
            DRV_BM64_SystemStatus = DRV_BM64_SYSTEM_POWER_ON;
            break;
        case DRV_BM64_DEC_EVENT_SYS_POWER_OFF:
            DRV_BM64_SystemStatus = DRV_BM64_SYSTEM_POWER_OFF;
            break;
        case DRV_BM64_DEC_EVENT_SYS_STANDBY:
            DRV_BM64_SystemStatus = DRV_BM64_SYSTEM_STANDBY;
            break;

        case DRV_BM64_DEC_EVENT_SYS_PAIRING_START:
            DRV_BM64_SystemStatus = DRV_BM64_SYSTEM_PAIRING;
            DRV_BM64_LinkbackStatus = DRV_BM64_PAIRING;
            break;
        case DRV_BM64_DEC_EVENT_SYS_PAIRING_OK:
            DRV_BM64_LinkbackStatus = DRV_BM64_PAIRING_OK;
            break;
        case DRV_BM64_DEC_EVENT_SYS_PAIRING_FAILED:
            DRV_BM64_LinkbackStatus = DRV_BM64_PAIRING_FAILED;
            break;

        case DRV_BM64_DEC_EVENT_LINKBACK_SUCCESS:
            DRV_BM64_LinkbackStatus = DRV_BM64_LINKBACK_OK;
            break;

        case DRV_BM64_DEC_EVENT_LINKBACK_FAILED:
            DRV_BM64_LinkbackStatus = DRV_BM64_LINKBACK_FAILED;
            break;

        case DRV_BM64_DEC_EVENT_BD_ADDR_RECEIVED:
            if(gDrvBm64Obj.state == DRV_BM64_STATE_INIT_READ_DEVICE_ADDR_WAIT)
            {
                _timer1ms = 0;
                gDrvBm64Obj.state = DRV_BM64_STATE_INIT_READ_DEVICE_NAME;
            }
            break;
            
        case DRV_BM64_DEC_EVENT_BT_NAME_RECEIVED:
            if(gDrvBm64Obj.state == DRV_BM64_STATE_INIT_READ_DEVICE_NAME_WAIT)
            {
                _timer1ms = 0;
                //gDrvBm64Obj.state = DRV_BM64_STATE_INIT_BLE_ADV_START;
                gDrvBm64Obj.state = DRV_BM64_STATE_POWER_OFF;
            }
            break;

        case DRV_BM64_DEC_EVENT_PAIR_RECORD_RECEIVED:
            gDrvBm64Obj.pairedRecordNumber = (uint8_t)para;
            if(gDrvBm64Obj.state == DRV_BM64_STATE_READ_PAIR_RECORD_WAIT)
            {
                gDrvBm64Obj.state = DRV_BM64_STATE_READ_LINKED_MODE;
            }
            break;

        case DRV_BM64_DEC_EVENT_LINK_MODE_RECEIVED:
            DRV_BM64_InitAckStatus();             //C34(READ_LINKED_MODE) has no ACK, BTM bug...
            gDrvBm64Obj.linkedMode = (uint8_t)para;
            if(gDrvBm64Obj.linkedMode == 2 || gDrvBm64Obj.linkedMode == 4)        //NSPK master, Broadcast master
            {
                para >>= 8;
                gDrvBm64Obj.nSPKLinkedCounter = (uint8_t)para;
            }
            if(gDrvBm64Obj.state == DRV_BM64_STATE_READ_LINKED_MODE_WAIT)
            {
                _timer1ms = 0; //clear time out timer
                gDrvBm64Obj.state = DRV_BM64_STATE_LINKBACK_START;
            }
            break;

        case DRV_BM64_DEC_EVENT_PLAYBACK_STATUS_CHANGED:
        {
            gDrvBm64Obj.playingStatus = (DRV_BM64_PLAYINGSTATUS)para;
            _clientCallBack(DRV_BM64_EVENT_PLAYBACK_STATUS_CHANGED,para);                    
            break;
        }

        case DRV_BM64_DEC_EVENT_NSPK_SYNC_POWER_OFF:
            //_timer1ms = 0;
            gDrvBm64Obj.state = DRV_BM64_STATE_POWER_OFF_START_NSPK;
            break;

        case DRV_BM64_DEC_EVENT_HFP_VOLUME_CHANGED:
            _set4bitVol((uint8_t)para, DRV_BM64_VOLUME_HFP);
            _setCodecVolCurrentMode();
            break;

        case DRV_BM64_DEC_EVENT_AVRCP_VOLUME_CTRL:        //AVRCP 1.0
            if(para == 0)       //vol up
            {
                _volumeUp(DRV_BM64_VOLUME_A2DP);
            }
            else if(para == 1)      //vol down
            {
                _volumeDown(DRV_BM64_VOLUME_A2DP);
            }
            _setCodecVolCurrentMode();
            break;

        case DRV_BM64_DEC_EVENT_AVRCP_ABS_VOLUME_CHANGED: //AVRCP > 1.0
            _set7bitVol((uint8_t)para, DRV_BM64_VOLUME_A2DP);
            _setCodecVolCurrentMode();
            break;

        case DRV_BM64_DEC_EVENT_NSPK_SYNC_VOL_CTRL:
            if(para == 0)       //vol up
            {
                _volumeUp(DRV_BM64_VOLUME_A2DP);
                if(gDrvBm64Obj.volume.currentVolMode == DRV_BM64_VOLUME_A2DP)
                {
                    _setCodecVolCurrentMode();
                    //BT_SetOverallGainCommand(4, _volumeFormatTo7bits(BTAPP_Volume.a2dpVol), _volumeFormatTo7bits(BTAPP_Volume.hfpVol), _volumeFormatTo7bits(BTAPP_Volume.lineInVol));
                    if (DRV_BM64_IsAllowedToSendCommand()) {
                        DRV_BM64_updateA2DPGain(_volumeFormatTo7bits(gDrvBm64Obj.volume.a2dpVol));
                    }
                    else {
                        gDrvBm64Obj.nextCommandReq.updateA2DPGainReq = 1;
                    }
                }
            }
            else if(para == 1)      //vol down
            {
                _volumeDown(DRV_BM64_VOLUME_A2DP);
                if(gDrvBm64Obj.volume.currentVolMode == DRV_BM64_VOLUME_A2DP)
                {
                    _setCodecVolCurrentMode();
                    //BT_SetOverallGainCommand(4, _volumeFormatTo7bits(BTAPP_Volume.a2dpVol), _volumeFormatTo7bits(BTAPP_Volume.hfpVol), _volumeFormatTo7bits(BTAPP_Volume.lineInVol));
                    if (DRV_BM64_IsAllowedToSendCommand()) {
                        DRV_BM64_updateA2DPGain(_volumeFormatTo7bits(gDrvBm64Obj.volume.a2dpVol));
                    }
                    else {
                        gDrvBm64Obj.nextCommandReq.updateA2DPGainReq = 1;
                    }
                }
            }
            break;

        case DRV_BM64_DEC_EVENT_NSPK_SYNC_INTERNAL_GAIN:
            _set4bitVol((uint8_t)(para&0x000f), DRV_BM64_VOLUME_A2DP);               //bit3~0 indicates DRV_BM64_VOLUME_A2DP gain
            _set4bitVol((uint8_t)((para>>4) & 0x000f), DRV_BM64_VOLUME_LINEIN);     //bit7~4 indicates LINE IN gain
            break;
            
        case DRV_BM64_DEC_EVENT_NSPK_SYNC_ABS_VOL:
            _set7bitVol((uint8_t)para, DRV_BM64_VOLUME_A2DP);
            if(gDrvBm64Obj.volume.currentVolMode == DRV_BM64_VOLUME_A2DP)
            {
                _setCodecVolCurrentMode();
                if (DRV_BM64_IsAllowedToSendCommand()) {
                    DRV_BM64_updateA2DPGain(_volumeFormatTo7bits(gDrvBm64Obj.volume.a2dpVol));
                } else {
                    gDrvBm64Obj.nextCommandReq.updateA2DPGainReq = 1;
                }
            }
            break;

        case DRV_BM64_DEC_EVENT_NSPK_ADD_SPEAKER3:
            if(DRV_BM64_eCSBStatus.nspk_link == DRV_BM64_NSPK_MASTER_LINK_TO_SLAVE2     //this is master, it connects to speaker 2 only
            || DRV_BM64_eCSBStatus.nspk_link == DRV_BM64_NSPK_MASTER_LINK_TO_SLAVE3)    //this is master, it connects to speaker 3 only
            {
                //go to add 3rd speaker in stereo mode
                if (DRV_BM64_IsAllowedToSendCommand())
                    DRV_BM64_MMI_ActionCommand ( 0xF6, gDrvBm64Obj.linkIndex);
                else
                    gDrvBm64Obj.nextMMIActionReq2.MMI_F6_Req = 1;
            }
            break;

        case DRV_BM64_DEC_EVENT_LE_STATUS_CHANGED:
            lowByte = (uint8_t)para;
            para >>= 8;
            highByte = (uint8_t)para;
            switch(highByte)
            {
                case 0x00:      //LE standby
                    if(((gDrvBm64Obj.state > DRV_BM64_STATE_POWER_ON) && (gDrvBm64Obj.state <= DRV_BM64_STATE_BT_RUNNING))
                            ||(gDrvBm64Obj.state == DRV_BM64_STATE_POWER_OFF))
                    {
                        DRV_BM64_BLE_UpdateAdvType(CONNECTABLE_UNDIRECT_ADV);
                        DRV_BM64_BLE_advUpdateBTconnectable(BT_CONNECTABLE);
                    }
                    break;
                case 0x01:      //advertising
                    break;
                case 0x02:      //scanning
                    break;
                case 0x03:      //connected
                    if(BM64_BLE_JUMPER_ENABLED)        //check jumper setting to determine if need to power on when BLE is connected
                    {
                        if(DRV_BM64_GetPowerStatus() == DRV_BM64_STATUS_OFF)
                        {
                            DRV_BM64_TaskReq(DRV_BM64_REQ_SYSTEM_ON);
                        }
                    }
                    break;
                default:
                    break;
            }
            _clientCallBack(DRV_BM64_EVENT_BLE_STATUS_CHANGED,highByte);
            break;
        case DRV_BM64_DEC_EVENT_LE_ADV_CONTROL_REPORT:
            break;
        case DRV_BM64_DEC_EVENT_LE_CONNECTION_PARA_REPORT:
            break;
        case DRV_BM64_DEC_EVENT_LE_CONNECTION_PARA_UPDATE_RSP:
            break;
        case DRV_BM64_DEC_EVENT_GATT_ATTRIBUTE_DATA:
            break;
            
        case DRV_BM64_DEC_EVENT_PORT0_INPUT_CHANGED:
            break;
        case DRV_BM64_DEC_EVENT_PORT1_INPUT_CHANGED:
            lowByte = (uint8_t)para;
            para >>= 8;
            highByte = (uint8_t)para;
            if(highByte & 0x20)                 //P1-5 indicator
            {
                if(lowByte&0x20)
                    gDrvBm64Obj.port1.bit_5 = 0;      //save P1-5
                else
                    gDrvBm64Obj.port1.bit_5 = 1;      //save P1-5
            }
            break;
        case DRV_BM64_DEC_EVENT_PORT2_INPUT_CHANGED:
            break;
        case DRV_BM64_DEC_EVENT_PORT3_INPUT_CHANGED:
            lowByte = (uint8_t)para;
            para >>= 8;
            highByte = (uint8_t)para;
            if(highByte & 0x40)                 //P3-6 indicator
            {
                if(lowByte&0x40)
                    gDrvBm64Obj.port3.bit_6 = 0;      //save P3-6
                else
                    gDrvBm64Obj.port3.bit_6 = 1;      //save P3-6
            }
            if(gDrvBm64Obj.state == DRV_BM64_STATE_INIT_WAIT_GPIO_EVENT)
            {
                gDrvBm64Obj.state = DRV_BM64_STATE_INIT_READ_DEVICE_ADDR;
            }
            break;

        default:
            break;
    }
}

//================================================
//1ms Timer
//================================================
void DRV_BM64_Timer1MS_event( void )
{
    if(_timer1ms)
        --_timer1ms;
}

/*-----------------------------------------------------------------------------*/
//below are functions for NSPK or broadcast link creating or canceling
/*-----------------------------------------------------------------------------*/
void DRV_BM64_NSPKAdd( void )
{
    DRV_BM64_MMI_ActionCommand ( 0xF4, gDrvBm64Obj.linkIndex);
    gDrvBm64Obj.nextMMIActionReq.SpeakerAddCommandReq = 1;
}

/*-----------------------------------------------------------------------------*/
void BTAPP_NSPKBtnLongPress( void )
{
    if(gDrvBm64Obj.state != DRV_BM64_STATE_BT_RUNNING
            && gDrvBm64Obj.state != DRV_BM64_STATE_POWERBACK_NSPK_MASTER_WAITING
            && gDrvBm64Obj.state != DRV_BM64_STATE_POWERBACK_NSPK_SLAVE_WAITING
            && gDrvBm64Obj.state != DRV_BM64_STATE_POWERBACK_BROADCAST_MASTER_WAITING
            && gDrvBm64Obj.state != DRV_BM64_STATE_POWERBACK_BROADCAST_SLAVE_WAITING )
        return;

    switch(DRV_BM64_eCSBStatus.nspk_status)
    {
        case DRV_BM64_CSB_STATUS_STANDBY:
            DRV_BM64_NSPKAdd();
            break;
        case DRV_BM64_CSB_STATUS_CONNECTING:
            if (DRV_BM64_IsAllowedToSendCommand())
                DRV_BM64_MMI_ActionCommand ( 0xE3, gDrvBm64Obj.linkIndex);     //cancel NSPK creation
            else
                gDrvBm64Obj.nextMMIActionReq2.CancelNSPKReq = 1;
            break;
        case DRV_BM64_CSB_STATUS_CONNECTED_AS_NSPK_MASTER:
        case DRV_BM64_CSB_STATUS_CONNECTED_AS_NSPK_SLAVE:
            if (DRV_BM64_IsAllowedToSendCommand())
                DRV_BM64_MMI_ActionCommand ( 0xE5, gDrvBm64Obj.linkIndex);     //cancel/terminate NSPK link
            else
                gDrvBm64Obj.nextMMIActionReq2.TerminateCancelNSPKReq = 1;
            break;
        case DRV_BM64_CSB_STATUS_NSPK_MASTER_CONNECTING:
            if (DRV_BM64_IsAllowedToSendCommand())
                DRV_BM64_MMI_ActionCommand ( 0xE3, gDrvBm64Obj.linkIndex);     //cancel NSPK creation
            else
                gDrvBm64Obj.nextMMIActionReq2.CancelNSPKReq = 1;
            break;
        case DRV_BM64_CSB_STATUS_CONNECTED_AS_BROADCAST_MASTER:
        case DRV_BM64_CSB_STATUS_CONNECTED_AS_BROADCAST_SLAVE:
            if (DRV_BM64_IsAllowedToSendCommand())
                DRV_BM64_MMI_ActionCommand ( 0xE4, gDrvBm64Obj.linkIndex);     //terminate NSPK link
            else
                gDrvBm64Obj.nextMMIActionReq2.TerminateNSPKReq = 1;
            break;
        case DRV_BM64_CSB_STATUS_BROADCAST_MASTER_CONNECTING:
            //N/A
            break;
    }
}

/*-----------------------------------------------------------------------------*/
void BTAPP_NSPKBtnDbClick( void )
{
    if(DRV_BM64_GetPowerStatus() != DRV_BM64_STATUS_READY)
        return;
    if(DRV_BM64_eCSBStatus.nspk_status == DRV_BM64_CSB_STATUS_CONNECTED_AS_NSPK_MASTER ||
    DRV_BM64_eCSBStatus.nspk_status == DRV_BM64_CSB_STATUS_CONNECTED_AS_NSPK_SLAVE)
    {
        if (DRV_BM64_IsAllowedToSendCommand())
            DRV_BM64_MMI_ActionCommand(0xF6, gDrvBm64Obj.linkIndex);
        else
            gDrvBm64Obj.nextMMIActionReq2.MMI_F6_Req = 1;
    }
}

/*-----------------------------------------------------------------------------*/
void BTAPP_NSPKBtnShortPress( void )
{
    if(DRV_BM64_GetPowerStatus() != DRV_BM64_STATUS_READY)
        return;

    if(DRV_BM64_eCSBStatus.nspk_link == DRV_BM64_NSPK_MASTER_LINK_TO_SLAVE2     //connect to speaker 2 only
            || DRV_BM64_eCSBStatus.nspk_link == DRV_BM64_NSPK_MASTER_LINK_TO_SLAVE3    //connect to speaker 3 only
            || DRV_BM64_eCSBStatus.nspk_link == DRV_BM64_NSPK_SLAVE_LINK_TO_MASTER)      //this is slave
    {
        //go to switch audio channel in stereo mode
        if (DRV_BM64_IsAllowedToSendCommand())
            DRV_BM64_MMI_ActionCommand ( 0xEC, gDrvBm64Obj.linkIndex);
        else
            gDrvBm64Obj.nextMMIActionReq2.switchNSPKChannel = 1;
    }
}

/*-----------------------------------------------------------------------------*/
void BTAPP_BroadcastAdd( void )
{
    DRV_BM64_MMI_ActionCommand ( 0xF5, gDrvBm64Obj.linkIndex);
    gDrvBm64Obj.nextMMIActionReq.BroadcastModeCommandReq = 1;
}

/*-----------------------------------------------------------------------------*/
void BTAPP_BroadcastBtnLongPress(void)
{
    if(gDrvBm64Obj.state != DRV_BM64_STATE_BT_RUNNING
            && gDrvBm64Obj.state != DRV_BM64_STATE_POWERBACK_NSPK_MASTER_WAITING
            && gDrvBm64Obj.state != DRV_BM64_STATE_POWERBACK_NSPK_SLAVE_WAITING
            && gDrvBm64Obj.state != DRV_BM64_STATE_POWERBACK_BROADCAST_MASTER_WAITING
            && gDrvBm64Obj.state != DRV_BM64_STATE_POWERBACK_BROADCAST_SLAVE_WAITING )
        return;
    switch(DRV_BM64_eCSBStatus.nspk_status)
    {
        case DRV_BM64_CSB_STATUS_STANDBY:
            BTAPP_BroadcastAdd();
            break;
        case DRV_BM64_CSB_STATUS_CONNECTING:
            if (DRV_BM64_IsAllowedToSendCommand())
                DRV_BM64_MMI_ActionCommand ( 0xE3, gDrvBm64Obj.linkIndex);     //cancel NSPK creation
            else
                gDrvBm64Obj.nextMMIActionReq2.CancelNSPKReq = 1;
            break;
        case DRV_BM64_CSB_STATUS_CONNECTED_AS_NSPK_MASTER:
        case DRV_BM64_CSB_STATUS_CONNECTED_AS_NSPK_SLAVE:
            if (DRV_BM64_IsAllowedToSendCommand())
                DRV_BM64_MMI_ActionCommand ( 0xE4, gDrvBm64Obj.linkIndex);     //terminate NSPK link
            else
                gDrvBm64Obj.nextMMIActionReq2.TerminateNSPKReq = 1;
            break;
        case DRV_BM64_CSB_STATUS_NSPK_MASTER_CONNECTING:
            //N/A
            break;
        case DRV_BM64_CSB_STATUS_CONNECTED_AS_BROADCAST_MASTER:
        case DRV_BM64_CSB_STATUS_CONNECTED_AS_BROADCAST_SLAVE:
            if (DRV_BM64_IsAllowedToSendCommand())
                DRV_BM64_MMI_ActionCommand ( 0xE5, gDrvBm64Obj.linkIndex);     //cancel/terminate NSPK link
            else
                gDrvBm64Obj.nextMMIActionReq2.TerminateCancelNSPKReq = 1;
            break;
        case DRV_BM64_CSB_STATUS_BROADCAST_MASTER_CONNECTING:
            if (DRV_BM64_IsAllowedToSendCommand())
                DRV_BM64_MMI_ActionCommand ( 0xE3, gDrvBm64Obj.linkIndex);     //cancel NSPK creation
            else
                gDrvBm64Obj.nextMMIActionReq2.CancelNSPKReq = 1;
            break;
    }
}

/*-----------------------------------------------------------------------------*/
void BTAPP_BroadcastBtnShortPress(void)
{
    //NA
}

/*-----------------------------------------------------------------------------*/
void BTAPP_BroadcastBtnDbClick(void)
{
    if(DRV_BM64_GetPowerStatus() != DRV_BM64_STATUS_READY)
        return;
    if(DRV_BM64_eCSBStatus.nspk_status == DRV_BM64_CSB_STATUS_CONNECTED_AS_BROADCAST_MASTER ||
    DRV_BM64_eCSBStatus.nspk_status == DRV_BM64_CSB_STATUS_CONNECTED_AS_BROADCAST_SLAVE)
    {
        if (DRV_BM64_IsAllowedToSendCommand())
            DRV_BM64_MMI_ActionCommand(0xF6, gDrvBm64Obj.linkIndex);
        else
            gDrvBm64Obj.nextMMIActionReq2.MMI_F6_Req = 1;
    }
}

/*-----------------------------------------------------------------------------*/
void BTAPP_ExitBroadcastRegisterMode( void )
{
    if(DRV_BM64_eCSBStatus.nspk_link == DRV_BM64_BROADCAST_MASTER)      //broadcast mode master speaker only
    {
        if (DRV_BM64_IsAllowedToSendCommand())
                DRV_BM64_MMI_ActionCommand ( 0xE3, gDrvBm64Obj.linkIndex);     //cancel NSPK creation
            else
                gDrvBm64Obj.nextMMIActionReq2.CancelNSPKReq = 1;
    }
}

/*-----------------------------------------------------------------------------*/
void BTAPP_GroupSpeakerSoundSync( void )
{
    if(DRV_BM64_eCSBStatus.nspk_link != DRV_BM64_NSPK_NO_LINK && DRV_BM64_eCSBStatus.nspk_link != DRV_BM64_BROADCAST_SLAVE)     //NSPK exist & not broadcast slave
    {
        if (DRV_BM64_IsAllowedToSendCommand())
            DRV_BM64_MMI_ActionCommand ( 0xF7, gDrvBm64Obj.linkIndex);
        else
            gDrvBm64Obj.nextMMIActionReq2.MMI_F7_Req = 1;
    }
}

/*-----------------------------------------------------------------------------*/
//below is calling handler for short button pressing event
void DRV_BM64_CallEventShort(const DRV_HANDLE handle)
{
    switch(DRV_BM64_CallStatus)
    {
        case DRV_BM64_CALL_IDLE:
            if (DRV_BM64_IsAllowedToSendCommand())
                DRV_BM64_MMI_ActionCommand ( 0x0A, gDrvBm64Obj.linkIndex);     //voice dial
            else
                gDrvBm64Obj.nextMMIActionReq2.VoiceDialReq = 1;
            break;
        case DRV_BM64_VOICE_DIAL:
            if (DRV_BM64_IsAllowedToSendCommand())
                DRV_BM64_MMI_ActionCommand ( 0x0B, gDrvBm64Obj.linkIndex);     //cancel voice dial
            else
                gDrvBm64Obj.nextMMIActionReq2.cancelVoiceDialReq  = 1;
            break;
        case DRV_BM64_CALL_INCOMMING:
            if (DRV_BM64_IsAllowedToSendCommand())
                DRV_BM64_MMI_ActionCommand ( DRV_BM64_MMI_ACCEPT_CALL, gDrvBm64Obj.linkIndex);
            else
                gDrvBm64Obj.nextMMIActionReq.AcceptCallReq = 1;
            break;
        case DRV_BM64_CALL_OUTGOING :
        case DRV_BM64_CALLING:
        case DRV_BM64_CALLING_WAITING:
        case DRV_BM64_CALLING_HOLD:
            if (DRV_BM64_IsAllowedToSendCommand())
                DRV_BM64_MMI_ActionCommand ( DRV_BM64_MMI_FORCE_END_CALL, gDrvBm64Obj.linkIndex);
            else
                gDrvBm64Obj.nextMMIActionReq.ForceEndCallReq = 1;
            break;
        default:
            break;
    }

}

/*-----------------------------------------------------------------------------*/
//below is calling handler for long button pressing event
void DRV_BM64_CallEventLong(const DRV_HANDLE handle)
{
    switch(DRV_BM64_CallStatus)
    {
        case DRV_BM64_CALL_IDLE:
            if (DRV_BM64_IsAllowedToSendCommand())
                DRV_BM64_MMI_ActionCommand ( DRV_BM64_MMI_LAST_NUMBER_REDIAL, gDrvBm64Obj.linkIndex);
            else
                gDrvBm64Obj.nextMMIActionReq.LastNumberRedialReq = 1;
            
            break;
        case DRV_BM64_VOICE_DIAL:
            //do nothing
            break;
        case DRV_BM64_CALL_INCOMMING:
            if (DRV_BM64_IsAllowedToSendCommand())
                DRV_BM64_MMI_ActionCommand ( DRV_BM64_MMI_REJECT_CALL, gDrvBm64Obj.linkIndex);
            else
                gDrvBm64Obj.nextMMIActionReq.RejectCallReq = 1;
            break;
        case DRV_BM64_CALL_OUTGOING :
            //do nothing
            break;
        case DRV_BM64_CALLING:
            if (DRV_BM64_IsAllowedToSendCommand())
                DRV_BM64_MMI_ActionCommand ( 0x0E, gDrvBm64Obj.linkIndex);     //transfer call
            else
                gDrvBm64Obj.nextMMIActionReq2.TransferCallReq = 1;
            break;
        case DRV_BM64_CALLING_WAITING:
        case DRV_BM64_CALLING_HOLD:
            //do nothing
            break;
        default:
            break;
    }

}

/*-----------------------------------------------------------------------------*/
/*below functions are for SPP process*/
/*-----------------------------------------------------------------------------*/
    
/*------------------------------------------------------------*/
static void _initSPP(void)
    {
    DRV_BM64_SPP_RxFifoHead = 0;
    DRV_BM64_SPP_RxFifoTail = 0;
    DRV_BM64_SPP_RxCounter = 0;
}

/*-----------------------------------------------------------------------------*/
void DRV_BM64_SPPBuffClear( void )
{
    uint16_t i;
    DRV_BM64_SPP_RxFifoHead = 0;
    DRV_BM64_SPP_RxFifoTail = 0;
    DRV_BM64_SPP_RxCounter = 0;
    for(i = 0; i< sizeof(DRV_BM64_SPP_RxFifo); i++)
        DRV_BM64_SPP_RxFifo[i] = 0;
}

/*-----------------------------------------------------------------------------*/
bool DRV_BM64_AddBytesToSPPBuff(uint8_t* data, uint8_t size)        //TRUE: data added ok, FALSE: data added fail, buffer is overflow
{
    uint8_t i;
    for(i=0; i<size; i++)
    {
        if(DRV_BM64_SPP_RxCounter < DRV_BM64_SPP_RxFifoSize)
        {
            DRV_BM64_SPP_RxFifo[DRV_BM64_SPP_RxFifoTail] = *data++;
            DRV_BM64_SPP_RxCounter++;
            if(DRV_BM64_SPP_RxFifoTail < DRV_BM64_SPP_RxFifoSize-1)
                DRV_BM64_SPP_RxFifoTail++;
            else
                DRV_BM64_SPP_RxFifoTail = 0;
        }
        else
        {
            return false;
        }
    }
    
    _clientCallBack(DRV_BM64_EVENT_BLESPP_MSG_RECEIVED, (uint8_t)size);
    return true;
}

/*-----------------------------------------------------------------------------*/
/*
 * void DRV_BM64_SaveLocalBDAddress(uint8_t* address)
 * void DRV_BM64_SaveMasterBDAddress(uint8_t* address)
 */
/*-----------------------------------------------------------------------------*/
void DRV_BM64_SaveLocalBDAddress(uint8_t* address)
{
    uint8_t i;
    for(i=0; i<6; i++)
        _localBDAddr[i] = *address++;
}
void DRV_BM64_SaveMasterBDAddress(uint8_t* address)
{
    uint8_t i;
    for(i=0; i<6; i++)
        _masterBDAddr[i] = *address++;
}
void DRV_BM64_SaveLocalBDName(uint8_t len, uint8_t* address)
{
    uint8_t i;
    for (i=0; (i < len) && (i < DRV_BM64_MAXBDNAMESIZE); i++)
    {
        _localBDName[i] = (char)*address++;
    }
    _localBDName[i] = '\0';
}

/*---------------------------------------------------------------------------------------------------------------*/
/*
 * bool BT_CustomerGATT_AttributeData(uint8_t attributeIndex, uint8_t* attributeData, uint8_t attributeDataLength)
 */
/*---------------------------------------------------------------------------------------------------------------*/
bool DRV_BM64_CustomerGATT_AttributeData(uint8_t attributeIndex, uint8_t* attributeData, uint8_t attributeDataLength)
{
    switch(attributeIndex)
    {
        case 1:
            if (DRV_BM64_IsAllowedToSendCommand()) {
                DRV_BM64_LinkBackToDeviceByBTAddress(attributeData);
            }
            else {
                //save BD address
                _linkbackBDAddr[0] = *attributeData++;
                _linkbackBDAddr[1] = *attributeData++;
                _linkbackBDAddr[2] = *attributeData++;
                _linkbackBDAddr[3] = *attributeData++;
                _linkbackBDAddr[4] = *attributeData++;
                _linkbackBDAddr[5] = *attributeData;
                gDrvBm64Obj.nextCommandReq.linkbackToDevAddr = 1;
            }
            break;
        case 2:
            if(*attributeData == 0x01)
            {
                if (DRV_BM64_IsAllowedToSendCommand()) {
                    DRV_BM64_MMI_ActionCommand(DRV_BM64_MMI_ANY_MODE_ENTERING_PAIRING, gDrvBm64Obj.linkIndex);
                    DRV_BM64_LinkbackStatus = DRV_BM64_PAIRING_START;
                }
                else {
                    gDrvBm64Obj.nextMMIActionReq.PairReq = 1;
                }
            }
            break;
        default:
            break;
    }
    return true;
}

static void _switchSampleFrequency(DRV_BM64_SAMPLE_FREQUENCY sampleFreq)
{             
    uint32_t iSampleFreq = 0;
    
    switch (sampleFreq)
    {
        case DRV_BM64_SAMPLEFREQ_8000:          // 8 kHz
            iSampleFreq = 8000;
            break;
       
        case DRV_BM64_SAMPLEFREQ_16000:          // 16 kHz
            iSampleFreq = 16000;            
            break;

        case DRV_BM64_SAMPLEFREQ_44100:          // 44.1 kHz
            iSampleFreq = 44100;
            break;

        case DRV_BM64_SAMPLEFREQ_48000:          // 48 kHz
            iSampleFreq = 48000;
            break;
            
        default:
            break;
    }

    if (iSampleFreq)
    {      
        gDrvBm64Obj.samplingRate = iSampleFreq;
        _clientCallBack(DRV_BM64_EVENT_SAMPLERATE_CHANGED, iSampleFreq);
    }
}

