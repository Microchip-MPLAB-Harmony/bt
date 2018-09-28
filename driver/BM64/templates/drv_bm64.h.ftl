/*******************************************************************************
  BM64 Bluetooth Static Driver implementation

  Company:
    Microchip Technology Inc.

  File Name:
    drv_bm64.h

  Summary:
    BM64 Bluetooth Static Driver main header file

  Description:
    This file is the header file for the external (public) API of the static 
    implementation of the BM64 driver.

    The BM64 is a Bluetooth 4.2 Stereo Module that supports classic A2DP, AVRCP,
    HFP, HSP, and SPP protocols as well as BLE (Bluetooth Low Energy).
    
    The BM64 streams I2S audio at up to 24-bit, 96 kHz.  It uses a UART to 
    receive commands from the host microcontroller (PIC32) and and send events
    back.
 
    All functions and constants in this file are named with the format
    DRV_BM64_xxx, where xxx is a function name or constant.  These names are
    redefined in the appropriate configuration?s system_config.h file to the
    format DRV_BT_xxx using #defines so that Bluetooth code in the application
    can be written as generically as possible (e.g. by writing DRV_BT_Open
    instead of DRV_BM64_Open etc.).

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

#ifndef DRV_BM64_H
#define DRV_BM64_H

// *****************************************************************************
// *****************************************************************************
// Section: Included Files
// *****************************************************************************
// *****************************************************************************

#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>     // for typedef __SIZE_TYPE__ size_t needed in  osal_impl_basic.h
#include "system/system_module.h"
#include "driver/driver_common.h"
#include "configuration.h"
<#if INCLUDE_BM64_I2S == true>
#include "driver/i2s/drv_i2s.h"
</#if>
#include "osal/osal.h"

// DOM-IGNORE-BEGIN
#ifdef __cplusplus  // Provide C++ Compatibility
extern "C" {
#endif
// DOM-IGNORE-END 

// *****************************************************************************
// *****************************************************************************
// Section: Constants
// *****************************************************************************
// *****************************************************************************

#define DRV_BM64_MAXBDNAMESIZE    32 

// BM64 defines based on I2S interface             
<#if INCLUDE_BM64_I2S == true>         
#define DRV_BM64_DATA32                   DRV_I2S_DATA32
#define DRV_BM64_BUFFER_HANDLE            DRV_I2S_BUFFER_HANDLE
#define DRV_BM64_BUFFER_HANDLE_INVALID    DRV_I2S_BUFFER_HANDLE_INVALID
#define DRV_BM64_BUFFER_EVENT             DRV_I2S_BUFFER_EVENT 
#define DRV_BM64_BUFFER_EVENT_COMPLETE    DRV_I2S_BUFFER_EVENT_COMPLETE
</#if>
    
// *****************************************************************************
// *****************************************************************************
// Section: Data Types
// *****************************************************************************
// *****************************************************************************
    
typedef enum {
    DRV_BM64_STATUS_NONE,
    DRV_BM64_STATUS_OFF,
    DRV_BM64_STATUS_ON,
    DRV_BM64_STATUS_READY
} DRV_BM64_DRVR_STATUS;         // BM64 driver status

typedef enum {
    DRV_BM64_PROTOCOL_A2DP = 1,
    DRV_BM64_PROTOCOL_AVRCP = 2,
    DRV_BM64_PROTOCOL_HFP_HSP = 4,
    DRV_BM64_PROTOCOL_SPP = 8,
    DRV_BM64_PROTOCOL_BLE = 16,
    DRV_BM64_PROTOCOL_ALL = 31
} DRV_BM64_PROTOCOL;            // BM64 protocols

typedef enum {
    DRV_BM64_SAMPLEFREQ_8000 = 0,
    DRV_BM64_SAMPLEFREQ_12000,
    DRV_BM64_SAMPLEFREQ_16000,
    DRV_BM64_SAMPLEFREQ_24000,
    DRV_BM64_SAMPLEFREQ_32000,
    DRV_BM64_SAMPLEFREQ_48000,
    DRV_BM64_SAMPLEFREQ_44100,
    DRV_BM64_SAMPLEFREQ_88000,
    DRV_BM64_SAMPLEFREQ_96000
} DRV_BM64_SAMPLE_FREQUENCY;    // BM64 sample frequency

typedef enum {
    DRV_BM64_NO_LINK_STATUS = 0,
    DRV_BM64_SCO_LINK_STATUS = 0x01,
    DRV_BM64_ACL_LINK_STATUS = 0x02, 
    DRV_BM64_HFP_LINK_STATUS = 0x04, 
    DRV_BM64_A2DP_LINK_STATUS = 0x08,
    DRV_BM64_AVRCP_LINK_STATUS = 0x10,
    DRV_BM64_SPP_LINK_STATUS = 0x20,
    DRV_BM64_IAP_LINK_STATUS = 0x40,
    DRV_BM64_MAP_LINK_STATUS = 0x80,
} DRV_BM64_LINKSTATUS;      // BM64 link status

typedef enum 
{
    DRV_BM64_REQ_NONE = 0,
    DRV_BM64_REQ_SYSTEM_ON, 
    DRV_BM64_REQ_SYSTEM_OFF,
} DRV_BM64_REQUEST;             // BM64 power on/off request

typedef enum 
{
    DRV_BM64_EVENT_NONE = 0,

    DRV_BM64_EVENT_NSPK_STATUS,
    DRV_BM64_EVENT_LINE_IN_STATUS,
    DRV_BM64_EVENT_A2DP_STATUS,
    DRV_BM64_EVENT_CALL_STATUS_CHANGED,

    DRV_BM64_EVENT_CODEC_TYPE,

    DRV_BM64_EVENT_HFP_CONNECTED,
    DRV_BM64_EVENT_HFP_DISCONNECTED,
    DRV_BM64_EVENT_A2DP_CONNECTED,
    DRV_BM64_EVENT_A2DP_DISCONNECTED,
    DRV_BM64_EVENT_AVRCP_CONNECTED,
    DRV_BM64_EVENT_AVRCP_DISCONNECTED,
    DRV_BM64_EVENT_SPP_CONNECTED,
    DRV_BM64_EVENT_IAP_CONNETED,
    DRV_BM64_EVENT_SPP_IAP_DISCONNECTED,
    DRV_BM64_EVENT_ACL_CONNECTED,
    DRV_BM64_EVENT_ACL_DISCONNECTED,
    DRV_BM64_EVENT_SCO_CONNECTED,
    DRV_BM64_EVENT_SCO_DISCONNECTED,
    DRV_BM64_EVENT_MAP_CONNECTED,
    DRV_BM64_EVENT_MAP_DISCONNECTED,

    DRV_BM64_EVENT_SYS_POWER_ON,
    DRV_BM64_EVENT_SYS_POWER_OFF,
    DRV_BM64_EVENT_SYS_STANDBY,
    DRV_BM64_EVENT_SYS_PAIRING_START,
    DRV_BM64_EVENT_SYS_PAIRING_OK,
    DRV_BM64_EVENT_SYS_PAIRING_FAILED,

    DRV_BM64_EVENT_LINKBACK_SUCCESS,
    DRV_BM64_EVENT_LINKBACK_FAILED,

    DRV_BM64_EVENT_BD_ADDR_RECEIVED,
    DRV_BM64_EVENT_PAIR_RECORD_RECEIVED,
    DRV_BM64_EVENT_LINK_MODE_RECEIVED,

    DRV_BM64_EVENT_PLAYBACK_STATUS_CHANGED,
    DRV_BM64_EVENT_AVRCP_VOLUME_CTRL,
    DRV_BM64_EVENT_AVRCP_ABS_VOLUME_CHANGED,
    DRV_BM64_EVENT_HFP_VOLUME_CHANGED,
    
    DRV_BM64_EVENT_VOLUME_CHANGED,     
    DRV_BM64_EVENT_SAMPLERATE_CHANGED,
            
    DRV_BM64_EVENT_NSPK_SYNC_POWER_OFF,
    DRV_BM64_EVENT_NSPK_SYNC_VOL_CTRL,
    DRV_BM64_EVENT_NSPK_SYNC_INTERNAL_GAIN,
    DRV_BM64_EVENT_NSPK_SYNC_ABS_VOL,
    DRV_BM64_EVENT_NSPK_CHANNEL_SETTING,
    DRV_BM64_EVENT_NSPK_ADD_SPEAKER3,

    DRV_BM64_EVENT_LE_STATUS_CHANGED,
    DRV_BM64_EVENT_LE_ADV_CONTROL_REPORT,
    DRV_BM64_EVENT_LE_CONNECTION_PARA_REPORT,
    DRV_BM64_EVENT_LE_CONNECTION_PARA_UPDATE_RSP,
    DRV_BM64_EVENT_GATT_ATTRIBUTE_DATA,
    
    DRV_BM64_EVENT_PORT0_INPUT_CHANGED,
    DRV_BM64_EVENT_PORT1_INPUT_CHANGED,
    DRV_BM64_EVENT_PORT2_INPUT_CHANGED,
    DRV_BM64_EVENT_PORT3_INPUT_CHANGED,
            
    DRV_BM64_EVENT_BLESPP_MSG_RECEIVED,
    DRV_BM64_EVENT_BLE_STATUS_CHANGED,
            
} DRV_BM64_EVENT;       // events that can be returned to a client via callback

typedef enum {
    DRV_BM64_PLAYING_STOPPED,
    DRV_BM64_PLAYING_PLAYING,
    DRV_BM64_PLAYING_PAUSED,
    DRV_BM64_PLAYING_FF,
    DRV_BM64_PLAYING_FR,
    DRV_BM64_PLAYING_ERROR
} DRV_BM64_PLAYINGSTATUS;

typedef enum {
    DRV_BM64_BLE_STATUS_STANDBY,
    DRV_BM64_BLE_STATUS_ADVERTISING,
    DRV_BM64_BLE_STATUS_SCANNING,
    DRV_BM64_BLE_STATUS_CONNECTED
} DRV_BM64_BLE_STATUS;

// prototype for callback for DRV_BM64_EventHandlerSet
typedef void (*DRV_BM64_EVENT_HANDLER) (DRV_BM64_EVENT event, uint32_t param, uintptr_t contextHandle);

<#if INCLUDE_BM64_I2S == true>
// prototype for callback for DRV_BM64_BufferEventHandlerSet
typedef void (*DRV_BM64_BUFFER_EVENT_HANDLER) (DRV_BM64_BUFFER_EVENT event, uintptr_t contextHandle);
</#if>
// *****************************************************************************
// *****************************************************************************
// Section: Interface Routines
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

  Example:
    <code>
    // (in SYS_Initialize, system_init.c)* 
 
 	DRV_BM64_Initialize(); 
    </code>

  Remarks:
    This routine must be called before any other BM64 driver routine is called.
    This routine should only be called once during system initialization.
    This routine will never block for hardware access.

*/

void DRV_BM64_Initialize( void );

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

  Example:
    <code>
    // (in SYS_Tasks, system_tasks.c)
 
    // Maintain Device Drivers
	DRV_BM64_Tasks(); 
    </code>

  Remarks:
    This routine is not normally called directly by an application.  Instead it
    is called by the system's Tasks routine (SYS_Tasks).
*/

void DRV_BM64_Tasks( void );

// *****************************************************************************
/* Function DRV_BM64_Status:
 
        SYS_STATUS DRV_BM64_Status( void );

  Summary:
    Gets the current system status of the BM64 Bluetooth driver module.

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
 * 
  Example:
    <code>
 * // note generic version of call (DRV_BT instead of DRV_BM64) is used
    if (SYS_STATUS_READY == DRV_BT_Status())
    {
        // This means the driver can be opened using the
        // DRV_BT_Open() function.
    }
    </code> 

  Remarks:
    A driver can opened only when its status is SYS_STATUS_READY.
*/

SYS_STATUS DRV_BM64_Status( void );

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

  Example:
    <code>    
    case APP_STATE_WAIT_INIT:
    {
       // note generic version of call (DRV_BT instead of DRV_BM64) is used
       if (DRV_BT_STATUS_READY == DRV_BT_GetPowerStatus())
       {           
           appData.state=APP_STATE_IDLE;
           // start can processing audio
       }
    }
    break;
    </code>

  Remarks:
    A status of DRV_BT_STATUS_READY means the drivers state machine has finished
    initialization and is ready to stream audio.
*/

DRV_BM64_DRVR_STATUS DRV_BM64_GetPowerStatus( void );

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

  Example:
    <code>
    // note generic version of call (DRV_BT instead of DRV_BM64) is used
    DRV_BT_TaskReq(DRV_BM64_REQ_SYSTEM_ON);
    </code>

  Remarks:
    None.
*/

void DRV_BM64_TaskReq(DRV_BM64_REQUEST request);

// *****************************************************************************
/* Function DRV_BM64_Open: 

        DRV_HANDLE DRV_BM64_Open(const DRV_IO_INTENT ioIntent,
                          const DRV_BM64_PROTOCOL protocol);

  Summary:
    Open the specified BM64 driver instance and returns a handle to it

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

  Example:
    <code>
    case APP_STATE_OPEN:
    {
        if (SYS_STATUS_READY == DRV_BT_Status())
        { 
            // open BT module, including RX audio stream
            // note generic version of call (DRV_BT instead of DRV_BM64) is used
            appData.bt.handle = DRV_BT_Open(DRV_IO_INTENT_READ, DRV_BT_PROTOCOL_ALL);

            if(appData.bt.handle != DRV_HANDLE_INVALID)
            {
                appData.state = APP_STATE_SET_BT_BUFFER_HANDLER;
            }
            else
            {
                // Got an Invalid Handle.  Wait for BT module to Initialize
            }
        }
    }
    break;
    </code>

  Remarks:
    The handle returned is valid until the DRV_BM64_Close routine is called.
    This routine will never block waiting for hardware.  If the requested intent
    flags are not supported, the routine will return DRV_HANDLE_INVALID.  This
    function is thread safe in a RTOS application. It should not be called in an
    ISR.
 
    Currently only one client is allowed at a time.
*/

DRV_HANDLE DRV_BM64_Open(const DRV_IO_INTENT ioIntent, const DRV_BM64_PROTOCOL protocol);

// *****************************************************************************
/* Function DRV_BM64_Close: 

        void DRV_BM64_Close(DRV_HANDLE handle);

  Summary:
    Close an opened-instance of the BM64 Bluetooth driver.

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
 
   Example:
    <code>
    // note generic version of call (DRV_BT instead of DRV_BM64) is used
    DRV_BT_Close(appData.bt.handle);
    </code>* 

  Remarks:
    Usually there is no need for the driver client to verify that the Close
    operation has completed.  The driver will abort any ongoing operations
    when this routine is called.
*/

void DRV_BM64_Close(const DRV_HANDLE handle);

<#if INCLUDE_BM64_I2S == true>
// *****************************************************************************
/* Function DRV_BM64_BufferEventHandlerSet:

       void DRV_BM64_EventHandlerSet(DRV_HANDLE handle,
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

  Example:
    <code>
    case APP_STATE_SET_BT_BUFFER_HANDLER:
    {
        // note generic version of call (DRV_BT instead of DRV_BM64) is used
        DRV_BT_BufferEventHandlerSet(appData.bt.handle,
                                      appData.bt.bufferHandler,
                                      appData.bt.context); 

        DRV_BT_EventHandlerSet(appData.bt.handle,
                                      appData.bt.eventHandler,
                                      (uintptr_t)0);                                  

        appData.state = APP_STATE_CODEC_OPEN;            
    }
    break; 
    </code>

  Remarks:
    If the client does not want to be notified when the command has completed, 
    it does not need to register a callback.
*/
void DRV_BM64_BufferEventHandlerSet(DRV_HANDLE handle, 
        const DRV_BM64_BUFFER_EVENT_HANDLER eventHandler, const uintptr_t contextHandle);
</#if>
// *****************************************************************************
/* Function DRV_BM64_EventHandlerSet:

        void DRV_BM64_EventHandlerSet(DRV_HANDLE handle,
            const DRV_BM64_EVENT_HANDLER eventHandler,
            const uintptr_t contextHandle);

  Summary:
    This function allows a client to identify an event handling function
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

  Example:
    <code>
    case APP_STATE_SET_BT_BUFFER_HANDLER:
    {
        DRV_BT_BufferEventHandlerSet(appData.bt.handle,
                                      appData.bt.bufferHandler,
                                      appData.bt.context); 

        // note generic version of call (DRV_BT instead of DRV_BM64) is used
        DRV_BT_EventHandlerSet(appData.bt.handle,
                                      appData.bt.eventHandler,
                                      (uintptr_t)0);                                  

        appData.state = APP_STATE_CODEC_OPEN;            
    }
    break; 
    </code>

  Remarks:
    If the client does not want to be notified when an event has occurred, it
    does not need to register a callback.
*/

void DRV_BM64_EventHandlerSet(DRV_HANDLE handle,
        const DRV_BM64_EVENT_HANDLER eventHandler, const uintptr_t contextHandle);

<#if INCLUDE_BM64_I2S == true>
// *****************************************************************************
/*
Function:
	void DRV_BM64_BufferAddRead(const DRV_HANDLE handle,
		DRV_BM6_BUFFER_HANDLE *bufferHandle, void *buffer, size_t size)

  Summary:
    Schedule a non-blocking driver read operation.

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
    buffer       - pointer to buffer that will contain received data
    size         - buffer size in bytes.

  Returns:
    The bufferHandle parameter will contain the return buffer handle. This will
    be DRV_BM64_BUFFER_HANDLE_INVALID if the function was not successful.
 
   Example:
    <code>
    case APP_STATE_BT_BUFFER_COMPLETE:
    {
       //BT RX
       if (!_bufferUsed[appData.readIndex])
       {
           //Next BT Read Queued
           // note generic version of call (DRV_BT instead of DRV_BM64) is used
           DRV_BT_BufferAddRead(appData.bt.handle, 
                                 &appData.bt.readBufHandle, 
                                 audioBuffer[appData.readIndex], 
                                 appData.bt.bufferSize); 

           if(appData.bt.readBufHandle != DRV_BT_BUFFER_HANDLE_INVALID)
           {
               appData.bt.readBufHandle = DRV_BT_BUFFER_HANDLE_INVALID; 
               _bufferUsed[appData.readIndex] = true;

               //QUEUE HEAD Index (for next BT read)  
               appData.readIndex++;      
               if(appData.readIndex >= AUDIO_QUEUE_SIZE)
               {
                   appData.readIndex = 0;
               }                                 
               appData.state = APP_STATE_BT_WAIT_FOR_BUFFER_COMPLETE;
           }
           else
           {
               SYS_DEBUG(0, "BT Buffer Read FAILED!!!");
           }
       }
       else
       {
           //Overrun -- Wait for Read buffer to become available.
           SYS_DEBUG(0, "Buffer Overrun\r\n");
       }                             
    }
    break;
    </code> 

  Remarks:
    This function is thread safe in a RTOS application. It can be called from
    within the BM64 Driver Buffer Event Handler that is registered by this
    client. It should not be called in the event handler associated with another
    BM64 driver instance. It should not otherwise be called directly in an ISR.

*/

void DRV_BM64_BufferAddRead (const DRV_HANDLE handle, 
        DRV_BM64_BUFFER_HANDLE *bufferHandle, void *buffer, size_t size);

// *****************************************************************************
/* Function DRV_BM64_VolumeUp:

        void DRV_BM64_VolumeUp(const DRV_HANDLE handle);

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

  Example:
    <code>
    case BUTTON_STATE_PRESSED:      // (debouncing not shown)
    {
        if (BSP_SwitchStateGet(BSP_SWITCH_1)==BSP_SWITCH_STATE_PRESSED))
        {
            // note generic version of call (DRV_BT instead of DRV_BM64) is used 
            DRV_BT_volumeUp(appData.bt.handle);               
            appData.buttonState=BUTTON_STATE_WAIT_FOR_RELEASE;                
        }
    }
    break; 
    </code>

  Remarks:
    This will result in a callback with the event DRV_BM64_EVENT_VOLUME_CHANGED
    specifying the new volume setting for the codec.
*/

void DRV_BM64_volumeUp(const DRV_HANDLE handle);

// *****************************************************************************
/* Function DRV_BM64_VolumeDown:

        void DRV_BM64_VolumeDown(const DRV_HANDLE handle);

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

  Example:
    <code>
    case BUTTON_STATE_PRESSED:      // (debouncing not shown)
    {
        if (BSP_SwitchStateGet(BSP_SWITCH_2)==BSP_SWITCH_STATE_PRESSED))
        {
            // note generic version of call (DRV_BT instead of DRV_BM64) is used 
            DRV_BT_volumeUp(appData.bt.handle);               
            appData.buttonState=BUTTON_STATE_WAIT_FOR_RELEASE;                
        }
    }
    break; 
    </code>

  Remarks:
    This will result in a callback with the event DRV_BM64_EVENT_VOLUME_CHANGED
    specifying the new volume setting for the codec.
*/

void DRV_BM64_volumeDown(const DRV_HANDLE handle);

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

  Example:
    <code>
    uint8_t volume;
 
    // note generic version of call (DRV_BT instead of DRV_BM64) is used 
    volume = DRV_BT_VolumeGet(appData.bt.handle);
    </code>

  Remarks:
    None.
*/

uint8_t DRV_BM64_VolumeGet(const DRV_HANDLE handle);	//returns 7-bit value 0-127

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

  Example:
    <code>
    // note generic version of call (DRV_BT instead of DRV_BM64) is used * 
    volume = DRV_BT_VolumeGet(appData.bt.handle,50);       // set volume to 50%
    </code>

  Remarks:
    None.
*/

void DRV_BM64_VolumeSet(const DRV_HANDLE handle, uint8_t volume);

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

  Example:
    <code>
    uint32_t sampleRate;
 
    // note generic version of call (DRV_BT instead of DRV_BM64) is used
    sampleRate = DRV_BT_SamplingRateGet(appData.bt.handle);
    </code>

  Remarks:
    None.
*/

uint32_t DRV_BM64_SamplingRateGet(DRV_HANDLE handle);

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

  Example:
    <code>
    // set sample rate to 44.1 kHz
    // note generic version of call (DRV_BT instead of DRV_BM64) is used
    DRV_BT_SamplingRateSet(appData.bt.handle, 44100);
    </code>

  Remarks:
    None.
*/

void DRV_BM64_SamplingRateSet(DRV_HANDLE handle, uint32_t samplingRate);
</#if>

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

  Example:
    <code>
    case BUTTON_STATE_PRESSED:      // (debouncing not shown)
    {
        if (BSP_SwitchStateGet(BSP_SWITCH_1)==BSP_SWITCH_STATE_PRESSED))
        {
            // note generic version of call (DRV_BT instead of DRV_BM64) is used
            DRV_BT_EnterBTPairingMode(appData.bt.handle);               
            appData.buttonState=BUTTON_STATE_WAIT_FOR_RELEASE;                
        }
    }
    break;
    </code>

  Remarks:
    None.
*/

void DRV_BM64_EnterBTPairingMode(const DRV_HANDLE handle);

// *****************************************************************************
/* Function DRV_BM64_ForgetAllLinks:

        void DRV_BM64_ForgetAllLinks(const DRV_HANDLE handle);

  Summary:
    Forget all pairings.

  Description:
    Forget (erase) all links and pairings stored in EEPROM.
 
  Precondition:
    DRV_BM64_Open must have been called to obtain a valid opened device handle.

  Parameters:
    handle      - valid handle to an opened BM64 device driver unique to client

  Returns:
    None.

  Example:
    <code>
    case BUTTON_STATE_PRESSED:      // (debouncing not shown)
    {
        if (BSP_SwitchStateGet(BSP_SWITCH_2)==BSP_SWITCH_STATE_PRESSED))
        {
            // note generic version of call (DRV_BT instead of DRV_BM64) is used
            DRV_BT_ForgetAllLinks(appData.bt.handle);               
            appData.buttonState=BUTTON_STATE_WAIT_FOR_RELEASE;                
        }
    }
    break; 
    </code>

  Remarks:
    After this is called, one must call DRV_BM64_EnterBTPairingMode to establish
    a connection to a Bluetooth host again. 
*/

void DRV_BM64_ForgetAllLinks(const DRV_HANDLE handle);

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

  Example:
    <code>
    case BUTTON_STATE_PRESSED:      // (debouncing not shown)
    {
        if (BSP_SwitchStateGet(BSP_SWITCH_2)==BSP_SWITCH_STATE_PRESSED))
        {
            // note generic version of call (DRV_BT instead of DRV_BM64) is used
            DRV_BT_LinkLastDevice(appData.bt.handle);               
            appData.buttonState=BUTTON_STATE_WAIT_FOR_RELEASE;                
        }
    }
    break; 
    </code>

  Remarks:
    None.
*/

void DRV_BM64_LinkLastDevice(const DRV_HANDLE handle);

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

  Example:
    <code>
    case BUTTON_STATE_PRESSED:      // (debouncing not shown)
    {
        if (BSP_SwitchStateGet(BSP_SWITCH_2)==BSP_SWITCH_STATE_PRESSED))
        {
            // note generic version of call (DRV_BT instead of DRV_BM64) is used
            DRV_BT_DisconnectAllLinks(appData.bt.handle);               
            appData.buttonState=BUTTON_STATE_WAIT_FOR_RELEASE;                
        }
    }
    break; 
    </code>

  Remarks:
    Does not unpair the device, just disconnects.  Use DRV_BM64_LinkLastDevice
    to reconnect.  Use DRV_BM64_ForgetAllLinks to forget all pairings.
*/

void DRV_BM64_DisconnectAllLinks(const DRV_HANDLE handle);

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

  Example:
    <code>
    case BUTTON_STATE_PRESSED:      // (debouncing not shown)
    {
        if (BSP_SwitchStateGet(BSP_SWITCH_3)==BSP_SWITCH_STATE_PRESSED))
        {             
            DRV_BT_PLAYINGSTATUS playingStatus = DRV_BT_GetPlayingStatus(appData.bt.handle);
            if ((playingStatus==DRV_BT_PLAYING_FF)||(playingStatus==DRV_BT_PLAYING_FR))
            {
               // note generic version of call (DRV_BT instead of DRV_BM64) is used
               if (DRV_BT_GetLinkStatus(appData.bt.handle) & DRV_BT_AVRCP_LINK_STATUS)
               {                 
                   DRV_BT_CancelForwardOrRewind(appData.bt.handle);
               }
            }
        }
    }
    break;
    </code>

  Remarks:
    None.
*/

DRV_BM64_LINKSTATUS DRV_BM64_GetLinkStatus(const DRV_HANDLE handle);

<#if INCLUDE_BM64_I2S == true>
// *****************************************************************************
/* Function DRV_BM64_Play:

        DRV_BM64_Play(const DRV_HANDLE handle);

  Summary:
    Start playback.

  Description:
    Send an AVRCP command to the host device to initiate or resume playback.
 
  Precondition:
    DRV_BM64_Open must have been called to obtain a valid opened device handle.

  Parameters:
    handle      - valid handle to an opened BM64 device driver unique to client

  Returns:
    None.

  Example:
    <code>
    case BUTTON_STATE_PRESSED:      // (debouncing not shown)
    {
        if (BSP_SwitchStateGet(BSP_SWITCH_3)==BSP_SWITCH_STATE_PRESSED))
        {
            // note generic version of call (DRV_BT instead of DRV_BM64) is used
            DRV_BT_Play(appData.bt.handle);               
            appData.buttonState=BUTTON_STATE_WAIT_FOR_RELEASE;                
        }
    }
    break; 
    </code>

  Remarks:
    None.
*/

void DRV_BM64_Play(const DRV_HANDLE handle);

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

  Example:
    <code>
    case BUTTON_STATE_PRESSED:      // (debouncing not shown)
    {
        if (BSP_SwitchStateGet(BSP_SWITCH_3)==BSP_SWITCH_STATE_PRESSED))
        {
            // note generic version of call (DRV_BT instead of DRV_BM64) is used
            DRV_BT_Pause(appData.bt.handle);               
            appData.buttonState=BUTTON_STATE_WAIT_FOR_RELEASE;                
        }
    }
    break; 
    </code>

  Remarks:
    None.
*/

void DRV_BM64_Pause(const DRV_HANDLE handle);

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

  Example:
    <code>
    case BUTTON_STATE_PRESSED:      // (debouncing not shown)
    {
        if (BSP_SwitchStateGet(BSP_SWITCH_3)==BSP_SWITCH_STATE_PRESSED))
        {
            // note generic version of call (DRV_BT instead of DRV_BM64) is used
            DRV_BT_Stop(appData.bt.handle);               
            appData.buttonState=BUTTON_STATE_WAIT_FOR_RELEASE;                
        }
    }
    break; 
    </code>

  Remarks:
    None.
*/

void DRV_BM64_Stop(const DRV_HANDLE handle);

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

  Example:
    <code>
    case BUTTON_STATE_PRESSED:      // (debouncing not shown)
    {
        if (BSP_SwitchStateGet(BSP_SWITCH_3)==BSP_SWITCH_STATE_PRESSED))
        {
            // note generic version of call (DRV_BT instead of DRV_BM64) is used
            DRV_BT_PlayPause(appData.bt.handle);               
            appData.buttonState=BUTTON_STATE_WAIT_FOR_RELEASE;                
        }
    }
    break; 
    </code>

  Remarks:
    None.
*/

void DRV_BM64_PlayPause(const DRV_HANDLE handle);

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

  Example:
    <code>
    case BUTTON_STATE_PRESSED:      // (debouncing not shown)
    {
        if (BSP_SwitchStateGet(BSP_SWITCH_5)==BSP_SWITCH_STATE_PRESSED))
        {
            // note generic version of call (DRV_BT instead of DRV_BM64) is used
            DRV_BT_PlayPreviousSong(appData.bt.handle);               
            appData.buttonState=BUTTON_STATE_WAIT_FOR_RELEASE;                
        }
    }
    break; 
    </code>

  Remarks:
    None.
*/

void DRV_BM64_PlayPreviousSong(const DRV_HANDLE handle);

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

  Example:
    <code>
    case BUTTON_STATE_PRESSED:      // (debouncing not shown)
    {
        if (BSP_SwitchStateGet(BSP_SWITCH_3)==BSP_SWITCH_STATE_PRESSED))
        {
            // note generic version of call (DRV_BT instead of DRV_BM64) is used
            DRV_BT_PlayNextSong(appData.bt.handle);               
            appData.buttonState=BUTTON_STATE_WAIT_FOR_RELEASE;                
        }
    }
    break; 
    </code>

  Remarks:
    None.
*/

void DRV_BM64_PlayNextSong(const DRV_HANDLE handle);

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

  Example:
    <code>
    case BUTTON_STATE_PRESSED:      // (debouncing not shown)
    {
        if (BSP_SwitchStateGet(BSP_SWITCH_5)==BSP_SWITCH_STATE_PRESSED))
        {
            // note generic version of call (DRV_BT instead of DRV_BM64) is used
            DRV_BT_Rewind(appData.bt.handle);               
            appData.buttonState=BUTTON_STATE_WAIT_FOR_RELEASE;                
        }
    }
    break; 
    </code>

  Remarks:
    None.
*/

void DRV_BM64_Rewind(const DRV_HANDLE handle);

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

  Example:
    <code>
    case BUTTON_STATE_PRESSED:      // (debouncing not shown)
    {
        if (BSP_SwitchStateGet(BSP_SWITCH_5)==BSP_SWITCH_STATE_PRESSED))
        {
            // note generic version of call (DRV_BT instead of DRV_BM64) is used
            DRV_BT_FastForward(appData.bt.handle);               
            appData.buttonState=BUTTON_STATE_WAIT_FOR_RELEASE;                
        }
    }
    break; 
    </code>

  Remarks:
    None.
*/

void DRV_BM64_FastForward(const DRV_HANDLE handle);

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

  Example:
    <code>
    case BUTTON_STATE_PRESSED:      // (debouncing not shown)
    {
        if (BSP_SwitchStateGet(BSP_SWITCH_3)==BSP_SWITCH_STATE_PRESSED))
        {
            // note generic version of call (DRV_BT instead of DRV_BM64) is used
            DRV_BT_CancelForwardOrRewind(appData.bt.handle);               
            appData.buttonState=BUTTON_STATE_WAIT_FOR_RELEASE;                
        }
    }
    break; 
    </code>

  Remarks:
    None.
*/

void DRV_BM64_CancelForwardOrRewind(const DRV_HANDLE handle);

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

  Example:
    <code>
    case BUTTON_STATE_PRESSED:      // (debouncing not shown)
    {
        if (BSP_SwitchStateGet(BSP_SWITCH_3)==BSP_SWITCH_STATE_PRESSED))
        { 
            // note generic version of call (DRV_BT instead of DRV_BM64) is used
            DRV_BT_PLAYINGSTATUS playingStatus = DRV_BT_GetPlayingStatus(appData.bt.handle);
            if ((playingStatus==DRV_BT_PLAYING_FF)||(playingStatus==DRV_BT_PLAYING_FR))
            {
               if (DRV_BT_GetLinkStatus(appData.bt.handle) & DRV_BT_AVRCP_LINK_STATUS)
               {                 
                   DRV_BT_CancelForwardOrRewind(appData.bt.handle);
               }
            }
        }
    }
    break;
    </code>

  Remarks:
    None.
*/

DRV_BM64_PLAYINGSTATUS DRV_BM64_GetPlayingStatus(const DRV_HANDLE handle);
</#if>

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

  Example:
    <code>
    laString tempStr;                                      
    char buf [18];
  
    // note generic version of call (DRV_BT instead of DRV_BM64) is used
    DRV_BT_GetBDAddress(appData.bt.handle, buf);
    tempStr = laString_CreateFromCharBuffer(buf, &LiberationSans12);
    laLabelWidget_SetText(GFX_BTADDRESS_VALUE, tempStr);    // display BT address
    laString_Destroy(&tempStr);
    </code>

  Remarks:
    Buffer must be at least 18 bytes in length (6 octets separated by ?:?, e.g.
    able to hold "12:34:56:78:90:12\0").
*/

void DRV_BM64_GetBDAddress(const DRV_HANDLE handle, char* buffer);

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

  Example:
    <code>
    laString tempStr;                                      
    char buf [DRV_BT_MAXBDNAMESIZE+1];

    // note generic version of call (DRV_BT instead of DRV_BM64) is used
    DRV_BT_GetBDName(appData.bt.handle, buf, DRV_BT_MAXBDNAMESIZE+1);
    tempStr = laString_CreateFromCharBuffer(buf, &LiberationSans12);
    laLabelWidget_SetText(GFX_BTNAME_VALUE, tempStr);   // display BT name
    laString_Destroy(&tempStr);
    </code>

  Remarks:
    If name is longer than buflen-1 bytes long, it will be truncated to fit
    inside the buffer.
*/

void DRV_BM64_GetBDName(const DRV_HANDLE handle, char* buffer, const uint8_t buflen);

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

  Example:
    <code>
    // note generic version of call (DRV_BT instead of DRV_BM64) is used
    DRV_BT_SetBDName(appData.bt.handle, "Temporary BM64 Name");
    </code>

  Remarks:
    The name is set for this session only; if the BM64 is reset (e.g. power is
    lost) the name will revert to the Bluetooth name stored in EEPROM.
*/

void DRV_BM64_SetBDName(const DRV_HANDLE handle, const char* buffer);

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

  Example:
    <code>
    uint8_t byte;
    
    // note generic versions of calls (DRV_BT instead of DRV_BM64) is used
    DRV_BT_ClearBLEData(appData.bt.handle);

    // wait for byte to arrive
    while (!DRV_BT_ReadByteFromBLE(appData.bt.handle, &byte))
    {
        // should have some sort of way to break out of here if byte never arrives
    }
    </code>

  Remarks:
 None.
*/

void DRV_BM64_ClearBLEData( const DRV_HANDLE handle );

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

  Example:
    <code>
    uint8_t byte;
 
    // note generic version of call (DRV_BT instead of DRV_BM64) is used
    if (DRV_BT_ReadByteFromBLE(appData.bt.handle, &byte))  // if byte received
    {
        // do something
    } 
    </code>

  Remarks:
    None.
*/

bool DRV_BM64_ReadByteFromBLE(const DRV_HANDLE handle, uint8_t* byte);

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

  Example:
    <code>
    #define BUFSIZE  100
    uint8_t buf [BUFSIZE];

    // note generic version of call (DRV_BT instead of DRV_BM64) is used
    if (DRV_BT_ReadDataFromBLE(appData.bt.handle, buf, BUFSIZE))  // if data received
    {
        // do something
    } 
    </code>

  Remarks:
    No more than size bytes will be returned, even if more are available.
*/

bool DRV_BM64_ReadDataFromBLE(const DRV_HANDLE handle, uint8_t* byte, uint16_t size );

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

  Example:
    <code>
    uint8_t byte;
 
    byte = 10;     // set to some value

    // note generic version of call (DRV_BT instead of DRV_BM64) is used
    DRV_BT_SendByteOverBLE(appData.bt.handle, byte);
    </code>

  Remarks:
    None.
*/

void DRV_BM64_SendByteOverBLE(const DRV_HANDLE handle, uint8_t byte);

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

  Example:
    <code>
    #define BUFSIZE    100
    uint8_t buf [BUFSIZE];
 
    // (code to fill in buffer with data)

    // note generic version of call (DRV_BT instead of DRV_BM64) is used
    DRV_BT_SendDataOverBLE(appData.bt.handle, buf, BUFSIZE);
    </code>

  Remarks:
    None.
*/

void DRV_BM64_SendDataOverBLE(const DRV_HANDLE handle, uint8_t* bytes, uint16_t size);

// *****************************************************************************
/* Function DRV_BM64_BLE_QueryStatus:

        void DRV_BM64_BLE_QueryStatus(const DRV_HANDLE handle);

  Summary:
    Query BM64 LE status.

  Description:
    Queries the BM64 to respond with a DRV_BM64_EVENT_BLE_STATUS_CHANGED event,
    which will indicate if the BM64 BLE status is standby, advertising,
    scanning or connected.

  Precondition:
    DRV_BM64_Open must have been called to obtain a valid opened device handle.

  Parameters:
    handle      - valid handle to an opened BM64 device driver unique to client

  Returns:
    None.

  Example:
    <code>
    // note generic version of call (DRV_BT instead of DRV_BM64) is used
    DRV_BT_BLE_QueryStatus(appData.bt.handle);
 
    . . .
 
    // later, a call will come back to the event handler callback function
    // (previously set up via a call to DRV_BM64_EventHandlerSet)
    static void _BLEEventHandler(DRV_BT_EVENT event, uint32_t param, uintptr_t context)
    {
        switch(event)
        {
            case DRV_BT_EVENT_BLE_STATUS_CHANGED:
            {           
                // do case switch based on param variable
            }
       }
    }    
    </code>

  Remarks:
    RV_BM64_BLE_QueryStatus is non-blocking; it returns right away and sometime
    later (perhaps tens or hundreds of ms) the event handler callback will be
    called.
*/

void DRV_BM64_BLE_QueryStatus(const DRV_HANDLE handle);

// *****************************************************************************
/* Function DRV_BM64_BLE_EnableAdvertising:

        void DRV_BM64_BLE_EnableAdvertising(const DRV_HANDLE handle, bool enable);

  Summary:
    Enable or disable advertising.

  Description:
    Enable or disable BLE advertising.
 
  Precondition:
    DRV_BM64_Open must have been called to obtain a valid opened device handle.

  Parameters:
    handle      - valid handle to an opened BM64 device driver unique to client
    enable      - true to enable advertising, false to disable advertising

  Returns:
    None.

  Example:
    <code>
    // note generic version of call (DRV_BT instead of DRV_BM64) is used
    DRV_BM64_BLE_EnableAdvertising(appData.bt.handle, true);
    </code>

  Remarks:
    None.
*/

void DRV_BM64_BLE_EnableAdvertising(const DRV_HANDLE handle, bool enable);

//DOM-IGNORE-BEGIN
#ifdef __cplusplus
}
#endif
//DOM-IGNORE-END

#endif
