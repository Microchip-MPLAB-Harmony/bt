/*******************************************************************************
  MPLAB Harmony Application Header File

  Company:
    Microchip Technology Inc.

  File Name:
    audio.h

  Summary:
    This header file provides prototypes and definitions for the application.

  Description:
    This header file provides function prototypes and data type definitions for
    the application.  Some of these are required by the system (such as the
    "APP_Initialize" and "APP_Tasks" prototypes) and some of them are only used
    internally by the application (such as the "APP_STATES" definition).  Both
    are defined here for convenience.
*******************************************************************************/

//DOM-IGNORE-BEGIN
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
//DOM-IGNORE-END

#ifndef _AUDIO_H
#define _AUDIO_H

// *****************************************************************************
// *****************************************************************************
// Section: Included Files
// *****************************************************************************
// *****************************************************************************

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdlib.h>
#include "configuration.h"
#include "definitions.h"
#include "user.h"           // defines switches and LEDs

// DOM-IGNORE-BEGIN
#ifdef __cplusplus  // Provide C++ Compatibility
extern "C" {
#endif
// DOM-IGNORE-END

#define AUDIO_BUFFER_SAMPLES   4000
#define AUDIO_QUEUE_SIZE          2  //PING/PONG buffer scheme
    
#define DELAY_AFTER_VOLME_CHG   1000    // must wait 1000 sec between volume changes from button    

// *****************************************************************************
// *****************************************************************************
// Section: Type Definitions
// *****************************************************************************
// *****************************************************************************

// *****************************************************************************
/* Application states

  Summary:
    Application states enumeration

  Description:
    This enumeration defines the valid application states.  These states
    determine the behavior of the application at various times.
*/

typedef enum
{
    AUDIO_STATE_WAIT_OPEN,    
    AUDIO_STATE_OPEN,
    AUDIO_STATE_SET_BT_BUFFER_HANDLER,     
    AUDIO_STATE_CODEC_OPEN,
    AUDIO_STATE_CODEC_SET_BUFFER_HANDLER,
    AUDIO_STATE_INIT_DONE,            
    AUDIO_STATE_BT_SUBMIT_INITIAL_READS,       
    AUDIO_STATE_BT_WAIT_FOR_BUFFER_COMPLETE,    
    AUDIO_STATE_BT_BUFFER_COMPLETE,   
} AUDIO_STATES;
// *****************************************************************************
/* Codec

  Summary:
    Application Codec data

  Description:
*/
typedef struct
{
    DRV_HANDLE handle;
    DRV_CODEC_BUFFER_HANDLE writeBufHandle;
    DRV_CODEC_BUFFER_EVENT_HANDLER bufferHandler;
    uintptr_t context;
    uint8_t *txbufferObject;
    size_t bufferSize;

} AUDIO_CODEC;

// *****************************************************************************
/* Codec

  Summary:
    Application Codec data

  Description:
*/
typedef struct
{
    DRV_HANDLE handle;
    DRV_BT_BUFFER_HANDLE readBufHandle;
    DRV_BT_BUFFER_EVENT_HANDLER bufferHandler;
    DRV_BT_EVENT_HANDLER eventHandler;
    uintptr_t context;
    uint8_t *rxbufferObject;
    size_t bufferSize;

} AUDIO_BT;


// *****************************************************************************
/* Application Data

  Summary:
    Holds application data

  Description:
    This structure holds the application's data.

  Remarks:
    Application strings and buffers are be defined outside this structure.
 */

typedef struct
{
    /* The application's current state */
    AUDIO_STATES state;

    AUDIO_CODEC codec;

    AUDIO_BT bt;
    
    uint16_t readIndex;   //Next buffer to read  (read_HEAD)
    uint16_t readyIndex;  //Next Buffer to RX Complete (readComplete_HEAD)
    uint16_t writeIndex;  //Next buffer to write (write_TAIL)
    uint16_t txCompleteIndex; //Next TX to complete (writeComplete_TAIL 
    
} AUDIO_DATA;



void audioTasks(void);

// *****************************************************************************
// *****************************************************************************
// Section: Application Callback Routines
// *****************************************************************************
// *****************************************************************************
/* These routines are called by drivers when certain events occur.
*/


// *****************************************************************************
// *****************************************************************************
// Section: Application Initialization and State Machine Functions
// *****************************************************************************
// *****************************************************************************

/*******************************************************************************
  Function:
    void audioInitialize(void);

  Summary:
     MPLAB Harmony application initialization routine.

  Description:
    This function initializes the Harmony application.  It places the
    application in its initial state and prepares it to run so that its
    APP_Tasks function can be called.

  Precondition:
    All other system initialization routines should be called before calling
    this routine (in "SYS_Initialize").

  Parameters:
    None.

  Returns:
    None.

  Example:
    <code>
    audioInitialize();
    </code>

  Remarks:
    This routine must be called from the SYS_Initialize function.
*/

void audioInitialize(void); 

/*******************************************************************************
  Function:
    void audioTasks ( void )

  Summary:
    MPLAB Harmony Demo application tasks function

  Description:
    This routine is the Harmony Demo application's tasks function.  It
    defines the application's state machine and core logic.

  Precondition:
    The system and application initialization ("SYS_Initialize") should be
    called before calling this.

  Parameters:
    None.

  Returns:
    None.

  Example:
    <code>
    APP_SDCARD_AUDIO_Tasks();
    </code>

  Remarks:
    This routine must be called from SYS_Tasks() routine.
 */

void audioTasks( void );

void AUDIO_CODEC_TxBufferEventHandler(DRV_CODEC_BUFFER_EVENT event,
    DRV_CODEC_BUFFER_HANDLE handle, uintptr_t context );

void AUDIO_BT_RxBufferEventHandler(DRV_BT_BUFFER_EVENT event,
    DRV_BT_BUFFER_HANDLE handle, uintptr_t context );

void SetCodecSamplingRate(uint32_t sampleFreq);

void audioStart();

#endif /* _APP_H */

//DOM-IGNORE-BEGIN
#ifdef __cplusplus
}
#endif
//DOM-IGNORE-END

/*******************************************************************************
 End of File
 */


