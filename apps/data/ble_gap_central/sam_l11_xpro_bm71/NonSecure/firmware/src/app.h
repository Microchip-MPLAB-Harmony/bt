/*******************************************************************************
  MPLAB Harmony Application Header File

  Company:
    Microchip Technology Inc.

  File Name:
    app.h

  Summary:
    This header file provides prototypes and definitions for the application.

  Description:
    This header file provides function prototypes and data type definitions for
    the application.  Some of these are required by the system (such as the
    "APP_Initialize" and "APP_Tasks" prototypes) and some of them are only used
    internally by the application (such as the "APP_STATES" definition).  Both
    are defined here for convenience.
*******************************************************************************/

#ifndef APP_H
#define APP_H

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

// DOM-IGNORE-BEGIN
#ifdef __cplusplus  // Provide C++ Compatibility

extern "C" {

#endif
// DOM-IGNORE-END

// *****************************************************************************
// *****************************************************************************
// Section: Type Definitions
// *****************************************************************************
// *****************************************************************************
#if defined DEBUG_LOG_DISABLED
	#define DBG_LOG_CONT	ALL_UNUSED
	#define DBG_LOG		    ALL_UNUSED
	#define DBG_LOG_ADV	    ALL_UNUSED
#else
	#define DBG_LOG_CONT	printf
	#define DBG_LOG		    printf("\r\n");\
							printf
	#define DBG_LOG_ADV	    printf("\r\nBLE-ADV: ");\
							printf
#endif
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
	/* Application's state machine's initial state. */
	APP_STATE_INIT=0,
    APP_STATE_WAIT_INIT,
    APP_STATE_IDLE,
    APP_STATE_START_SCAN,
    APP_STATE_SCAN_COMPLETED,
    APP_STATE_DEVICE_CONNECTED
} APP_STATES;      

#define QUERY_DELAY        500        // 1/2 sec

/* UUID for service and characteristics */
#define TEMP_SENSOR_SERVICE_UUID	0x1b, 0xc5, 0xd5, 0xa5, 0x02, 0x00, 0xa6, 0x87, 0xe5, 0x11, 0x36, 0x39, 0xc1, 0xba, 0x5a, 0xf0
#define TEMP_UUID					0x1b, 0xc5, 0xd5, 0xa5, 0x02, 0x00, 0xa6, 0x87, 0xe5, 0x11, 0x36, 0x39, 0xd7, 0xba, 0x5a, 0xf0

/* This application supports temperature sensor function */
#define NUMBER_OF_SENSORS		1

typedef enum
{
	SENSOR_1_TEMPERATURE,
    /* Add more */
}sensor_list_t;

typedef struct __PACKED
{
    uint16_t handle;
	float temp;
    
}temp_sensor_value_t;

typedef struct
{
	uint8_t conn_handle;
	ble_addr_t peer_addr;
	ble_conn_param_t conn_param;
}app_ble_remote_device_info_t;

typedef struct __PACKED
{
	ble_handle_t char_attr_handle;
	uint8_t property;
	ble_handle_t char_value_attr_handle;
	ble_uuid_t	uuid;
	ble_handle_t cccd_handle;
	uint16_t cccd_value;
} sensor_char_t;

typedef struct __PACKED
{
	ble_uuid_t service_uuid;
	uint8_t num_of_sensor_chars;
	sensor_char_t *sensor_char_list;
} sensor_service_t;

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
    APP_STATES state;
    
    /* handle to opened timer */
    DRV_HANDLE tmrHandle;

    bool deviceConnected;
} APP_DATA;

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
    void APP_Initialize ( void )

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
    APP_Initialize();
    </code>

  Remarks:
    This routine must be called from the SYS_Initialize function.
*/

void APP_Initialize ( void );


/*******************************************************************************
  Function:
    void APP_Tasks ( void )

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
    APP_Tasks();
    </code>

  Remarks:
    This routine must be called from SYS_Tasks() routine.
 */

void APP_Tasks( void );


uint8_t* memcpy_inplace_reorder(uint8_t* data, uint16_t len);

// make boards with multiple switches/LEDs compatible with those with just one
#ifdef SWITCH1_STATE_PRESSED
#define SWITCH_Get             SWITCH1_Get
#define SWITCH_STATE_PRESSED   SWITCH1_STATE_PRESSED
#endif

#ifdef LED1_On
#define LED_On                 LED1_On
#define LED_Off                LED1_Off
#define LED_Toggle             LED1_Toggle
#else
#define LED1_On                LED_On
#define LED1_Off               LED_Off
#define LED1_Toggle            LED_Toggle
#endif

#endif /* APP_H */

//DOM-IGNORE-BEGIN
#ifdef __cplusplus
}
#endif
//DOM-IGNORE-END

/*******************************************************************************
 End of File
 */

