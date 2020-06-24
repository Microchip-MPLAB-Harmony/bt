/*******************************************************************************
  MPLAB Harmony Application Header File

  Company:
    Microchip Technology Inc.

  File Name:
    ble.h

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

#ifndef _BLE_H
#define _BLE_H

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
#define MAX_ADV_PAYLOAD_LEN					31
#define MAX_ADV_REPORT_EVENT_LIST			20
#define EVENT_BUFFER_LENGTH                 512
    
#define MAX_GAP_EVENT_SUBSCRIBERS			3
#define MAX_GATT_SERVER_EVENT_SUBSCRIBERS	3
#define MAX_GATT_CLIENT_EVENT_SUBSCRIBERS	3
#define MAX_COMMON_EVENT_SUBSCRIBERS		3
#define MAX_PAIRING_SUBSCRIBERS				3
#define MAX_GATT_TRANSPARENT_SUBSCRIBERS	3
    
// *****************************************************************************
typedef enum
{
	BLE_MGR_STATE_INIT,
	BLE_MGR_STATE_ADVERTISING,
	BLE_MGR_STATE_SCANNING,
	BLE_MGRSTATE_CONNECTING,
	BLE_MGR_STATE_CONNECTED,
	BLE_MGR_STATE_DISCOVERING_SERVICE,
}ble_mgr_state_t;

typedef enum
{
	BLE_GATT_NONE,
	BLE_GATT_NOTIFY,
	BLE_GATT_INDICATE,
}ble_mgr_handle_value_operation_t;

/** \enum ble_mgr_event_t 
 *  \brief BLE Manager event type. This event types are used to
 *    subscribe the Group of BLE Event callbacks or 
 *    un-subscribe the group of BLE Event callbacks
*/
typedef enum {
	/* BLE GAP Events types */
	BLE_GAP_EVENT_TYPE,
	/* BLE GATT Server Event types */
	BLE_GATT_SERVER_EVENT_TYPE,
	/* BLE GATT Client Event types */
	BLE_GATT_CLIENT_EVENT_TYPE,
	/* Common Event types */
	BLE_COMMON_EVENT_TYPE,
	/* BLE Pairing Event types */
	BLE_PAIRING_EVENT_TYPE,
	/* BLE GATT Transparent Event types */
	BLE_GATT_TP_EVENT_TYPE,
}ble_mgr_event_t;

typedef struct __PACKED
{
	uint8_t *data;
	uint32_t data_len;
}event_msg_t;

typedef struct __PACKED
{
	event_msg_t event_msg;
	uint8_t event_id;
}event_t;
    
/**	\typedef ble_event_callback_t 
 *	\brief BLE Event callback generic type */
typedef ble_status_t (*ble_event_callback_t) (event_msg_t* message);

/**	\struct ble_gap_event_cb_t 
 *	\brief BLE GAP Event callback types
*/
typedef struct _ble_gap_event_cb_t {
	ble_event_callback_t adv_report;
	ble_event_callback_t connected;
	ble_event_callback_t disconnected;
	ble_event_callback_t conn_param_update;
}ble_gap_event_cb_t;

/**	\struct ble_gatt_server_event_cb_t 
 *	\brief All BLE GATT Server callback types
*/
typedef struct _ble_gatt_server_event_cb_t {
	ble_event_callback_t char_value_write;
    ble_event_callback_t prepare_write_request;
    ble_event_callback_t execute_write_request;
}ble_gatt_server_event_cb_t;

/**	\struct ble_gatt_client_event_cb_t 
 *	\brief All BLE GATT Client callback types
*/
typedef struct _ble_gatt_client_event_cb_t {
	ble_event_callback_t service_disc_resp;
	ble_event_callback_t char_disc_resp;
	ble_event_callback_t char_descriptor_disc_resp;
	ble_event_callback_t char_value_received;
    ble_event_callback_t prepare_write_response;
    ble_event_callback_t execute_write_response;
}ble_gatt_client_event_cb_t;

/**	\struct ble_common_event_cb_t 
 *	\brief All BLE Common callback types
*/
typedef struct _ble_common_event_cb_t {
	ble_event_callback_t cmd_complete;
	ble_event_callback_t status_report;
	ble_event_callback_t le_end_test_result;
	ble_event_callback_t config_mode_status;
}ble_common_event_cb_t;

/**	\struct ble_pairing_event_cb_t 
 *	\brief All BLE Pairing callback types
*/
typedef struct _ble_pairing_event_cb_t {
	ble_event_callback_t passkey_entry_req;
	ble_event_callback_t pairing_complete;
	ble_event_callback_t passkey_confirm_req;
}ble_pairing_event_cb_t;

/**	\struct ble_gatt_transparent_event_cb_t 
 *	\brief All BLE GATT Transparent callback types
*/
typedef struct _ble_gatt_transparent_event_cb_t {
	ble_event_callback_t trans_data_received;
}ble_gatt_transparent_event_cb_t;

typedef enum
{ 
    BLE_STATE_OPEN,
    BLE_STATE_SET_BT_EVENT_HANDLER,     
    BLE_STATE_INIT_DONE,               
} BLE_STATES;

typedef struct
{
    DRV_HANDLE handle;
    DRV_BT_EVENT_HANDLER eventHandler;
    uintptr_t context;
} DATA_BT;

/**   \enum ble_mgr_match_param_t 
	\brief BLE Manager advertisement report matching parameter to initiate connection. 
		Upon matching parameter, the connection will be initiated to remote device.
*/
typedef enum
{
	/* None */
	BLE_MATCHING_PARAM_NONE,
	/* Address */
	BLE_MATCHING_PARAM_ADDRESS,
	/* Rssi */
	BLE_MATCHING_PARAM_RSSI,
	/* payload */
	BLE_MATCHING_PARAM_PAYLOAD,
}ble_mgr_match_param_t;

/**	\struct ble_mgr_adv_report_event_t 
 *	\brief Advertisement report event data
*/
typedef struct __PACKED
{
	ble_adv_event_type_t adv_event_type;
	ble_addr_t addr;
	uint8_t data_len;
	uint8_t data[MAX_ADV_PAYLOAD_LEN];
	int8_t rssi;
}ble_mgr_adv_report_event_t;

/**	\struct ble_mgr_adv_report_event_t 
 *	\brief Advertisement report event data
*/
typedef struct __PACKED
{
	ble_mgr_match_param_t matching_param;
	ble_adv_event_type_t adv_event_type;
	ble_addr_t addr;
	uint8_t data_start;
	uint8_t data_len;
	uint8_t data[MAX_ADV_PAYLOAD_LEN];
	int8_t rssi;
}ble_mgr_adv_report_match_param_t;

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
    BLE_STATES state;
    DATA_BT bt;   
    
} BLE_DATA;

// *****************************************************************************
// *****************************************************************************
// Section: Application Initialization and State Machine Functions
// *****************************************************************************
// *****************************************************************************

/*******************************************************************************
  Function:
    void bleInitialize(void);

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
    bleInitialize();
    </code>

  Remarks:
    This routine must be called from the SYS_Initialize function.
*/

void bleInitialize(bool all); 

/*******************************************************************************
  Function:
    void bleTasks ( void )

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
    bleTasks();
    </code>

  Remarks:
    This routine must be called from SYS_Tasks() routine.
 */

void bleTasks( void );

/*! \fn bool ble_mgr_events_register_callback(ble_mgr_event_t event_type, const void *ble_event_handler)
 *  \brief Register callback functions for BLE events.
 *  \param event_type Type of event, like GAP, GATT-Client, GATT-Server... etc.
 *  \param ble_event_handler Function pointer to group of event handler callbacks.
 *  \param scan_resp_data Scan response data.
 *  \param scan_reap_data_len Scan response data length.
 *  \pre Platform and BLE interface has to be initialized using ble_mgr_device_init.
 *  \return ble_status_t Status of setting advertisement data, scan response data, advertisement parameters or start advertisement operation.
 */
bool ble_mgr_events_register_callback(ble_mgr_event_t event_type, const void *ble_event_handler);

///*! \fn void ble_mgr_peripheral_device_match_params(ble_mgr_adv_report_match_param_t *match_param)
// *  \brief Sets a matching parameter to find and connect with the device.
// *  \param match_param Matching parameter, it could be BLE address, RSSI threshold or advertisement payload.
// *  \pre None.
// *  \return None.
// */
void ble_mgr_peripheral_device_match_params(ble_mgr_adv_report_match_param_t *match_param);

/*! \fn bool ble_mgr_check_match_param(ble_mgr_adv_report_event_t *adv_report)
 *  \brief Check against the matching parameter set by the user, it could be BLE address, RSSI threshold or advertisement payload.
 *  \param adv_report Advertisement report info got from scanning operation.
 *  \pre Peripheral matching parameter has to be set using ble_mgr_peripheral_device_match_params.
 *  \return bool Status of checking against matching parameter.
 */
bool ble_mgr_check_match_param(ble_mgr_adv_report_event_t *adv_report);

/*! \fn ble_status_t ble_mgr_characteristic_notify_set(ble_handle_t conn_handle, uint16_t desc_handle, bool enabled)
 *  \brief Enable/disable characteristic notification.
 *  \param conn_handle Connection handle.
 *  \param desc_handle Client characteristic config descriptor handle.
 *  \param enabled Enable/disable the notification.
 *  \pre None.
 *  \return bool Status of enable/disable notification.
 */
ble_status_t ble_mgr_characteristic_notify_set(ble_handle_t conn_handle, uint16_t desc_handle, bool enabled);

/*
 * \fn ble_status_t ble_set_connection_params(ble_set_conn_param_t *conn_params)
 * \brief Set connection parameters
 * \param[in] conn_params provides connection parameter for upcoming connections. @ref ble_set_conn_param_t 
 * \return Upon successful completion of the function shall return @ref BLE_SUCCESS, otherwise it shall return @ref ble_status_t
*/
ble_status_t ble_mgr_set_connection_params(ble_set_conn_param_t *conn_params);

#endif /* _APP_H */

//DOM-IGNORE-BEGIN
#ifdef __cplusplus
}
#endif
//DOM-IGNORE-END

/*******************************************************************************
 End of File
 */


