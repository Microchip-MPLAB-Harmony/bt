/*******************************************************************************
  BLE Central Demo Application Source File.

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
#include "ble/ble.h"
#include "stdio.h"

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

app_ble_remote_device_info_t remote_device_info = {0};
bool service_disc_pending = false;
bool is_cccd_found = false;
temp_sensor_value_t sensor_data = {0};
uint8_t notify_enabled_sensor_index = 0;

 ble_uuid_t service_uuid = {
		.type = BLE_UUID_128B, 
		.uuid.uuid_128b = {TEMP_SENSOR_SERVICE_UUID}
		};
 
 sensor_char_t sensor_char_list[NUMBER_OF_SENSORS] = {
	{
		.char_attr_handle  = 0,
		.property = 0x00,
		.char_value_attr_handle = 0,
		.uuid.type = BLE_UUID_128B,
		.uuid.uuid.uuid_128b = {TEMP_UUID},
		.cccd_handle = 0,
		.cccd_value = 0,
	}
};
 
 sensor_service_t sensor_service = {
	.service_uuid.type = BLE_UUID_128B,
	.service_uuid.uuid.uuid_128b = {TEMP_SENSOR_SERVICE_UUID},
	.num_of_sensor_chars = NUMBER_OF_SENSORS,
	.sensor_char_list = sensor_char_list,
};

static ble_status_t app_adv_report_cb(event_msg_t* msg);
static ble_status_t app_connected_cb(event_msg_t* msg);
static ble_status_t app_disconnected_cb(event_msg_t* msg);
static ble_status_t app_conn_param_update_cb(event_msg_t* msg);
static ble_status_t app_char_value_write_cb(event_msg_t* msg);
static ble_status_t app_service_disc_resp_cb(event_msg_t* msg);
static ble_status_t app_char_disc_resp_cb(event_msg_t* msg);
static ble_status_t app_char_descriptor_disc_resp_cb(event_msg_t* msg);
static ble_status_t app_char_value_received_cb(event_msg_t* msg);
static ble_status_t app_cmd_complete_cb(event_msg_t* msg);
static ble_status_t app_status_report_cb(event_msg_t* msg);
static ble_status_t app_le_end_test_result_cb(event_msg_t* msg);
static ble_status_t app_config_mode_status_cb(event_msg_t* msg);
static ble_status_t app_passkey_entry_req_cb(event_msg_t* msg);
static ble_status_t app_pairing_complete_cb(event_msg_t* msg);
static ble_status_t app_passkey_confirm_req_cb(event_msg_t* msg);
static ble_status_t app_trans_data_received_cb(event_msg_t* msg);

static const ble_gap_event_cb_t app_gap_event_handle = {
	.adv_report = app_adv_report_cb,
	.connected = app_connected_cb,
	.disconnected = app_disconnected_cb,
	.conn_param_update = app_conn_param_update_cb,
};

static const ble_gatt_server_event_cb_t app_gatt_server_event_handle = {
	.char_value_write = app_char_value_write_cb,
};

static const ble_gatt_client_event_cb_t app_gatt_client_event_handle = {
	.service_disc_resp = app_service_disc_resp_cb,
	.char_disc_resp = app_char_disc_resp_cb,
	.char_descriptor_disc_resp = app_char_descriptor_disc_resp_cb,
	.char_value_received = app_char_value_received_cb,
};

static const ble_common_event_cb_t app_common_event_handle = {
	.cmd_complete = app_cmd_complete_cb,
	.status_report = app_status_report_cb,
	.le_end_test_result = app_le_end_test_result_cb,
	.config_mode_status = app_config_mode_status_cb,
};

static const ble_pairing_event_cb_t app_pairing_event_handle = {
	.passkey_entry_req = app_passkey_entry_req_cb,
	.pairing_complete = app_pairing_complete_cb,
	.passkey_confirm_req = app_passkey_confirm_req_cb,
};

static const ble_gatt_transparent_event_cb_t app_transparent_event_handle = {
	.trans_data_received = app_trans_data_received_cb,
};

// *****************************************************************************
// *****************************************************************************
// Section: Application Callback Functions
// *****************************************************************************
// *****************************************************************************
static ble_status_t app_adv_report_cb(event_msg_t* msg)
{
	ble_adv_report_event_t *adv_report = (ble_adv_report_event_t *)msg->data;
	
	DBG_LOG("\nAdv_Report - Adv type = 0x%02X", adv_report->adv_event_type);
	DBG_LOG("\nAdv_Report - Addr type = 0x%02X", adv_report->addr.type);
	DBG_LOG("\nAdv_Report - Adv payload len = 0x%02X", adv_report->data_len);
	DBG_LOG("\nAdv_Report - RSSI = %d", adv_report->rssi);
	
	return BLE_SUCCESS;
}

static ble_status_t app_connected_cb(event_msg_t* msg)
{
	ble_conn_complete_event_t *connected = (ble_conn_complete_event_t*)msg->data;
	
	if(BLE_SUCCESS == connected->status)
	{
		remote_device_info.conn_handle = connected->conn_handle;
		remote_device_info.peer_addr.type = connected->peer_addr.type;
		memcpy(remote_device_info.peer_addr.addr, connected->peer_addr.addr, BLE_ADDR_LEN);
		memcpy(&remote_device_info.conn_param, &connected->conn_param, sizeof(ble_conn_param_t));
		/* Service discover will be initiated */
		service_disc_pending = true;
        
        appData.state = APP_STATE_DEVICE_CONNECTED;
	}
	
	return BLE_SUCCESS;
}

static ble_status_t app_disconnected_cb(event_msg_t* msg)
{
	ble_disconnect_complete_event_t *disconnectd = (ble_disconnect_complete_event_t *)msg->data;
	
	DBG_LOG("\nDevice disconnected reason = 0x%02X", disconnectd->reason);
	service_disc_pending = false;
	memset(&remote_device_info, 0, sizeof(app_ble_remote_device_info_t));
    // Start the scan once the device gets disconnected
	DRV_BT_BLE_StartScan();
	return BLE_SUCCESS;
}

static ble_status_t app_conn_param_update_cb(event_msg_t* msg)
{
	return BLE_SUCCESS;
}

static ble_status_t app_char_value_write_cb(event_msg_t* msg)
{
	return BLE_SUCCESS;
}

static ble_status_t app_service_disc_resp_cb(event_msg_t* msg)
{
	DBG_LOG("\napp_service_disc_resp_cb");
	return BLE_SUCCESS;
}

static ble_status_t app_char_disc_resp_cb(event_msg_t* msg)
{
	primary_service_char_discovery_resp_t *char_disc_resp = (primary_service_char_discovery_resp_t *)msg->data;
	uint8_t num_of_attrib = (msg->data_len - (SERVICE_DISC_CHAR_RESP_CONN_HANDLE_LEN + SERVICE_DISC_CHAR_RESP_LENGTH_LEN)) / char_disc_resp->length;
	
	DBG_LOG("\n*** app_char_disc_resp_cb ***");
	DBG_LOG("\nConn handle = 0x%02X", char_disc_resp->conn_handle);
	DBG_LOG("\nAttrib length = 0x%02X", char_disc_resp->length);
	DBG_LOG("\nnum_of_attrib = 0x%02X", num_of_attrib);           
            
	if(char_disc_resp->length == 21)
	{
		for(uint8_t attrib_index = 0; attrib_index < num_of_attrib; attrib_index++)
		{
			/* Reverse the UUID in incoming parameter */
			memcpy_inplace_reorder(char_disc_resp->attrib_data + (attrib_index * char_disc_resp->length) + SERVICE_DISC_CHAR_RESP_CHAR_UUID_START, BLE_ATTRIB_UUID_LENGTH_16);
			for(uint8_t sensor_index = 0; sensor_index < NUMBER_OF_SENSORS; sensor_index++)
			{
				if(BLE_UUID_128B == sensor_service.sensor_char_list[sensor_index].uuid.type)
				{
					if(!memcmp(sensor_service.sensor_char_list[sensor_index].uuid.uuid.uuid_128b, 
								char_disc_resp->attrib_data + (attrib_index * char_disc_resp->length) + SERVICE_DISC_CHAR_RESP_CHAR_UUID_START, 
								BLE_ATTRIB_UUID_LENGTH_16))
					{
						memcpy(&sensor_service.sensor_char_list[sensor_index].char_attr_handle, (char_disc_resp->attrib_data + (attrib_index * char_disc_resp->length) + SERVICE_DISC_CHAR_RESP_ATTRIB_HANDLE_START), 5);                       
						break;
					}
				}
			}
		}
	}
	else if(char_disc_resp->length == 7)
	{
		for(uint8_t attrib_index = 0; attrib_index < num_of_attrib; attrib_index++)
		{
			/* Reverse the UUID in incoming parameter */
			memcpy_inplace_reorder(char_disc_resp->attrib_data + (attrib_index * char_disc_resp->length) + SERVICE_DISC_CHAR_RESP_CHAR_UUID_START, BLE_ATTRIB_UUID_LENGTH_2);
			for(uint8_t sensor_index = 0; sensor_index < NUMBER_OF_SENSORS; sensor_index++)
			{
				if(BLE_UUID_16B == sensor_service.sensor_char_list[sensor_index].uuid.type)
				{
					if(!memcmp(sensor_service.sensor_char_list[sensor_index].uuid.uuid.uuid_128b,
								char_disc_resp->attrib_data + (attrib_index * char_disc_resp->length) + SERVICE_DISC_CHAR_RESP_CHAR_UUID_START,
								BLE_ATTRIB_UUID_LENGTH_2))
					{
						memcpy(&sensor_service.sensor_char_list[sensor_index].char_attr_handle, char_disc_resp->attrib_data + (attrib_index * char_disc_resp->length) + SERVICE_DISC_CHAR_RESP_ATTRIB_HANDLE_START, 5);
						break;
					}
				}
			}
		}
	}
	
	return BLE_SUCCESS;
}

static ble_status_t app_char_descriptor_disc_resp_cb(event_msg_t* msg)
{
	char_desc_discovery_resp_t *desc_discovery_resp = (char_desc_discovery_resp_t *)msg->data;
	uint8_t num_of_attrib = 0;
	uint8_t cccd_uuid[BLE_UUID_16B_LEN] = {0x02, 0x29};
	uint16_t cccd_handle = 0x0000;
	
	DBG_LOG("\n*** app_char_descriptor_disc_resp_cb ***");
	
	if(1 == desc_discovery_resp->format)
	{
		num_of_attrib = (msg->data_len - (DESC_DISC_RESP_CONN_HANDLE_LEN + DESC_DISC_RESP_FORMAT_LEN)) / 4;
		
		for(uint8_t attrib_index = 0; attrib_index < num_of_attrib; attrib_index++)
		{
			if(!memcmp(desc_discovery_resp->desc_attrib_data + (attrib_index * 4) + DESC_DISC_RESP_UUID_START, cccd_uuid, BLE_UUID_16B_LEN))
			{
				memcpy(&cccd_handle, desc_discovery_resp->desc_attrib_data + (attrib_index * 4) + DESC_DISC_RESP_ATTRIB_HANDLE_START, 2);
				
				for(uint8_t sensor_index = 0; sensor_index < NUMBER_OF_SENSORS; sensor_index++)
				{
					if(cccd_handle == sensor_service.sensor_char_list[sensor_index].char_value_attr_handle + 1)
					{
						sensor_service.sensor_char_list[sensor_index].cccd_handle = cccd_handle;
						sensor_service.sensor_char_list[sensor_index].cccd_value = 0;
						is_cccd_found = true;
						break;
					}
				}
			}
		}
	}
	
	return BLE_SUCCESS;
}

static ble_status_t app_char_value_received_cb(event_msg_t* msg)
{
	uint8_t *p_data = (uint8_t *)msg->data;
   
	
	memcpy(&sensor_data, &p_data[1], 6);
	memcpy_inplace_reorder((uint8_t *)&sensor_data.handle, sizeof(uint16_t));
    memcpy_inplace_reorder((uint8_t *)&sensor_data.temp, sizeof(float));
	
	if(sensor_data.handle == sensor_service.sensor_char_list[SENSOR_1_TEMPERATURE].char_value_attr_handle)
	{        
        DBG_LOG("\nTemp Value = %i.%02i C", (int)sensor_data.temp, (int)((sensor_data.temp - (int)sensor_data.temp)*100.0));
	}

	return BLE_SUCCESS;
}

static ble_status_t app_cmd_complete_cb(event_msg_t* msg)
{
	return BLE_SUCCESS;
}

static ble_status_t app_status_report_cb(event_msg_t* msg)
{
	return BLE_SUCCESS;
}

static ble_status_t app_le_end_test_result_cb(event_msg_t* msg)
{
	return BLE_SUCCESS;
}

static ble_status_t app_config_mode_status_cb(event_msg_t* msg)
{
	return BLE_SUCCESS;
}

static ble_status_t app_passkey_entry_req_cb(event_msg_t* msg)
{
	return BLE_SUCCESS;
}

static ble_status_t app_pairing_complete_cb(event_msg_t* msg)
{
	return BLE_SUCCESS;
}

static ble_status_t app_passkey_confirm_req_cb(event_msg_t* msg)
{
	DBG_LOG("\napp_passkey_confirm_req_cb");
	return BLE_SUCCESS;
}

static ble_status_t app_trans_data_received_cb(event_msg_t* msg)
{
	return BLE_SUCCESS;
}

void button_cb(void)
{
	/* Add button callback functionality here */
}


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
  //Add neccessary actions for timer callback
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
    ble_mgr_adv_report_match_param_t match_param;
	uint8_t adv_data[] = {0x15, 0xFF, 0xCD, 0x00, 0xFE, 0x14, 0xAD, 0x11, 0xCF, 0x40, 0x06, 0x3F, 0x11, 0xE5, 0xBE, 0x3E, 0x00, 0x02, 0xA5, 0xD5, 0xC5, 0x2C};
    
    ble_set_conn_param_t conn_params = {
		.min_conn_interval = 0x0018, 
		.max_conn_interval = 0x0018, 
		.conn_latency = 0x0010, 
		.link_sv_to = 0x0100
		};
    
    DBG_LOG("\n--APP_Initialize---");
    
    LED_On();       // initially on, until connected
    appData.state = APP_STATE_INIT;
    
    /* Set a expected matching parameters from remote device, to connect with */
	match_param.matching_param = BLE_MATCHING_PARAM_PAYLOAD;
	match_param.data_start = 0x03;
	match_param.data_len = sizeof(adv_data);
	memcpy(match_param.data, adv_data, sizeof(adv_data));
	ble_mgr_peripheral_device_match_params(&match_param);

    bleInitialize(true);
    
    /* BLE event callback registration */
	ble_mgr_events_register_callback(BLE_GAP_EVENT_TYPE, &app_gap_event_handle);
	ble_mgr_events_register_callback(BLE_GATT_SERVER_EVENT_TYPE, &app_gatt_server_event_handle);
	ble_mgr_events_register_callback(BLE_GATT_CLIENT_EVENT_TYPE, &app_gatt_client_event_handle);
	ble_mgr_events_register_callback(BLE_COMMON_EVENT_TYPE, &app_common_event_handle);
	ble_mgr_events_register_callback(BLE_PAIRING_EVENT_TYPE, &app_pairing_event_handle);
	ble_mgr_events_register_callback(BLE_GATT_TP_EVENT_TYPE, &app_transparent_event_handle);
       
    /* Set GAP connection parameters */    
    DRV_BT_BLE_ConnectionParameterSet(conn_params.min_conn_interval, conn_params.max_conn_interval, conn_params.conn_latency, conn_params.link_sv_to);
    
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
            if (DRV_BM71_STATUS_READY == DRV_BT_GetPowerStatus())
            {           
                appData.state = APP_STATE_IDLE;
            }
        }
        break;
        
        // Initialized 
        case APP_STATE_IDLE:
        {
            appData.state = APP_STATE_START_SCAN;
            DRV_BT_BLE_StartScan();
 
		}
		break;
        
        case APP_STATE_DEVICE_CONNECTED:
        {
            /* Connection is established, but service discovery not initiated yet */
            if(remote_device_info.conn_handle && service_disc_pending)
            {
                DRV_BT_BLE_ClientDiscoverCharacteristics(remote_device_info.conn_handle, &service_uuid );
                service_disc_pending = false;
            }
            
            if(is_cccd_found)
            {
                uint8_t sensor_index;
                uint16_t notify_val = BLE_GATT_NOTIFY;

                is_cccd_found = false;

                for(sensor_index = 0; sensor_index < NUMBER_OF_SENSORS;sensor_index++)
                {
                    if(sensor_service.sensor_char_list[sensor_index].cccd_handle && (sensor_service.sensor_char_list[sensor_index].cccd_value == 0))
                    {
                        ble_handle_t cccd_handle = sensor_service.sensor_char_list[sensor_index].cccd_handle;

                        sensor_service.sensor_char_list[sensor_index].cccd_value = 0x0001;
                        DRV_BT_BLE_ClientWriteCharacteristics(remote_device_info.conn_handle, cccd_handle, 0x00,(uint8_t*)&notify_val, 2 );
                        
                        if(sensor_index == SENSOR_1_TEMPERATURE)
                        {
                            DBG_LOG("***Temperature notification enabled***");
                        }
                        break;
                    }
                }
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

uint8_t* memcpy_inplace_reorder(uint8_t* data, uint16_t len)
{
      uint8_t *a, *b;

      if (!data) return data;
      for (a = data, b = (data + len - 1); b > a; ++a, --b)
      {
            *a ^= *b;
            *b ^= *a;
            *a ^= *b;
      }
      return data;
}


/*******************************************************************************
 End of File
 */
