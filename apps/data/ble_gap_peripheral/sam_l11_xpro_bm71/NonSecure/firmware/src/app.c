/*******************************************************************************
  BLE Peripheral Demo Application Source File.

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
#include "tempSensor/tempSensor.h"

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

DRV_HANDLE tmrHandle;

uint8_t temp_service_uuid[BLE_UUID_128B_LEN] = {TEMP_SENSOR_SERVICE_UUID};
ble_uuid_t serviceUUID;
static float celVal; 

/* Event parameters */
uint8_t event_buf[EVENT_BUFFER_LENGTH];
event_t evt_param = {.event_msg.data = event_buf, .event_msg.data_len = 0, .event_id = 0};

/* Advertisement data */
static const uint8_t adv_data[]	= {	0x02,ADV_FLAGS,0x05,          /* Flags */
	0x15, ADV_MANUFATURER_SPECIFIC_DATA, 0xCD, 0x00, 0xFE, 0x14, 0xAD, 0x11, 0xCF, 0x40, 0x06, 0x3F, 0x11, 0xE5, 0xBE, 0x3E, 0x00, 0x02, 0xA5, 0xD5, 0xC5, 0x2C,
};

app_ble_remote_device_info_t remote_device_info;
bool app_notify_timer_expired = false;
bool temp_notify_enabled = false;
bool is_char_desc_found = false;

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

static void send_temp_sensor_data(uint8_t *temp_data);


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

static ble_status_t app_adv_report_cb(event_msg_t* msg)
{
	return BLE_SUCCESS;
}

static ble_status_t app_connected_cb(event_msg_t* msg)
{
	ble_conn_complete_event_t *connected = (ble_conn_complete_event_t *)msg->data;
	
	if(BLE_SUCCESS == connected->status)
	{
		remote_device_info.conn_handle = connected->conn_handle;
		remote_device_info.peer_addr.type = connected->peer_addr.type;
		memcpy(remote_device_info.peer_addr.addr, connected->peer_addr.addr, BLE_ADDR_LEN);
		memcpy(&remote_device_info.conn_param, &connected->conn_param, sizeof(ble_conn_param_t));
        appData.state = APP_STATE_DEVICE_CONNECTED;
	}
	
	return BLE_SUCCESS;
}

static ble_status_t app_disconnected_cb(event_msg_t* msg)
{
	ble_disconnect_complete_event_t *disconnectd = (ble_disconnect_complete_event_t *)msg->data;
	
	DBG_LOG("\nDevice disconnected reason = 0x%02X", disconnectd->reason);
	temp_notify_enabled = false;

	// Start Advertisement   
    appData.state = APP_STATE_IDLE;
    
	return BLE_SUCCESS;
}

static ble_status_t app_conn_param_update_cb(event_msg_t* msg)
{
	return BLE_SUCCESS;
}

static ble_status_t app_char_value_write_cb(event_msg_t* msg)
{
	ble_write_char_value_event_t *write_char_value = (ble_write_char_value_event_t *)msg->data;
	uint8_t temp_uuid[BLE_UUID_128B_LEN] = {TEMP_UUID};
    
    memcpy_inplace_reorder((uint8_t *)&(write_char_value->char_value_handle), 2);
	
	for(uint8_t sensor_index = 0; sensor_index < NUMBER_OF_SENSORS; sensor_index++)
	{
		if(sensor_service.sensor_char_list[sensor_index].cccd_handle == write_char_value->char_value_handle)
		{
			sensor_service.sensor_char_list[sensor_index].cccd_value = (uint16_t)write_char_value->char_value[1] << 8;
			sensor_service.sensor_char_list[sensor_index].cccd_value |= (uint16_t)write_char_value->char_value[0];
		
			if(sensor_service.sensor_char_list[sensor_index].cccd_value & BLE_CCCD_NOTIFICATION_ENABLED)
			{
				DBG_LOG("\nNotification enabled!!!");
				if(!memcmp(sensor_service.sensor_char_list[sensor_index].uuid.uuid.uuid_128b, temp_uuid, BLE_UUID_128B_LEN))
				{
					temp_notify_enabled = true;
                    
                    //Start periodic Timer
                    SYS_TIME_TimerStart(tmrHandle);
               
				}
				
			}
			else
			{
				DBG_LOG("Notification disabled!!!");
				if(!memcmp(sensor_service.sensor_char_list[sensor_index].uuid.uuid.uuid_128b, temp_uuid, BLE_UUID_128B_LEN))
				{
					temp_notify_enabled = false;
				}
			   //If more sensors included in future then need to check the UUID against each of them

				
				if(!temp_notify_enabled)
				{
                    //Stop timer 
                    SYS_TIME_TimerStop(tmrHandle);
				}
			}
		}
	}
	
	return BLE_SUCCESS;
}

static ble_status_t app_service_disc_resp_cb(event_msg_t* msg)
{
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
			memcpy_inplace_reorder(char_disc_resp->attrib_data + (attrib_index * char_disc_resp->length) + SERVICE_DISC_CHAR_RESP_CHAR_UUID_START, BLE_UUID_128B_LEN);
			for(uint8_t sensor_index = 0; sensor_index < NUMBER_OF_SENSORS; sensor_index++)
			{
				if(BLE_UUID_128B == sensor_service.sensor_char_list[sensor_index].uuid.type)
				{
					if(!memcmp(sensor_service.sensor_char_list[sensor_index].uuid.uuid.uuid_128b, 
								char_disc_resp->attrib_data + (attrib_index * char_disc_resp->length) + SERVICE_DISC_CHAR_RESP_CHAR_UUID_START, 
								BLE_UUID_128B_LEN))
					{
						memcpy(&sensor_service.sensor_char_list[sensor_index].char_attr_handle, char_disc_resp->attrib_data + (attrib_index * char_disc_resp->length) + SERVICE_DISC_CHAR_RESP_ATTRIB_HANDLE_START,5);
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
			memcpy_inplace_reorder(char_disc_resp->attrib_data + (attrib_index * char_disc_resp->length) + SERVICE_DISC_CHAR_RESP_CHAR_UUID_START, BLE_UUID_16B_LEN);
			for(uint8_t sensor_index = 0; sensor_index < NUMBER_OF_SENSORS; sensor_index++)
			{
				if(BLE_UUID_16B == sensor_service.sensor_char_list[sensor_index].uuid.type)
				{
					if(!memcmp(sensor_service.sensor_char_list[sensor_index].uuid.uuid.uuid_16b,
								char_disc_resp->attrib_data + (attrib_index * char_disc_resp->length) + SERVICE_DISC_CHAR_RESP_CHAR_UUID_START,
								BLE_UUID_16B_LEN))
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
                        is_char_desc_found = true;
                        DBG_LOG("\nDescriptor found");
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
    appData.tmrExpired = true;
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
     
    DBG_LOG("\n--APP_Initialize---");
    
    LED_On();       // initially on, until connected
    
    /* Initialize internal temp sensor*/
    temp_sensor_init();
    /* Initialize the Application State*/
    appData.state = APP_STATE_INIT;
    
    bleInitialize(true);
  
    /* BLE event callback registration */
	ble_mgr_events_register_callback(BLE_GAP_EVENT_TYPE, &app_gap_event_handle);
	ble_mgr_events_register_callback(BLE_GATT_SERVER_EVENT_TYPE, &app_gatt_server_event_handle);
	ble_mgr_events_register_callback(BLE_GATT_CLIENT_EVENT_TYPE, &app_gatt_client_event_handle);
	ble_mgr_events_register_callback(BLE_COMMON_EVENT_TYPE, &app_common_event_handle);
	ble_mgr_events_register_callback(BLE_PAIRING_EVENT_TYPE, &app_pairing_event_handle);
	ble_mgr_events_register_callback(BLE_GATT_TP_EVENT_TYPE, &app_transparent_event_handle); 
    
}

/******************************************************************************
  Function:
    void APP_Tasks ( void )

  Remarks:
    See prototype in app.h.
 */


void APP_Tasks ( void )
{    
    bleTasks(); 
    
    /* Check the application's current state. */
    switch ( appData.state )
    {       
        /* Application's initial state. */
        case APP_STATE_INIT:
        {
            tmrHandle = SYS_TIME_TimerCreate(0, SYS_TIME_MSToCount(TEMP_DATA_SEND_INTERVAL), App_TimerCallback, (uintptr_t)0, SYS_TIME_PERIODIC);
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
            serviceUUID.type = BLE_UUID_128B;
            memcpy(&(serviceUUID.uuid.uuid_128b), &temp_service_uuid, 16 );
            memcpy_inplace_reorder((uint8_t *)&(serviceUUID.uuid.uuid_128b), 16 );
            DRV_BT_BLE_ServerPrimaryServiceRead(&serviceUUID); 
            appData.state = APP_STATE_START_ADV_WAIT;          
		}
		break;
        case APP_STATE_START_ADV_WAIT:
        {
              if(is_char_desc_found)
            {                          
                appData.state = APP_STATE_START_ADV;
                is_char_desc_found = false;
            }
           
        }
        break;
        case APP_STATE_START_ADV:
        {
            DRV_BT_BLE_StartAdvertising((uint8_t *)& adv_data, sizeof(adv_data));
            appData.state = APP_STATE_DEVICE_CONNECT_WAIT;
        }
        break;
        case APP_STATE_DEVICE_CONNECT_WAIT:
        {
           //Wait till device gets connected 
        }
        break;
        case APP_STATE_DEVICE_CONNECTED:
        {
            if(temp_notify_enabled & appData.tmrExpired)
            {
                get_temp_sensor_data((uint8_t *)&celVal);
                DBG_LOG("\nTemp Value - %d.%02d C", (int)(celVal), (int)((celVal - (int)celVal)*100.0));
                send_temp_sensor_data((uint8_t *)&celVal);
                appData.tmrExpired = false;
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

static void send_temp_sensor_data(uint8_t *temp_data)
{
    memcpy_inplace_reorder(temp_data, sizeof(float));
    
    DRV_BT_BLE_ServerCharacteristicSend(remote_device_info.conn_handle, sensor_service.sensor_char_list[0].char_value_attr_handle, temp_data, sizeof(float));
    
}


/*******************************************************************************
 End of File
 */
