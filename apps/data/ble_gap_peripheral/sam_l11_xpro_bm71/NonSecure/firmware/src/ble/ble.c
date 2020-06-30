/*******************************************************************************
  MPLAB Harmony Application Source File

  Company:
    Microchip Technology Inc.

  File Name:
    ble.c

  Summary:
    Source code for BLE operations

  Description:
    This file contains the source code for the MPLAB Harmony application.  It
    implements the logic of the application's BLE state machine and calls
    API routines of other MPLAB Harmony modules in the system, including the
    Bluetooth driver and system services.

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

#include <stdint.h> 
#include <string.h>
#include <ctype.h>      // for tolower

#include "ble.h"

// *****************************************************************************
// *****************************************************************************
// Section: Global Variable Definitions
// *****************************************************************************
// *****************************************************************************
BLE_DATA bleData;
uint8_t bleDataMsg[EVENT_BUFFER_LENGTH];

static ble_mgr_adv_report_event_t ble_mgr_adv_report_list = {0};
static ble_mgr_adv_report_match_param_t ble_mgr_adv_report_match_param = {0};

ble_mgr_state_t ble_mgr_state = BLE_MGR_STATE_INIT;

ble_gap_event_cb_t *ble_mgr_gap_event_cb[MAX_GAP_EVENT_SUBSCRIBERS] = {NULL, };
ble_gatt_server_event_cb_t *ble_mgr_gatt_server_event_cb[MAX_GATT_SERVER_EVENT_SUBSCRIBERS] = {NULL, };
ble_gatt_client_event_cb_t *ble_mgr_gatt_client_event_cb[MAX_GATT_CLIENT_EVENT_SUBSCRIBERS] = {NULL, };
ble_common_event_cb_t *ble_mgr_common_event_cb[MAX_COMMON_EVENT_SUBSCRIBERS] = {NULL, };
ble_pairing_event_cb_t *ble_mgr_pairing_event_cb[MAX_PAIRING_SUBSCRIBERS] = {NULL, };
ble_gatt_transparent_event_cb_t *ble_mgr_gatt_tp_event_cb[MAX_GATT_TRANSPARENT_SUBSCRIBERS] = {NULL, };

static void ble_mgr_init(void);
static void ble_mgr_event_manager(event_t *event_param);
static ble_status_t ble_mgr_adv_report_cb(event_msg_t* msg);
static ble_status_t ble_mgr_connected_cb(event_msg_t* msg);
static ble_status_t ble_mgr_disconnected_cb(event_msg_t* msg);
static ble_status_t ble_mgr_conn_param_update_cb(event_msg_t* msg);
static ble_status_t ble_mgr_char_value_write_cb(event_msg_t* msg);
static ble_status_t ble_mgr_prepare_write_request_cb(event_msg_t* msg);
static ble_status_t ble_mgr_execute_write_request_cb(event_msg_t* msg);
static ble_status_t ble_mgr_service_disc_resp_cb(event_msg_t* msg);
static ble_status_t ble_mgr_char_disc_resp_cb(event_msg_t* msg);
static ble_status_t ble_mgr_char_descriptor_disc_resp_cb(event_msg_t* msg);
static ble_status_t ble_mgr_char_value_received_cb(event_msg_t* msg);
static ble_status_t ble_mgr_prepare_write_repose_cb(event_msg_t* msg);
static ble_status_t ble_mgr_execute_write_repose_cb(event_msg_t* msg);
static ble_status_t ble_mgr_cmd_complete_cb(event_msg_t* msg);
static ble_status_t ble_mgr_status_report_cb(event_msg_t* msg);
static ble_status_t ble_mgr_le_end_test_result_cb(event_msg_t* msg);
static ble_status_t ble_mgr_config_mode_status_cb(event_msg_t* msg);
static ble_status_t ble_mgr_passkey_entry_req_cb(event_msg_t* msg);
static ble_status_t ble_mgr_pairing_complete_cb(event_msg_t* msg);
static ble_status_t ble_mgr_passkey_confirm_req_cb(event_msg_t* msg);
static ble_status_t ble_mgr_trans_data_received_cb(event_msg_t* msg);
static void _BLEEventHandler(DRV_BT_EVENT event, uint32_t param, uintptr_t context);

static const ble_gap_event_cb_t ble_mgr_gap_event_handle = {
	.adv_report = ble_mgr_adv_report_cb,
	.connected = ble_mgr_connected_cb,
	.disconnected = ble_mgr_disconnected_cb,
	.conn_param_update = ble_mgr_conn_param_update_cb,
};

static const ble_gatt_server_event_cb_t ble_mgr_gatt_server_event_handle = {
	.char_value_write = ble_mgr_char_value_write_cb,
    .prepare_write_request = ble_mgr_prepare_write_request_cb,
    .execute_write_request = ble_mgr_execute_write_request_cb,
};

static const ble_gatt_client_event_cb_t ble_mgr_gatt_client_event_handle = {
	.service_disc_resp = ble_mgr_service_disc_resp_cb,
	.char_disc_resp = ble_mgr_char_disc_resp_cb,
	.char_descriptor_disc_resp = ble_mgr_char_descriptor_disc_resp_cb,
	.char_value_received = ble_mgr_char_value_received_cb,
    .prepare_write_response = ble_mgr_prepare_write_repose_cb,
    .execute_write_response = ble_mgr_execute_write_repose_cb,
};

static const ble_common_event_cb_t ble_mgr_common_event_handle = {
	.cmd_complete = ble_mgr_cmd_complete_cb,
	.status_report = ble_mgr_status_report_cb,
	.le_end_test_result = ble_mgr_le_end_test_result_cb,
	.config_mode_status = ble_mgr_config_mode_status_cb,
};

static const ble_pairing_event_cb_t ble_mgr_pairing_event_handle = {
	.passkey_entry_req = ble_mgr_passkey_entry_req_cb,
	.pairing_complete = ble_mgr_pairing_complete_cb,
	.passkey_confirm_req = ble_mgr_passkey_confirm_req_cb,
};

static const ble_gatt_transparent_event_cb_t ble_mgr_transparent_event_handle = {
	.trans_data_received = ble_mgr_trans_data_received_cb,
};


void bleInitialize(bool all)        // parameter not used in non_gui version
{
    bleData.state = BLE_STATE_OPEN;    
    bleData.bt.eventHandler = (DRV_BT_EVENT_HANDLER) _BLEEventHandler; 
    
    ble_mgr_init();
    
    DRV_BT_ApplicationModeSet();
    
} //End bleInitialize()

void bleTasks()
{               
    switch(bleData.state)
    {
        //----------------------------------------------------------------------
        // Open BT module 
        //----------------------------------------------------------------------
        case BLE_STATE_OPEN:
        {
            if (SYS_STATUS_READY == DRV_BT_Status())
            {            
                // open BT module
                bleData.bt.handle = DRV_BT_Open(DRV_IO_INTENT_READ, DRV_BT_PROTOCOL_BLE);

                if(bleData.bt.handle != DRV_HANDLE_INVALID)
                {
                        bleData.state = BLE_STATE_SET_BT_EVENT_HANDLER;
                }
                else
                {
                    /* Got an Invalid Handle.  Wait for BT module to Initialize */;
                }
            }
        }
        break;
        
        //----------------------------------------------------------------------
        //Set the BT RX Buffer Handler
        //----------------------------------------------------------------------
        case BLE_STATE_SET_BT_EVENT_HANDLER:
        {          
            DRV_BT_EventHandlerSet(bleData.bt.handle,
                                          bleData.bt.eventHandler,
                                          (uintptr_t)0);                                  

            bleData.state = BLE_STATE_INIT_DONE;            
        }
        break;               
        
        // Initialized 
        case BLE_STATE_INIT_DONE:
        {
            // waits in this state
            break;
        }        

        default:
        {
        }
        break;

    } //End switch(bleData.state)

}

static void _BLEEventHandler(DRV_BT_EVENT event, uint32_t param, uintptr_t context)
{
    event_t bleEvent;
    switch(event)
    {
        case DRV_BT_EVENT_BLESPP_MSG_RECEIVED:
        {                                               
            uint8_t buf[EVENT_BUFFER_LENGTH];
           
            if (DRV_BT_ReadDataFromBLE(bleData.bt.handle, (uint8_t*)buf, param ))
            {
                memcpy(bleDataMsg, buf, param);
                
                bleEvent.event_id = bleDataMsg[0];
           
                bleEvent.event_msg.data = &bleDataMsg[1];
                bleEvent.event_msg.data_len = param;  
                
                ble_mgr_event_manager(&bleEvent);
                          
            }
        }
        break;
        
        case DRV_BT_EVENT_BLE_STATUS_CHANGED:
        {           
            switch(param)
            {             
                case DRV_BT_BLE_STATUS_STANDBY:
                case DRV_BT_BLE_STATUS_SCANNING:
                case DRV_BT_BLE_STATUS_ADVERTISING:
                    LED_On();
                    break;
                case DRV_BT_BLE_STATUS_CONNECTED:                        
                    LED_Off();
                    break;                    
            }
        }
        break;
        
        default:
            break;
    }
}


/*! \fn bool ble_mgr_events_register_callback(ble_mgr_event_t event_type, const void *ble_event_handler)
 *  \brief Register callback functions for BLE events.
 *  \param event_type Type of event, like GAP, GATT-Client, GATT-Server... etc.
 *  \param ble_event_handler Function pointer to group of event handler callbacks.
 *  \param scan_resp_data Scan response data.
 *  \param scan_reap_data_len Scan response data length.
 *  \pre Platform and BLE interface has to be initialized using ble_mgr_device_init.
 *  \return ble_status_t Status of setting advertisement data, scan response data, advertisement parameters or start advertisement operation.
 */
bool ble_mgr_events_register_callback(ble_mgr_event_t event_type, const void *ble_event_handler)
{
	uint8_t index;
	bool status = false;
	
	if(ble_event_handler != NULL)
	{
		switch(event_type)
		{
			case BLE_GAP_EVENT_TYPE:
			{
				ble_gap_event_cb_t *ble_gap_event_cb = (ble_gap_event_cb_t *)ble_event_handler;
				
				for(index = 0; index < MAX_GAP_EVENT_SUBSCRIBERS; index++)
				{
					if(NULL == ble_mgr_gap_event_cb[index])
					{
						ble_mgr_gap_event_cb[index] = ble_gap_event_cb;
						status = true;
						break;
					}
				}
				break;
			}
			case BLE_GATT_SERVER_EVENT_TYPE:
			{
				ble_gatt_server_event_cb_t *ble_gatt_server_event_cb = (ble_gatt_server_event_cb_t *)ble_event_handler;
				
				for(index = 0; index < MAX_GATT_SERVER_EVENT_SUBSCRIBERS; index++)
				{
					if(NULL == ble_mgr_gatt_server_event_cb[index])
					{
						ble_mgr_gatt_server_event_cb[index] = ble_gatt_server_event_cb;
						status = true;
						break;
					}
				}
				break;
			}
			case BLE_GATT_CLIENT_EVENT_TYPE:
			{
				ble_gatt_client_event_cb_t *ble_gatt_client_event_cb = (ble_gatt_client_event_cb_t *)ble_event_handler;
				
				for(index = 0; index < MAX_GATT_CLIENT_EVENT_SUBSCRIBERS; index++)
				{
					if(NULL == ble_mgr_gatt_client_event_cb[index])
					{
						ble_mgr_gatt_client_event_cb[index] = ble_gatt_client_event_cb;
						status = true;
						break;
					}
				}
				break;
			}
			case BLE_COMMON_EVENT_TYPE:
			{
				ble_common_event_cb_t *ble_common_event_cb = (ble_common_event_cb_t *)ble_event_handler;
				
				for(index = 0; index < MAX_COMMON_EVENT_SUBSCRIBERS; index++)
				{
					if(NULL == ble_mgr_common_event_cb[index])
					{
						ble_mgr_common_event_cb[index] = ble_common_event_cb;
						status = true;
						break;
					}
				}
				break;
			}
			case BLE_PAIRING_EVENT_TYPE:
			{
				ble_pairing_event_cb_t *ble_pairing_event_cb = (ble_pairing_event_cb_t *)ble_event_handler;
				
				for(index = 0; index < MAX_PAIRING_SUBSCRIBERS; index++)
				{
					if(NULL == ble_mgr_pairing_event_cb[index])
					{
						ble_mgr_pairing_event_cb[index] = ble_pairing_event_cb;
						status = true;
						break;
					}
				}
				break;
			}
			case BLE_GATT_TP_EVENT_TYPE:
			{
				ble_gatt_transparent_event_cb_t *ble_transparent_event_cb = (ble_gatt_transparent_event_cb_t *)ble_event_handler;
				
				for(index = 0; index < MAX_GATT_TRANSPARENT_SUBSCRIBERS; index++)
				{
					if(NULL == ble_mgr_gatt_tp_event_cb[index])
					{
						ble_mgr_gatt_tp_event_cb[index] = ble_transparent_event_cb;
						status = true;
						break;
					}
				}
				break;
			}
			default:
				break;
		}
	}
	
	return status;
}

static void ble_mgr_event_manager(event_t *event_param)
{
	switch(event_param->event_id)
	{
		case BM_ADVERTISING_REPORT:
		case BM_LE_CONNECT_COMPLETE:
		case BM_DISCONNECT_COMPLETE:
		case BM_CONNECTION_PARAMTER_UPDATE:
		{
			/* GAP events */
			uint8_t index;
			
			event_param->event_id -= BM_ADVERTISING_REPORT;
			
			for(index = 0; index < MAX_GAP_EVENT_SUBSCRIBERS; index++)
			{
				if(ble_mgr_gap_event_cb[index] != NULL)
				{
					const ble_event_callback_t *event_cb_fn = (ble_event_callback_t *)ble_mgr_gap_event_cb[index];
					
					if(event_cb_fn[event_param->event_id] != NULL)
					{
						event_cb_fn[event_param->event_id](&event_param->event_msg);
					}
				}
			}
			break;
		}
		case BM_SERVER_CHARACTERISTIC_VALUE_READ:
		case BM_SERVER_BLOB_READ_REQUEST:
		{
			break;
		}
		case BM_SERVER_CHARACTERICTIC_VALUE_WRITE:
        case BM_SERVER_PREPARE_WRITE_REQUEST:
        case BM_SERVER_EXECUTE_WRITE_REQUEST:
		{
			/* GATT Server events */
			uint8_t index;
			
			event_param->event_id -= BM_SERVER_CHARACTERICTIC_VALUE_WRITE;
			
			for(index = 0; index < MAX_GATT_SERVER_EVENT_SUBSCRIBERS; index++)
			{
				if(ble_mgr_gatt_server_event_cb[index] != NULL)
				{
					const ble_event_callback_t *event_cb_fn = (ble_event_callback_t *)ble_mgr_gatt_server_event_cb[index];
					
					if(event_cb_fn[event_param->event_id] != NULL)
					{
						event_cb_fn[event_param->event_id](&event_param->event_msg);
					}
				}
			}
			break;
		}
		case BM_CLIENT_DISCOVER_ALL_SERVICES_RESULT:
		case BM_CLIENT_DISCOVER_CHARACTERISTICS_RESULT:
		case BM_CLIENT_DISCOVER_CHARACTERISTICS_DESCRIPTORS_RESULT:
		case BM_CLIENT_CHARACTERISTIC_VALUE_RECEIVED:
        case BM_CLIENT_PREPARE_WRITE_RESPONSE:
        case BM_CLIENT_EXECUTE_WRITE_RESPONSE:
		{
			/* GATT Client events */
			uint8_t index;
			
			event_param->event_id -= BM_CLIENT_DISCOVER_ALL_SERVICES_RESULT;
			
			for(index = 0; index < MAX_GATT_CLIENT_EVENT_SUBSCRIBERS; index++)
			{
				if(ble_mgr_gatt_client_event_cb[index] != NULL)
				{
					const ble_event_callback_t *event_cb_fn = (ble_event_callback_t *)ble_mgr_gatt_client_event_cb[index];
					
					if(event_cb_fn[event_param->event_id] != NULL)
					{
						event_cb_fn[event_param->event_id](&event_param->event_msg);
					}
				}
			}
			break;
		}
		case BM_COMMAND_COMPLETE:
		case BM_STATUS_REPORT:
		case BM_LE_END_TEST_RESULT:
		case BM_CONFIGURE_MODE_STATUS:
		{
			/* Common events */
			uint8_t index;
			
			event_param->event_id -= BM_COMMAND_COMPLETE;
			
			for(index = 0; index < MAX_COMMON_EVENT_SUBSCRIBERS; index++)
			{
				if(ble_mgr_common_event_cb[index] != NULL)
				{
					const ble_event_callback_t *event_cb_fn = (ble_event_callback_t *)ble_mgr_common_event_cb[index];
					
					if(event_cb_fn[event_param->event_id] != NULL)
					{
						event_cb_fn[event_param->event_id](&event_param->event_msg);
					}
				}
			}
			break;
		}
		case BM_PASSKEY_REQUEST:
		case BM_PAIR_COMPLETE:
		case BM_PASSKEY_YESNO_REQUEST:
		{
			/* Pairing events */
			uint8_t index;
			
			event_param->event_id -= BM_PASSKEY_REQUEST;
			
			for(index = 0; index < MAX_PAIRING_SUBSCRIBERS; index++)
			{
				if(ble_mgr_pairing_event_cb[index] != NULL)
				{
					const ble_event_callback_t *event_cb_fn = (ble_event_callback_t *)ble_mgr_pairing_event_cb[index];
					
					if(event_cb_fn[event_param->event_id] != NULL)
					{
						event_cb_fn[event_param->event_id](&event_param->event_msg);
					}
				}
			}
			break;
		}
		case BM_TRANSPARENT_DATA_RECEIVED:
		{
			/* GATT Transparent events */
			uint8_t index;
			
			event_param->event_id -= BM_TRANSPARENT_DATA_RECEIVED;
			
			for(index = 0; index < MAX_GATT_TRANSPARENT_SUBSCRIBERS; index++)
			{
				if(ble_mgr_gatt_tp_event_cb[index] != NULL)
				{
					const ble_event_callback_t *event_cb_fn = (ble_event_callback_t *)ble_mgr_gatt_tp_event_cb[index];
					
					if(event_cb_fn[event_param->event_id] != NULL)
					{
						event_cb_fn[event_param->event_id](&event_param->event_msg);
					}
				}
			}
			break;
		}
		case BM_EVENT_NONE:
		case BM_ERROR:
		{
			/* Just to avoid compiler error */
			break;
		}
	}
}

/*! \fn void ble_mgr_init(void)
 *  \brief Registers BLE manager event callbacks.
 *  \param None.
 *  \pre BLE manager event callbacks have to be defined in initialized.
 *  \return None.
 */
static void ble_mgr_init(void)
{
	/* BLE event callback registration */
	ble_mgr_events_register_callback(BLE_GAP_EVENT_TYPE, &ble_mgr_gap_event_handle);
	ble_mgr_events_register_callback(BLE_GATT_SERVER_EVENT_TYPE, &ble_mgr_gatt_server_event_handle);
	ble_mgr_events_register_callback(BLE_GATT_CLIENT_EVENT_TYPE, &ble_mgr_gatt_client_event_handle);
	ble_mgr_events_register_callback(BLE_COMMON_EVENT_TYPE, &ble_mgr_common_event_handle);
	ble_mgr_events_register_callback(BLE_PAIRING_EVENT_TYPE, &ble_mgr_pairing_event_handle);
	ble_mgr_events_register_callback(BLE_GATT_TP_EVENT_TYPE, &ble_mgr_transparent_event_handle);
}

static ble_status_t ble_mgr_adv_report_cb(event_msg_t* msg)
{
	ble_adv_report_event_t *adv_report = (ble_adv_report_event_t *)msg->data;
	bool match = false;
	
	memcpy(&ble_mgr_adv_report_list.addr, &adv_report->addr, sizeof(ble_addr_t));
	ble_mgr_adv_report_list.adv_event_type = adv_report->adv_event_type;
	ble_mgr_adv_report_list.rssi = adv_report->rssi;
	ble_mgr_adv_report_list.data_len = adv_report->data_len;
	memcpy(ble_mgr_adv_report_list.data, adv_report->data, adv_report->data_len);
		
	if(ble_mgr_adv_report_match_param.matching_param != BLE_MATCHING_PARAM_NONE)
	{
		if((adv_report->adv_event_type == ADV_IND) || (adv_report->adv_event_type == ADV_DIRECT_IND))
		{
			match = ble_mgr_check_match_param(&ble_mgr_adv_report_list);
			
			if(match)
			{
				ble_mgr_state = BLE_MGRSTATE_CONNECTING;
				DRV_BT_BLE_CreateConnection(BLE_CONN_WHITELIST_FILTER_DISABLED, ble_mgr_adv_report_list.addr.type, ble_mgr_adv_report_list.addr.addr);
				return match;
			}	
		}
	}
		
	return BLE_SUCCESS;
}

static ble_status_t ble_mgr_connected_cb(event_msg_t* msg)
{
	ble_conn_complete_event_t *connected = (ble_conn_complete_event_t *) msg->data;
	uint8_t index;
    
	if(BLE_SUCCESS == connected->status)
	{
		ble_mgr_state = BLE_MGR_STATE_CONNECTED;
	}
	else
	{
		ble_mgr_state = BLE_MGR_STATE_INIT;
	}
	
	printf("\nDevice connected\r\n");
	printf("\nStatus = 0x%02X\r\n", connected->status);
	printf("\nConn_handle = 0x%02X\r\n", connected->conn_handle);
	printf("\nConn_interval = 0x%04X\r\n", connected->conn_param.conn_interval);
	printf("\nConn_latency = 0x%04X\r\n", connected->conn_param.conn_latency);
	printf("\nConn_sv_timeout = 0x%04X\r\n", connected->conn_param.link_sv_to);
	printf("Remote device Address :");
	for(index = 0; index < BLE_ADDR_LEN; index++)
	{
		printf(" 0x%02X", connected->peer_addr.addr[index]);
	}
	
	return BLE_SUCCESS;
}

static ble_status_t ble_mgr_disconnected_cb(event_msg_t* msg)
{
	return BLE_SUCCESS;
}

static ble_status_t ble_mgr_conn_param_update_cb(event_msg_t* msg)
{
	return BLE_SUCCESS;
}

static ble_status_t ble_mgr_char_value_write_cb(event_msg_t* msg)
{
	return BLE_SUCCESS;
}

static ble_status_t ble_mgr_prepare_write_request_cb(event_msg_t* msg)
{
	return BLE_SUCCESS;
}

static ble_status_t ble_mgr_execute_write_request_cb(event_msg_t* msg)
{
	return BLE_SUCCESS;
}

static ble_status_t ble_mgr_service_disc_resp_cb(event_msg_t* msg)
{
	return BLE_SUCCESS;
}

static ble_status_t ble_mgr_char_disc_resp_cb(event_msg_t* msg)
{
	return BLE_SUCCESS;
}

static ble_status_t ble_mgr_char_descriptor_disc_resp_cb(event_msg_t* msg)
{
	return BLE_SUCCESS;
}

static ble_status_t ble_mgr_char_value_received_cb(event_msg_t* msg)
{
	return BLE_SUCCESS;
}

static ble_status_t ble_mgr_prepare_write_repose_cb(event_msg_t* msg)
{
	return BLE_SUCCESS;
}

static ble_status_t ble_mgr_execute_write_repose_cb(event_msg_t* msg)
{
	return BLE_SUCCESS;
}

static ble_status_t ble_mgr_cmd_complete_cb(event_msg_t* msg)
{
	return BLE_SUCCESS;
}

static ble_status_t ble_mgr_status_report_cb(event_msg_t* msg)
{
	return BLE_SUCCESS;
}

static ble_status_t ble_mgr_le_end_test_result_cb(event_msg_t* msg)
{
	return BLE_SUCCESS;
}

static ble_status_t ble_mgr_config_mode_status_cb(event_msg_t* msg)
{
	return BLE_SUCCESS;
}

static ble_status_t ble_mgr_passkey_entry_req_cb(event_msg_t* msg)
{
	return BLE_SUCCESS;
}

static ble_status_t ble_mgr_pairing_complete_cb(event_msg_t* msg)
{
	return BLE_SUCCESS;
}

static ble_status_t ble_mgr_passkey_confirm_req_cb(event_msg_t* msg)
{
	return BLE_SUCCESS;
}

static ble_status_t ble_mgr_trans_data_received_cb(event_msg_t* msg)
{
	return BLE_SUCCESS;
}

/*
 * /fn ble_status_t ble_mgr_set_connection_params(ble_set_conn_param_t *conn_params)
 * /brief Set connection parameters
 * /param[in] conn_params provides connection parameter for upcoming connections. @ref ble_set_conn_param_t
 * /return Upon successful completion of the function shall return @ref BLE_SUCCESS, otherwise it shall return @ref ble_status_t
*/
ble_status_t ble_mgr_set_connection_params(ble_set_conn_param_t *conn_params)
{
	ble_status_t status = BLE_INVALID_COMMAND_PARAMETERS;
	
	if(NULL == conn_params)
	{
		return status;
	}
    
    DRV_BT_BLE_ConnectionParameterSet(conn_params->min_conn_interval, conn_params->max_conn_interval, conn_params->conn_latency, conn_params->link_sv_to);
    
    return BLE_SUCCESS;
}

/*! \fn void ble_mgr_peripheral_device_match_params(ble_mgr_adv_report_match_param_t *match_param)
 *  \brief Sets a matching parameter to find and connect with the device.
 *  \param match_param Matching parameter, it could be BLE address, RSSI threshold or advertisement payload.
 *  \pre None.
 *  \return None.
 */
void ble_mgr_peripheral_device_match_params(ble_mgr_adv_report_match_param_t *match_param)
{
	memset(&ble_mgr_adv_report_match_param, 0, sizeof(ble_mgr_adv_report_match_param_t));
	memcpy(&ble_mgr_adv_report_match_param, match_param, sizeof(ble_mgr_adv_report_match_param_t));
}


/*! \fn bool ble_mgr_check_match_param(ble_mgr_adv_report_event_t *adv_report)
 *  \brief Check against the matching parameter set by the user, it could be BLE address, RSSI threshold or advertisement payload.
 *  \param adv_report Advertisement report info got from scanning operation.
 *  \pre Peripheral matching parameter has to be set using ble_mgr_peripheral_device_match_params.
 *  \return bool Status of checking against matching parameter.
 */
bool ble_mgr_check_match_param(ble_mgr_adv_report_event_t *adv_report)
{
	bool match = false;
	
	switch(ble_mgr_adv_report_match_param.matching_param)
	{
		case BLE_MATCHING_PARAM_ADDRESS:
		{
			if(!memcmp(&ble_mgr_adv_report_match_param.addr, &adv_report->addr, sizeof(ble_addr_t)))
			{
				match = true;
			}
			break;
		}
		case BLE_MATCHING_PARAM_PAYLOAD:
		{
			if(!memcmp(ble_mgr_adv_report_match_param.data, adv_report->data + ble_mgr_adv_report_match_param.data_start, ble_mgr_adv_report_match_param.data_len))
			{
				match = true;
			}
			break;
		}
		case BLE_MATCHING_PARAM_RSSI:
		{
			if(ble_mgr_adv_report_match_param.rssi >= adv_report->rssi)
			{
				match = true;
			}
			break;
		}
		default:
			return match;
	}
	
	return match;
}