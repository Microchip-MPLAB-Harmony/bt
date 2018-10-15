/*******************************************************************************
  BM64 Bluetooth Static Driver implementation

  Company:
    Microchip Technology Inc.

  File Name:
    drv_bm64_command_send.h

  Summary:
   BM64 Bluetooth Static Driver header file for sending commands.

  Description:
    This file is the header file for the internal functions of the BM64
    driver related to sending commands to the BM64 module.
 
*******************************************************************************/

/******************************************************************************
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
********************************************************************/
#ifndef DRV_BM64_COMMAND_SEND_H
#define DRV_BM64_COMMAND_SEND_H

//#include <stdbool.h>
//#include <stdint.h>

enum
{
  DRV_BM64_MAKE_CALL_CMD = 0x00,
  DRV_BM64_MAKE_EXTENSION_CALL_CMD = 0x01,
  DRV_BM64_MMI_CMD = 0x02,
  DRV_BM64_REPORT_MASK_CMD = 0x03,
  DRV_BM64_MUSIC_CONTROL_CMD = 0x04,
  DRV_BM64_CHANGE_DEVICE_NAME_CMD = 0x05,
  DRV_BM64_CHANGE_PIN_CODE_CMD = 0x06,
  DRV_BM64_SET_PAIRING_TIMER_CMD = 0x07,
  DRV_BM64_GET_UART_VERSION_CMD = 0x08,
  DRV_BM64_GET_PB_BY_AT_CMD = 0x09,
  DRV_BM64_VENDOR_AT_CMD = 0x0A,
  DRV_BM64_AVRCP_SPEC_CMD = 0x0B,
  DRV_BM64_AVRCP_GROUP_NAVIAGE_CMD = 0x0C,
  DRV_BM64_READ_LINK_STATUS_CMD = 0x0D,
  DRV_BM64_READ_PAIRING_RECORD_CMD = 0x0E,
  DRV_BM64_READ_LOCAL_BD_ADDR_CMD = 0x0F,
  DRV_BM64_READ_LOCAL_DEVICE_NAME_CMD = 0x10,
  DRV_BM64_SET_ACCESS_PB_METHOD_CMD = 0x11,            //Item 18
  DRV_BM64_SEND_SPP_DATA_CMD = 0x12,
  DRV_BM64_BTM_UTILITY_FUNCTION_CMD = 0x13,
  DRV_BM64_MCU_SEND_EVENT_ACK_CMD = 0x14,
  DRV_BM64_ADDITIONAL_PROFILE_LINK_SETUP_CMD = 0x15,
  DRV_BM64_READ_LINKED_DEVICE_INFOR_CMD = 0x16,
  DRV_BM64_PROFILE_LINK_BACK_CMD = 0x17,
  DRV_BM64_DISCONNECT_CMD = 0x18,
  DRV_BM64_MCU_STATUS_INDICATION_CMD = 0x19,
  DRV_BM64_USER_CONFIRM_SPP_REQ_REPLY_CMD = 0x1A,
  DRV_BM64_SET_HF_GAIN_LEVEL_CMD = 0x1B,
  DRV_BM64_EQ_MODE_SETTING_CMD = 0x1C,
  DRV_BM64_DSP_NR_CTRL_CMD = 0x1D,
  DRV_BM64_GPIO_CTRL_CMD = 0x1E,
  DRV_BM64_BT_MCU_UART_RX_BUFF_SIZE_CMD = 0x1F,
  DRV_BM64_VOICE_PROMPT_CMD = 0x20,
  DRV_BM64_MAP_REQUEST_CMD = 0x21,
  DRV_BM64_SECURITY_BONDING_REQ_CMD = 0x22,
  DRV_BM64_SET_OVERALL_GAIN_CMD = 0x23,
  DRV_BM64_LE_SIGNALING_CMD = 0x29,
  DRV_BM64_DISCOVERABLE_CMD = 0x33,
  DRV_BM64_READ_LINK_MODE_CMD = 0x34
};                    // uart_cmdset_type_en;

enum
{
  DRV_BM64_MMI_ADD_REMOVE_SCO_LINK = 0x01,
  DRV_BM64_MMI_FORCE_END_CALL = 0x02,
  DRV_BM64_MMI_ACCEPT_CALL = 0x04,
  DRV_BM64_MMI_REJECT_CALL = 0x05,
  DRV_BM64_MMI_ENDCALL_OR_TRANSFER_TO_HEADSET = 0x06,
  DRV_BM64_MMI_MIC_MUTE_TOGGLE = 0x07,
  DRV_BM64_MMI_MUTE_MIC = 0x08,
  DRV_BM64_MMI_UNMUTE_MIC  = 0x09,
  DRV_BM64_MMI_VOICE_DIAL  = 0x0A,
  DRV_BM64_MMI_CANCLE_VOICE_DAIL = 0x0B,
  DRV_BM64_MMI_LAST_NUMBER_REDIAL = 0x0C,
  DRV_BM64_MMI_ACTIVE_CALL_HOLD_ACCEPT_HELD_CALL = 0x0D,
  DRV_BM64_MMI_VOICE_TRANSFER = 0x0E,
  DRV_BM64_MMI_QUERY_CALL_LIST = 0x0F,
  DRV_BM64_MMI_THREE_WAY_CALL = 0x10,
  DRV_BM64_MMI_RELEASE_CALL = 0x11,
  DRV_BM64_MMI_ACCEPT_WAITING_HOLD_CALL_RLS_ACTIVE_CALL = 0x12,
  DRV_BM64_MMI_INITIAL_HF_CONNECTION = 0x16,
  DRV_BM64_MMI_DISCONNECT_HF_LINK = 0x17,
  DRV_BM64_MMI_INC_MIC_GAIN = 0x24,
  DRV_BM64_MMI_DEC_MIC_GAIN = 0x25,
  DRV_BM64_MMI_SWITCH_PRIMARY_SECONDARY_HF_DEVICE = 0x26,
  DRV_BM64_MMI_INC_SPK_GAIN = 0x30,
  DRV_BM64_MMI_DEC_SPK_GAIN = 0x31,
  DRV_BM64_MMI_INITIAL_A2DP_CONNECT_PLAY_PAUSE = 0x32,
  DRV_BM64_MMI_STOP_MEDIA = 0x33,
  DRV_BM64_MMI_NEXT_SONG = 0x34,
  DRV_BM64_MMI_PREVIOUS_SONG = 0x35,
  DRV_BM64_MMI_DISCONNECT_A2DP = 0x3B,
  DRV_BM64_MMI_STANDBY_ENTERING_PAIRING = 0x50,
  DRV_BM64_MMI_POWERON_BUTTON_PRESS = 0x51,
  DRV_BM64_MMI_POWERON_BUTTON_RELEASE = 0x52,
  DRV_BM64_MMI_POWEROFF_BUTTON_PRESS = 0x53,
  DRV_BM64_MMI_POWEROFF_BUTTON_RELEASE = 0x54,
  DRV_BM64_MMI_RESET_EEPROM_SETTING = 0x56,
  DRV_BM64_MMI_ANY_MODE_ENTERING_PAIRING = 0x5D,
  DRV_BM64_MMI_POWEROFF_BT = 0x5E,
  DRV_BM64_MMI_BUZZER_TOGGLE = 0x60,
  DRV_BM64_MMI_DISABLE_BUZZER = 0x61,
  DRV_BM64_MMI_ENABLE_BUZZER = 0x62,
  DRV_BM64_MMI_TONE_CHANGE = 0x63,
  DRV_BM64_MMI_RETRIEVE_PHONE_BOOK = 0x64,
  DRV_BM64_MMI_RETRIEVE_MISS_CALL_HISTORY = 0x65,
  DRV_BM64_MMI_RETRIEVE_RECEIVED_CALL_HISTORY = 0x66,
  DRV_BM64_MMI_RETRIEVE_DIALED_CALL_HISTORY = 0x67,
  DRV_BM64_MMI_RETRIEVE_ALL_CALL_HISTORY = 0x68,
  DRV_BM64_MMI_CANCLE_RETRIEVE = 0x69,
  DRV_BM64_MMI_INDICATE_BATTERY_STATUS = 0x6A,
  DRV_BM64_MMI_EXIT_PAIRING_MODE = 0x6B,
  DRV_BM64_MMI_LINK_BACK_DEVICE = 0x6C,
  DRV_BM64_MMI_DISCONNECT_ALL_LINK = 0x6D,
  DRV_BM64_MMI_MASTERSPK_ENTER_CSB_PAGE = 0xE0,          //r3
  DRV_BM64_MMI_SLAVESPK_ENTER_CSB_PAGESCAN = 0xE1,          //r3
  DRV_BM64_MMI_NSPK_ADD_SPEAKER = 0xE2,
  DRV_BM64_MMI_MASTERSPK_TERMINAL_CSB = 0xE5,          //r3
  DRV_BM64_MMI_MASTERSPK_ENTER_AUXMODE = 0xE7,          //r3
  DRV_BM64_MMI_MASTERSPK_EXIT_AUXMODE = 0xE8,          //r3
  DRV_BM64_MMI_POWER_OFF_ALL_SPK = 0xED,
  DRV_BM64_MMI_MASTERSPK_REPAIR_TO_SLAVE = 0xF0
};            //uart_mmi_cmdset_type_en;

//#define SPPTX_PAYLOAD_MAX_SIZE 255

uint8_t DRV_BM64_IsAllowedToSendCommand( void );
void DRV_BM64_SendBytesAsCompleteCommand(uint8_t* command, uint8_t command_length);
void DRV_BM64_MMI_ActionCommand(uint8_t MMI_ActionCode, uint8_t link_index);
void DRV_BM64_MusicControlCommand(uint8_t CtrlCode);
void DRV_BM64_SendAckToEvent(uint8_t ack_event);
void DRV_BM64_SendDiscoverableCommand(uint8_t discoverable);
void DRV_BM64_ReadBTMLinkModeCommand( void );
void DRV_BM64_ReadDeviceAddressCommand(void);
void DRV_BM64_ReadDeviceNameCommand(void);
void DRV_BM64_GetPairRecordCommand(void);
void DRV_BM64_LinkBackToLastDevice(void);
void DRV_BM64_LinkBackToDeviceByBTAddress(uint8_t* address);
void DRV_BM64_LinkBackMultipoint(void);
void DRV_BM64_DisconnectAllProfile(void);
void DRV_BM64_SetOverallGainCommand(uint8_t set_type, uint8_t gain1, uint8_t gain2, uint8_t gain3);
void DRV_BM64_SetOverallGain(uint8_t gain1, uint8_t gain2, uint8_t gain3);
void DRV_BM64_updateA2DPGain(uint8_t gain);
void DRV_BM64_updateHFPGain(uint8_t gain);
void DRV_BM64_updateLineInGain(uint8_t gain);
void DRV_BM64_SetupBTMGPIO( void );
void DRV_BM64_SetRXBufferSize( void );
void DRV_BM64_ProfileLinkBack(uint8_t linked_profile, uint8_t link_index);

void DRV_BM64_CommandSendInit(void);
void DRV_BM64_CommandSendTask( void );
void DRV_BM64_CommandSend1MS_event(void);

void DRV_BM64_UART_TransferFirstByte( void );
void DRV_BM64_UART_TransferNextByte( void );

//-----------------------------------------
enum {
    DRV_BM64_ACK_STS_OK = 0,
    DRV_BM64_ACK_DISALLOWED = 1,
    DRV_BM64_ACK_COMMAND_UNKNOW = 2,
    DRV_BM64_ACK_PARA_ERROR = 3,
    DRV_BM64_ACK_BTM_BUSY = 4,
    DRV_BM64_ACK_OUT_OF_MEMORY = 5,
    DRV_BM64_COMMAND_IS_SENT = 0x0E,
    DRV_BM64_COMMAND_STS_INIT = 0x0F,
};  //Command ACK status

void DRV_BM64_InitAckStatus(void);
void DRV_BM64_UpdateAckStatusWhenReceived(uint8_t command_id, uint8_t ack_status) ;
void DRV_BM64_UpdateAckStatusWhenSent(uint8_t command_id) ;
uint8_t DRV_BM64_GetAckStatusByCommandID(uint8_t command_id);
uint8_t DRV_BM64_GetAckStatus_MMIAction( void );
uint8_t DRV_BM64_GetAckStatusMusicControl( void );
uint8_t DRV_BM64_GetAckStatusDiscoverable( void );
uint8_t DRV_BM64_GetAckStatusBTMLinkMode( void );
uint8_t DRV_BM64_GetAckStatusDeviceAddress( void );
uint8_t DRV_BM64_GetAckStatusDeviceName( void );
uint8_t DRV_BM64_GetAckStatusReadPairRecord( void );
uint8_t DRV_BM64_GetAckStatusLinkBack( void );
uint8_t DRV_BM64_GetAckStatusDisconnecProfile( void );
uint8_t DRV_BM64_GetAckStatusSetOverallGainCommand( void );
uint8_t DRV_BM64_GetAckStatusSendSPPData( void );
uint8_t DRV_BM64_GetAckStatusBTMGPIOCtrl( void );
uint8_t DRV_BM64_GetAckStatusBTMUtinityReq( void );
uint8_t DRV_BM64_GetAckStatusSetRXBufferSize( void );
void DRV_BM64_ChangeDeviceNameCommand(const char *name);
void DRV_BM64_SendSPPData(uint8_t* addr, uint16_t size, uint8_t link_index);
void DRV_BM64_EnterLineInMode(uint8_t disable0_enable1, uint8_t analog0_I2S1);
#endif
