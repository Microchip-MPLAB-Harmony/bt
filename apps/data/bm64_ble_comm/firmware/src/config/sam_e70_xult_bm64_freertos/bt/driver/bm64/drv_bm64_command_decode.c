/*******************************************************************************
  BM64 Bluetooth Static Driver implementation

  Company:
    Microchip Technology Inc.

  File Name:
    drv_bm64_command_decode.c

  Summary:
   BM64 Bluetooth Static Driver source file for decoding events.

  Description:
    This file is the implementation of the internal functions of the BM64
    driver related to decoding events coming back from the BM64 module.
 
*******************************************************************************/

/*****************************************************************************
 Copyright (C) 2017-2018 Microchip Technology Inc. and its subsidiaries.

Subject to your compliance with these terms, you may use Microchip software 
and any derivatives exclusively with Microchip products. It is your 
responsibility to comply with third party license terms applicable to your 
use of third party software (including open source software) that may 
accompany Microchip software.

THIS SOFTWARE IS SUPPLIED BY MICROCHIP "AS IS". NO WARRANTIES, WHETHER 
EXPRESS, IMPLIED OR STATUTORY, APPLY TO THIS SOFTWARE, INCLUDING ANY IMPLIED 
WARRANTIES OF NON-INFRINGEMENT, MERCHANTABILITY, AND FITNESS FOR A PARTICULAR 
PURPOSE.

IN NO EVENT WILL MICROCHIP BE LIABLE FOR ANY INDIRECT, SPECIAL, PUNITIVE, 
INCIDENTAL OR CONSEQUENTIAL LOSS, DAMAGE, COST OR EXPENSE OF ANY KIND 
WHATSOEVER RELATED TO THE SOFTWARE, HOWEVER CAUSED, EVEN IF MICROCHIP HAS 
BEEN ADVISED OF THE POSSIBILITY OR THE DAMAGES ARE FORESEEABLE. TO THE 
FULLEST EXTENT ALLOWED BY LAW, MICROCHIP'S TOTAL LIABILITY ON ALL CLAIMS IN 
ANY WAY RELATED TO THIS SOFTWARE WILL NOT EXCEED THE AMOUNT OF FEES, IF ANY, 
THAT YOU HAVE PAID DIRECTLY TO MICROCHIP FOR THIS SOFTWARE.
*****************************************************************************/

#include <xc.h>
#include "bt/driver/bm64/drv_bm64.h"
#include "bt/driver/bm64/drv_bm64_local.h"
#include "bt/driver/bm64/drv_bm64_command_decode.h"
#include "bt/driver/bm64/drv_bm64_command_send.h"
#include "bt/driver/bm64/drv_bm64_uart.h"

extern DRV_BM64_OBJ    gDrvBm64Obj;

// all #defines, enums and non-static functions and variables prefixed by
// DRV_BM64 to avoid name conflicts

#define DRV_BM64_CMD_SIZE_MAX   200

//command decode state machine
typedef enum {
	DRV_BM64_RX_DECODE_CMD_SYNC_AA,
	DRV_BM64_RX_DECODE_CMD_SYNC_00,
	DRV_BM64_RX_DECODE_CMD_LENGTH,
	DRV_BM64_RX_DECODE_CMD_DATA,
	DRV_BM64_RX_DECODE_CMD_CHECKSUM
} DRV_BM64_RX_DECODE_MODE;

// events coming from the BM64 over the UART RX interface as defined in the document 
// UART_CommandSet_v2_0(DUAL_MODE).xlsx and decoded here
enum            
{
  DRV_BM64_BM64EVENT_ACK = 0x00,
  DRV_BM64_BM64EVENT_DEVICE_STATE = 0x01,
  DRV_BM64_BM64EVENT_CALL_STATUS = 0x02,
  DRV_BM64_BM64EVENT_CALL_ID = 0x03,
  DRV_BM64_BM64EVENT_SMS_INDICATION = 0x04,
  DRV_BM64_BM64EVENT_MISS_CALL_INDICATION = 0x05,
  DRV_BM64_BM64EVENT_PHONE_MAX_BATTERY_LEVEL = 0x06,
  DRV_BM64_BM64EVENT_PHONE_BATTERY_LEVEL = 0x07,
  DRV_BM64_BM64EVENT_PHONE_ROAMING_STATUS = 0x08,
  DRV_BM64_BM64EVENT_PHONE_MAX_SIGNAL_STRENGTH = 0x09,
  DRV_BM64_BM64EVENT_PHONE_SIGNAL_STRENGTH = 0x0A,
  DRV_BM64_BM64EVENT_PHONE_SERVICE_STATUS = 0x0B,
  DRV_BM64_BM64EVENT_BATTERY_LEVEL = 0x0C,
  DRV_BM64_BM64EVENT_CHARGER_STATUS = 0x0D,
  DRV_BM64_BM64EVENT_RESET_TO_DEFAULT = 0x0E,
  DRV_BM64_BM64EVENT_VOLUME_LEVEL = 0x0F,
  DRV_BM64_BM64EVENT_EQ_MODE = 0x10,
  DRV_BM64_BM64EVENT_MISS_CALL_HISTORY = 0x11,
  DRV_BM64_BM64EVENT_RECEIVED_CALL_HISTORY = 0x12,
  DRV_BM64_BM64EVENT_DIALED_CALL_HISTORY = 0x13,
  DRV_BM64_BM64EVENT_COMBINE_CALL_HISTORY = 0x14,
  DRV_BM64_BM64EVENT_PHONE_BOOK = 0x15,
  DRV_BM64_BM64EVENT_ACCESS_FINISH = 0x16,
  DRV_BM64_BM64EVENT_REMOTE_DEVICE_NAME = 0x17,
  DRV_BM64_BM64EVENT_UART_VERSION = 0x18,
  DRV_BM64_BM64EVENT_CALL_LIST_REPORT = 0x19,
  DRV_BM64_BM64EVENT_AVRCP_SPEC_RSP = 0x1A,
  DRV_BM64_BM64EVENT_BTM_UTILITY_REQ = 0x1B,
  DRV_BM64_BM64EVENT_VENDOR_AT_CMD_RSP = 0x1C,
  DRV_BM64_BM64EVENT_UNKNOW_AT_RESULT = 0x1D,
  DRV_BM64_BM64EVENT_REPORT_LINK_STATUS = 0x1E,
  DRV_BM64_BM64EVENT_REPORT_PAIRING_RECORD = 0x1F,
  DRV_BM64_BM64EVENT_REPORT_LOCAL_BD_ADDR = 0x20,
  DRV_BM64_BM64EVENT_REPORT_LOCAL_DEVICE_NAME = 0x21,
  DRV_BM64_BM64EVENT_REPORT_SPP_DATA = 0x22,
  DRV_BM64_BM64EVENT_REPORT_LINK_BACK_STATUS = 0x23,
  DRV_BM64_BM64EVENT_RINGTONE_FINISH_INDICATION = 0x24,
  DRV_BM64_BM64EVENT_USER_CONFIRM_SSP_REQ = 0x25,
  DRV_BM64_BM64EVENT_REPORT_AVRCP_VOL_CTRL = 0x26,
  DRV_BM64_BM64EVENT_REPORT_INPUT_SIGNAL_LEVEL = 0x27,
  DRV_BM64_BM64EVENT_REPORT_IAP_INFO = 0x28,
  DRV_BM64_BM64EVENT_REPORT_AVRCP_ABS_VOL_CTRL = 0x29,
  DRV_BM64_BM64EVENT_REPORT_VOICE_PROMPT_STATUS = 0x2A,
  DRV_BM64_BM64EVENT_REPORT_MAP_DATA = 0x2B,
  DRV_BM64_BM64EVENT_SECURITY_BONDLING_RES = 0x2C,
  DRV_BM64_BM64EVENT_REPORT_TYPE_CODEC = 0x2D,
  DRV_BM64_BM64EVENT_REPORT_TYPE_BTM_SETTING = 0x2E,
  DRV_BM64_BM64EVENT_REPORT_MCU_UPDATE_REPLY = 0x2F,
  DRV_BM64_BM64EVENT_REPORT_BTM_INITIAL_STATUS = 0x30,
  DRV_BM64_BM64EVENT_REPORT_LE_EVENT = 0x32,
  DRV_BM64_BM64EVENT_REPORT_nSPK_STATUS = 0x33,
  DRV_BM64_BM64EVENT_REPORT_nSPK_VENDOR_EVENT = 0x34,
  DRV_BM64_BM64EVENT_REPORT_CUSTOMER_GATT_ATTRIBUTE_DATA = 0x39,
  DRV_BM64_BM64EVENT_REPORT_LINK_MODE = 0x3A
};

//command decode: BM64 status
enum
{
  DRV_BM64_BM64STATUS_OFF = 0x00,
  DRV_BM64_BM64STATUS_DISCOVERABLE = 0x01,
  DRV_BM64_BM64STATUS_ON = 0x02,
  DRV_BM64_BM64STATUS_PAIRING_COMPLETE = 0x03,
  DRV_BM64_BM64STATUS_PAIRING_FAIL = 0x04,
  DRV_BM64_BM64STATUS_HFP_CONNECTED = 0x05,
  DRV_BM64_BM64STATUS_A2DP_CONNECTED = 0x06,
  DRV_BM64_BM64STATUS_HFP_DISCONNECTED = 0x07,
  DRV_BM64_BM64STATUS_A2DP_DISCONNECTED = 0x08,
  DRV_BM64_BM64STATUS_SCO_CONNECTED = 0x09,
  DRV_BM64_BM64STATUS_SCO_DISCONNECTED = 0x0A,
  DRV_BM64_BM64STATUS_ARVCP_CONNECTED = 0x0B,
  DRV_BM64_BM64STATUS_AVRCP_DISCONNECTED = 0x0C,
  DRV_BM64_BM64STATUS_SPP_CONNECTED = 0x0D,
  DRV_BM64_BM64STATUS_SPP_DISCONNECTED = 0x0E,
  DRV_BM64_BM64STATUS_STANDBY = 0x0F,
  DRV_BM64_BM64STATUS_iAP_CONNECTED = 0x10,
  DRV_BM64_BM64STATUS_ACL_DISCONNECTED = 0x11,
  DRV_BM64_BM64STATUS_MAP_CONNECTED = 0x12,
  DRV_BM64_BM64STATUS_MAP_FORBIDDEN = 0x13,
  DRV_BM64_BM64STATUS_MAP_DISCONNECTED = 0x14,
  DRV_BM64_BM64STATUS_ACL_CONNECTED = 0x15
};

//Command decode: BTM_Utility_Req
enum
{
    DRV_BM64_BTM_CONTROL_AMP = 0x00,
    DRV_BM64_BTM_REPORT_LINE_IN_STATUS = 0x01,
    DRV_BM64_BTM_NOTIFY_HANDLE_BTM = 0x02,
    DRV_BM64_BTM_NOTIFY_EEPROM_UPDATE_FINISH = 0x03,
    DRV_BM64_BTM_REPORT_CODEC_STATUS = 0x04,
    DRV_BM64_NSPK_SYNC_POWER_OFF = 0x05,
    DRV_BM64_NSPK_SYNC_VOLUME_CTRL = 0x06,
    DRV_BM64_NSPK_SYNC_INTERNAL_GAIN = 0x07,
    DRV_BM64_NSPK_SYNC_A2DP_ABS_VOL = 0x08,
    DRV_BM64_NSPK_SYNC_CURRENT_CHANNEL_SETTING = 0x09,
    DRV_BM64_NSPK_NOTIFY_POWER_SYNCED = 0x0A,
    DRV_BM64_NSPK_NOTIFY_COMMAND_SUCCESS = 0x0B,
    DRV_BM64_NSPK_NOTIFY_COMMAND_FAIL = 0x0C,
    DRV_BM64_NSPK_NOTIFY_SLAVE_STATUS_CHANGED = 0x0D,
    DRV_BM64_NSPK_RESERVED = 0x0E,
    DRV_BM64_NSPK_NOTIFY_ADD_THIRD_SPEAKER = 0x0F
};

/*======================*/
/*  external variables  */
/*======================*/
static uint8_t  CmdDecodedFlag;
static uint8_t  CmdBuffer[DRV_BM64_CMD_SIZE_MAX];

/*======================================*/
/*  internal variables          */
/*======================================*/
static DRV_BM64_RX_DECODE_MODE  CmdDecodeState;
static uint8_t  CmdDecodeCmdLength;
static uint8_t  CmdDecodeChecksum;			
static uint8_t  CmdDecodeDataCnt;                    //temporary variable in decoding
static unsigned short CmdBufferPt;                    //

/*======================================*/
/*  function implemention       */
/*======================================*/
void DRV_BM64_CommandDecodeInit( void )
{
    CmdDecodedFlag = 0;
    CmdDecodeState = DRV_BM64_RX_DECODE_CMD_SYNC_AA;
    while(DRV_BM64_eusartRxCount)
    {
        DRV_BM64_EUSART_Read();     // flush buffer
    }
    DRV_BM64_InitAckStatus();
	DRV_BM64_SPPBuffClear();
}

void DRV_BM64_CommandDecodeMain( void )
{
	DRV_BM64_CommandHandler();
	if(CmdDecodedFlag)
	{
            DRV_BM64_CommandDecode();
            CmdDecodedFlag = 0;
	}
}

extern volatile uint8_t DRV_BM64_eusartRxCount;

void DRV_BM64_CommandHandler(void) {
    uint8_t current_byte;

    while (DRV_BM64_eusartRxCount) {
        current_byte = DRV_BM64_EUSART_Read();

        switch (CmdDecodeState) {
            case DRV_BM64_RX_DECODE_CMD_SYNC_AA:
                if (current_byte == 0xaa)
                    CmdDecodeState = DRV_BM64_RX_DECODE_CMD_SYNC_00;
                break;

            case DRV_BM64_RX_DECODE_CMD_SYNC_00:
                if (current_byte == 0x00)
                    CmdDecodeState = DRV_BM64_RX_DECODE_CMD_LENGTH;
                else
                    CmdDecodeState = DRV_BM64_RX_DECODE_CMD_SYNC_AA;
                break;

            case DRV_BM64_RX_DECODE_CMD_LENGTH:
                CmdDecodedFlag = 0; //command receive flag clear
                CmdBufferPt = 0; //buffer reset for command parameter
                CmdDecodeCmdLength = current_byte;
                CmdDecodeChecksum = current_byte; //checksum calculation start!
                CmdDecodeDataCnt = current_byte; //save bytes number, use to check where is command end
                CmdDecodeState = DRV_BM64_RX_DECODE_CMD_DATA; //next state
                break;

            case DRV_BM64_RX_DECODE_CMD_DATA:
                CmdDecodeChecksum += current_byte;
                CmdDecodeDataCnt--;
                CmdBuffer[CmdBufferPt++] = current_byte;
                if (CmdDecodeDataCnt == 0) //no data remained?
                    CmdDecodeState = DRV_BM64_RX_DECODE_CMD_CHECKSUM; //yes, next mode: checksum
                break;

            case DRV_BM64_RX_DECODE_CMD_CHECKSUM:
                if ((uint8_t) (CmdDecodeChecksum + current_byte) == 0) {
                    CmdDecodedFlag = 1;
                } else {
                }
                CmdDecodeState = DRV_BM64_RX_DECODE_CMD_SYNC_AA;
                break;
            default:
                break;
        }

        if (CmdDecodedFlag) {
            break;
        }
    }
}

void DRV_BM64_CommandDecode( void )
{
    uint16_t spp_total_length;
    uint16_t spp_payload_length;
    uint16_t para;
    
    switch(CmdBuffer[0])
    {
        case DRV_BM64_BM64EVENT_ACK:
            DRV_BM64_UpdateAckStatusWhenReceived(CmdBuffer[1],CmdBuffer[2]);
            break;
            
        case DRV_BM64_BM64EVENT_DEVICE_STATE:
            DRV_BM64_SendAckToEvent(CmdBuffer[0]);     //send ACK to this event
            switch(CmdBuffer[1])
            {
                //power on, power off, standby etc.
                case DRV_BM64_BM64STATUS_OFF:        //power off state
                    DRV_BM64_EventHandler(DRV_BM64_DEC_EVENT_SYS_POWER_OFF, 0, &CmdBuffer[1]);
                    break;
                case DRV_BM64_BM64STATUS_ON:         //power on state
                    DRV_BM64_EventHandler(DRV_BM64_DEC_EVENT_SYS_POWER_ON, 0, &CmdBuffer[1]);
                    break;
                case DRV_BM64_BM64STATUS_STANDBY:     //standby state
                    DRV_BM64_EventHandler(DRV_BM64_DEC_EVENT_SYS_STANDBY, 0, &CmdBuffer[1]);
                    break;

                //pairing etc
                case DRV_BM64_BM64STATUS_DISCOVERABLE:       //pairing state
                    DRV_BM64_EventHandler(DRV_BM64_DEC_EVENT_SYS_PAIRING_START, 0, &CmdBuffer[1]);
                    break;
                case DRV_BM64_BM64STATUS_PAIRING_COMPLETE:       //pairing ok
                    DRV_BM64_EventHandler(DRV_BM64_DEC_EVENT_SYS_PAIRING_OK, 0, &CmdBuffer[1]);
                    break;
                case DRV_BM64_BM64STATUS_PAIRING_FAIL:           //pairing failed
                    DRV_BM64_EventHandler(DRV_BM64_DEC_EVENT_SYS_PAIRING_FAILED, 0, &CmdBuffer[1]);
                    break;

                    //HFP
                case DRV_BM64_BM64STATUS_HFP_CONNECTED:
                    gDrvBm64Obj.linkIndex = CmdBuffer[2]&0x0f;
                    DRV_BM64_EventHandler(DRV_BM64_DEC_EVENT_HFP_CONNECTED, (uint16_t)CmdBuffer[2], &CmdBuffer[1]);
                    break;
                case DRV_BM64_BM64STATUS_HFP_DISCONNECTED:
                    DRV_BM64_EventHandler(DRV_BM64_DEC_EVENT_HFP_DISCONNECTED, (uint16_t)CmdBuffer[2], &CmdBuffer[1]);
                    break;
                    
                    //spp,iap etc
                case DRV_BM64_BM64STATUS_SPP_CONNECTED:
                    DRV_BM64_EventHandler(DRV_BM64_DEC_EVENT_SPP_CONNECTED, (uint16_t)CmdBuffer[2], &CmdBuffer[1]);
                    break;
                case DRV_BM64_BM64STATUS_iAP_CONNECTED:
                    DRV_BM64_EventHandler(DRV_BM64_DEC_EVENT_IAP_CONNETED, (uint16_t)CmdBuffer[2], &CmdBuffer[1]);
                    break;
                case DRV_BM64_BM64STATUS_SPP_DISCONNECTED:
                    DRV_BM64_EventHandler(DRV_BM64_DEC_EVENT_SPP_IAP_DISCONNECTED, (uint16_t)CmdBuffer[2], &CmdBuffer[1]);
                    break;
                    
                    //a2dp etc
                case DRV_BM64_BM64STATUS_A2DP_CONNECTED:
                    gDrvBm64Obj.linkIndex = CmdBuffer[2]&0x0f;
                    DRV_BM64_EventHandler(DRV_BM64_DEC_EVENT_A2DP_CONNECTED, (uint16_t)CmdBuffer[2], &CmdBuffer[1]);
                    break;
                case DRV_BM64_BM64STATUS_A2DP_DISCONNECTED:
                    DRV_BM64_EventHandler(DRV_BM64_DEC_EVENT_A2DP_DISCONNECTED, (uint16_t)CmdBuffer[2], &CmdBuffer[1]);
                    break;

                    //avrcp etc
                case DRV_BM64_BM64STATUS_ARVCP_CONNECTED:
                    DRV_BM64_EventHandler(DRV_BM64_DEC_EVENT_AVRCP_CONNECTED, (uint16_t)CmdBuffer[2], &CmdBuffer[1]);
                    break;
                case DRV_BM64_BM64STATUS_AVRCP_DISCONNECTED:
                    DRV_BM64_EventHandler(DRV_BM64_DEC_EVENT_AVRCP_DISCONNECTED, (uint16_t)CmdBuffer[2], &CmdBuffer[1]);
                    break;

                    //ACL, SCO etc
                case DRV_BM64_BM64STATUS_ACL_CONNECTED:
                    DRV_BM64_EventHandler(DRV_BM64_DEC_EVENT_ACL_CONNECTED, (uint16_t)CmdBuffer[2], &CmdBuffer[1]);
                    break;
                case DRV_BM64_BM64STATUS_ACL_DISCONNECTED:
                    DRV_BM64_EventHandler(DRV_BM64_DEC_EVENT_ACL_DISCONNECTED, (uint16_t)CmdBuffer[2], &CmdBuffer[1]);
                    break;
                case DRV_BM64_BM64STATUS_SCO_CONNECTED:
                    DRV_BM64_EventHandler(DRV_BM64_DEC_EVENT_SCO_CONNECTED, (uint16_t)CmdBuffer[2], &CmdBuffer[1]);
                    break;
                case DRV_BM64_BM64STATUS_SCO_DISCONNECTED:
                    DRV_BM64_EventHandler(DRV_BM64_DEC_EVENT_SCO_DISCONNECTED, (uint16_t)CmdBuffer[2], &CmdBuffer[1]);
                    break;
                    
                case DRV_BM64_BM64STATUS_MAP_CONNECTED:
                    DRV_BM64_EventHandler(DRV_BM64_DEC_EVENT_MAP_CONNECTED, (uint16_t)CmdBuffer[2], &CmdBuffer[1]);
                    break;
                case DRV_BM64_BM64STATUS_MAP_DISCONNECTED:
                    DRV_BM64_EventHandler(DRV_BM64_DEC_EVENT_MAP_DISCONNECTED, (uint16_t)CmdBuffer[2], &CmdBuffer[1]);
                    break;

                case DRV_BM64_BM64STATUS_MAP_FORBIDDEN:
                    break;                  
            }
            break;

        case DRV_BM64_BM64EVENT_BTM_UTILITY_REQ:
            DRV_BM64_SendAckToEvent(CmdBuffer[0]);     //send ACK to this event
            switch(CmdBuffer[1])
            {
                case DRV_BM64_BTM_CONTROL_AMP:
                    break;
                case DRV_BM64_BTM_REPORT_LINE_IN_STATUS:
                    DRV_BM64_EventHandler(DRV_BM64_DEC_EVENT_LINE_IN_STATUS,(uint16_t)CmdBuffer[2], &CmdBuffer[1]);
                    break;
                case DRV_BM64_BTM_NOTIFY_HANDLE_BTM:
                    break;
                case DRV_BM64_BTM_NOTIFY_EEPROM_UPDATE_FINISH:
                    break;                    
                case DRV_BM64_BTM_REPORT_CODEC_STATUS:
                    DRV_BM64_EventHandler(DRV_BM64_DEC_EVENT_A2DP_STATUS,(uint16_t)CmdBuffer[2], &CmdBuffer[1]);
					break;

                case DRV_BM64_NSPK_SYNC_POWER_OFF:
                    DRV_BM64_EventHandler(DRV_BM64_DEC_EVENT_NSPK_SYNC_POWER_OFF,0, &CmdBuffer[1]);
                    break;
                case DRV_BM64_NSPK_SYNC_VOLUME_CTRL:
                    DRV_BM64_EventHandler(DRV_BM64_DEC_EVENT_NSPK_SYNC_VOL_CTRL,(uint16_t)CmdBuffer[2], &CmdBuffer[1]);
                    break;
                case DRV_BM64_NSPK_SYNC_INTERNAL_GAIN:
                    DRV_BM64_EventHandler(DRV_BM64_DEC_EVENT_NSPK_SYNC_INTERNAL_GAIN,(uint16_t)CmdBuffer[2], &CmdBuffer[1]);
                    break;
                case DRV_BM64_NSPK_SYNC_A2DP_ABS_VOL:
                    DRV_BM64_EventHandler(DRV_BM64_DEC_EVENT_NSPK_SYNC_ABS_VOL,(uint16_t)CmdBuffer[2], &CmdBuffer[1]);
                    break;
                case DRV_BM64_NSPK_SYNC_CURRENT_CHANNEL_SETTING:
                    DRV_BM64_EventHandler(DRV_BM64_DEC_EVENT_NSPK_CHANNEL_SETTING,(uint16_t)CmdBuffer[2], &CmdBuffer[1]);
                    break;
                case DRV_BM64_NSPK_NOTIFY_POWER_SYNCED:
                    break;
                case DRV_BM64_NSPK_NOTIFY_COMMAND_SUCCESS:
                    break;
                case DRV_BM64_NSPK_NOTIFY_COMMAND_FAIL:
                    break;
                case DRV_BM64_NSPK_NOTIFY_SLAVE_STATUS_CHANGED:
                    break;
                case DRV_BM64_NSPK_NOTIFY_ADD_THIRD_SPEAKER:
                    DRV_BM64_EventHandler(DRV_BM64_DEC_EVENT_NSPK_ADD_SPEAKER3,(uint16_t)CmdBuffer[2], &CmdBuffer[1]);
                    break;                    
            }
            break;
        
        case DRV_BM64_BM64EVENT_CALL_STATUS:
            DRV_BM64_SendAckToEvent(CmdBuffer[0]);     //send ACK to this event
            DRV_BM64_EventHandler(DRV_BM64_DEC_EVENT_CALL_STATUS_CHANGED, (uint16_t)CmdBuffer[2], &CmdBuffer[1]);
            break;
            
        case DRV_BM64_BM64EVENT_REPORT_nSPK_STATUS:
            DRV_BM64_SendAckToEvent(CmdBuffer[0]);     //send ACK to this event
            para = CmdBuffer[1];
            para <<= 8;
            para |= CmdBuffer[2];
            DRV_BM64_EventHandler(DRV_BM64_DEC_EVENT_NSPK_STATUS, para, &CmdBuffer[1]);
            break;
            
        case DRV_BM64_BM64EVENT_REPORT_LINK_BACK_STATUS:
            DRV_BM64_SendAckToEvent(CmdBuffer[0]);     //send ACK to this event
            if(CmdBuffer[1] == 0 )       //ACL link-back
            {
                if(CmdBuffer[2] == 0xFF)     //00 FF: ACL failed
                {
                    DRV_BM64_EventHandler(DRV_BM64_DEC_EVENT_LINKBACK_FAILED, 0, &CmdBuffer[1]);
                }
                else
                {
                    DRV_BM64_EventHandler(DRV_BM64_DEC_EVENT_LINKBACK_SUCCESS, 0, &CmdBuffer[1]);
                }
            }
            else if(CmdBuffer[1] == 1 || CmdBuffer[1] ==  2 || CmdBuffer[1] == 3)  //01 00 or 02 00 or 03 00: HFP/A2DP/SPP success
            {
                if(CmdBuffer[2] == 0x00)
                {
                    DRV_BM64_EventHandler(DRV_BM64_DEC_EVENT_LINKBACK_SUCCESS, 0, &CmdBuffer[1]);
                }
                else
                {
                    DRV_BM64_EventHandler(DRV_BM64_DEC_EVENT_LINKBACK_FAILED, 0, &CmdBuffer[1]);
                }
            }
            else
            {
                
            }
            break;

        case DRV_BM64_BM64EVENT_REPORT_LOCAL_BD_ADDR:
            DRV_BM64_SendAckToEvent(CmdBuffer[0]);     //send ACK to this event
            DRV_BM64_SaveLocalBDAddress(&CmdBuffer[1]);
            DRV_BM64_EventHandler(DRV_BM64_DEC_EVENT_BD_ADDR_RECEIVED, 0, &CmdBuffer[1]);
            break;
            
        case DRV_BM64_BM64EVENT_REPORT_LOCAL_DEVICE_NAME:
            DRV_BM64_SendAckToEvent(CmdBuffer[0]);     //send ACK to this event
            DRV_BM64_SaveLocalBDName(CmdBuffer[1],&CmdBuffer[2]);
            DRV_BM64_EventHandler(DRV_BM64_DEC_EVENT_BT_NAME_RECEIVED, 0, &CmdBuffer[1]);
            break;            
            
        case DRV_BM64_BM64EVENT_REPORT_PAIRING_RECORD:
            DRV_BM64_SendAckToEvent(CmdBuffer[0]);     //send ACK to this event
            DRV_BM64_EventHandler(DRV_BM64_DEC_EVENT_PAIR_RECORD_RECEIVED, (uint16_t)(CmdBuffer[1]), &CmdBuffer[1]);
            break;

        case DRV_BM64_BM64EVENT_VOLUME_LEVEL:              //0x0f command: HFP Gain Level
            DRV_BM64_SendAckToEvent(CmdBuffer[0]);     //send ACK to this event
            DRV_BM64_EventHandler(DRV_BM64_DEC_EVENT_HFP_VOLUME_CHANGED, (uint16_t)(CmdBuffer[2]), &CmdBuffer[1]);
            break;
            
        case DRV_BM64_BM64EVENT_REPORT_AVRCP_VOL_CTRL:     //0x26 command
            DRV_BM64_SendAckToEvent(CmdBuffer[0]);     //send ACK to this event
            DRV_BM64_EventHandler(DRV_BM64_DEC_EVENT_AVRCP_VOLUME_CTRL, (uint16_t)(CmdBuffer[2]), &CmdBuffer[1]);
            break;
            
        case DRV_BM64_BM64EVENT_REPORT_AVRCP_ABS_VOL_CTRL: //0x29 command
            DRV_BM64_SendAckToEvent(CmdBuffer[0]);     //send ACK to this event
            DRV_BM64_EventHandler(DRV_BM64_DEC_EVENT_AVRCP_ABS_VOLUME_CHANGED, (uint16_t)(CmdBuffer[2]), &CmdBuffer[1]);
            break;

        case DRV_BM64_BM64EVENT_AVRCP_SPEC_RSP:
            DRV_BM64_SendAckToEvent(CmdBuffer[0]);     //send ACK to this event
            //CmdBuffer[0] = AVRCP_SPEC_RSP
            //CmdBuffer[1] = database 0 or 1 link
            //CmdBuffer[2] = 0x0F or 0x0D(INTRIM or CHANGED)
            //CmdBuffer[3] = 0x48 fixed(Subunit_type + Subunit_ID)
            //CmdBuffer[4] = 0x00 fixed(Opcode)
            //CmdBuffer[5 ~ 7] = 00 19 58 fixed(company ID
            //CmdBuffer[8] = 0x31 fixed(PDU ID)
            //CmdBuffer[9] = 0x00 fixed(packet ID)
            //CmdBuffer[10 - 11] = parameter length
            //CmdBuffer[12] = EventID2 (EVENT_PLAYBACK_STATUS_CHANGED (0x01), EVENT_VOLUME_CHANGED (0x0d))
            //CmdBuffer[13] = PLAYING STATUS
            if(CmdBuffer[12] == 0x01)        //EVENT_PLAYBACK_STATUS_CHANGED (0x01)
            {
                DRV_BM64_EventHandler(DRV_BM64_DEC_EVENT_PLAYBACK_STATUS_CHANGED, (uint16_t)(CmdBuffer[13]), &CmdBuffer[1]);
            }
            break;

        case DRV_BM64_BM64EVENT_REPORT_SPP_DATA:
            //spp_type = CmdBuffer[2];

            spp_total_length = (uint16_t)CmdBuffer[3];
            spp_total_length <<= 8;
            spp_total_length |= (uint16_t)CmdBuffer[4];

            spp_payload_length = (uint16_t)CmdBuffer[5];
            spp_payload_length <<= 8;
            spp_payload_length |= (uint16_t)CmdBuffer[6];

            DRV_BM64_SendAckToEvent(CmdBuffer[0]);     //send ACK to this event
            DRV_BM64_AddBytesToSPPBuff(&CmdBuffer[7], spp_payload_length);
            break;

        case DRV_BM64_BM64EVENT_REPORT_CUSTOMER_GATT_ATTRIBUTE_DATA:
            DRV_BM64_SendAckToEvent(CmdBuffer[0]);     //send ACK to this event
            DRV_BM64_EventHandler(DRV_BM64_DEC_EVENT_GATT_ATTRIBUTE_DATA, (uint16_t)(CmdBuffer[3]), &CmdBuffer[1]);
            DRV_BM64_CustomerGATT_AttributeData(CmdBuffer[3], &CmdBuffer[4], CmdDecodeCmdLength-4);
            break;
        
        case DRV_BM64_BM64EVENT_REPORT_LINK_MODE:
            DRV_BM64_SendAckToEvent(CmdBuffer[0]);     //send ACK to this event
            para = CmdBuffer[2];     //second byte put to higher part
            para <<= 8;
            para |= CmdBuffer[1];    //first byte put to lower part
            DRV_BM64_EventHandler(DRV_BM64_DEC_EVENT_LINK_MODE_RECEIVED, para, &CmdBuffer[1]);
            break;

        case DRV_BM64_BM64EVENT_REPORT_LE_EVENT:
            DRV_BM64_SendAckToEvent(CmdBuffer[0]);     //send ACK to this event
            switch(CmdBuffer[1])     //LE event sub command
            {
                case 0:
                    para = CmdBuffer[2];
                    para <<= 8;
                    para |= CmdBuffer[3];
                    DRV_BM64_EventHandler(DRV_BM64_DEC_EVENT_LE_STATUS_CHANGED, para, &CmdBuffer[1]);
                    break;
                case 1:
                    DRV_BM64_EventHandler(DRV_BM64_DEC_EVENT_LE_ADV_CONTROL_REPORT, (uint16_t)(CmdBuffer[2]), &CmdBuffer[1]);
                    break;
                case 2:
                    DRV_BM64_EventHandler(DRV_BM64_DEC_EVENT_LE_CONNECTION_PARA_REPORT, 0, &CmdBuffer[1]);
                    break;
                case 3:
                    para = CmdBuffer[2];
                    para <<= 8;
                    para |= CmdBuffer[3];
                    DRV_BM64_EventHandler(DRV_BM64_DEC_EVENT_LE_CONNECTION_PARA_UPDATE_RSP, para, &CmdBuffer[1]);
                    break;
                default:
                    break;
            }
            break;
        
        case DRV_BM64_BM64EVENT_REPORT_INPUT_SIGNAL_LEVEL:
            DRV_BM64_SendAckToEvent(CmdBuffer[0]);     //send ACK to this event
            CmdBuffer[1] ^= 0xff;
            CmdBuffer[2] ^= 0xff;
            CmdBuffer[3] ^= 0xff;
            CmdBuffer[4] ^= 0xff;
            if(CmdBuffer[1])
            {
                para = CmdBuffer[1];
                para <<= 8;
                para |= CmdBuffer[5];
                DRV_BM64_EventHandler(DRV_BM64_DEC_EVENT_PORT0_INPUT_CHANGED, para, &CmdBuffer[1]);
            }
            if(CmdBuffer[2])
            {
                para = CmdBuffer[2];
                para <<= 8;
                para |= CmdBuffer[6];
                DRV_BM64_EventHandler(DRV_BM64_DEC_EVENT_PORT1_INPUT_CHANGED, para, &CmdBuffer[1]);
            }
            if(CmdBuffer[3])
            {
                para = CmdBuffer[3];
                para <<= 8;
                para |= CmdBuffer[7];
                DRV_BM64_EventHandler(DRV_BM64_DEC_EVENT_PORT2_INPUT_CHANGED, para, &CmdBuffer[1]);
            }
            if(CmdBuffer[4])
            {
                para = CmdBuffer[4];
                para <<= 8;
                para |= CmdBuffer[8];
                DRV_BM64_EventHandler(DRV_BM64_DEC_EVENT_PORT3_INPUT_CHANGED, para, &CmdBuffer[1]);
            }
            break;
         
        case DRV_BM64_BM64EVENT_REPORT_TYPE_CODEC:     // added for HFP function
            DRV_BM64_SendAckToEvent(CmdBuffer[0]);     //send ACK to this event
            para = CmdBuffer[1];     //sample frequency
            para <<= 8;
            para |= CmdBuffer[2];    // mode
            DRV_BM64_EventHandler(DRV_BM64_DEC_EVENT_CODEC_TYPE, para, &CmdBuffer[1]);          
            break;

        default:
            DRV_BM64_SendAckToEvent(CmdBuffer[0]);     //send ACK to this event
            break;
    }
}

