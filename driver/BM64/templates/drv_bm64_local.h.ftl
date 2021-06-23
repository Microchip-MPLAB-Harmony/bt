<#----------------------------------------------------------------------------
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
----------------------------------------------------------------------------->

/*******************************************************************************
  BM64 Bluetooth Static Driver implementation

  Company:
    Microchip Technology Inc.

  File Name:
    drv_bm64_local.h

  Summary:
   BM64 Bluetooth Static Driver header file for local data.

  Description:
    This file is the header file for the local definitions and data declarations
    that are local to the the BM64 driver (not exposed to the client).
 
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
#ifndef DRV_BM64_LOCAL_H
#define DRV_BM64_LOCAL_H

//#include <stdbool.h>
//#include <stdint.h>

#include "driver/driver_common.h"
#include "configuration.h"
#include "bt/driver/bm64/drv_bm64.h"

/**********************************************
 * Driver Client Obj
 **********************************************/

typedef enum 
{
    DRV_BM64_STATE_INITIALIZE_START = 0,
    DRV_BM64_STATE_INIT_MFB_HIGH,
    DRV_BM64_STATE_INIT_RESET_HIGH,
    DRV_BM64_STATE_INIT_RESET_HIGH_WAIT,        
    
    DRV_BM64_STATE_INIT_COMMAND_START,    //send first command
    DRV_BM64_STATE_INIT_SETUP_GPIO,
    DRV_BM64_STATE_INIT_WAIT_GPIO_EVENT,
    DRV_BM64_STATE_INIT_READ_DEVICE_ADDR,
    DRV_BM64_STATE_INIT_READ_DEVICE_ADDR_WAIT,
    DRV_BM64_STATE_INIT_READ_DEVICE_NAME,
    DRV_BM64_STATE_INIT_READ_DEVICE_NAME_WAIT,                        
    DRV_BM64_STATE_INIT_BLE_ADV_START,

    DRV_BM64_STATE_POWER_ON_START,        //wait 500ms. after 500ms, send SYSTEM_ON and CSB_PAGE command
    DRV_BM64_STATE_POWER_ON_BUTTON_PRESSED,
    DRV_BM64_STATE_POWER_ON,          //system on finished, wait for BT or AUX instruction
    DRV_BM64_STATE_BLE_ADV_WAIT,
    
    DRV_BM64_STATE_VOL_SYNC,
    DRV_BM64_STATE_READ_PAIR_RECORD,
    DRV_BM64_STATE_READ_PAIR_RECORD_WAIT,
    DRV_BM64_STATE_READ_LINKED_MODE,
    DRV_BM64_STATE_READ_LINKED_MODE_WAIT,
    DRV_BM64_STATE_LINKBACK_START,
    DRV_BM64_STATE_LINKBACK_TO_LAST_DEVICE,
    DRV_BM64_STATE_POWERBACK_TO_NSPK_MODE,
    DRV_BM64_STATE_POWERBACK_NSPK_MASTER_WAITING,
    DRV_BM64_STATE_POWERBACK_NSPK_SLAVE_WAITING,
    DRV_BM64_STATE_POWERBACK_BROADCAST_MASTER_WAITING,
    DRV_BM64_STATE_POWERBACK_BROADCAST_SLAVE_WAITING,
       
    DRV_BM64_STATE_BT_RUNNING,

    DRV_BM64_STATE_POWER_OFF_START,
    DRV_BM64_STATE_POWER_OFF_START_NSPK,
    DRV_BM64_STATE_POWER_OFF_WAIT,
    DRV_BM64_STATE_POWER_OFF_WAIT_NSPK,
    DRV_BM64_STATE_POWER_OFF_WAIT2,
    DRV_BM64_STATE_POWER_OFF_WAIT2_NSPK,
    DRV_BM64_STATE_POWER_OFF_WAIT_NSPK_EVENT,
    DRV_BM64_STATE_POWER_OFF
} DRV_BM64_TASKSTATE;   // Bluetooth task state define

typedef enum {
    DRV_BM64_VOLUME_A2DP = 0,
    DRV_BM64_VOLUME_HFP,
    DRV_BM64_VOLUME_LINEIN
} DRV_BM64_VOLUME_MODE; // BM64 volume mode

typedef struct
{
    uint8_t a2dpVol;
    uint8_t hfpVol;
    uint8_t lineInVol;
    DRV_BM64_VOLUME_MODE currentVolMode;
} DRV_BM64_VOLUME;      // BM64 volume settings

typedef union 
{
    uint16_t value;
    struct {
        uint16_t SpeakerAddCommandReq        : 1;       //0xE2(NSPK)
        uint16_t BroadcastModeCommandReq     : 1;       //0xE2(broadcast)
        uint16_t NSpeakerTriggerMasterReq    : 1;       //0xE0
        uint16_t NSpeakerTriggerSlaveReq     : 1;       //0xE1
        uint16_t BroadcastTriggerMasterReq   : 1;       //0xE0
        uint16_t BroadcastTriggerSlaveReq    : 1;       //0xE1
        uint16_t PairReq                    : 1;        //ANY_MODE_ENTERING_PAIRING       
        uint16_t ResetEEPROMReq             : 1;        //RESET_EEPROM_SETTING (forget all links)
<#if INCLUDE_DEPRECATED_MMI_COMMANDS == true>          
        uint16_t DisconnectAllLinksReq      : 1;        //DISCONNECT_ALL_LINK
        uint16_t LinkLastDeviceReq          : 1;        //LINK_BACK_DEVICE       
        uint16_t NextSongReq                : 1;        //NEXT_SONG
        uint16_t PreviousSongReq            : 1;        //PREVIOUS_SONG
</#if>        
        uint16_t IncSpkGainReq              : 1;        //INC_SPK_GAIN
        uint16_t DecSpkGainReq              : 1;        //DEC_SPK_GAIN
        uint16_t AcceptCallReq              : 1;        //ACCEPT_CALL
        uint16_t ForceEndCallReq            : 1;        //FORCE_END_CALL
        uint16_t LastNumberRedialReq        : 1;        //LAST_NUMBER_REDIAL
        uint16_t RejectCallReq              : 1;        //REJECT_CALL
    };
} DRV_BM64_NEXTMMIACTIONREQ;

typedef union
{
    uint16_t value;
    struct {
        uint16_t updateA2DPGainReq           : 1;
        uint16_t updateHFPGainReq            : 1;
        uint16_t updateLineInGainReq         : 1;
        uint16_t linkbackToDevAddr           : 1;
        uint16_t musicCtrlReq_00             : 1;
        uint16_t musicCtrlReq_01             : 1;
        uint16_t musicCtrlReq_03             : 1;
        uint16_t musicCtrlReq_05             : 1;
        uint16_t musicCtrlReq_06             : 1;        
        uint16_t musicCtrlReq_07             : 1;
        uint16_t musicCtrlReq_08             : 1;
<#if INCLUDE_DEPRECATED_MMI_COMMANDS == false>
        uint16_t musicCtrlReq_09             : 1;
        uint16_t musicCtrlReq_0A             : 1;
</#if>        
        uint16_t SPPLinkBackReq              : 1;
        uint16_t DisconnectAllLinksReq       : 1;        //was MMI/DISCONNECT_ALL_LINK
        uint16_t LinkLastDeviceReq           : 1;        //was MMI/LINK_BACK_DEVICE         
    };
} DRV_BM64_NEXTCOMMANDREQ;

typedef union
{
    uint16_t value;
    struct {
        uint16_t TransferCallReq            : 1;        //0x0E
        uint16_t VoiceDialReq               : 1;        //0x0A
        uint16_t cancelVoiceDialReq         : 1;        //0x0B
<#if INCLUDE_DEPRECATED_MMI_COMMANDS == true>          
        uint16_t PlayPauseReq               : 1;        //0x32
</#if>        
        uint16_t CancelNSPKReq              : 1;        //0xE3
        uint16_t TerminateNSPKReq           : 1;        //0xE4
        uint16_t TerminateCancelNSPKReq     : 1;        //0xE5
        uint16_t switchNSPKChannel          : 1;        //0xEC
        uint16_t enterNSPKModeReq           : 1;        //0xF4
        uint16_t enterBroadcastModeReq      : 1;        //0xF5
        uint16_t MMI_F6_Req                 : 1;        //0xF6
        uint16_t MMI_F7_Req                 : 1;        //0xF7
    };
} DRV_BM64_NEXTMMIACTIONREQ2;

typedef union
{
    uint8_t port;
    struct{
        uint8_t bit_0    : 1;       // bit 0 of port
        uint8_t bit_1    : 1;       // bit 1 of port
        uint8_t bit_2    : 1;       // bit 2 of port
        uint8_t bit_3    : 1;       // bit 3 of port
        uint8_t bit_4    : 1;       // bit 4 of port
        uint8_t bit_5    : 1;       // bit 5 of port
        uint8_t bit_6    : 1;       // bit 6 of port
        uint8_t bit_7    : 1;       // bit 7 of port
    };
} DRV_BM64_PORT;

// *****************************************************************************
/* BM64 Driver Global Instances Object

  Summary:
    Object used to keep track of data that is common to all instances of the
    BM64 Bluetooth driver.

  Description:
    This object is used to keep track of any data that is common to all
    instances of the  BM64 Bluetooth driver.

  Remarks:
    None.
*/

typedef struct _DRV_BM64_OBJ_STRUCT
{  
    SYS_STATUS status;              // status of this driver instance   
    bool inUse;                     // indicates this object is in use
    bool isExclusive;               // flag to indicate that the hardware instance is used in exclusive access mode */
    uint8_t numClients;             // number of clients possible with the hardware instance
    SYS_MODULE_INDEX i2sDriverModuleIndex;  // identifies data module(I2S) driver ID for data interface of BM64
    DRV_HANDLE i2sDriverHandle;     // identifies data module(I2S) driver open handle
    SYS_MODULE_INDEX tmrDriverModuleIndex;  // identifies control module timer ID for control interface of CODEC 
    DRV_HANDLE tmrDriverHandle;     // identifies control module(Timer) driver open handle
    uint32_t samplingRate;          // sampling rate
    bool isInInterruptContext;      // keeps track if the driver is in interrupt context
    OSAL_MUTEX_DECLARE(mutexDriverInstance);    // hardware instance mutex
    uint8_t linkIndex;
    uint8_t request;
    DRV_BM64_LINKSTATUS linkStatus;
    uint8_t linkedMode;
    uint8_t pairedRecordNumber;
    uint8_t nSPKLinkedCounter;
    uint8_t nSPKLinkingBackCounter;  
    DRV_BM64_TASKSTATE state;    
    DRV_BM64_NEXTCOMMANDREQ nextCommandReq;
    DRV_BM64_NEXTMMIACTIONREQ nextMMIActionReq;
    DRV_BM64_NEXTMMIACTIONREQ2 nextMMIActionReq2;    
    DRV_BM64_VOLUME volume;    
    DRV_BM64_PORT port1;
    DRV_BM64_PORT port3;    
    uint16_t mclk_multiplier;
    uint16_t bclk_divider;
    DRV_BM64_PLAYINGSTATUS playingStatus;
} DRV_BM64_OBJ;

typedef struct
{
    /* Indicates that this object is in use */
    bool inUse;

    /* Indicate whether the client is open in
     * read,write or read/write mode */
    DRV_IO_INTENT ioIntent;
    
    DRV_BM64_PROTOCOL protocol;

    /* Call back function for this client */
    //DRV_BM64_BUFFER_EVENT_HANDLER  pBufferEventCallBack;
    DRV_BM64_EVENT_HANDLER  pEventCallBack;

    /* Client data(Event Context) that will be
     * returned at callback */
    uintptr_t hClientArg;

} DRV_BM64_CLIENT_OBJ;

typedef struct
{
    /* Set to true if all members of this structure
       have been initialized once */
    bool membersAreInitialized;

    /* Mutex to protect client object pool */
    OSAL_MUTEX_DECLARE(mutexClientObjects);

} DRV_BM64_COMMON_DATA_OBJ;

// beginning of old file

// decode events (passed between drv_bm64_command_decode.c and drv_bm64.c)
typedef enum 
{
    DRV_BM64_DEC_EVENT_NONE = 0,

    DRV_BM64_DEC_EVENT_NSPK_STATUS,
    DRV_BM64_DEC_EVENT_LINE_IN_STATUS,
    DRV_BM64_DEC_EVENT_A2DP_STATUS,
    DRV_BM64_DEC_EVENT_CALL_STATUS_CHANGED,

    DRV_BM64_DEC_EVENT_CODEC_TYPE,    // added

    DRV_BM64_DEC_EVENT_HFP_CONNECTED,
    DRV_BM64_DEC_EVENT_HFP_DISCONNECTED,
    DRV_BM64_DEC_EVENT_A2DP_CONNECTED,
    DRV_BM64_DEC_EVENT_A2DP_DISCONNECTED,
    DRV_BM64_DEC_EVENT_AVRCP_CONNECTED,
    DRV_BM64_DEC_EVENT_AVRCP_DISCONNECTED,
    DRV_BM64_DEC_EVENT_SPP_CONNECTED,
    DRV_BM64_DEC_EVENT_IAP_CONNETED,
    DRV_BM64_DEC_EVENT_SPP_IAP_DISCONNECTED,
    DRV_BM64_DEC_EVENT_ACL_CONNECTED,
    DRV_BM64_DEC_EVENT_ACL_DISCONNECTED,
    DRV_BM64_DEC_EVENT_SCO_CONNECTED,
    DRV_BM64_DEC_EVENT_SCO_DISCONNECTED,
    DRV_BM64_DEC_EVENT_MAP_CONNECTED,
    DRV_BM64_DEC_EVENT_MAP_DISCONNECTED,

    DRV_BM64_DEC_EVENT_SYS_POWER_ON,
    DRV_BM64_DEC_EVENT_SYS_POWER_OFF,
    DRV_BM64_DEC_EVENT_SYS_STANDBY,
    DRV_BM64_DEC_EVENT_SYS_PAIRING_START,
    DRV_BM64_DEC_EVENT_SYS_PAIRING_OK,
    DRV_BM64_DEC_EVENT_SYS_PAIRING_FAILED,

    DRV_BM64_DEC_EVENT_LINKBACK_SUCCESS,
    DRV_BM64_DEC_EVENT_LINKBACK_FAILED,

    DRV_BM64_DEC_EVENT_BD_ADDR_RECEIVED,
    DRV_BM64_DEC_EVENT_PAIR_RECORD_RECEIVED,
    DRV_BM64_DEC_EVENT_LINK_MODE_RECEIVED,
    DRV_BM64_DEC_EVENT_BT_NAME_RECEIVED,            

    DRV_BM64_DEC_EVENT_PLAYBACK_STATUS_CHANGED,
    DRV_BM64_DEC_EVENT_AVRCP_VOLUME_CTRL,
    DRV_BM64_DEC_EVENT_AVRCP_ABS_VOLUME_CHANGED,
    DRV_BM64_DEC_EVENT_HFP_VOLUME_CHANGED,
    
    DRV_BM64_DEC_EVENT_NSPK_SYNC_POWER_OFF,
    DRV_BM64_DEC_EVENT_NSPK_SYNC_VOL_CTRL,
    DRV_BM64_DEC_EVENT_NSPK_SYNC_INTERNAL_GAIN,
    DRV_BM64_DEC_EVENT_NSPK_SYNC_ABS_VOL,
    DRV_BM64_DEC_EVENT_NSPK_CHANNEL_SETTING,
    DRV_BM64_DEC_EVENT_NSPK_ADD_SPEAKER3,
    
    DRV_BM64_DEC_EVENT_LE_STATUS_CHANGED,
    DRV_BM64_DEC_EVENT_LE_ADV_CONTROL_REPORT,
    DRV_BM64_DEC_EVENT_LE_CONNECTION_PARA_REPORT,
    DRV_BM64_DEC_EVENT_LE_CONNECTION_PARA_UPDATE_RSP,
    DRV_BM64_DEC_EVENT_GATT_ATTRIBUTE_DATA,
    
    DRV_BM64_DEC_EVENT_PORT0_INPUT_CHANGED,
    DRV_BM64_DEC_EVENT_PORT1_INPUT_CHANGED,
    DRV_BM64_DEC_EVENT_PORT2_INPUT_CHANGED,
    DRV_BM64_DEC_EVENT_PORT3_INPUT_CHANGED,
} DRV_BM64_DEC_EVENT;           // BM64 decoded events

typedef struct {
    uint8_t LinkedProfileStatus;
    uint8_t DeviceID_LinkIndex;
} DRV_BM64_CONNECTION_STATUS;

#define DRV_BM64_CONNECTION_MAX 3

typedef struct {
    uint8_t activeIndex;
    DRV_BM64_CONNECTION_STATUS allConnection[DRV_BM64_CONNECTION_MAX];
} DRV_BM64_ALLCONNECTIONS;

typedef enum {
    DRV_BM64_LINKBACK_INIT,       //init
    DRV_BM64_LINKBACK_CONNECTING, //sent link_back command but no event yet
    DRV_BM64_LINKBACK_OK,         //link back success, event
    DRV_BM64_LINKBACK_FAILED,     //linkback failed, event
    DRV_BM64_PAIRING_START,       //sent pair command but no event yet
    DRV_BM64_PAIRING,             //pair event
    DRV_BM64_PAIRING_OK,          //pair event
    DRV_BM64_PAIRING_FAILED,      //pair event
    DRV_BM64_LINK_CONNECTED,         //other event
} DRV_BM64_LINKBACKSTATUS;       // linkback status

typedef enum {
    DRV_BM64_SYSTEM_INIT,         //init
    DRV_BM64_SYSTEM_POWER_OFF,    //event
    DRV_BM64_SYSTEM_POWER_ON,     //event
    DRV_BM64_SYSTEM_STANDBY,      //event
    DRV_BM64_SYSTEM_CONNECTED,    //event
    DRV_BM64_SYSTEM_PAIRING,      //event
} DRV_BM64_SYSTEMSTATUS;        // BT internal system status

enum DRV_BM64_NSPK_LINK_STATUS {
    DRV_BM64_NSPK_NO_LINK = 0,
    DRV_BM64_NSPK_MASTER_LINK_TO_SLAVE2 = 1,
    DRV_BM64_NSPK_MASTER_LINK_TO_SLAVE3 = 2,
    DRV_BM64_NSPK_MASTER_LINK_TO_BOTH = 3,
    DRV_BM64_NSPK_SLAVE_LINK_TO_MASTER = 4,
    DRV_BM64_BROADCAST_MASTER = 5,
    DRV_BM64_BROADCAST_SLAVE = 6
};
enum DRV_BM64_NSPK_EVENT {
    DRV_BM64_CSB_EVENT_STANDBY = 0,
    DRV_BM64_CSB_EVENT_BUSY = 1,
    DRV_BM64_CSB_EVENT_CONNECTING = 2,
    DRV_BM64_CSB_EVENT_CONNECTED = 3,
    DRV_BM64_CSB_EVENT_LOSS = 4,
    DRV_BM64_NSPK_EVENT_BACK_TO_MASTER = 5,
    DRV_BM64_NSPK_EVENT_BACK_TO_SLAVE = 6,
    DRV_BM64_CSB_EVENT_CHANGE_ROLE = 7,
    DRV_BM64_CSB_EVENT_DISCONNECTED_BY_NFC = 8,
    DRV_BM64_CSB_EVENT_CONTINUE_CONNECTING = 9
};
enum DRV_BM64_NSPK_SYSTEM_STATUS {
    DRV_BM64_CSB_STATUS_STANDBY = 0,
    DRV_BM64_CSB_STATUS_CONNECTING,
    DRV_BM64_CSB_STATUS_CONNECTED_AS_NSPK_MASTER,
    DRV_BM64_CSB_STATUS_CONNECTED_AS_NSPK_SLAVE,
    DRV_BM64_CSB_STATUS_NSPK_MASTER_CONNECTING,
    DRV_BM64_CSB_STATUS_CONNECTED_AS_BROADCAST_MASTER,
    DRV_BM64_CSB_STATUS_CONNECTED_AS_BROADCAST_SLAVE,
    DRV_BM64_CSB_STATUS_BROADCAST_MASTER_CONNECTING,
};

typedef struct {
    enum DRV_BM64_NSPK_LINK_STATUS nspk_link;
    enum DRV_BM64_NSPK_EVENT snpk_event;
    enum DRV_BM64_NSPK_SYSTEM_STATUS  nspk_status;
} DRV_BM64_ECSBSTATUS;

typedef enum  {
    DRV_BM64_CALL_IDLE = 0,
    DRV_BM64_VOICE_DIAL = 1,
    DRV_BM64_CALL_INCOMMING = 2,
    DRV_BM64_CALL_OUTGOING = 3,
    DRV_BM64_CALLING = 4,
    DRV_BM64_CALLING_WAITING = 5,
    DRV_BM64_CALLING_HOLD = 6
} DRV_BM64_CALLSTATUS;

typedef enum {
    LINE_IN_INACTIVE = 0,
    LINE_IN_ACTIVE,
    LINE_IN_ACTIVE_WITH_AUDIO,
    LINE_IN_WITH_SILENCE_AUDIO
} DRV_BM64_LINEINSTATUS;

typedef enum {
    DRV_BM64_A2DP_IDLE = 0,
    DRV_BM64_A2DP_ACTIVE = 1
} DRV_BM64_A2DPSTATUS;

void DRV_BM64_Timer_1ms( uintptr_t context);
void DRV_BM64_Timer1MS_event( void );
void DRV_BM64_EventHandler(uint8_t event, uint16_t para, uint8_t* para_full);

        void DRV_BM64_SaveLocalBDAddress(uint8_t* address);
void DRV_BM64_SaveLocalBDName(uint8_t len, uint8_t* address);

void DRV_BM64_volumeUpCurrentMode(const DRV_HANDLE handle);

void DRV_BM64_setVolCurrentMode( void );        // A2DP, HFP etc

void DRV_BM64_SPPBuffClear( void );
bool DRV_BM64_AddBytesToSPPBuff(uint8_t* data, uint8_t size);
bool DRV_BM64_CustomerGATT_AttributeData(uint8_t attributeIndex, 
        uint8_t* attributeData, uint8_t attributeDataLength);
#endif