/*******************************************************************************
  BM64 Bluetooth Static Driver implementation

  Company:
    Microchip Technology Inc.

  File Name:
    drv_bm64_ble.c

  Summary:
   BM64 Bluetooth Static Driver source file for BLE

  Description:
    This file is the implementation of the internal functions of the BM64
    driver related to BLE.
 
*******************************************************************************/
// DOM-IGNORE-BEGIN
/*******************************************************************************
* Copyright (C) 2018 Microchip Technology Inc. and its subsidiaries.
*
* Subject to your compliance with these terms, you may use Microchip software
* and any derivatives exclusively with Microchip products. It is your
* responsibility to comply with third party license terms applicable to your
* use of third party software (including open source software) that may
* accompany Microchip software.
*
* THIS SOFTWARE IS SUPPLIED BY MICROCHIP "AS IS". NO WARRANTIES, WHETHER
* EXPRESS, IMPLIED OR STATUTORY, APPLY TO THIS SOFTWARE, INCLUDING ANY IMPLIED
* WARRANTIES OF NON-INFRINGEMENT, MERCHANTABILITY, AND FITNESS FOR A
* PARTICULAR PURPOSE.
*
* IN NO EVENT WILL MICROCHIP BE LIABLE FOR ANY INDIRECT, SPECIAL, PUNITIVE,
* INCIDENTAL OR CONSEQUENTIAL LOSS, DAMAGE, COST OR EXPENSE OF ANY KIND
* WHATSOEVER RELATED TO THE SOFTWARE, HOWEVER CAUSED, EVEN IF MICROCHIP HAS
* BEEN ADVISED OF THE POSSIBILITY OR THE DAMAGES ARE FORESEEABLE. TO THE
* FULLEST EXTENT ALLOWED BY LAW, MICROCHIP'S TOTAL LIABILITY ON ALL CLAIMS IN
* ANY WAY RELATED TO THIS SOFTWARE WILL NOT EXCEED THE AMOUNT OF FEES, IF ANY,
* THAT YOU HAVE PAID DIRECTLY TO MICROCHIP FOR THIS SOFTWARE.
*******************************************************************************/
// DOM-IGNORE-END

#include <stdbool.h>
#include <stdint.h>
#include "definitions.h"
#include "bt/driver/bm64/drv_bm64.h"
#include "bt/driver/bm64/drv_bm64_ble.h"
#include "bt/driver/bm64/drv_bm64_command_send.h"
#include "bt/driver/bm64/drv_bm64_command_decode.h"
#include "bt/driver/bm64/drv_bm64_sha1.h"
//#define _HASH_TEST

// all #defines, enums and non-static functions and variables prefixed by
// DRV_BM64 to avoid name conflicts

enum {
    DRV_BM64_BLE_ADV_TASK_IDLE,
    DRV_BM64_BLE_ADV_STOP,
    DRV_BM64_BLE_ADV_UPDATE_RESPONSE_DATA,
    DRV_BM64_BLE_SET_ADV_TYPE,
    DRV_BM64_BLE_ADV_START,
    DRV_BM64_BLE_ADV_START_WAITING,
} DRV_BM64_BLE_advTaskState;

DRV_BM64_BLE_ADV_TYPE BLE_advType;

static bool BLE_advUpdateReq;

struct {
    uint8_t AD_size;            //fixed to 22
    uint8_t AD_Type;            //fixed to 0xff, as manufacturer type
    uint8_t company_ID[2];      //big endian, uint16 is little endian
    uint8_t product_ID[2];
    uint8_t version;
    uint8_t model_ID;
    uint8_t model_number;
    uint8_t color;
    uint8_t local_unique_ID[4];
    uint8_t all_capability_flags_byte_H;
    union {
        uint8_t all_capability_flags_byte_L;
        struct {
            uint8_t flag_wifi_connection_service_supported      : 1;
            uint8_t flag_bt_connection_service_supported        : 1;
            uint8_t flag_bt_multi_audio_service_supported       : 1;
        };
    };
    uint8_t TX_power;
    union {
        uint8_t all_value_of_group_status;
        struct {
            uint8_t BT_connectable          : 1;
            uint8_t output_channel          : 3;
            uint8_t role_in_group           : 1;
            uint8_t group_status            : 3;
        };
    };
    uint8_t master_unique_ID[4];
    uint8_t number_of_player;
} DRV_BM64_BLE_scanResponseInfo;

DRV_BM64_SHA1_CONTEXT context1;              // Context for SHA-1
#if defined (CRYPTO_HASH_CONFIG_SHA_SMALL_RAM)
uint32_t workingBuffer32[16];       // Working buffer for SHA-1/224/256
#else
static uint32_t workingBuffer32[80];       // Working buffer for SHA-1/224/256
#endif
static uint8_t digest[64];
static uint8_t BDAdress[17];       //text buffer of BD address
#ifdef _HASH_TEST
const uint8_t message3[] = "message digest";
//const uint8_t message3_1_digest[] = {0xC1,0x22,0x52,0xCE,0xDA,0x8B,0xE8,0x99,0x4D,0x5F,0xA0,0x29,0x0A,0x47,0x23,0x1C,0x1D,0x16,0xAA,0xE3,};
const uint8_t message[] = "12:34:56:78:90:ab";
#endif

void DRV_BM64_BLE_Query_status( void );
void DRV_BM64_BLE_EnabAdvertising(bool enable);

/*-----------------------global functions --------------------*/
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

  Remarks:
    RV_BM64_BLE_QueryStatus is non-blocking; it returns right away and sometime
    later (perhaps tens or hundreds of ms) the event handler callback will be
    called.
*/

void DRV_BM64_BLE_QueryStatus(const DRV_HANDLE handle)
{
    DRV_BM64_BLE_Query_status();
}

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

  Remarks:
    None.
*/

void DRV_BM64_BLE_EnableAdvertising(const DRV_HANDLE handle, bool enable)
{
    DRV_BM64_BLE_EnabAdvertising(enable);
}

/*----------------------------------------------------------- */
static uint8_t hexToASCII(uint8_t hex)
{
    if(hex<=9)
        return (hex + 0x30);
    else
        return (hex - 0x0a + 0x61);
}
static void formatBTAddressToText(uint8_t * BT_address)
{
    BDAdress[0] = hexToASCII(*BT_address >> 4);
    BDAdress[1] = hexToASCII(*BT_address & 0x0f);
    BT_address++;
    BDAdress[3] = hexToASCII(*BT_address >> 4);
    BDAdress[4] = hexToASCII(*BT_address & 0x0f);
    BT_address++;
    BDAdress[6] = hexToASCII(*BT_address >> 4);
    BDAdress[7] = hexToASCII(*BT_address & 0x0f);
    BT_address++;
    BDAdress[9] = hexToASCII(*BT_address >> 4);
    BDAdress[10] = hexToASCII(*BT_address & 0x0f);
    BT_address++;
    BDAdress[12] = hexToASCII(*BT_address >> 4);
    BDAdress[13] = hexToASCII(*BT_address & 0x0f);
    BT_address++;
    BDAdress[15] = hexToASCII(*BT_address >> 4);
    BDAdress[16] = hexToASCII(*BT_address & 0x0f);
}
/*------------------------------------------------------------*/
void DRV_BM64_BLE_advertiser_Init(void)
{
    BDAdress[2] = ':';
    BDAdress[5] = ':';
    BDAdress[8] = ':';
    BDAdress[11] = ':';
    BDAdress[14] = ':';
#ifdef _HASH_TEST
    SHA1_Initialize (&context1, workingBuffer32);
    SHA1_DataAdd (&context1, (uint8_t *)message3, 14);      //testing
    SHA1_Calculate (&context1, digest);
    SHA1_Initialize (&context1, workingBuffer32);
    SHA1_DataAdd (&context1, (uint8_t *)message, 17);       //BD address testing
    SHA1_Calculate (&context1, digest);
#endif

#if 0       //for SONY specific
    DRV_BM64_BLE_scanResponseInfo.AD_size = 22;
    DRV_BM64_BLE_scanResponseInfo.AD_Type = 0xff;
    DRV_BM64_BLE_scanResponseInfo.company_ID[0] = 0x01;  //0x012d, big endian
    DRV_BM64_BLE_scanResponseInfo.company_ID[1] = 0x2d;
    DRV_BM64_BLE_scanResponseInfo.product_ID[0] = 0x00;  //0x0004, big endian
    DRV_BM64_BLE_scanResponseInfo.product_ID[1] = 0x04;
    DRV_BM64_BLE_scanResponseInfo.version = 0x01;
    DRV_BM64_BLE_scanResponseInfo.model_ID = 0x00;       //for developing testing
    DRV_BM64_BLE_scanResponseInfo.model_number = 0x00;   //use 0x00 for BM63-3G development
    DRV_BM64_BLE_scanResponseInfo.color = 0x00;          //default color
    DRV_BM64_BLE_scanResponseInfo.local_unique_ID[0] = 0;   //init unique ID
    DRV_BM64_BLE_scanResponseInfo.local_unique_ID[1] = 0;   //init unique ID
    DRV_BM64_BLE_scanResponseInfo.local_unique_ID[2] = 0;   //init unique ID
    DRV_BM64_BLE_scanResponseInfo.local_unique_ID[3] = 0;   //init unique ID
    DRV_BM64_BLE_scanResponseInfo.all_capability_flags_byte_H = 0x00;
    DRV_BM64_BLE_scanResponseInfo.all_capability_flags_byte_L = 0x06;
    DRV_BM64_BLE_scanResponseInfo.TX_power = 0x46;
    DRV_BM64_BLE_scanResponseInfo.all_value_of_group_status = 0x00;
    DRV_BM64_BLE_scanResponseInfo.master_unique_ID[0] = 0; //init unique ID
    DRV_BM64_BLE_scanResponseInfo.master_unique_ID[1] = 0; //init unique ID
    DRV_BM64_BLE_scanResponseInfo.master_unique_ID[2] = 0; //init unique ID
    DRV_BM64_BLE_scanResponseInfo.master_unique_ID[3] = 0; //init unique ID
    DRV_BM64_BLE_scanResponseInfo.number_of_player = 0;
#else       //for general
    DRV_BM64_BLE_scanResponseInfo.AD_size = 22;
    DRV_BM64_BLE_scanResponseInfo.AD_Type = 0xff;
    DRV_BM64_BLE_scanResponseInfo.company_ID[0] = 0x00;
    DRV_BM64_BLE_scanResponseInfo.company_ID[1] = 0x01;
    DRV_BM64_BLE_scanResponseInfo.product_ID[0] = 0x00;
    DRV_BM64_BLE_scanResponseInfo.product_ID[1] = 0x01;
    DRV_BM64_BLE_scanResponseInfo.version = 0x01;
    DRV_BM64_BLE_scanResponseInfo.model_ID = 0x00;       //for developing testing
    DRV_BM64_BLE_scanResponseInfo.model_number = 0x00;   //use 0x00 for BM63-3G development
    DRV_BM64_BLE_scanResponseInfo.color = 0x00;          //default color
    DRV_BM64_BLE_scanResponseInfo.local_unique_ID[0] = 0;   //init unique ID
    DRV_BM64_BLE_scanResponseInfo.local_unique_ID[1] = 0;   //init unique ID
    DRV_BM64_BLE_scanResponseInfo.local_unique_ID[2] = 0;   //init unique ID
    DRV_BM64_BLE_scanResponseInfo.local_unique_ID[3] = 0;   //init unique ID
    DRV_BM64_BLE_scanResponseInfo.all_capability_flags_byte_H = 0x00;
    DRV_BM64_BLE_scanResponseInfo.all_capability_flags_byte_L = 0x06;
    DRV_BM64_BLE_scanResponseInfo.TX_power = 0x46;
    DRV_BM64_BLE_scanResponseInfo.all_value_of_group_status = 0x00;
    DRV_BM64_BLE_scanResponseInfo.master_unique_ID[0] = 0; //init unique ID
    DRV_BM64_BLE_scanResponseInfo.master_unique_ID[1] = 0; //init unique ID
    DRV_BM64_BLE_scanResponseInfo.master_unique_ID[2] = 0; //init unique ID
    DRV_BM64_BLE_scanResponseInfo.master_unique_ID[3] = 0; //init unique ID
    DRV_BM64_BLE_scanResponseInfo.number_of_player = 0;
#endif
}
/*------------------------------------------------------------*/
void DRV_BM64_BLE_advUpdateLocalUniqueID(uint8_t * BT_address) 
{
    formatBTAddressToText(BT_address);
    DRV_BM64_SHA1_Initialize (&context1, workingBuffer32);
    DRV_BM64_SHA1_DataAdd (&context1, BDAdress, 17);
    DRV_BM64_SHA1_Calculate (&context1, digest);
    DRV_BM64_BLE_scanResponseInfo.local_unique_ID[0] = digest[0];
    DRV_BM64_BLE_scanResponseInfo.local_unique_ID[1] = digest[1];
    DRV_BM64_BLE_scanResponseInfo.local_unique_ID[2] = digest[2];
    DRV_BM64_BLE_scanResponseInfo.local_unique_ID[3] = digest[3];
    if(DRV_BM64_BLE_advTaskState > DRV_BM64_BLE_ADV_UPDATE_RESPONSE_DATA || DRV_BM64_BLE_advTaskState == DRV_BM64_BLE_ADV_TASK_IDLE)
        BLE_advUpdateReq = true;
}
/*------------------------------------------------------------*/
void DRV_BM64_BLE_advUpdateMasterUniqueID(uint8_t * BT_address)
{
    formatBTAddressToText(BT_address);
    DRV_BM64_SHA1_Initialize (&context1, workingBuffer32);
    DRV_BM64_SHA1_DataAdd (&context1, BDAdress, 17);
    DRV_BM64_SHA1_Calculate (&context1, digest);
    DRV_BM64_BLE_scanResponseInfo.master_unique_ID[0] = digest[0];
    DRV_BM64_BLE_scanResponseInfo.master_unique_ID[1] = digest[1];
    DRV_BM64_BLE_scanResponseInfo.master_unique_ID[2] = digest[2];
    DRV_BM64_BLE_scanResponseInfo.master_unique_ID[3] = digest[3];
    if(DRV_BM64_BLE_advTaskState > DRV_BM64_BLE_ADV_UPDATE_RESPONSE_DATA || DRV_BM64_BLE_advTaskState == DRV_BM64_BLE_ADV_TASK_IDLE)
        BLE_advUpdateReq = true;
}
/*------------------------------------------------------------*/
void DRV_BM64_BLE_advUpdateNumOfPlayer(uint8_t num)
{
    if(DRV_BM64_BLE_scanResponseInfo.number_of_player != num)
    {
        DRV_BM64_BLE_scanResponseInfo.number_of_player = num;
        if(DRV_BM64_BLE_advTaskState > DRV_BM64_BLE_ADV_UPDATE_RESPONSE_DATA || DRV_BM64_BLE_advTaskState == DRV_BM64_BLE_ADV_TASK_IDLE)
            BLE_advUpdateReq = true;
    }
}
/*------------------------------------------------------------*/
void DRV_BM64_BLE_advUpdateGroupStatus(DRV_BM64_BLE_GROUP_STATUS group_status)
{
    if(DRV_BM64_BLE_scanResponseInfo.group_status != group_status)
    {
        DRV_BM64_BLE_scanResponseInfo.group_status = group_status;
        if(DRV_BM64_BLE_advTaskState > DRV_BM64_BLE_ADV_UPDATE_RESPONSE_DATA || DRV_BM64_BLE_advTaskState == DRV_BM64_BLE_ADV_TASK_IDLE)
            BLE_advUpdateReq = true;
    }
}
/*------------------------------------------------------------*/
void DRV_BM64_BLE_advUpdateRoleInGroup(DRV_BM64_BLE_GROUP_ROLE role)
{
    if(DRV_BM64_BLE_scanResponseInfo.role_in_group != role)
    {
        DRV_BM64_BLE_scanResponseInfo.role_in_group = role;
        if(DRV_BM64_BLE_advTaskState > DRV_BM64_BLE_ADV_UPDATE_RESPONSE_DATA || DRV_BM64_BLE_advTaskState == DRV_BM64_BLE_ADV_TASK_IDLE)
            BLE_advUpdateReq = true;
    }
}
/*------------------------------------------------------------*/
void DRV_BM64_BLE_advUpdateOutputChannel(DRV_BM64_BLE_GROUP_OUTPUT_CHANNEL channel)
{
    if(DRV_BM64_BLE_scanResponseInfo.output_channel != channel)
    {
        DRV_BM64_BLE_scanResponseInfo.output_channel = channel;
        if(DRV_BM64_BLE_advTaskState > DRV_BM64_BLE_ADV_UPDATE_RESPONSE_DATA || DRV_BM64_BLE_advTaskState == DRV_BM64_BLE_ADV_TASK_IDLE)
            BLE_advUpdateReq = true;
    }
}
/*------------------------------------------------------------*/
void DRV_BM64_BLE_advUpdateBTconnectable(DRV_BM64_BLE_CONNECTABLE_STATUS connectable)
{
    DRV_BM64_BLE_scanResponseInfo.BT_connectable = connectable;
    if(DRV_BM64_BLE_advTaskState > DRV_BM64_BLE_ADV_UPDATE_RESPONSE_DATA || DRV_BM64_BLE_advTaskState == DRV_BM64_BLE_ADV_TASK_IDLE)
        BLE_advUpdateReq = true;
}
/*------------------------------------------------------------*/

void DRV_BM64_BLE_Query_status( void )
{
    uint8_t command[6];
    uint8_t chksum;

    command[0] = 0xAA;      //header byte 0
    command[1] = 0x00;      //header byte 1
    command[2] = 0x02;      //length
    command[3] = DRV_BM64_LE_SIGNALING_CMD;      //command ID
    command[4] = 0x00;      //subCommand 0x00: query status
    chksum = command[2] + command[3] + command[4];
    chksum = ~chksum + 1;
    command[5] = chksum;
    DRV_BM64_SendBytesAsCompleteCommand(&command[0], 6);
}

void DRV_BM64_BLE_EnabAdvertising(bool enable)
{
    uint8_t command[7];
    uint8_t chksum;

    command[0] = 0xAA;      //header byte 0
    command[1] = 0x00;      //header byte 1
    command[2] = 0x03;      //length
    command[3] = DRV_BM64_LE_SIGNALING_CMD;      //command ID
    command[4] = 0x01;      //subCommand 0x01: Enable advertising or disable advertising
    command[5] = enable;
    chksum = command[2] + command[3] + command[4] + command[5];
    chksum = ~chksum + 1;
    command[6] = chksum;
    DRV_BM64_SendBytesAsCompleteCommand(&command[0], 7);
}

void DRV_BM64_BLE_SetAdvertisingType(void)
{
    uint8_t command[7];
    uint8_t chksum;

    command[0] = 0xAA;      //header byte 0
    command[1] = 0x00;      //header byte 1
    command[2] = 0x03;      //length
    command[3] = DRV_BM64_LE_SIGNALING_CMD;      //command ID
    command[4] = 0x04;      //subCommand 0x04: set advertising type
    command[5] = BLE_advType;
    chksum = command[2] + command[3] + command[4] + command[5];
    chksum = ~chksum + 1;
    command[6] = chksum;
    DRV_BM64_SendBytesAsCompleteCommand(&command[0], 7);
}

void DRV_BM64_BLE_SetAdvResponseData(void)
{
    uint8_t command[50];
    uint8_t chksum;
    uint8_t* p;
    uint8_t i,size;
    
    p = (uint8_t *)&(DRV_BM64_BLE_scanResponseInfo.AD_size);
    size = sizeof(DRV_BM64_BLE_scanResponseInfo);
    command[0] = 0xAA;      //header byte 0
    command[1] = 0x00;      //header byte 1
    command[2] = 0x03+size;      //length
    command[3] = DRV_BM64_LE_SIGNALING_CMD;      //command ID
    command[4] = 0x06;      //subCommand 0x06: set response data
    command[5] = 00;
    chksum = command[2] + command[3] + command[4] + command[5];
    
    for(i = 0; i< size; i++)
    {
        command[6+i] = *p;
        chksum += *p++;
    }
    
    chksum = ~chksum + 1;
    command[6+size] = chksum;
    DRV_BM64_SendBytesAsCompleteCommand(&command[0], 7+size);
}
/*------------------------------------------------------------*/
/*------------------------------------------------------------*/
void DRV_BM64_BLE_advertiserUpdateTask( void )
{
    if(DRV_BM64_BLE_advTaskState == DRV_BM64_BLE_ADV_TASK_IDLE && BLE_advUpdateReq == true)
    {
        BLE_advUpdateReq = false;
        DRV_BM64_BLE_advTaskState = DRV_BM64_BLE_ADV_STOP;
    }
    
    switch(DRV_BM64_BLE_advTaskState)
    {
        case DRV_BM64_BLE_ADV_STOP:
            if(DRV_BM64_IsAllowedToSendCommand())
            {
                DRV_BM64_BLE_EnabAdvertising(false);
                DRV_BM64_BLE_advTaskState = DRV_BM64_BLE_ADV_UPDATE_RESPONSE_DATA;//BLE_ADV_STOP_WAITING;
            }
            break;
        case DRV_BM64_BLE_ADV_UPDATE_RESPONSE_DATA:
            if(DRV_BM64_IsAllowedToSendCommand())
            {
                DRV_BM64_BLE_SetAdvResponseData();
                DRV_BM64_BLE_advTaskState = DRV_BM64_BLE_SET_ADV_TYPE;//BLE_ADV_UPDATE_DATA_WAITING;
                //BLE_timer1ms = 100;
            }
            break;
        case DRV_BM64_BLE_SET_ADV_TYPE:
            if(DRV_BM64_IsAllowedToSendCommand())
            {
                DRV_BM64_BLE_SetAdvertisingType();
                DRV_BM64_BLE_advTaskState = DRV_BM64_BLE_ADV_START;//BLE_SET_ADV_TYPE_WAITING;
                //BLE_timer1ms = 100;
            }
            break;
        case DRV_BM64_BLE_ADV_START:
            if(DRV_BM64_IsAllowedToSendCommand())
            {
                DRV_BM64_BLE_EnabAdvertising(true);
                DRV_BM64_BLE_advTaskState = DRV_BM64_BLE_ADV_START_WAITING;
                //BLE_timer1ms = 100;
            }
            break;
        case DRV_BM64_BLE_ADV_START_WAITING:
            if(DRV_BM64_GetAckStatusByCommandID(DRV_BM64_LE_SIGNALING_CMD) == DRV_BM64_ACK_STS_OK)
            {
                DRV_BM64_BLE_advTaskState = DRV_BM64_BLE_ADV_TASK_IDLE;
            }
            break;
        default:
            break;
    }
}
/*------------------------------------------------------------*/
void DRV_BM64_BLE_advTaskInit(void)
{
    DRV_BM64_BLE_advertiser_Init();
    BLE_advUpdateReq = false;
    DRV_BM64_BLE_advTaskState = DRV_BM64_BLE_ADV_TASK_IDLE;
}
/*------------------------------------------------------------*/
void DRV_BM64_BLE_UpdateAdvType(DRV_BM64_BLE_ADV_TYPE type)
{
    BLE_advType = type;
    if(DRV_BM64_BLE_advTaskState > DRV_BM64_BLE_ADV_UPDATE_RESPONSE_DATA || DRV_BM64_BLE_advTaskState == DRV_BM64_BLE_ADV_TASK_IDLE)
        BLE_advUpdateReq = true;
}
/*------------------------------------------------------------*/
void DRV_BM64_BLE_advUpdateRequest( void )
{
    BLE_advUpdateReq = true;
}
/*------------------------------------------------------------*/
bool DRV_BM64_BLE_advUpdateIsEnd( void )
{
    if(BLE_advUpdateReq == true)
        return false;
    if(DRV_BM64_BLE_advTaskState == DRV_BM64_BLE_ADV_TASK_IDLE)
        return true;
    else
        return false;
}
