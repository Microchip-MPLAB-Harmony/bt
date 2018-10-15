/*******************************************************************************
  BM64 Bluetooth Static Driver implementation

  Company:
    Microchip Technology Inc.

  File Name:
    drv_bm64_ble.h

  Summary:
   BM64 Bluetooth Static Driver header file for BLE

  Description:
    This file is the header file for the internal functions of the BM64
    driver related to BLE.
 
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
#ifndef DRV_BM64_BLE_H
#define DRV_BM64_BLE_H

#include <stdbool.h>
#include <stdint.h>

typedef enum {
    DRV_BM64_BLE_SINGLE = 0,
    DRV_BM64_BLE_GROUP = 1
} DRV_BM64_BLE_GROUP_STATUS;

typedef enum {
    DRV_BM64_BLE_MASTER_ROLE = 0,
    DRV_BM64_BLE_PLAYER_ROLE = 1
} DRV_BM64_BLE_GROUP_ROLE;

typedef enum {
    DRV_BM64_BLE_STEREO = 0,
    DRV_BM64_BLE_LEFT = 1,
    DRV_BM64_BLE_RIGHT = 2,
    DRV_BM64_BLE_MONAURAL = 3
} DRV_BM64_BLE_GROUP_OUTPUT_CHANNEL;

typedef enum {
    BT_CONNECTABLE = 0,
    BT_NON_CONNECTABLE = 1
} DRV_BM64_BLE_CONNECTABLE_STATUS;

typedef enum {
    CONNECTABLE_UNDIRECT_ADV = 0,           //connectable
    CONNECTABLE_RESERVED = 1,
    SCANNABLE_UNDIRECT_ADV = 2,
    NON_CONNECTABLE_UNDIRECT_ADV = 3,       //non-connectable
} DRV_BM64_BLE_ADV_TYPE;

void DRV_BM64_BLE_advertiser_Init(void);
void DRV_BM64_BLE_advUpdateLocalUniqueID(uint8_t* BT_address);
void DRV_BM64_BLE_advUpdateMasterUniqueID(uint8_t* BT_address);
void DRV_BM64_BLE_advUpdateNumOfPlayer(uint8_t num);
void DRV_BM64_BLE_advUpdateGroupStatus(DRV_BM64_BLE_GROUP_STATUS group_status);
void DRV_BM64_BLE_advUpdateRoleInGroup(DRV_BM64_BLE_GROUP_ROLE role);
void DRV_BM64_BLE_advUpdateOutputChannel(DRV_BM64_BLE_GROUP_OUTPUT_CHANNEL channel);
void DRV_BM64_BLE_advUpdateBTconnectable(DRV_BM64_BLE_CONNECTABLE_STATUS connectable);

void DRV_BM64_BLE_advertiserUpdateTask( void );
void DRV_BM64_BLE_advTaskInit(void);
void DRV_BM64_BLE_UpdateAdvType(DRV_BM64_BLE_ADV_TYPE type);
void DRV_BM64_BLE_advUpdateRequest( void );          //basically don't use it
bool DRV_BM64_BLE_advUpdateIsEnd( void );

void DRV_BM64_BLE_Timer1MS_event( void );

#endif
