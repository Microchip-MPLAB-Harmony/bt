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
