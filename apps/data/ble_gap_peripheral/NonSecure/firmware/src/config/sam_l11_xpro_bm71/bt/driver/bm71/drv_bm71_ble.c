/*******************************************************************************
  BM71 Bluetooth Static Driver implementation

  Company:
    Microchip Technology Inc.

  File Name:
    drv_bm71_ble.c

  Summary:
   BM71 Bluetooth Static Driver source file for BLE

  Description:
    This file is the implementation of the internal functions of the BM71
    driver related to BLE.
 
*******************************************************************************/
// DOM-IGNORE-BEGIN
/*******************************************************************************
* Copyright (C) 2019 Microchip Technology Inc. and its subsidiaries.
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
#include <string.h>
#include "definitions.h"
#include "bt/driver/bm71/drv_bm71.h"
#include "bt/driver/bm71/drv_bm71_local.h"
#include "bt/driver/bm71/drv_bm71_ble.h"
#include "bt/driver/bm71/drv_bm71_command_send.h"
#include "bt/driver/bm71/drv_bm71_command_decode.h"

extern DRV_BM71_OBJ    gDrvBm71Obj;

// all #defines, enums and non-static functions and variables prefixed by
// DRV_BM71 to avoid name conflicts

void DRV_BM71_BLE_Query_status( void );
void DRV_BM71_BLE_EnabAdvertising(bool enable);


/*-----------------------global functions --------------------*/
// *****************************************************************************
/* Function DRV_BM71_BLE_QueryStatus:

        void DRV_BM71_BLE_QueryStatus(const DRV_HANDLE handle);

  Summary:
    Query BM71 LE status.

  Description:
    Queries the BM71 to respond with a DRV_BM71_EVENT_BLE_STATUS_CHANGED event,
    which will indicate if the BM71 BLE status is standby, advertising,
    scanning or connected.

  Precondition:
    DRV_BM71_Open must have been called to obtain a valid opened device handle.

  Parameters:
    handle      - valid handle to an opened BM71 device driver unique to client

  Returns:
    None.

  Remarks:
    RV_BM71_BLE_QueryStatus is non-blocking; it returns right away and sometime
    later (perhaps tens or hundreds of ms) the event handler callback will be
    called.
*/

void DRV_BM71_BLE_QueryStatus(const DRV_HANDLE handle)
{
    DRV_BM71_BLE_Query_status();
}

// *****************************************************************************
/* Function DRV_BM71_BLE_EnableAdvertising:

        void DRV_BM71_BLE_EnableAdvertising(const DRV_HANDLE handle, bool enable);

  Summary:
    Enable or disable advertising.

  Description:
    Enable or disable BLE advertising.
 
  Precondition:
    DRV_BM71_Open must have been called to obtain a valid opened device handle.

  Parameters:
    handle      - valid handle to an opened BM71 device driver unique to client
    enable      - true to enable advertising, false to disable advertising

  Returns:
    None.

  Remarks:
    None.
*/

void DRV_BM71_BLE_EnableAdvertising(const DRV_HANDLE handle, bool enable)
{
    DRV_BM71_BLE_EnabAdvertising(enable);
}

void DRV_BM71_BLE_Query_status( void )
{
    uint8_t command[5];
    uint8_t chksum;

    command[0] = 0xAA;      //header byte 0
    command[1] = 0x00;      //header byte 1
    command[2] = 0x01;      //length
    command[3] = BM_STATUS_READ;      //command ID
    chksum = command[2] + command[3];
    chksum = ~chksum + 1;
    command[4] = chksum;
    DRV_BM71_SendBytesAsCompleteCommand(&command[0], 5);  
}

void DRV_BM71_BLE_Reset( void )
{
    uint8_t command[5];
    uint8_t chksum;

    command[0] = 0xAA;      //header byte 0
    command[1] = 0x00;      //header byte 1
    command[2] = 0x01;      //length
    command[3] = BM_RESET;      //command ID
    chksum = command[2] + command[3];
    chksum = ~chksum + 1;
    command[4] = chksum;
    DRV_BM71_SendBytesAsCompleteCommand(&command[0], 5);
}

void DRV_BM71_SendSPPData(uint8_t *data, uint16_t size, uint8_t connHandle)
{
    uint8_t command[646];
    uint8_t chksum;
    uint16_t i;

    if (size > 640)
    {
        size = 640;
    }
    uint8_t *p = data;  

    command[0] = 0xAA;      //header byte 0
    command[1] = 0x00;      //header byte 1
    command[2] = 0x02+size;
    command[3] = BM_TRANSPARENT_DATA_SEND;  //command ID
    command[4] = connHandle; 
    chksum = command[2] + command[3] + command[4];
    
    for(i = 0; i<size; i++)
    {
        command[5+i] = *p;
        chksum += *p++;
    }
    
    chksum = ~chksum + 1;
    command[5+size] = chksum;
    DRV_BM71_SendBytesAsCompleteCommand(&command[0], 6+size);
}

void DRV_BM71_BLE_WriteAdvertisingData(void)
{
    uint8_t command[50];
    uint8_t chksum;
    uint8_t i,size;

    char *name = "TransparentUARTDemo";
    size = strlen(name);
    char *p = name;   

    command[0] = 0xAA;      //header byte 0
    command[1] = 0x00;      //header byte 1
    command[2] = 0x07+size;
    command[3] = BM_ADV_DATA_WRITE;      //command ID
    command[4] = 0x01;
    command[5] = 0x02;    
    command[6] = ADV_FLAGS;
    command[7] = 0x05;
    command[8] = size+1;   
    command[9] = ADV_COMPLETE_NAME;  
    chksum = command[2] + command[3] + command[4] + command[5] + command[6] + command[7] + command[8] + command[9];
    
    for(i = 0; i<size; i++)
    {
        command[10+i] = (uint8_t)*p;
        chksum += (uint8_t)*p++;
    }
    
    chksum = ~chksum + 1;
    command[10+size] = chksum;
    DRV_BM71_SendBytesAsCompleteCommand(&command[0], 11+size);
}

void DRV_BM71_BLE_EnabAdvertising(bool enable)
{
    uint8_t command[6];
    uint8_t chksum;

    command[0] = 0xAA;      //header byte 0
    command[1] = 0x00;      //header byte 1
    command[2] = 0x02;      //length
    command[3] = BM_SET_ADV_ENABLE;      //command ID
    command[4] = (enable) ? 0x81 : 0;      //0x81=enter stby, beacon enabled, 0=leave standby
    chksum = command[2] + command[3] + command[4];
    chksum = ~chksum + 1;
    command[5] = chksum;
    DRV_BM71_SendBytesAsCompleteCommand(&command[0], 6);
}

void DRV_BM71_BLE_SetAdvertisingParams(void)
{
    uint8_t command[15];
    uint8_t chksum;

    command[0] = 0xAA;      //header byte 0
    command[1] = 0x00;      //header byte 1
    command[2] = 0x0B;      //length
    command[3] = BM_ADV_PARAM_SET;      //command ID
    command[4] = 0x06;      // advertising interval (1600*0.625 ms = 1 sec)
    command[5] = 0x40;      // advertising interval (lo byte)
    command[6] = 0x00;      // advertising type      
    command[7] = 0x00;      // direct address type      
    command[8] = 0x00;      // public or random device address
    command[9] = 0x00;
    command[10] = 0x00;
    command[11] = 0x00;
    command[12] = 0x00;
    command[13] = 0x00;
    chksum = command[2] + command[3] + command[4] + command[5]; // remainder 0
    chksum = ~chksum + 1;
    command[14] = chksum;
    DRV_BM71_SendBytesAsCompleteCommand(&command[0], 15);
}

void DRV_BM71_BLE_EnabTransparentMode(bool enable)
{
    uint8_t command[8];
    uint8_t chksum;

    command[0] = 0xAA;      //header byte 0
    command[1] = 0x00;      //header byte 1
    command[2] = 0x04;      //length
    command[3] = BM_TRANSPARENT_ENABLE;      //command ID
    command[4] = gDrvBm71Obj.linkIndex;      // last conn handle
    command[5] = (enable) ? 0x01 : 0;
    command[6] = 0x00;
    chksum = command[2] + command[3] + command[4] + command[5] + command[6];
    chksum = ~chksum + 1;
    command[7] = chksum;
    DRV_BM71_SendBytesAsCompleteCommand(&command[0], 8);
}

void DRV_BM71_BLE_ConnectionParameterSet(uint16_t min_conn_interval, uint16_t max_conn_interval, uint16_t slave_latency, uint16_t so_timeout)
{
    uint8_t command[13];
    uint8_t chksum;
    
    command[0] = 0xAA;      //header byte 0
    command[1] = 0x00;      //header byte 1
    command[2] = 0x08;      //length
    command[3] = BM_CONN_PARAM_SET;      //command ID
    command[4] = (uint8_t)min_conn_interval;      //min conn interval
    command[5] = (uint8_t)(min_conn_interval >> 8);
    command[6] = (uint8_t)max_conn_interval;      //max conn interval
    command[7] = (uint8_t)(max_conn_interval >> 8);
    command[8] = (uint8_t)slave_latency;      //Slave latency
    command[9] = (uint8_t)(slave_latency >> 8); 
    command[10]= (uint8_t)so_timeout;      //Supervision Timeout
    command[11]= (uint8_t)(so_timeout >> 8);
    chksum = command[2] + command[3] + command[5] + command[7] + command[9] + command[10];
    chksum = ~chksum + 1;
    command[12] = chksum;
    DRV_BM71_SendBytesAsCompleteCommand(&command[0], 13);
}

void DRV_BM71_BLE_ScanParameterSet(uint16_t scan_interval, uint16_t scan_window, ble_scan_type_t scan_type)
{
    uint8_t command[10];
    uint8_t chksum;
    
    command[0] = 0xAA;      //header byte 0
    command[1] = 0x00;      //length MSB
    command[2] = 0x06;      //length LSB
    command[3] = BM_SCAN_PARAM_SET;      //command ID
    command[4] = (uint8_t)(scan_interval);      //Scan interval
    command[5] = (uint8_t)(scan_interval >> 8);
    command[6] = (uint8_t)(scan_window);      //Scan window
    command[7] = (uint8_t)(scan_window >> 8);
    command[8] = scan_type;      //Scan Type - Active (1)
    chksum = command[2] + command[3] + command[5] + command[7] + command[8] ;
    chksum = ~chksum + 1;
    command[9] = chksum;
    DRV_BM71_SendBytesAsCompleteCommand(&command[0], 10);
}

void DRV_BM71_BLE_ScanEnable(ble_scan_enable_t scan_enable, ble_scan_duplicate_filter_t scan_dup_filter)
{
    uint8_t command[7];
    uint8_t chksum;
    
    command[0] = 0xAA;      //header byte 0
    command[1] = 0x00;      //length MSB
    command[2] = 0x03;      //length LSB
    command[3] = BM_SCAN_ENABLE_SET;      //command ID
    command[4] = scan_enable;      //Scan enable/disable
    command[5] = scan_dup_filter;  //Filter duplicate Adv packet
    chksum = command[2] + command[3] + command[4] + command[5] ;
    chksum = ~chksum + 1;
    command[6] = chksum;
    DRV_BM71_SendBytesAsCompleteCommand(&command[0], 7);
}

void DRV_BM71_BLE_StartScan(void)
{
    DRV_BM71_BLE_ScanParameterSet(DEFAULT_SCAN_INTERVAL, DEFAULT_SCAN_WINDOW, BLE_SCAN_ACTIVE);
    
    gDrvBm71Obj.state = DRV_BM71_STATE_SET_BLE_SCAN_PARAM;
    
}

void DRV_BM71_BLE_CreateConnection(bool wlFilter, BM_ADV_ADDRESS addrType, uint8_t *addr)
{
    uint8_t command[13];
    uint8_t chksum;
    
    command[0] = 0xAA;      //header byte 0
    command[1] = 0x00;      //length MSB
    command[2] = 0x09;      //length LSB
    command[3] = BM_CONNECT;      //command ID
    command[4] = wlFilter;      //Whitelist Filter Disabled
    command[5] = addrType;      //Device Address Type - Public Address - Hardcoded Value
    
    memcpy((uint8_t *)&command[6], addr, 6);
    //command[6] = 0x01;      //Device Address (6 Bytes)
    //command[7] = 0x02;
    //command[8] = 0x03;
    //command[9] = 0x04;
    //command[10] = 0x05;
    //command[11] = 0x06;
    chksum = command[2] + command[3] + command[4] + command[5] + command[6] + command[7] 
            + command[8] + command[9] + command[10] + command[11];
    chksum = ~chksum + 1;
    command[12] = chksum;
    DRV_BM71_SendBytesAsCompleteCommand(&command[0], 13);
}

void DRV_BM71_BLE_ClientDiscoverCharacteristics(uint8_t connHandle, ble_uuid_t *serviceUUID)
{
    uint8_t command[22];
    uint8_t chksum;
    uint8_t type = serviceUUID->type;
    uint8_t UUIDLen = 0;
    
    
    command[0] = 0xAA;      //header byte 0
    command[1] = 0x00;      //length MSB
    command[2] = 0x02;      //length LSB
    command[3] = BM_CLIENT_DISCOVER_CHARACTERISTICS;      //command ID
    command[4] = connHandle;      //Whitelist Filter Disabled
    
    if (type == BLE_UUID_16B)
    { 
        UUIDLen = 2;
        memcpy((uint8_t *)&command[5], serviceUUID->uuid.uuid_16b, UUIDLen );
    }
    else
    {
        UUIDLen = 16;
        memcpy((uint8_t *)&command[5], serviceUUID->uuid.uuid_128b, UUIDLen );
    }
    
    command[2] = command[2] + UUIDLen;
    chksum = command[2] + command[3] + command[4] + command[5] + command[6] + command[7] 
            + command[8] + command[9] + command[10] + command[11] + command[12] + command[13] 
            + command[14] + command[15] + command[16] + command[17] + command[18] + command[19]+ command[20];
    chksum = ~chksum + 1;
    command[21] = chksum;
    DRV_BM71_SendBytesAsCompleteCommand(&command[0], 22);
}

void DRV_BM71_BLE_ClientWriteCharacteristics(uint8_t connectionHandle, uint16_t characteristicHandle, uint8_t writeType, uint8_t* characteriticValue, uint8_t characteriticValueLength)
{
    uint8_t command[11];
    uint8_t chksum;    
    
    command[0] = 0xAA;      //header byte 0
    command[1] = 0x00;      //length MSB
    command[2] = 0x07;      //length LSB
    command[3] = BM_CLIENT_CHARACTERISTIC_WRITE;      //command ID
    command[4] = connectionHandle;      //Whitelist Filter Disabled
    command[5] = writeType;
    
    command[6] = (uint8_t)(characteristicHandle >> 8);
    command[7] = (uint8_t)(characteristicHandle);
    
    memcpy(&command[8], characteriticValue, characteriticValueLength);
    
    chksum = command[2] + command[3] + command[4] + command[5] + command[6] + command[7] 
            + command[8] + command[9];
    chksum = ~chksum + 1;
    command[10] = chksum;
    DRV_BM71_SendBytesAsCompleteCommand(&command[0], 11);
    
}

void DRV_BM71_BLE_ServerPrimaryServiceRead(ble_uuid_t *serviceUUID)
{
    uint8_t command[21];
    uint8_t chksum;
    uint8_t type = serviceUUID->type;
    uint8_t UUIDLen = 0;
    
    
    command[0] = 0xAA;      //header byte 0
    command[1] = 0x00;      //length MSB
    command[2] = 0x01;      //length LSB
    command[3] = BM_SERVER_SERVICE_READ;      //command ID
        
    if (type == BLE_UUID_16B)
    { 
        UUIDLen = 2;
        memcpy((uint8_t *)&command[4], serviceUUID->uuid.uuid_16b, UUIDLen );
    }
    else
    {
        UUIDLen = 16;
        memcpy((uint8_t *)&command[4], serviceUUID->uuid.uuid_128b, UUIDLen );
    }
    
    command[2] = command[2] + UUIDLen;
    chksum = command[2] + command[3] + command[4] + command[5] + command[6] + command[7] 
            + command[8] + command[9] + command[10] + command[11] + command[12] + command[13] 
            + command[14] + command[15] + command[16] + command[17] + command[18] + command[19];
    chksum = ~chksum + 1;
    command[20] = chksum;
    DRV_BM71_SendBytesAsCompleteCommand(&command[0], 21);
}

void DRV_BM71_BLE_StartAdvertising(uint8_t *adv_data, uint8_t adv_data_len)
{
    DRV_BM71_BLE_SetAdvertisingData(adv_data, adv_data_len);
    gDrvBm71Obj.state = DRV_BM71_STATE_SET_BLE_ADV_PARAM;

}

void DRV_BM71_BLE_SetAdvertisingData(uint8_t *adv_data, uint8_t adv_data_len)
{
    uint8_t command[50];
    uint8_t chksum;
    uint8_t i;
    
    uint8_t *p = adv_data;   

    command[0] = 0xAA;      //header byte 0
    command[1] = 0x00;      //header byte 1
    command[2] = 0x02+adv_data_len;
    command[3] = BM_ADV_DATA_WRITE;      //command ID
    command[4] = 0x01;
    
    chksum = command[2] + command[3] + command[4];
    
    for(i = 0; i<adv_data_len; i++)
    {
        command[5+i] = (uint8_t)*p;
        chksum += (uint8_t)*p;
        p++;
    }
    
    chksum = ~chksum + 1;
    command[5+adv_data_len] = chksum;
    DRV_BM71_SendBytesAsCompleteCommand(&command[0], 6+adv_data_len);
}

void DRV_BM71_BLE_ServerCharacteristicSend(ble_handle_t connHandle, ble_handle_t characteristicHandle, uint8_t* data, uint16_t length)
{
    uint8_t command[11];
    uint8_t chksum; 
    
    uint8_t *p = data;
    
    command[0] = 0xAA;      //header byte 0
    command[1] = 0x00;      //length MSB
    command[2] = 0x04 + length;      //length LSB
    command[3] = BM_SERVER_CHARACTERISTIC_SEND;      //command ID
    command[4] = connHandle;      //Whitelist Filter Disabled
    command[5] = (uint8_t)(characteristicHandle >> 8);
    command[6] = (uint8_t)(characteristicHandle);
    
    chksum = command[2] + command[3] + command[4] + command[5] + command[6];

    
    for(uint8_t i = 0; i<length; i++)
    {
        command[7+i] = (uint8_t)*p;
        chksum += (uint8_t)*p;
        p++;
    }
    
    chksum = ~chksum + 1;
    command[7+length] = chksum;
    DRV_BM71_SendBytesAsCompleteCommand(&command[0], 8+length);
}


