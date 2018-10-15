/*******************************************************************************
  BM64 Bluetooth Static Driver implementation

  Company:
    Microchip Technology Inc.

  File Name:
    drv_bm64_command_send.c

  Summary:
   BM64 Bluetooth Static Driver source file for sending commands.

  Description:
    This file is the implementation of the internal functions of the BM64
    driver related to sending commands to the BM64 module.
 
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
#include "bt/driver/bm64/drv_bm64_command_send.h"
#include "bt/driver/bm64/drv_bm64_command_decode.h"
#include "bt/driver/bm64/drv_bm64_gpio.h"
#include "bt/driver/bm64/drv_bm64_uart.h"

// all #defines, enums and non-static functions and variables prefixed by
// DRV_BM64 to avoid name conflicts

#define DRV_BM64_UR_TX_BUF_SIZE              300
static uint8_t      UR_TxBuf[DRV_BM64_UR_TX_BUF_SIZE];
static uint16_t     UR_TxBufHead;
static uint16_t     UR_TxBufTail;
typedef enum {
	DRV_BM64_TXRX_BUF_EMPTY,
	DRV_BM64_TXRX_BUF_OK,
	DRV_BM64_TXRX_BUF_FULL
} DRV_BM64_TXRX_BUF_STATUS;
static DRV_BM64_TXRX_BUF_STATUS  UR_TxBufStatus;

typedef enum {
    DRV_BM64_CMD_SEND_STATE_IDLE = 0,         //no data in the queue, MFB is low
    DRV_BM64_CMD_SEND_MFB_HIGH_WAITING,       //set MFB to high, and wait for 2ms
    DRV_BM64_CMD_SEND_DATA_SENDING,           //UART interface is working
    DRV_BM64_CMD_SEND_DATA_SENT_WAITING,      //wait for another 10ms after all data are sent out
} DRV_BM64_CMD_SEND_STATE;

DRV_BM64_CMD_SEND_STATE CMD_SendState;
static uint8_t CommandStartMFBWaitTimer;
static uint8_t CommandSentMFBWaitTimer;

static struct {
    uint8_t MMI_ACTION_status;// : 4;
    uint8_t MUSIC_CTRL_status;// : 4;
    uint8_t DISCOVERABLE_status;// : 4;
    uint8_t READ_LINK_MODE_status;// : 4;
    uint8_t CHG_BD_NAME_status;// : 4;    
    uint8_t READ_BD_ADDRESS_status;// : 4;
    uint8_t READ_BD_NAME_status;// : 4;    
    uint8_t READ_PAIR_RECORD_status;// : 4;
    uint8_t LINK_BACK_status;// : 4;
    uint8_t DISCONNECT_PROFILE_status;// : 4;
    uint8_t SET_OVERALL_GAIN_status;// : 4;
    uint8_t SET_GPIO_CTRL_status;// : 4;
    uint8_t SPP_DATA_status;// : 4;
    uint8_t BTM_UTILITY_REQ_status;// : 4;
    uint8_t LE_SIGNALING_status;// : 4;
    uint8_t SET_RX_BUFFER_SIZE_status;
    uint8_t PROFILE_LINK_BACK_status;
} CommandAckStatus; //Command ACK status

/*======================================*/
/*  function implementation  */
/*======================================*/
/*------------------------------------------------------------*/
static void StartToSendCommand( void )
{
    switch(CMD_SendState)
    {
        case DRV_BM64_CMD_SEND_MFB_HIGH_WAITING:
            //MFB waiting, do nothing
            break;

        case DRV_BM64_CMD_SEND_DATA_SENDING:
            //data is going, do nothing
            break;

        case DRV_BM64_CMD_SEND_DATA_SENT_WAITING:
            CMD_SendState = DRV_BM64_CMD_SEND_DATA_SENDING;
            DRV_BM64_UART_TransferFirstByte();
            break;

        case DRV_BM64_CMD_SEND_STATE_IDLE:
            CMD_SendState = DRV_BM64_CMD_SEND_MFB_HIGH_WAITING;
            //DRV_BM64_MFB_SetHigh();
            CommandStartMFBWaitTimer = 3;      //wait 2 - 3ms
            break;
    }
}
/*------------------------------------------------------------*/
static bool copySendingCommandToBuffer(uint8_t* data, uint16_t size)
{
    bool buf_result = true;
    uint8_t ur_tx_buf_status_save = UR_TxBufStatus;
    uint16_t ur_tx_buf_head_save = UR_TxBufHead;

    if(UR_TxBufStatus !=  DRV_BM64_TXRX_BUF_FULL)
    {
        while(size--)
        {
            UR_TxBuf[UR_TxBufHead++] = *data++;

            if(UR_TxBufHead >= DRV_BM64_UR_TX_BUF_SIZE)
                UR_TxBufHead = 0;

            if(UR_TxBufHead ==  UR_TxBufTail)
            {
                if(size)
                {
                    buf_result = false;
                    UR_TxBufStatus = ur_tx_buf_status_save;		//restore in this case
                    UR_TxBufHead = ur_tx_buf_head_save;                 //restore in this case
                }
                else
                {
                    UR_TxBufStatus =  DRV_BM64_TXRX_BUF_FULL;
                }
                break;
            }
            else
            {
                UR_TxBufStatus = DRV_BM64_TXRX_BUF_OK;
            }
        }

        if(buf_result)
        {
            StartToSendCommand();
        }
    }
    else
    {
        buf_result = false;
    }
    return buf_result;
}

/*------------------------------------------------------------*/
static uint8_t calculateChecksum(uint8_t* startByte, uint8_t* endByte)
{
    uint8_t checksum = 0;
    while(startByte <= endByte)
    {
        checksum += *startByte;
        startByte++;
    }
    checksum = ~checksum + 1;
    return checksum;
}

static uint8_t calculateChecksum2(uint8_t checksum, uint8_t* startByte, uint16_t length)
{
    while(length)
    {
        checksum += *startByte++;
        length--;
    }
    return checksum;
}

/*------------------------------------------------------------*/
uint8_t DRV_BM64_IsAllowedToSendCommand( void )
{
    uint16_t i, size = sizeof(CommandAckStatus);
    uint8_t* p = &CommandAckStatus.MMI_ACTION_status;
    for(i=0; i<size; i++)
    {
        if(*p == DRV_BM64_COMMAND_IS_SENT)
            return false;
        p++;
    }
    return true;
}
/*------------------------------------------------------------*/
void DRV_BM64_SendBytesAsCompleteCommand(uint8_t* command, uint8_t command_length)
{
    copySendingCommandToBuffer(command, command_length);
    DRV_BM64_UpdateAckStatusWhenSent(command[3]);
}

/*------------------------------------------------------------*/
void DRV_BM64_MMI_ActionCommand(uint8_t MMI_ActionCode, uint8_t link_index)
{
    uint8_t command[8];
    command[0] = 0xAA;      //header byte 0
    command[1] = 0x00;      //header byte 1
    command[2] = 0x03;      //length
    command[3] = DRV_BM64_MMI_CMD;      //command ID
    command[4] = link_index;      //link_index, set to 0
    command[5] = MMI_ActionCode;
    command[6] = calculateChecksum(&command[2], &command[5]);
    copySendingCommandToBuffer(&command[0], 7);
    CommandAckStatus.MMI_ACTION_status = DRV_BM64_COMMAND_IS_SENT;
}

/*------------------------------------------------------------*/
void DRV_BM64_MusicControlCommand(uint8_t CtrlCode)
{
    uint8_t command[8];
    command[0] = 0xAA;      //header byte 0
    command[1] = 0x00;      //header byte 1
    command[2] = 0x03;      //length
    command[3] = DRV_BM64_MUSIC_CONTROL_CMD;      //command ID
    command[4] = 0x00;      //link_index, set to 0
    command[5] = CtrlCode;
    command[6] = calculateChecksum(&command[2], &command[5]);
    copySendingCommandToBuffer(&command[0], 7);
    CommandAckStatus.MUSIC_CTRL_status = DRV_BM64_COMMAND_IS_SENT;
}

/*------------------------------------------------------------*/
void DRV_BM64_SendAckToEvent(uint8_t ack_event)
{
    uint8_t command[6];
    command[0] = 0xAA;                      //header byte 0
    command[1] = 0x00;                      //header byte 1
    command[2] = 0x02;                      //length
    command[3] = DRV_BM64_MCU_SEND_EVENT_ACK_CMD;        //command ID
    command[4] = ack_event;                 //event to ack
    command[5] = calculateChecksum(&command[2], &command[4]);
    copySendingCommandToBuffer(&command[0], 6);
}

/*------------------------------------------------------------*/
void DRV_BM64_SendDiscoverableCommand(uint8_t discoverable)
{
    uint8_t command[6];
    command[0] = 0xAA;
    command[1] = 0x00;
    command[2] = 0x02;
    command[3] = DRV_BM64_DISCOVERABLE_CMD;
    command[4] = discoverable;      //0: disable, 1: enable
    command[5] = calculateChecksum(&command[2], &command[4]);
    copySendingCommandToBuffer(&command[0], 6);
    CommandAckStatus.DISCOVERABLE_status = DRV_BM64_COMMAND_IS_SENT;
}

/*------------------------------------------------------------*/
void DRV_BM64_ReadBTMLinkModeCommand( void )
{
    uint8_t command[6];
    command[0] = 0xAA;
    command[1] = 0x00;
    command[2] = 0x02;
    command[3] = DRV_BM64_READ_LINK_MODE_CMD;
    command[4] = 0;         //dummy byte
    command[5] = calculateChecksum(&command[2], &command[4]);
    copySendingCommandToBuffer(&command[0], 6);
    CommandAckStatus.READ_LINK_MODE_status = DRV_BM64_COMMAND_IS_SENT;
}
          
/*------------------------------------------------------------*/
void DRV_BM64_ChangeDeviceNameCommand(const char *name)
{
    char ch;
    uint8_t i;
    uint8_t command[38];
    command[0] = 0xAA;                      //header byte 0
    command[1] = 0x00;                      //header byte 1
    command[2] = 0x21;                      //length
    command[3] = DRV_BM64_CHANGE_DEVICE_NAME_CMD;         //command ID
    for (i=0; i < 32; i++)
    {
        ch = name[i];
        if (ch=='\0')
        {
            break;
        }
        command[4+i] = name[i];
    }
    for (; i < 32; i++)
    {
        command[4+i] = '\0';
    }    
    command[36] = calculateChecksum(&command[2], &command[35]);
    copySendingCommandToBuffer(&command[0], 37);
    CommandAckStatus.CHG_BD_NAME_status = DRV_BM64_COMMAND_IS_SENT;
}          

/*------------------------------------------------------------*/
void DRV_BM64_ReadDeviceAddressCommand(void)
{
    uint8_t command[6];
    command[0] = 0xAA;                      //header byte 0
    command[1] = 0x00;                      //header byte 1
    command[2] = 0x02;                      //length
    command[3] = DRV_BM64_READ_LOCAL_BD_ADDR_CMD;         //command ID
    command[4] = 0x00;                      //dummy byte
    command[5] = calculateChecksum(&command[2], &command[4]);
    copySendingCommandToBuffer(&command[0], 6);
    CommandAckStatus.READ_BD_ADDRESS_status = DRV_BM64_COMMAND_IS_SENT;
}

/*------------------------------------------------------------*/
void DRV_BM64_ReadDeviceNameCommand(void)
{
    uint8_t command[6];
    command[0] = 0xAA;                      //header byte 0
    command[1] = 0x00;                      //header byte 1
    command[2] = 0x02;                      //length
    command[3] = DRV_BM64_READ_LOCAL_DEVICE_NAME_CMD;         //command ID
    command[4] = 0x00;                      //dummy byte
    command[5] = calculateChecksum(&command[2], &command[4]);
    copySendingCommandToBuffer(&command[0], 6);
    CommandAckStatus.READ_BD_NAME_status = DRV_BM64_COMMAND_IS_SENT;
}

/*------------------------------------------------------------*/
void DRV_BM64_GetPairRecordCommand(void)
{
    uint8_t command[6];
    command[0] = 0xAA;                      //header byte 0
    command[1] = 0x00;                      //header byte 1
    command[2] = 0x02;                      //length
    command[3] = DRV_BM64_READ_PAIRING_RECORD_CMD;         //command ID
    command[4] = 0x00;                      //dummy byte
    command[5] = calculateChecksum(&command[2], &command[4]);
    copySendingCommandToBuffer(&command[0], 6);
    CommandAckStatus.READ_PAIR_RECORD_status = DRV_BM64_COMMAND_IS_SENT;
}

/*------------------------------------------------------------*/
void DRV_BM64_LinkBackToLastDevice(void)
{
    uint8_t command[6];
    command[0] = 0xAA;                      //header byte 0
    command[1] = 0x00;                      //header byte 1
    command[2] = 0x02;                      //length
    command[3] = DRV_BM64_PROFILE_LINK_BACK_CMD;         //command ID
    command[4] = 0x00;                      //0x00: last device, 0x01: last HFP device, 0x02: last A2DP device
    command[5] = calculateChecksum(&command[2], &command[4]);
    copySendingCommandToBuffer(&command[0], 6);
    CommandAckStatus.LINK_BACK_status = DRV_BM64_COMMAND_IS_SENT;
}

/*------------------------------------------------------------*/
void DRV_BM64_LinkBackMultipoint(void)
{
    uint8_t command[6];
    command[0] = 0xAA;                      //header byte 0
    command[1] = 0x00;                      //header byte 1
    command[2] = 0x02;                      //length
    command[3] = DRV_BM64_PROFILE_LINK_BACK_CMD;         //command ID
    command[4] = 0x06;                      //multipoint devices
    command[5] = calculateChecksum(&command[2], &command[4]);
    copySendingCommandToBuffer(&command[0], 6);
    CommandAckStatus.LINK_BACK_status = DRV_BM64_COMMAND_IS_SENT;
}

/*------------------------------------------------------------*/
void DRV_BM64_LinkBackToDeviceByBTAddress(uint8_t* address)
{
    uint8_t command[14];
    command[0] = 0xAA;                      //header byte 0
    command[1] = 0x00;                      //header byte 1
    command[2] = 10;                      //length
    command[3] = DRV_BM64_PROFILE_LINK_BACK_CMD;         //command ID
    command[4] = 0x05;              //0x05: link back to device with specified address
    command[5] = 0x00;
    command[6] = 0x07;
    command[7] = *address++;        //byte 0
    command[8] = *address++;        //byte 1
    command[9] = *address++;        //byte 2
    command[10] = *address++;        //byte 3
    command[11] = *address++;        //byte 4
    command[12] = *address++;        //byte 5
    command[13] = calculateChecksum(&command[2], &command[12]);
    copySendingCommandToBuffer(&command[0], 14);
    CommandAckStatus.LINK_BACK_status = DRV_BM64_COMMAND_IS_SENT;
}

/*------------------------------------------------------------*/
void DRV_BM64_DisconnectAllProfile(void)
{
    uint8_t command[6];
    command[0] = 0xAA;                      //header byte 0
    command[1] = 0x00;                      //header byte 1
    command[2] = 0x02;                      //length
    command[3] = DRV_BM64_DISCONNECT_CMD;                //command ID
    command[4] = 0x0f;                      //event to ack
    command[5] = calculateChecksum(&command[2], &command[4]);
    copySendingCommandToBuffer(&command[0], 6);
    CommandAckStatus.DISCONNECT_PROFILE_status = DRV_BM64_COMMAND_IS_SENT;
}

/*------------------------------------------------------------*/
void DRV_BM64_SetOverallGainCommand(uint8_t set_type, uint8_t gain1, uint8_t gain2, uint8_t gain3)
{
    uint8_t command[11];
    command[0] = 0xAA;                      //header byte 0
    command[1] = 0x00;                      //header byte 1
    command[3] = DRV_BM64_SET_OVERALL_GAIN_CMD;                //command ID
    command[4] = 0x00;                      //link index
    command[5] = 0x00;                      //mask bits
    command[6] = set_type;                      //type
    if(set_type == 1 || set_type == 2)
    {
        command[2] = 0x04;                    //length
        command[7] = calculateChecksum(&command[2], &command[6]);
        copySendingCommandToBuffer(&command[0], 8);
    }
    else if(set_type == 3)
    {
        command[2] = 0x07;                    //length
        command[7] = gain1&0x0f;
        command[8] = gain2&0x0f;
        command[9] = gain3&0x0f;
        command[10] = calculateChecksum(&command[2], &command[9]);
        copySendingCommandToBuffer(&command[0], 11);
    }
    else
    {
        command[2] = 0x07;                    //lengthcommand[2] = 0x07;                    //length
        command[7] = gain1&0x7f;
        command[8] = gain2&0x7f;
        command[9] = gain3&0x7f;
        command[10] = calculateChecksum(&command[2], &command[9]);
        copySendingCommandToBuffer(&command[0], 11);
    }
    CommandAckStatus.SET_OVERALL_GAIN_status = DRV_BM64_COMMAND_IS_SENT;
}
/*------------------------------------------------------------*/
void DRV_BM64_SetOverallGain(uint8_t gain1, uint8_t gain2, uint8_t gain3)
{
    uint8_t command[11];
    command[0] = 0xAA;                      //header byte 0
    command[1] = 0x00;                      //header byte 1
    command[2] = 0x07; //lengthcommand[2] = 0x07;                    //length
    command[3] = DRV_BM64_SET_OVERALL_GAIN_CMD;                //command ID
    command[4] = 0x00;                      //link index
    command[5] = 0x00;                      //mask bits
    command[6] = 0x05;                      //type
    command[7] = gain1 & 0x7f;
    command[8] = gain2 & 0x7f;
    command[9] = gain3 & 0x7f;
    command[10] = calculateChecksum(&command[2], &command[9]);
    copySendingCommandToBuffer(&command[0], 11);
    CommandAckStatus.SET_OVERALL_GAIN_status = DRV_BM64_COMMAND_IS_SENT;
}
/*------------------------------------------------------------*/
void DRV_BM64_updateA2DPGain(uint8_t gain)
{
    uint8_t command[11];
    command[0] = 0xAA;                      //header byte 0
    command[1] = 0x00;                      //header byte 1
    command[2] = 0x07; //lengthcommand[2] = 0x07;                    //length
    command[3] = DRV_BM64_SET_OVERALL_GAIN_CMD;                //command ID
    command[4] = 0x00;                      //link index
    command[5] = 0x01;                      //mask bits
    command[6] = 0x04;                      //type
    command[7] = gain & 0x7f;
    command[8] = 0;
    command[9] = 0;
    command[10] = calculateChecksum(&command[2], &command[9]);
    copySendingCommandToBuffer(&command[0], 11);
    
    CommandAckStatus.SET_OVERALL_GAIN_status = DRV_BM64_COMMAND_IS_SENT;
}

/*------------------------------------------------------------*/
void DRV_BM64_updateHFPGain(uint8_t gain)
{
    uint8_t command[11];
    command[0] = 0xAA;                      //header byte 0
    command[1] = 0x00;                      //header byte 1
    command[2] = 0x07;                    //length
    command[3] = DRV_BM64_SET_OVERALL_GAIN_CMD;                //command ID
    command[4] = 0x00;                      //link index
    command[5] = 0x02;                      //mask bits
    command[6] = 0x03;                      //type
    command[7] = 0;
    command[8] = gain & 0x0f;
    command[9] = 0;
    command[10] = calculateChecksum(&command[2], &command[9]);
    copySendingCommandToBuffer(&command[0], 11);
    
    CommandAckStatus.SET_OVERALL_GAIN_status = DRV_BM64_COMMAND_IS_SENT;
}

/*------------------------------------------------------------*/
void DRV_BM64_updateLineInGain(uint8_t gain)
{
    uint8_t command[11];
    command[0] = 0xAA;                      //header byte 0
    command[1] = 0x00;                      //header byte 1
    command[2] = 0x07;                    //length
    command[3] = DRV_BM64_SET_OVERALL_GAIN_CMD;                //command ID
    command[4] = 0x00;                      //link index
    command[5] = 0x04;                      //mask bits
    command[6] = 0x03;                      //type
    command[7] = 0;
    command[8] = 0;
    command[9] = gain & 0x0f;
    command[10] = calculateChecksum(&command[2], &command[9]);
    copySendingCommandToBuffer(&command[0], 11);
    CommandAckStatus.SET_OVERALL_GAIN_status = DRV_BM64_COMMAND_IS_SENT;
}

/*------------------------------------------------------------*/
void DRV_BM64_SendSPPData(uint8_t* addr, uint16_t size, uint8_t link_index)
{
    uint8_t command[11];
    uint8_t checksum = 0;
    uint16_t cmd_length = size + 7;
    command[0] = 0xAA;                      //header byte 0
    command[1] = (uint8_t)(cmd_length>>8);                 //header byte 1
    command[2] = (uint8_t)(cmd_length&0xff);                      //length
    command[3] = DRV_BM64_SEND_SPP_DATA_CMD;             //command ID
    command[4] = link_index;                      //link_index, set to 0
    command[5] = 0x00;          //single packet format
    //total_length: 2byte
    command[6] = (uint8_t)(size>>8);
    command[7]= (uint8_t)(size&0xff);
    //payload_length: 2byte
    command[8] = (uint8_t)(size>>8);
    command[9] = (uint8_t)(size&0xff);
    copySendingCommandToBuffer(&command[0], 10);
    checksum = calculateChecksum2(checksum, &command[1], 9);
    copySendingCommandToBuffer(addr, size);
    checksum = calculateChecksum2(checksum, addr, size);
    checksum = ~checksum + 1;
    copySendingCommandToBuffer(&checksum, 1);
    CommandAckStatus.SPP_DATA_status = DRV_BM64_COMMAND_IS_SENT;
}

void DRV_BM64_LoopBackSPPData(uint8_t* addr, uint16_t total_command_size)
{
    uint8_t command[11];
    uint8_t checksum = 0;
    uint16_t cmd_length = total_command_size+1;
    command[0] = 0xAA;                                  //header byte 0
    command[1] = (uint8_t)(cmd_length>>8);                 //header byte 1(length high byte)
    command[2] = (uint8_t)(cmd_length&0xff);                      //length(length low byte)
    command[3] = DRV_BM64_SEND_SPP_DATA_CMD;             //command ID
    copySendingCommandToBuffer(&command[0], 4);
    checksum = calculateChecksum2(checksum, &command[1], 3);
    //total_command_size -= 1;
    copySendingCommandToBuffer(addr, total_command_size);
    checksum = calculateChecksum2(checksum, addr, total_command_size);
    checksum = ~checksum + 1;
    copySendingCommandToBuffer(&checksum, 1);
    CommandAckStatus.SPP_DATA_status = DRV_BM64_COMMAND_IS_SENT;
}

/*------------------------------------------------------------*/
void DRV_BM64_SetupBTMGPIO( void )
{
    uint8_t command[20];
    command[0] = 0xAA;                      //header byte 0
    command[1] = 0x00;                      //header byte 1
    command[2] = 13;                        //length
    command[3] = DRV_BM64_GPIO_CTRL_CMD;                //command ID
    command[4] = 0xFF;                      //IO_Ctrl_Mask_P0,
    command[5] = 0xDF;                      //IO_Ctrl_Mask_P1,
    command[6] = 0xFF;                      //IO_Ctrl_Mask_P2,
    command[7] = 0xBF;                      //IO_Ctrl_Mask_P3,
    command[8] = 0x00;                      //IO_Setting_P0,
    command[9] = 0x00;                      //IO_Setting_P1,
    command[10] = 0x00;                     //IO_Setting_P2,
    command[11] = 0x00;                     //IO_Setting_P3,
    command[12] = 0x00;                     //Output_Value_P0,
    command[13] = 0x00;                     //Output_Value_P1,
    command[14] = 0x00;                     //Output_Value_P2,
    command[15] = 0x00;                     //Output_Value_P3,
    command[16] = calculateChecksum(&command[2], &command[15]);
    copySendingCommandToBuffer(&command[0], 17);
    CommandAckStatus.SET_GPIO_CTRL_status = DRV_BM64_COMMAND_IS_SENT;
}

/*------------------------------------------------------------*/
void DRV_BM64_EnterLineInMode(uint8_t disable0_enable1, uint8_t analog0_I2S1)
{
    uint8_t command[10];
    command[0] = 0xAA;                      //header byte 0
    command[1] = 0x00;                      //header byte 1
    command[2] = 3;                        //length
    command[3] = DRV_BM64_BTM_UTILITY_FUNCTION_CMD;                //command ID
    command[4] = 1;                         //utility_function_type=0x01
    if(analog0_I2S1)
    {
        if(disable0_enable1)
            command[5] = 3;
        else
            command[5] = 2;
    }
    else
    {
        if(disable0_enable1)
            command[5] = 1;
        else
            command[5] = 0;
    }
    command[6] = calculateChecksum(&command[2], &command[5]);
    copySendingCommandToBuffer(&command[0], 7);
    CommandAckStatus.BTM_UTILITY_REQ_status = DRV_BM64_COMMAND_IS_SENT;
}

/*------------------------------------------------------------*/
void DRV_BM64_SetRXBufferSize( void )
{
    uint8_t command[10];
    command[0] = 0xAA;                      //header byte 0
    command[1] = 0x00;                      //header byte 1
    command[2] = 3;                        //length
    command[3] = DRV_BM64_BT_MCU_UART_RX_BUFF_SIZE_CMD;                //command ID
    command[4] = 0;
    command[5] = 200;
    command[6] = calculateChecksum(&command[2], &command[5]);
    copySendingCommandToBuffer(&command[0], 7);
    CommandAckStatus.SET_RX_BUFFER_SIZE_status = DRV_BM64_COMMAND_IS_SENT;
}
void DRV_BM64_ProfileLinkBack(uint8_t linked_profile, uint8_t link_index)
{
    uint8_t command[10];
    command[0] = 0xAA;                      //header byte 0
    command[1] = 0x00;                      //header byte 1
    command[2] = 3;                        //length
    command[3] = DRV_BM64_ADDITIONAL_PROFILE_LINK_SETUP_CMD;                //command ID
    command[4] = link_index;
    command[5] = linked_profile;
    command[6] = calculateChecksum(&command[2], &command[5]);
    copySendingCommandToBuffer(&command[0], 7);
    CommandAckStatus.PROFILE_LINK_BACK_status = DRV_BM64_COMMAND_IS_SENT;
}
/*------------------------------------------------------------*/
void DRV_BM64_CommandSendInit( void )
{
    UR_TxBufHead = 0;
    UR_TxBufTail = 0;
    UR_TxBufStatus = DRV_BM64_TXRX_BUF_EMPTY;
    DRV_BM64_InitAckStatus();
}

/*------------------------------------------------------------*/
void DRV_BM64_CommandSendTask( void )
{
    switch(CMD_SendState)
    {
        case DRV_BM64_CMD_SEND_MFB_HIGH_WAITING:
            //do nothing, timer interrupt will handle
            break;

        case DRV_BM64_CMD_SEND_DATA_SENDING:
            //do nothing, UART interrupt will handle
            break;

        case DRV_BM64_CMD_SEND_DATA_SENT_WAITING:
            if(!CommandSentMFBWaitTimer)
            {
                DRV_BM64_MFB_SetLow();
                CMD_SendState = DRV_BM64_CMD_SEND_STATE_IDLE;
            }
            break;

        case DRV_BM64_CMD_SEND_STATE_IDLE:
            break;

        default:
            break;
    }
}

/*------------------------------------------------------------*/
void DRV_BM64_CommandSend1MS_event(void)        // gets control every 1 ms
{
    if(CommandSentMFBWaitTimer)
        --CommandSentMFBWaitTimer;

    if( CommandStartMFBWaitTimer)
    {
        -- CommandStartMFBWaitTimer;
        if(! CommandStartMFBWaitTimer)
        {
            if(CMD_SendState == DRV_BM64_CMD_SEND_MFB_HIGH_WAITING)
            {
                CMD_SendState = DRV_BM64_CMD_SEND_DATA_SENDING;
                DRV_BM64_UART_TransferFirstByte();
            }
        }
    }
}

/*------------------------------------------------------------*/
void DRV_BM64_UART_TransferFirstByte( void )
{
    uint8_t data;
    if(UR_TxBufStatus != DRV_BM64_TXRX_BUF_EMPTY)
    {
        data = UR_TxBuf[UR_TxBufTail++];
        if(UR_TxBufTail >= DRV_BM64_UR_TX_BUF_SIZE)
            UR_TxBufTail = 0;
        if(UR_TxBufHead == UR_TxBufTail)
            UR_TxBufStatus = DRV_BM64_TXRX_BUF_EMPTY;
        else
            UR_TxBufStatus = DRV_BM64_TXRX_BUF_OK;
        DRV_BM64_EUSART_Write(data);
    }
}

/*------------------------------------------------------------*/
//uint16_t xx0=0, xx1=0;
void DRV_BM64_UART_TransferNextByte( void )
{
    uint8_t data;
    //xx0++;
    if(UR_TxBufStatus != DRV_BM64_TXRX_BUF_EMPTY)
    {
        data = UR_TxBuf[UR_TxBufTail++];
        if(UR_TxBufTail >= DRV_BM64_UR_TX_BUF_SIZE)
            UR_TxBufTail = 0;
        if(UR_TxBufHead == UR_TxBufTail)
            UR_TxBufStatus = DRV_BM64_TXRX_BUF_EMPTY;
        else
            UR_TxBufStatus = DRV_BM64_TXRX_BUF_OK;
        DRV_BM64_EUSART_Write(data);
    }
    else
    {
        if(CMD_SendState == DRV_BM64_CMD_SEND_DATA_SENDING)
        {
            CMD_SendState = DRV_BM64_CMD_SEND_DATA_SENT_WAITING;
            CommandSentMFBWaitTimer = 10;
        }
    }
}

/*------------------------------------------------------------*/
void DRV_BM64_InitAckStatus(void)
{
    uint16_t i, size = sizeof(CommandAckStatus);
    uint8_t* p = &CommandAckStatus.MMI_ACTION_status;
    for(i=0; i<size; i++)
    {
        *p++ = DRV_BM64_COMMAND_STS_INIT;
    }
}
/*------------------------------------------------------------*/
void DRV_BM64_UpdateAckStatusWhenReceived(uint8_t command_id, uint8_t ack_status)
{
    ack_status &= 0x0f;
    switch (command_id) {
        case DRV_BM64_MMI_CMD:
            CommandAckStatus.MMI_ACTION_status = ack_status;
            break;
        case DRV_BM64_MUSIC_CONTROL_CMD:
            CommandAckStatus.MUSIC_CTRL_status = ack_status;
            break;
        case DRV_BM64_DISCOVERABLE_CMD:
            CommandAckStatus.DISCOVERABLE_status = ack_status;
            break;
        case DRV_BM64_READ_LINK_MODE_CMD:
            CommandAckStatus.READ_LINK_MODE_status = ack_status;
            break;
        case DRV_BM64_CHANGE_DEVICE_NAME_CMD:
            CommandAckStatus.CHG_BD_NAME_status = ack_status;
            break;            
        case DRV_BM64_READ_LOCAL_BD_ADDR_CMD:
            CommandAckStatus.READ_BD_ADDRESS_status = ack_status;
            break;
        case DRV_BM64_READ_LOCAL_DEVICE_NAME_CMD:
            CommandAckStatus.READ_BD_NAME_status = ack_status;
            break;           
        case DRV_BM64_READ_PAIRING_RECORD_CMD:
            CommandAckStatus.READ_PAIR_RECORD_status = ack_status;
            break;
        case DRV_BM64_PROFILE_LINK_BACK_CMD:
            CommandAckStatus.LINK_BACK_status = ack_status;
            break;
        case DRV_BM64_DISCONNECT_CMD:
            CommandAckStatus.DISCONNECT_PROFILE_status = ack_status;
            break;
        case DRV_BM64_SET_OVERALL_GAIN_CMD:
            CommandAckStatus.SET_OVERALL_GAIN_status = ack_status;
            break;
        case DRV_BM64_SEND_SPP_DATA_CMD:
            CommandAckStatus.SPP_DATA_status = ack_status;
            break;
        case DRV_BM64_GPIO_CTRL_CMD:
            CommandAckStatus.SET_GPIO_CTRL_status = ack_status;
            break;
        case DRV_BM64_BTM_UTILITY_FUNCTION_CMD:
            CommandAckStatus.BTM_UTILITY_REQ_status = ack_status;
            break;
        case DRV_BM64_LE_SIGNALING_CMD:
            CommandAckStatus.LE_SIGNALING_status = ack_status;
            break;
        case DRV_BM64_BT_MCU_UART_RX_BUFF_SIZE_CMD:
            CommandAckStatus.SET_RX_BUFFER_SIZE_status = ack_status;
            break;
        case DRV_BM64_ADDITIONAL_PROFILE_LINK_SETUP_CMD:
            CommandAckStatus.PROFILE_LINK_BACK_status = ack_status;
            break;
        default:
            break;
    }
}

/*------------------------------------------------------------*/
void DRV_BM64_UpdateAckStatusWhenSent(uint8_t command_id)
{
    switch (command_id) {
        case DRV_BM64_MMI_CMD:
            CommandAckStatus.MMI_ACTION_status = DRV_BM64_COMMAND_IS_SENT;
            break;
        case DRV_BM64_MUSIC_CONTROL_CMD:
            CommandAckStatus.MUSIC_CTRL_status = DRV_BM64_COMMAND_IS_SENT;
            break;
        case DRV_BM64_DISCOVERABLE_CMD:
            CommandAckStatus.DISCOVERABLE_status = DRV_BM64_COMMAND_IS_SENT;
            break;
        case DRV_BM64_READ_LINK_MODE_CMD:
            CommandAckStatus.READ_LINK_MODE_status = DRV_BM64_COMMAND_IS_SENT;
            break;
        case DRV_BM64_CHANGE_DEVICE_NAME_CMD:
            CommandAckStatus.CHG_BD_NAME_status = DRV_BM64_COMMAND_IS_SENT;
            break;            
        case DRV_BM64_READ_LOCAL_BD_ADDR_CMD:
            CommandAckStatus.READ_BD_ADDRESS_status = DRV_BM64_COMMAND_IS_SENT;
            break;
        case DRV_BM64_READ_LOCAL_DEVICE_NAME_CMD:
            CommandAckStatus.READ_BD_NAME_status = DRV_BM64_COMMAND_IS_SENT;
            break;            
        case DRV_BM64_READ_PAIRING_RECORD_CMD:
            CommandAckStatus.READ_PAIR_RECORD_status = DRV_BM64_COMMAND_IS_SENT;
            break;
        case DRV_BM64_PROFILE_LINK_BACK_CMD:
            CommandAckStatus.LINK_BACK_status = DRV_BM64_COMMAND_IS_SENT;
            break;
        case DRV_BM64_DISCONNECT_CMD:
            CommandAckStatus.DISCONNECT_PROFILE_status = DRV_BM64_COMMAND_IS_SENT;
            break;
        case DRV_BM64_SET_OVERALL_GAIN_CMD:
            CommandAckStatus.SET_OVERALL_GAIN_status = DRV_BM64_COMMAND_IS_SENT;
            break;
        case DRV_BM64_SEND_SPP_DATA_CMD:
            CommandAckStatus.SPP_DATA_status = DRV_BM64_COMMAND_IS_SENT;
            break;
        case DRV_BM64_GPIO_CTRL_CMD:
            CommandAckStatus.SET_GPIO_CTRL_status = DRV_BM64_COMMAND_IS_SENT;
            break;
        case DRV_BM64_BTM_UTILITY_FUNCTION_CMD:
            CommandAckStatus.BTM_UTILITY_REQ_status = DRV_BM64_COMMAND_IS_SENT;
            break;
        case DRV_BM64_LE_SIGNALING_CMD:
            CommandAckStatus.LE_SIGNALING_status = DRV_BM64_COMMAND_IS_SENT;
            break;
        case DRV_BM64_BT_MCU_UART_RX_BUFF_SIZE_CMD:
            CommandAckStatus.SET_RX_BUFFER_SIZE_status = DRV_BM64_COMMAND_IS_SENT;
            break;
        case DRV_BM64_ADDITIONAL_PROFILE_LINK_SETUP_CMD:
            CommandAckStatus.PROFILE_LINK_BACK_status = DRV_BM64_COMMAND_IS_SENT;
        default:
            break;
    }
}

/*------------------------------------------------------------*/
uint8_t DRV_BM64_GetAckStatusByCommandID(uint8_t command_id)
{
    uint8_t ack_status;
    switch (command_id) {
        case DRV_BM64_MMI_CMD:
            ack_status = CommandAckStatus.MMI_ACTION_status;
            break;
        case DRV_BM64_MUSIC_CONTROL_CMD:
            ack_status = CommandAckStatus.MUSIC_CTRL_status;
            break;
        case DRV_BM64_DISCOVERABLE_CMD:
            ack_status = CommandAckStatus.DISCOVERABLE_status;
            break;
        case DRV_BM64_READ_LINK_MODE_CMD:
            ack_status = CommandAckStatus.READ_LINK_MODE_status;
            break;
        case DRV_BM64_CHANGE_DEVICE_NAME_CMD:
            ack_status = CommandAckStatus.CHG_BD_NAME_status;
            break;            
        case DRV_BM64_READ_LOCAL_BD_ADDR_CMD:
            ack_status = CommandAckStatus.READ_BD_ADDRESS_status;
            break;
        case DRV_BM64_READ_LOCAL_DEVICE_NAME_CMD:
            ack_status = CommandAckStatus.READ_BD_NAME_status;
            break;            
        case DRV_BM64_READ_PAIRING_RECORD_CMD:
            ack_status = CommandAckStatus.READ_PAIR_RECORD_status;
            break;
        case DRV_BM64_PROFILE_LINK_BACK_CMD:
            ack_status = CommandAckStatus.LINK_BACK_status;
            break;
        case DRV_BM64_DISCONNECT_CMD:
            ack_status = CommandAckStatus.DISCONNECT_PROFILE_status;
            break;
        case DRV_BM64_SET_OVERALL_GAIN_CMD:
            ack_status = CommandAckStatus.SET_OVERALL_GAIN_status;
            break;
        case DRV_BM64_SEND_SPP_DATA_CMD:
            ack_status = CommandAckStatus.SPP_DATA_status;
            break;
        case DRV_BM64_GPIO_CTRL_CMD:
            ack_status = CommandAckStatus.SET_GPIO_CTRL_status;
            break;
        case DRV_BM64_BTM_UTILITY_FUNCTION_CMD:
            ack_status = CommandAckStatus.BTM_UTILITY_REQ_status;
            break;
        case DRV_BM64_LE_SIGNALING_CMD:
            ack_status = CommandAckStatus.LE_SIGNALING_status;
            break;
        case DRV_BM64_BT_MCU_UART_RX_BUFF_SIZE_CMD:
            ack_status = CommandAckStatus.SET_RX_BUFFER_SIZE_status;
            break;
        case DRV_BM64_ADDITIONAL_PROFILE_LINK_SETUP_CMD:
            ack_status = CommandAckStatus.PROFILE_LINK_BACK_status;
            break;
        default:
            ack_status = DRV_BM64_ACK_STS_OK;            //for unprocessed command returns ACK_OK
            break;
    }
    return ack_status;
}
/*------------------------------------------------------------*/
uint8_t DRV_BM64_GetAckStatus_MMIAction( void )
{
    return CommandAckStatus.MMI_ACTION_status;
}
uint8_t DRV_BM64_GetAckStatusMusicControl( void )
{
    return CommandAckStatus.MUSIC_CTRL_status;
}
uint8_t DRV_BM64_GetAckStatusDiscoverable( void )
{
    return CommandAckStatus.DISCOVERABLE_status;
}
uint8_t DRV_BM64_GetAckStatusBTMLinkMode( void )
{
    return CommandAckStatus.READ_LINK_MODE_status;
}
uint8_t DRV_BM64_GetAckStatusDeviceAddress( void )
{
    return CommandAckStatus.READ_BD_ADDRESS_status;
}
uint8_t DRV_BM64_GetAckStatusDeviceName( void )
{
    return CommandAckStatus.READ_BD_NAME_status;
}
uint8_t DRV_BM64_GetAckStatusReadPairRecord( void )
{
    return CommandAckStatus.READ_PAIR_RECORD_status;
}
uint8_t DRV_BM64_GetAckStatusLinkBack( void )
{
    return CommandAckStatus.LINK_BACK_status;
}
uint8_t DRV_BM64__GetAckStatusDisconnecProfile( void )
{    return CommandAckStatus.DISCONNECT_PROFILE_status;
}
uint8_t DRV_BM64_GetAckStatusSetOverallGainCommand( void )
{
    return CommandAckStatus.SET_OVERALL_GAIN_status;
}
uint8_t DRV_BM64_GetAckStatusSendSPPData( void )
{
    return CommandAckStatus.SPP_DATA_status;
}
uint8_t DRV_BM64_GetAckStatusBTMGPIOCtrl( void )
{
    return CommandAckStatus.SET_GPIO_CTRL_status;
}
uint8_t DRV_BM64_GetAckStatusBTMUtinityReq( void )
{
    return CommandAckStatus.BTM_UTILITY_REQ_status;
}

uint8_t DRV_BM64_GetAckStatusSetRXBufferSize( void )
{
    return CommandAckStatus.SET_RX_BUFFER_SIZE_status;
}

