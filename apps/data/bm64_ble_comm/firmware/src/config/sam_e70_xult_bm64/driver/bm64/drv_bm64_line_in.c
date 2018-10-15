/*******************************************************************************
  BM64 Bluetooth Static Driver implementation

  Company:
    Microchip Technology Inc.

  File Name:
    drv_bm64_line_in.c

  Summary:
   BM64 Bluetooth Static Driver source file for line-in functions.

  Description:
    This file is the implementation of the internal functions of the BM64
    driver related to the line-in interface of the BM64.
 
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
#include <xc.h>
//#include "mcc/mcc.h"
#include "driver/bm64/drv_bm64_line_in.h"
#include "driver/bm64/drv_bm64.h"
#include "driver/bm64/drv_bm64_command_send.h"

// all #defines, enums and non-static functions and variables prefixed by
// DRV_BM64 to avoid name conflicts

#define DRV_BM64_PORT_DETECT_INTERVAL_MS        50
#define DRV_BM64_ANG_POSITIVE_VALIDATE_CNT       3
#define DRV_BM64_ANG_LOSS_VALIDATE_TIME_CNT      3
static  uint16_t ang_audio_detect_cnt = 0;
static uint8_t ang_audio_detect_input;
static uint8_t audio_selected = 0;   //0: BT, 1: aux in

static bool lineInCmdSendReq = false;
uint16_t DRV_BM64_AnalogDetectTimer;

void DRV_BM64_AnalogAudioDetectInit(void)
{
    DRV_BM64_AnalogDetectTimer = 10;
    audio_selected = 0;
    ang_audio_detect_input = 0;     //depend on circuit...
}

void DRV_BM64_AnalogAudioDetectTask()
{   
    if(DRV_BM64_AnalogDetectTimer==0)
    {
        DRV_BM64_AnalogAudioDetect();
        DRV_BM64_AnalogDetectTimer = DRV_BM64_PORT_DETECT_INTERVAL_MS;
    }
    if (DRV_BM64_GetPowerStatus() == DRV_BM64_STATUS_ON || DRV_BM64_GetPowerStatus() == DRV_BM64_STATUS_READY) {
        if (lineInCmdSendReq) {
            if (DRV_BM64_IsAllowedToSendCommand()) {
                lineInCmdSendReq = false;

                if(1/*getI2SAuxInJumper()*/) //HIGH: I2S    -- NEEDS FIXUP!!
                    DRV_BM64_EnterLineInMode(audio_selected, 1);
                else
                    DRV_BM64_EnterLineInMode(audio_selected, 0);
            }
        }
    }
}

void DRV_BM64_AnalogAudioDetect_Timer1MS_event()
{
    if(DRV_BM64_AnalogDetectTimer)
        --DRV_BM64_AnalogDetectTimer;
}

void DRV_BM64_AnalogAudioDetect(void)
{
    uint8_t current_ang_input = 0;//LINE_IN_DETECT_GetValue();

    if(current_ang_input != ang_audio_detect_input)
    {
        ang_audio_detect_cnt = 0;
        ang_audio_detect_input = current_ang_input;
    }
    else
    {
        if(ang_audio_detect_cnt <= 60000)
            ang_audio_detect_cnt ++;
    }

    if(audio_selected == 0) //current is BT selected
    {
        if(!ang_audio_detect_input && (ang_audio_detect_cnt > DRV_BM64_ANG_POSITIVE_VALIDATE_CNT))
        {
            audio_selected = 1;
            lineInCmdSendReq = true;    //BTAPP_LineInEvent(1);
        }
    }
    else        //current is AUX selected
    {
        if(ang_audio_detect_input && (ang_audio_detect_cnt > DRV_BM64_ANG_LOSS_VALIDATE_TIME_CNT))
        {
            audio_selected = 0;
            lineInCmdSendReq = true;    //BTAPP_LineInEvent(0);
        }
    }
}
