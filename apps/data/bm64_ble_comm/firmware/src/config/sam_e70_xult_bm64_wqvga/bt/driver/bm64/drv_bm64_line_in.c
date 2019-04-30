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

#include "definitions.h"
#include "bt/driver/bm64/drv_bm64_line_in.h"
#include "bt/driver/bm64/drv_bm64.h"
#include "bt/driver/bm64/drv_bm64_command_send.h"

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
