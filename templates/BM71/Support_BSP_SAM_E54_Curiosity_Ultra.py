# coding: utf-8
##############################################################################
# Copyright (C) 2019 Microchip Technology Inc. and its subsidiaries.
#
# Subject to your compliance with these terms, you may use Microchip software
# and any derivatives exclusively with Microchip products. It is your
# responsibility to comply with third party license terms applicable to your
# use of third party software (including open source software) that may
# accompany Microchip software.
#
# THIS SOFTWARE IS SUPPLIED BY MICROCHIP "AS IS". NO WARRANTIES, WHETHER
# EXPRESS, IMPLIED OR STATUTORY, APPLY TO THIS SOFTWARE, INCLUDING ANY IMPLIED
# WARRANTIES OF NON-INFRINGEMENT, MERCHANTABILITY, AND FITNESS FOR A
# PARTICULAR PURPOSE.
#
# IN NO EVENT WILL MICROCHIP BE LIABLE FOR ANY INDIRECT, SPECIAL, PUNITIVE,
# INCIDENTAL OR CONSEQUENTIAL LOSS, DAMAGE, COST OR EXPENSE OF ANY KIND
# WHATSOEVER RELATED TO THE SOFTWARE, HOWEVER CAUSED, EVEN IF MICROCHIP HAS
# BEEN ADVISED OF THE POSSIBILITY OR THE DAMAGES ARE FORESEEABLE. TO THE
# FULLEST EXTENT ALLOWED BY LAW, MICROCHIP'S TOTAL LIABILITY ON ALL CLAIMS IN
# ANY WAY RELATED TO THIS SOFTWARE WILL NOT EXCEED THE AMOUNT OF FEES, IF ANY,
# THAT YOU HAVE PAID DIRECTLY TO MICROCHIP FOR THIS SOFTWARE.
##############################################################################

same54cultComponentIDList = ["sercom2", "sys_time", "tc0"]		
same54cultAutoConnectList = [["bluetooth_bm71", "USART PLIB", "sercom2", "SERCOM2_UART"],
                          ["sys_time", "sys_time_TMR_dependency", "tc0", "TC0_TMR"]]
same54cultPinConfigs = [{"pin": 102, "name": "SERCOM2_PAD0", "type": "SERCOM2_PAD0", "direction": "", "latch": "", "abcd": "C"},    # PB26
                        {"pin": 103, "name": "SERCOM2_PAD1", "type": "SERCOM2_PAD1", "direction": "", "latch": "", "abcd": "C"},    # PB27
                        {"pin": 105, "name": "BM71_RX_IND", "type": "GPIO", "direction": "Out", "latch": "High", "abcd": ""},       # PB29
                        {"pin": 50,  "name": "STBYRST", "type": "GPIO", "direction": "Out", "latch": "High", "abcd": ""}]           # PD11 

sam_e54_curiosity_ultra = bspSupportObj(same54cultPinConfigs, same54cultComponentIDList, None, same54cultAutoConnectList, None)					

addBSPSupport("BSP_SAM_E54_Curiosity_Ultra", "E54_CURIOSITY_ULTRA", sam_e54_curiosity_ultra)