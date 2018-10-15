# coding: utf-8
##############################################################################
# Copyright (C) 2018 Microchip Technology Inc. and its subsidiaries.
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

noaudioComponentIDList = ["usart0", "sys_time", "tc0"]		
noaudioAutoConnectList = [["bluetooth_bm64", "USART PLIB", "usart0", "USART0_UART"],
                          ["sys_time", "sys_time_TMR_dependency", "tc0", "TC0_TMR"]]
noaudioPinConfigs = [{"pin": 20, "name": "USART0_TXD0", "type": "USART0_TXD0", "direction": "", "latch": "", "abcd": "C"},  # PB1
                     {"pin": 21, "name": "USART0_RXD0", "type": "USART0_RXD0", "direction": "", "latch": "", "abcd": "C"},  # PB0
                     {"pin": 26, "name": "BM64_MFB", "type": "GPIO", "direction": "Out", "latch": "Low", "abcd": ""},       # PB2
                     {"pin": 98, "name": "STBYRST", "type": "GPIO", "direction": "Out", "latch": "High", "abcd": ""}]       # PD11 

i2scComponentIDList = ["drv_i2s", "i2sc1", "usart0", "sys_time", "tc0"]
i2scAutoConnectList = [["bluetooth_bm64", "I2S driver", "drv_i2s_0", "drv_i2s"],
                       ["drv_i2s_0", "drv_i2s_I2S_dependency", "i2sc1", "I2SC1_I2S"],
                       ["bluetooth_bm64", "USART PLIB", "usart0", "USART0_UART"],
                       ["sys_time", "sys_time_TMR_dependency", "tc0", "TC0_TMR"]]
i2scPinConfigs =   [{"pin": 22, "name": "I2SC1_CK", "type": "I2SC1_CK", "direction": "", "latch": "", "abcd": "D"},         # PA20
                    {"pin":  4, "name": "I2SC1_WS", "type": "I2SC1_WS", "direction": "", "latch": "", "abcd": "C"},         # PE0
                    {"pin":  6, "name": "I2SC1_DO0", "type": "I2SC1_DO0", "direction": "", "latch": "", "abcd": "C"},       # PE1
                    {"pin":  7, "name": "I2SC1_DI0", "type": "I2SC1_DI0", "direction": "", "latch": "", "abcd": "C"},       # PE2
                    {"pin": 20, "name": "USART0_TXD0", "type": "USART0_TXD0", "direction": "", "latch": "", "abcd": "C"},   # PB1
                    {"pin": 21, "name": "USART0_RXD0", "type": "USART0_RXD0", "direction": "", "latch": "", "abcd": "C"},   # PB0
                    {"pin": 26, "name": "BM64_MFB", "type": "GPIO", "direction": "Out", "latch": "Low", "abcd": ""},        # PB2
                    {"pin": 98, "name": "STBYRST", "type": "GPIO", "direction": "Out", "latch": "High", "abcd": ""}]        # PD11 

sam_e70_xplained_ultra_noaudio = bspSupportObj(noaudioPinConfigs, noaudioComponentIDList, None, noaudioAutoConnectList, None)					
sam_e70_xplained_ultra_I2SC = bspSupportObj(i2scPinConfigs, i2scComponentIDList, None, i2scAutoConnectList, None)

addBSPSupport("BSP_SAM_E70_Xplained_Ultra", "No Audio", sam_e70_xplained_ultra_noaudio)
addBSPSupport("BSP_SAM_E70_Xplained_Ultra", "I2SC", sam_e70_xplained_ultra_I2SC)