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

pic32mxBtadkI2sComponentIDList = ["a_drv_i2s", "a_i2s2", "uart2", "sys_time", "core_timer"]		
pic32mxBtadkI2sAutoConnectList = [["bluetooth_bm64", "I2S driver", "a_drv_i2s_1", "drv_i2s"],
                      ["a_drv_i2s_1", "drv_i2s_I2S_dependency", "a_i2s2", "SPI2_I2S"],
					  ["bluetooth_bm64", "USART PLIB", "uart2", "UART2_UART"],
                      ["sys_time", "sys_time_TMR_dependency", "core_timer", "CORE_TIMER_TMR"]]
pic32mxBtadkI2sPinConfigs = [{"pin": 11, "name": "SDI2", "type": "SDI2", "direction": "", "latch": "", "abcd": ""},           # RG7
                    {"pin": 53, "name": "REFCLKO", "type": "REFCLKO", "direction": "", "latch": "", "abcd": ""},     # RF8
                    {"pin": 7,  "name": "SS2 (out)", "type": "SS2 (out)", "direction": "", "latch": "", "abcd": ""}, # RC2
                    {"pin": 10, "name": "SCK2", "type": "SCK2", "direction": "", "latch": "", "abcd": ""},           # RG6
                    {"pin": 12, "name": "SDO2", "type": "SDO2", "direction": "", "latch": "", "abcd": ""},           # RG8
                    {"pin": 49, "name": "U2RX", "type": "U2RX", "direction": "", "latch": "", "abcd": ""},   		 # RF4
                    {"pin": 50, "name": "U2TX", "type": "U2TX", "direction": "", "latch": "", "abcd": ""},   		 # RF5					
					{"pin": 23, "name": "BM64_MFB", "type": "GPIO", "direction": "Out", "latch": "Low", "abcd": ""}, # RB3					
                    {"pin": 1,  "name": "STBYRST", "type": "GPIO", "direction": "Out", "latch": "High", "abcd": ""}] # RG15
					
pic32mx_bluetooth_audio_dev_kit_I2S = bspSupportObj(pic32mxBtadkI2sPinConfigs, pic32mxBtadkI2sComponentIDList, None, pic32mxBtadkI2sAutoConnectList, None)

addBSPSupport("BSP_PIC32MX_Bluetooth_Audio_Development_Kit", "PIC32MX_BTADK_I2S", pic32mx_bluetooth_audio_dev_kit_I2S)