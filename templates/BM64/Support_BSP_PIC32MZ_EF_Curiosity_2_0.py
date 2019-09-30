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

pic32mzCuriosity20ComponentIDList = ["uart1", "sys_time", "core_timer"]		
pic32mzCuriosity20AutoConnectList = [["bluetooth_bm64", "USART PLIB", "uart1", "UART1_UART"],
                      ["sys_time", "sys_time_TMR_dependency", "core_timer", "CORE_TIMER_TMR"]]
pic32mzCuriosity20PinConfigs = [{"pin": 6,   "name": "U1RX", "type": "U1RX", "direction": "", "latch": "", "abcd": ""},					# RRC1
                                {"pin": 13,  "name": "U1TX", "type": "U1TX", "direction": "", "latch": "", "abcd": ""},					# RC4
                                {"pin": 58,  "name": "BM64_MFB", "type": "GPIO", "direction": "Out", "latch": "Low", "abcd": ""}, 	    # RF12
                                {"pin": 9,   "name": "BT_STBYRST", "type": "GPIO", "direction": "Out", "latch": "High", "abcd": ""}]	# RJ12
					
pic32mz_ef_curiosity20 = bspSupportObj(pic32mzCuriosity20PinConfigs, pic32mzCuriosity20ComponentIDList, None, pic32mzCuriosity20AutoConnectList, None)

addBSPSupport("BSP_PIC32MZ_EF_Curiosity_2.0", "PIC32MZ_CURIOSITY_20", pic32mz_ef_curiosity20)
