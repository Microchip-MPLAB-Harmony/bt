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

pic32mzCuriosity20ComponentIDList = ["uart3", "sys_time", "core_timer"]		
pic32mzCuriosity20AutoConnectList = [["bluetooth_bm71", "USART PLIB", "uart3", "UART3_UART"],
                                     ["sys_time", "sys_time_TMR_dependency", "core_timer", "CORE_TIMER_TMR"]]
pic32mzCuriosity20PinConfigs = [{"pin": 70,  "name": "U3RX", "type": "U3RX", "direction": "", "latch": "", "abcd": ""},                 # RD15
                                {"pin": 90,  "name": "U3TX", "type": "U3TX", "direction": "", "latch": "", "abcd": ""},                 # RF4
                                {"pin": 94,  "name": "BM71_RX_IND", "type": "GPIO", "direction": "Out", "latch": "High", "abcd": ""},   # RK6
                                {"pin": 59, "name": "STBYRST", "type": "GPIO", "direction": "Out", "latch": "High", "abcd": ""}]        # RB12 
					
pic32mz_ef_curiosity20 = bspSupportObj(pic32mzCuriosity20PinConfigs, pic32mzCuriosity20ComponentIDList, None, pic32mzCuriosity20AutoConnectList, None)

addBSPSupport("BSP_PIC32MZ_EF_Curiosity_2.0", "PIC32MZ_CURIOSITY_20", pic32mz_ef_curiosity20)